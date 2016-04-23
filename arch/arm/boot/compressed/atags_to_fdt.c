#include <asm/setup.h>
#include <libfdt.h>

#ifdef	CONFIG_ARCH_LM2
#include <asm/string.h>
#include <asm/page.h>
/*
 * 0x7fff_f000 start virtual address
 */
#define	PARAM_ADDR	0x7ffff000
#define	ATAGS_ADDR	0x80000100

struct	lm2_param {
	unsigned int	magic;
	unsigned char	macaddr[6];
	unsigned long long	ramsize;
	unsigned char	bootparam[512];
	unsigned long	initrd_addr;
	unsigned long	initrd_size;
	unsigned long	firm_addr;
	unsigned long	firm_size;
};

static	void	prepare_atag_list(void)
{
	struct	lm2_param	*p_ptr = PARAM_ADDR;
	struct	tag		*t_ptr = ATAGS_ADDR;

	t_ptr->hdr.size = sizeof(struct tag_header) + sizeof(struct tag_core);
	t_ptr->hdr.tag = ATAG_CORE;
	t_ptr->u.core.flags= 0x1;
	t_ptr->u.core.pagesize = PAGE_SIZE;
	t_ptr->u.core.rootdev = 0xff;

	t_ptr = t_ptr + t_ptr->hdr.size;

	t_ptr->hdr.size = sizeof(struct tag_header) + sizeof(struct tag_mem32);
	t_ptr->hdr.tag = ATAG_MEM;
	t_ptr->u.mem.size = (unsigned long)p_ptr->ramsize;
	t_ptr->u.mem.start = 0x80000000;

	t_ptr = t_ptr + t_ptr->hdr.size;

	t_ptr->hdr.size = sizeof(struct tag_header) + sizeof(struct tag_initrd);
	t_ptr->hdr.tag = ATAG_INITRD;
	t_ptr->u.initrd.start = p_ptr->initrd_addr;
	t_ptr->u.initrd.size = p_ptr->initrd_size;

	t_ptr = t_ptr + t_ptr->hdr.size;

	t_ptr->hdr.size = sizeof(struct tag_header) + 512;
	t_ptr->hdr.tag = ATAG_CMDLINE;
	memcpy(t_ptr->u.cmdline.cmdline, p_ptr->bootparam, 512);

	return;
	
}

#endif	/* CONFIG_ARCH_LM2 */

#if defined(CONFIG_ARM_ATAG_DTB_COMPAT_CMDLINE_EXTEND)
#define do_extend_cmdline 1
#else
#define do_extend_cmdline 0
#endif

static int node_offset(void *fdt, const char *node_path)
{
	int offset = fdt_path_offset(fdt, node_path);
	if (offset == -FDT_ERR_NOTFOUND)
		offset = fdt_add_subnode(fdt, 0, node_path);
	return offset;
}

static int setprop(void *fdt, const char *node_path, const char *property,
		   uint32_t *val_array, int size)
{
	int offset = node_offset(fdt, node_path);
	if (offset < 0)
		return offset;
	return fdt_setprop(fdt, offset, property, val_array, size);
}

static int setprop_string(void *fdt, const char *node_path,
			  const char *property, const char *string)
{
	int offset = node_offset(fdt, node_path);
	if (offset < 0)
		return offset;
	return fdt_setprop_string(fdt, offset, property, string);
}

static int setprop_cell(void *fdt, const char *node_path,
			const char *property, uint32_t val)
{
	int offset = node_offset(fdt, node_path);
	if (offset < 0)
		return offset;
	return fdt_setprop_cell(fdt, offset, property, val);
}

static const void *getprop(const void *fdt, const char *node_path,
			   const char *property, int *len)
{
	int offset = fdt_path_offset(fdt, node_path);

	if (offset == -FDT_ERR_NOTFOUND)
		return NULL;

	return fdt_getprop(fdt, offset, property, len);
}

static uint32_t get_cell_size(const void *fdt)
{
	int len;
	uint32_t cell_size = 1;
	const uint32_t *size_len =  getprop(fdt, "/", "#size-cells", &len);

	if (size_len)
		cell_size = fdt32_to_cpu(*size_len);
	return cell_size;
}

static void merge_fdt_bootargs(void *fdt, const char *fdt_cmdline)
{
	char cmdline[COMMAND_LINE_SIZE];
	const char *fdt_bootargs;
	char *ptr = cmdline;
	int len = 0;

	/* copy the fdt command line into the buffer */
	fdt_bootargs = getprop(fdt, "/chosen", "bootargs", &len);
	if (fdt_bootargs)
		if (len < COMMAND_LINE_SIZE) {
			memcpy(ptr, fdt_bootargs, len);
			/* len is the length of the string
			 * including the NULL terminator */
			ptr += len - 1;
		}

	/* and append the ATAG_CMDLINE */
	if (fdt_cmdline) {
		len = strlen(fdt_cmdline);
		if (ptr - cmdline + len + 2 < COMMAND_LINE_SIZE) {
			*ptr++ = ' ';
			memcpy(ptr, fdt_cmdline, len);
			ptr += len;
		}
	}
	*ptr = '\0';

	setprop_string(fdt, "/chosen", "bootargs", cmdline);
}

/*
 * Convert and fold provided ATAGs into the provided FDT.
 *
 * REturn values:
 *    = 0 -> pretend success
 *    = 1 -> bad ATAG (may retry with another possible ATAG pointer)
 *    < 0 -> error from libfdt
 */
int atags_to_fdt(void *atag_list, void *fdt, int total_space)
{
	struct tag *atag = atag_list;
	/* In the case of 64 bits memory size, need to reserve 2 cells for
	 * address and size for each bank */
	uint32_t mem_reg_property[2 * 2 * NR_BANKS];
	int memcount = 0;
	int ret, memsize;

#ifdef	CONFIG_ARCH_LM2
	prepare_atag_list();
#endif	/* CONFIG_ARCH_LM2 */

	/* make sure we've got an aligned pointer */
	if ((u32)atag_list & 0x3)
		return 1;

	/* if we get a DTB here we're done already */
	if (*(u32 *)atag_list == fdt32_to_cpu(FDT_MAGIC))
	       return 0;

	/* validate the ATAG */
	if (atag->hdr.tag != ATAG_CORE ||
	    (atag->hdr.size != tag_size(tag_core) &&
	     atag->hdr.size != 2))
		return 1;

	/* let's give it all the room it could need */
	ret = fdt_open_into(fdt, fdt, total_space);
	if (ret < 0)
		return ret;

	for_each_tag(atag, atag_list) {
		if (atag->hdr.tag == ATAG_CMDLINE) {
			/* Append the ATAGS command line to the device tree
			 * command line.
			 * NB: This means that if the same parameter is set in
			 * the device tree and in the tags, the one from the
			 * tags will be chosen.
			 */
			if (do_extend_cmdline)
				merge_fdt_bootargs(fdt,
						   atag->u.cmdline.cmdline);
			else
				setprop_string(fdt, "/chosen", "bootargs",
					       atag->u.cmdline.cmdline);
		} else if (atag->hdr.tag == ATAG_MEM) {
			if (memcount >= sizeof(mem_reg_property)/4)
				continue;
			if (!atag->u.mem.size)
				continue;
			memsize = get_cell_size(fdt);

			if (memsize == 2) {
				/* if memsize is 2, that means that
				 * each data needs 2 cells of 32 bits,
				 * so the data are 64 bits */
				uint64_t *mem_reg_prop64 =
					(uint64_t *)mem_reg_property;
				mem_reg_prop64[memcount++] =
					cpu_to_fdt64(atag->u.mem.start);
				mem_reg_prop64[memcount++] =
					cpu_to_fdt64(atag->u.mem.size);
			} else {
				mem_reg_property[memcount++] =
					cpu_to_fdt32(atag->u.mem.start);
				mem_reg_property[memcount++] =
					cpu_to_fdt32(atag->u.mem.size);
			}

		} else if (atag->hdr.tag == ATAG_INITRD2) {
			uint32_t initrd_start, initrd_size;
			initrd_start = atag->u.initrd.start;
			initrd_size = atag->u.initrd.size;
			setprop_cell(fdt, "/chosen", "linux,initrd-start",
					initrd_start);
			setprop_cell(fdt, "/chosen", "linux,initrd-end",
					initrd_start + initrd_size);
		}
	}

	if (memcount) {
		setprop(fdt, "/memory", "reg", mem_reg_property,
			4 * memcount * memsize);
	}

	return fdt_pack(fdt);
}
