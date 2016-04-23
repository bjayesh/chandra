/*
 * misc.c
 * 
 * This is a collection of several routines from gzip-1.0.3 
 * adapted for Linux.
 *
 * malloc by Hannu Savolainen 1993 and Matthias Urlichs 1994
 *
 * Modified for ARM Linux by Russell King
 *
 * Nicolas Pitre <nico@visuaide.com>  1999/04/14 :
 *  For this code to run directly from Flash, all constant variables must
 *  be marked with 'const' and all other variables initialized at run-time 
 *  only.  This way all non constant variables will end up in the bss segment,
 *  which should point to addresses in RAM and cleared to 0 on start.
 *  This allows for a much quicker boot time.
 */

unsigned int __machine_arch_type;

#include <linux/compiler.h>	/* for inline */
#include <linux/types.h>
#include <linux/linkage.h>

static void putstr(const char *ptr);
/*extern void error(char *x);*/

#include CONFIG_UNCOMPRESS_INCLUDE

#ifdef CONFIG_DEBUG_ICEDCC

#if defined(CONFIG_CPU_V6) || defined(CONFIG_CPU_V6K) || defined(CONFIG_CPU_V7)

static void icedcc_putc(int ch)
{
	int status, i = 0x4000000;

	do {
		if (--i < 0)
			return;

		asm volatile ("mrc p14, 0, %0, c0, c1, 0" : "=r" (status));
	} while (status & (1 << 29));

	asm("mcr p14, 0, %0, c0, c5, 0" : : "r" (ch));
}


#elif defined(CONFIG_CPU_XSCALE)

static void icedcc_putc(int ch)
{
	int status, i = 0x4000000;

	do {
		if (--i < 0)
			return;

		asm volatile ("mrc p14, 0, %0, c14, c0, 0" : "=r" (status));
	} while (status & (1 << 28));

	asm("mcr p14, 0, %0, c8, c0, 0" : : "r" (ch));
}

#else

static void icedcc_putc(int ch)
{
	int status, i = 0x4000000;

	do {
		if (--i < 0)
			return;

		asm volatile ("mrc p14, 0, %0, c0, c0, 0" : "=r" (status));
	} while (status & 2);

	asm("mcr p14, 0, %0, c1, c0, 0" : : "r" (ch));
}

#endif

#define putc(ch)	icedcc_putc(ch)
#endif

static void putstr(const char *ptr)
{
	char c;

	while ((c = *ptr++) != '\0') {
		if (c == '\n')
			putc('\r');
		putc(c);
	}

	flush();
}

/*
 * gzip declarations
 */
extern char input_data[];
extern char input_data_end[];

unsigned char *output_data;
unsigned long free_mem_ptr;
unsigned long free_mem_end_ptr;

#ifndef arch_error
#define arch_error(x)
#endif

static void	num_to_string(unsigned int data)
{
	int i,j;
	char	num;

	for(i=7,j=0; i>=0 ; i--,j++){
		num = (data >> (i * 4)) & 0x0000000f;
		if((num - 9) > 0)
			putc((num-10) + 0x41);
		else
			putc(num + 0x30);
	}
	flush();
}

void error(char *x)
{
	arch_error(x);

	putstr("\n\n");
	putstr(x);
	putstr("\n\n -- System halted");

	while(1);	/* Halt */
}

asmlinkage void __div0(void)
{
	error("Attempting division by 0!");
}

extern int do_decompress(u8 *input, int len, u8 *output, void (*error)(char *x));


void
decompress_kernel(unsigned long output_start, unsigned long free_mem_ptr_p,
		unsigned long free_mem_ptr_end_p,
		int arch_id)
{
	int ret;
	output_data		= (unsigned char *)output_start;
	free_mem_ptr		= free_mem_ptr_p;
	free_mem_end_ptr	= free_mem_ptr_end_p;
	__machine_arch_type	= arch_id;

	arch_decomp_setup();
#if 0
	putstr("Debug mode enter\n");


	num_to_string((unsigned int)input_data);
	putstr("\n");
	num_to_string((unsigned int)input_data_end);
	putstr("\n");
	num_to_string((unsigned int)in_dat);
	putstr("\n");
	num_to_string((unsigned int)in_dat_end);
	putstr("\n");
	num_to_string((unsigned int)output_data);
	putstr("\n");


	ret = getc();
	putc(ret);
#endif	/* yamano debug */
	putstr("Uncompressing Linux...");
	ret = do_decompress(input_data, input_data_end - input_data,
			    output_data, error);
#if 0
	if (ret == -6)	putstr("retunr -6");
	if (ret == -5)	putstr("retunr -5");
	if (ret == -4)	putstr("retunr -4");
	if (ret == -3)	putstr("retunr -3");
	if (ret == -2)	putstr("retunr -2");
#endif
	if (ret)
		error("decompressor returned an error");
	else
		putstr(" done, booting the kernel.\n");
}
