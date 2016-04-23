/*
 * synopsys_pcie.h
 * Copyright (C) Wind River Systems, Inc.
 */
#ifndef SYNOPSYS_PCIE_H
#define SYNOPSYS_PCIE_H

//-------------------------------------------
// AXI to PEX initialization
//-------------------------------------------

#define ENABLE_PORT 0x1
#define ENABLE_ALL_ACCESS_TYPE 0xF

#define PAB_AXI_AMAP_CTRL_32_CFG  0x02000001
#define PAB_AXI_AMAP_CTRL_64_CFG  0x04000001
#define PAB_AXI_AMAP_CTRL_128_CFG 0x08000001
#define PAB_AXI_AMAP_CTRL_256_CFG 0x10000001

#define PAB_AXI_AMAP_CTRL_32_MEM  0x02000005
#define PAB_AXI_AMAP_CTRL_64_MEM  0x04000005
#define PAB_AXI_AMAP_CTRL_128_MEM 0x08000005
#define PAB_AXI_AMAP_CTRL_256_MEM 0x10000005


//-------------------------------------------
// PEX to AXI initialization for RC
//-------------------------------------------

// PAB_PEX_AMAP_CTRL only applies to root complex
#define PAB_PEX_AMAP_CTRL0      0x04000005 // 64MB prefetchable (DDR access)
#define PAB_PEX_AMAP_CTRL0X      0x80000005 // 2G prefetchable (DDR access)
#define PAB_PEX_AMAP_CTRL1      0x00100005 // 1MB  prefetchable (SP access)
#define PAB_PEX_AMAP_CTRL2      0x04000007 // 64MB non-prefetchable (XYZ access)
#define PAB_PEX_AMAP_CTRL3      0x00000401 // Interrupt for Mailbox, size of window is 1 KB (must be >0)

//-------------------------------------------
// 55xx AXI Target Space
//-------------------------------------------

// - define AXI address of 55xx internal resources
//   - this is fixed by 55xx definition
#define AXI_ADDR_DDR		0x80000000
#define AXI_ADDR_SP		0x05400000
#define AXI_ADDR_XYZ		0xF0000000
#define ENABLE_BIT		0x00000001 

/* DDR space 0x8_0000_0000 and PCI same address */
#define AXI_ADDR_L_DDR		0x00000000
#define AXI_ADDR_H_DDR		0x00000008

// - NOTE, AXI address for partitioning fixed 1.0 GB AXI address space
//   that goes out to PCIE links amongst 3 PCIE controllers for 55xx internal
//   intiator to get out to PCIE link
//   - PCIE1, 512 MB, 0x40000000
//   - PCIE2, 256 MB, 0x60000000
//   - PCIE3, 256 MB, 0x70000000

// - define AXI address used for 55xx internal intiator to go out
//   onto PCIE link to get to PCIE2 or PCIE3
//   - our simulation only supports looping the RC to 1 EP so the
//     address is the same whether PCIE1 loops to PCIE2 or PCIE3
//   - this address must be compatible with the programmed partitioning
//     and the programmable windows setup in PCIE controller
#define AXI_ADDR_PCIE1_TO_PCIE23_CFG	0x40000000
#define AXI_ADDR_PCIE1_TO_PCIE23_DDR	0x50000000
#define AXI_ADDR_PCIE1_TO_PCIE23_SP	0x58000000
#define AXI_ADDR_PCIE1_TO_PCIE23_XYZ	0x5C000000
// - same principle for PCIE2 going out onto PCIElink
//   - note this is only 256 MB space, not 512 MB, so when enumerating
//     in real life you will need to map entire 256 MB for CFG, then when
//     done with that you can remap the 256 MB for things like DDR, SP, etc.
#define AXI_ADDR_PCIE2_TO_PCIE13_CFG	0x60000000
#define AXI_ADDR_PCIE2_TO_PCIE13_DDR	0x68000000
#define AXI_ADDR_PCIE2_TO_PCIE13_SP	0x6c000000
#define AXI_ADDR_PCIE2_TO_PCIE13_XYZ	0x6e000000
// - same principle for PCIE3 going out onto PCIElink
//   - note this is only 256 MB space, not 512 MB, so when enumerating
//     in real life you will need to map entire 256 MB for CFG, then when
//     done with that you can remap the 256 MB for things like DDR, SP, etc.
#define AXI_ADDR_PCIE3_TO_PCIE12_CFG	0x70000000
#define AXI_ADDR_PCIE3_TO_PCIE12_DDR	0x78000000
#define AXI_ADDR_PCIE3_TO_PCIE12_SP	0x7c000000
#define AXI_ADDR_PCIE3_TO_PCIE12_XYZ	0x7e000000

// - KEY NOTE, if the CFG/DDR/SP/XYZ partiitoning above changes the
//   qlAxiToPexInit must change as well so that the correct sizes
//   are programmed into the PCIE#_PAC_AXI_AMAP_CTRL# registers


//*********************************************************************
// - The following is the memory map definition for a sample application
//
//   - PCIE Express (PEX) Address Map Info
//     - In a customer application each of the 3 PCIE controllers will be
//       on a different PCIE link so there will be 3 independent PEX address
//       maps (1 per link)
//     - For simulations, due to the fact that we do many loopback tests
//       from 1 PCIE controller to another, we have defined the PEX memory
//       map so that all 3 controllers are within a single map.  
//
//     - PCIE1_DDR  0000_1000_0000_0000
//     - PCIE1_SP   0000_1100_0000_0000
//     - PCIE1_MSI  0000_1200_0000_0000
//     - PCIE1_XYZ            1000_0000
//
//     - PCIE2_DDR  0000_2000_0000_0000  ----|
//     - PCIE2_SP   0000_2100_0000_0000      |--> when PCIE2 is EP
//     - PCIE2_MSI  0000_2200_0000_0000      |
//     - PCIE2_XYZ            2000_0000  ----|
//
//     - PCIE3_DDR  0000_3000_0000_0000
//     - PCIE3_SP   0000_3100_0000_0000
//     - PCIE3_MSI  0000_3200_0000_0000
//     - PCIE3_XYZ            3000_0000
//
//     - PCIE2_DDR  0000_4000_0000_0000  ----|
//     - PCIE2_SP   0000_4100_0000_0000      |--> when PCIE2 is RC
//     - PCIE2_MSI  0000_4200_0000_0000      |
//     - PCIE2_XYZ            4000_0000  ----|
//
//
//   - AXI Address Map Info
//     - AXI address for resources in 55xx (such as DDR, SP, etc) is fixed
//       by 55xx definition
//     - AXI address for 55xx internal intiator going out on PCIE link falls 
//       within a 1.0 GB window whose location is fixed by 55xx definition, and
//       the partitioning of that 1.0 GB window is also fixed
//
//*********************************************************************

// - define PEX address used to get to PCIE1 internal resources
#define PEX_ADDR_H_PCIE1_DDR 	0x00001000
#define PEX_ADDR_L_PCIE1_DDR 	0x00000000
#define PEX_ADDR_H_PCIE1_SP		0x00001100
#define PEX_ADDR_L_PCIE1_SP		0x00000000
#define PEX_ADDR_H_PCIE1_MSI	0x00001200 
#define PEX_ADDR_L_PCIE1_MSI	0x00000000 
#define PEX_ADDR_PCIE1_XYZ		0x10000000

// - define PEX address used to get to PCIE2 internal resources FOR EP!!!!!
#define PEX_ADDR_H_PCIE2_DDR 	0x00002000
#define PEX_ADDR_L_PCIE2_DDR 	0x00000000
#define PEX_ADDR_H_PCIE2_SP		0x00002100
#define PEX_ADDR_L_PCIE2_SP		0x00000000
#define PEX_ADDR_H_PCIE2_MSI	0x00002200 
#define PEX_ADDR_L_PCIE2_MSI	0x00000000 
#define PEX_ADDR_PCIE2_XYZ		0x20000000

// - define PEX address used to get to PCIE3 internal resources
#define PEX_ADDR_H_PCIE3_DDR 	0x00003000
#define PEX_ADDR_L_PCIE3_DDR 	0x00000000
#define PEX_ADDR_H_PCIE3_SP		0x00003100
#define PEX_ADDR_L_PCIE3_SP		0x00000000
#define PEX_ADDR_H_PCIE3_MSI	0x00003200 
#define PEX_ADDR_L_PCIE3_MSI	0x00000000 
#define PEX_ADDR_PCIE3_XYZ		0x30000000


//*********************************************************************
// - define PEX address used to get to PCIE2 internal resources when PCIE2 is RC
//   - need 2 options because if PCIE2 is RC then the PEX address for
//     its internal resources can not fall between the PEX address for
//     the internal resources of other devices (PCIE1, PCIE3) on the same link
//     - this is because of the way the BASE and LIMIT registers work
//*********************************************************************

// - define PEX address used to get to PCIE2 internal resources FOR RC!!!!!
#define PEX_ADDR_H_PCIE2_DDR_RC 0x00004000
#define PEX_ADDR_L_PCIE2_DDR_RC 0x00000000
#define PEX_ADDR_H_PCIE2_SP_RC  0x00004100
#define PEX_ADDR_L_PCIE2_SP_RC	0x00000000
#define PEX_ADDR_H_PCIE2_MSI_RC	0x00004200 
#define PEX_ADDR_L_PCIE2_MSI_RC	0x00000000 
#define PEX_ADDR_PCIE2_XYZ_RC	0x40000000



//*********************************************************************
// - RC uses BASE and LIMIT registers instead of BARS to determine when
//   a PEX address is meant for its internal resources
//   - this can be very confusing (counter intuitive)
//   - program BASE and LIMIT registers of RC to cover the PEX address
//     space of the EPs on the link
//   - then the PEX address used to get to RC internal resources must be 
//     outside the area specified by the RC BASE and LIMIT registers
//   - see PCI Express to PCI/PCI-X Bridge Specification for details
//     - 32 bit address access
//       - base  pcie address 31:0 = PCIE#_MEM_LIMIT_BASE 15:4,  20'h00000
//       - limit pcie address 31:0 = PCIE#_MEM_LIMIT_BASE 31:20, 20'hfffff 
//     - 64 bit address access
//       - base  pcie address 63:0 = PCIE#_PMEM_BASE_U  31:0, PCIE#_PMEM_LIMIT_BASE_L 15:4,  20'h00000
//       - limit pcie address 63:0 = PCIE#_PMEM_LIMIT_U 31:0, PCIE#_PMEM_LIMIT_BASE_L 31:20, 20'hfffff
//
// - KEY NOTE
//   - the values below need to track the values in PEX_ADDR_*_PCIE#_*
//
//*********************************************************************

#define PCIE1_MEM_LIMIT_BASE 		0x3FFF2000
#define PCIE1_PMEM_LIMIT_BASE_L		0xFFFF0000
#define PCIE1_PMEM_BASE_U		0x00002000
#define PCIE1_PMEM_LIMIT_U		0x000032FF

#define PCIE2_MEM_LIMIT_BASE 		0x3FFF1000
#define PCIE2_PMEM_LIMIT_BASE_L		0xFFFF0000
#define PCIE2_PMEM_BASE_U		0x00001000
#define PCIE2_PMEM_LIMIT_U		0x000032FF

#define PCIE3_MEM_LIMIT_BASE 		0x2FFF1000
#define PCIE3_PMEM_LIMIT_BASE_L		0xFFFF0000
#define PCIE3_PMEM_BASE_U		0x00001000
#define PCIE3_PMEM_LIMIT_U		0x000022FF

#define	PCIE_AXI_INT_INTA		0x00000020

#define	PCIE_PAB_AXI_INT_MISC_STAT	0x00000c00	/* Clear interrupt */


//--------------------------------------------------------------
// - define offset address in DDR that will be used by tests
//   as a scratch area of memeory for data transfers
//   - Jim W says stay away from the upper 25% of DDR since that
//     is where C compiler puts the instruction code
//   - Derek says stay away from first N% since the C compiler puts
//     the canned test pattern that I defined in pcie_test_pattern.h
//     in DDR starting at the first location of DDR
// - make sure this offset is big enough that I do not step
//   on the test pattern, and small enough that I do not step
//   on the instruction code
//--------------------------------------------------------------
#define DDR_SCRATCH_OFFSET    0x00010000



// DMA definitions
//void qlPcieDma (int which, int type,  ql_bits32 src, ql_bits32 dst, ql_bits32 size);
#define DMA_READ  0x0
#define DMA_WRITE 0x1

/**********************************************************************************************************/
#define	RSTGEN_BASE			0x04010000
#define	PCIEWRAP_BASE			0x04A70000
#define	PCIE_GEN3_CONTROLLERS_1_BASE	0x04A40000
#define	PCIE_GEN3_CONTROLLERS_2_BASE	0x04A50000
#define	PCIE_GEN3_CONTROLLERS_3_BASE	0x04A60000
// RSTGEN
#define	RSTGENSWRSTSTATIC10		0x0104
// PCIEWRAP: PCIe General Function registers
#define	PCIE_PHY_RST_CTRL		0x0108
#define	PCIE1_SW_RST			0x0010
#define	PCIE2_SW_RST			0x0014
#define	PCIE3_SW_RST			0x0018
#define	PCIE_SW_BOOTSTRAP		0x003C
#define	PCIE_PHY_CLK_CTRL		0x010C
#define	PCIE_PHY_PIPE_CTRL		0x0110
#define	PCIE_PHY_PIPE_STAT		0x0114
#define	PCIE1_MISC_STAT			0x0030
#define	PCIE2_MISC_STAT			0x0034
#define	PCIE3_MISC_STAT			0x0038
#define	PCIE1_MISC_CTRL			0x0020
#define	PCIE_INT_EN			0x005c
#define	PCIE_INT_EN__PCIETP_WRAP_rerror	0x00000001
#define	PCIE1_INT_EN			0x0060
#define	PCIE1_INT_EN__GDA_PAB		0x00000001
#define	PCIE2_INT_EN			0x0064
#define	PCIE3_INT_EN			0x0068
#define	PCIE_INT_CLR			0x006c
#define	PCIE_INT_CLR__PCIETP_WRAP	0x00000001
#define	PCIE1_INT_CLR			0x0070
#define	PCIE1_INT_CLR__PERST_N_PIN	0x00000020
#define PCIE1_INT_CLR__GDA_PAB		0x00000001
#define	PCIE2_INT_CLR			0x0074
#define	PCIE3_INT_CLR			0x0078
// PCIe Gen3 Controllers registers
#define	PCIE_GPEXD_CORE_CLK_RATIO	0x0440
#define	PCIE_PAB_CTRL			0x0808
#define	PCIE_PAB_AXI_PIO_CTRL0		0x0844
#define	PCIE1_PAB_AXI_PIO_STAT0		0x0848
#define	PCIE1_PAB_PEX_INT_STAT		0x0ba8
#define	PCIE_PAB_AXI_INT_MISC_EN	0x0BEC
#define	PCIE1_PAB_PEX_INT_EN		0x0ba4
#define	PCIE_PAB_AXI_AMAP_CTRL0		0x0CA4
#define	PCIE_PAB_AXI_AMAP_AXI_BASE0	0x0CA8
#define	PCIE_PAB_AXI_AMAP_PEX_BASEL0	0x0CAC
#define	PCIE_PAB_AXI_AMAP_PEX_BASEH0	0x0CB0
#define	PCIE_PAB_AXI_AMAP_CTRL1		0x0CB4
#define	PCIE_PAB_AXI_AMAP_AXI_BASE1	0x0CB8
#define	PCIE_PAB_AXI_AMAP_PEX_BASEL1	0x0CBC
#define	PCIE_PAB_AXI_AMAP_PEX_BASEH1	0x0CC0
#define	PCIE_PAB_AXI_AMAP_CTRL2		0x0CC4
#define	PCIE_PAB_AXI_AMAP_AXI_BASE2	0x0CC8
#define	PCIE_PAB_AXI_AMAP_PEX_BASEL2	0x0CCC
#define	PCIE_PAB_AXI_AMAP_PEX_BASEH2	0x0CD0
#define	PCIE_PAB_AXI_AMAP_CTRL3		0x0CD4
#define	PCIE_PAB_AXI_AMAP_AXI_BASE3	0x0CD8
#define	PCIE_PAB_AXI_AMAP_PEX_BASEL3	0x0CDC
#define	PCIE_PAB_AXI_AMAP_PEX_BASEH3	0x0CE0
#define	PCIE_PAB_PEX_PIO_CTRL0		0x08E4
#define	PCIE_PAB_PEX_AMAP_CTRL0		0x0EA0
#define	PCIE_PAB_PEX_AMAP_AXI_BASE0	0x0EA4
#define	PCIE_PAB_PEX_AMAP_PEX_BASEL0	0x0EA8
#define	PCIE_PAB_PEX_AMAP_PEX_BASEH0	0x0EAC
#define	PCIE_PAB_PEX_AMAP_CTRL1		0x0EB0
#define	PCIE_PAB_PEX_AMAP_AXI_BASE1	0x0EB4
#define	PCIE_PAB_PEX_AMAP_PEX_BASEL1	0x0EB8
#define	PCIE_PAB_PEX_AMAP_PEX_BASEH1	0x0EBC
#define	PCIE_PAB_PEX_AMAP_CTRL2		0x0EC0
#define	PCIE_PAB_PEX_AMAP_AXI_BASE2	0x0EC4
#define	PCIE_PAB_PEX_AMAP_PEX_BASEL2	0x0EC8
#define	PCIE_PAB_PEX_AMAP_PEX_BASEH2	0x0ECC
#define	PCIE_PAB_PEX_AMAP_CTRL3		0x0ED0
#define	PCIE_PAB_PEX_AMAP_AXI_BASE3	0x0ED4
#define	PCIE_PAB_PEX_AMAP_PEX_BASEL3	0x0ED8
#define	PCIE_PAB_PEX_AMAP_PEX_BASEH3	0x0EDC
#define	PCIE_PAB_PEX_AMAP_BAR0_F0	0x0DE4
#define	PCIE_PAB_PEX_AMAP_BAR1_F0	0x0DE8
#define	PCIE_PAB_PEX_AMAP_BAR2_F0	0x0DEC
/* New for 55xx local high address */
#define	PCIE_PAB_AXI_AMAP_AXI_BASE0X	0x0F00
#define	PCIE_PAB_AXI_AMAP_AXI_BASE1X	0x0F04
#define	PCIE_PAB_AXI_AMAP_AXI_BASE2X	0x0F08
#define	PCIE_PAB_AXI_AMAP_AXI_BASE3X	0x0F0C
#define	PCIE_PAB_PEX_AMAP_AXI_BASE0X	0x0F40
#define	PCIE_PAB_PEX_AMAP_AXI_BASE1X	0x0F44
#define	PCIE_PAB_PEX_AMAP_AXI_BASE2X	0x0F48
#define	PCIE_PAB_PEX_AMAP_AXI_BASE3X	0x0F4c	/* May be */
#define	PCIE_PAB_PEX_AMAP_BAR0_F0X	0x0F80
#define	PCIE_PAB_PEX_AMAP_BAR1_F0X	0x0F84
#define	PCIE_PAB_PEX_AMAP_BAR2_F0X	0x0F88

#define	PCIE_GPEXP_CFG_VENDORID		0x0000
#define	PCIE_GPEXP_CFG_COMMAND			0x0004

#define	PCIE_GPEXD_ID			0x0470
#define	PCIE_GPEXD_CLASSCODE		0x0474
#define	PCIE_GPEXP_CFG_SUBVENID_PLIMITUDW	0x002C
#define	PCIE_GPEXD_SUBSYS_ID			0x0478
#define	PCIE_GPEXD_CFG_RDY			0x04B0
#define	PCIE_GPEXP_CFG_CACHE			0x000C
#define	PCIE_GPEXP_CFG_BASE2_PRIBUS		0x0018
#define	PCIE_GPEXP_CFG_BASE3_IOBASE		0x001C
#define	PCIE_GPEXP_CFG_BASE4_MEMBASE		0x0020
#define	PCIE_GPEXP_CFG_BASE5_PMEMBASE		0x0024
#define	PCIE_GPEXP_CFG_X_PBASEUDW		0x0028
#define	PCIE_GPEXP_CFG_SUBVENID_PLIMITUDW	0x002C

#define	PCIE_PHY_RST_CTRL__PHY_RESET__INV_MASK		0xffffffef
#define	PCIE_PHY_RST_CTRL__PHY_RESET__MASK		0x00000010
#define PCIE_PHY_RST_CTRL__PIPE0_RESET_N__MASK		0x00000001
#define	PCIE_PHY_RST_CTRL__PIPE0_RESET_N__INV_MASK	0xfffffffe
#define	PCIE_PHY_RST_CTRL__PIPE1_RESET_N__MASK		0x00000002
#define	PCIE_PHY_RST_CTRL__PIPE1_RESET_N__INV_MASK	0xfffffffd
#define	PCIE_PHY_RST_CTRL__PIPE2_RESET_N__MASK		0x00000004
#define	PCIE_PHY_RST_CTRL__PIPE2_RESET_N__INV_MASK	0xfffffffb
#define	PCIE1_SW_RST__PERST_N__MASK			0x00000020
#define	PCIE1_SW_RST__PAB_N__MASK			0x00000008
#define	PCIE1_SW_RST__PAB_N__INV_MASK			0xfffffff7
#define	PCIE1_SW_RST__AMBA_N__MASK			0x00000004
#define	PCIE1_SW_RST__AMBA_N__INV_MASK			0xfffffffb
#define	PCIE1_SW_RST__PBUS_N__MASK			0x00000002
#define	PCIE1_SW_RST__PBUS_N__INV_MASK			0xfffffffd
#define	PCIE1_SW_RST__LINK_N__MASK			0x00000001
#define	PCIE1_SW_RST__LINK_N__INV_MASK			0xfffffffe
#define	PCIE2_SW_RST__PAB_N__MASK			0x00000008
#define	PCIE2_SW_RST__PAB_N__INV_MASK			0xfffffff7
#define	PCIE2_SW_RST__AMBA_N__MASK			0x00000004
#define	PCIE2_SW_RST__AMBA_N__INV_MASK			0xfffffffb
#define	PCIE2_SW_RST__PBUS_N__MASK			0x00000002
#define	PCIE2_SW_RST__PBUS_N__INV_MASK			0xfffffffd
#define	PCIE2_SW_RST__LINK_N__MASK			0x00000001
#define	PCIE2_SW_RST__LINK_N__INV_MASK			0xfffffffe
#define	PCIE3_SW_RST__PAB_N__MASK			0x00000008
#define	PCIE3_SW_RST__PAB_N__INV_MASK			0xfffffff7
#define	PCIE3_SW_RST__AMBA_N__MASK			0x00000004
#define	PCIE3_SW_RST__AMBA_N__INV_MASK			0xfffffffb
#define	PCIE3_SW_RST__PBUS_N__MASK			0x00000002
#define	PCIE3_SW_RST__PBUS_N__INV_MASK			0xfffffffd
#define	PCIE3_SW_RST__LINK_N__MASK			0x00000001
#define	PCIE3_SW_RST__LINK_N__INV_MASK			0xfffffffe
#define	PCIE_SW_BOOTSTRAP__PCIE1_EP_RC_SEL__MASK	0x00000040
#define	PCIE_SW_BOOTSTRAP__PCIE1_EP_RC_SEL__INV_MASK	0xffffffbf
#define	PCIE_SW_BOOTSTRAP__PCIE2_EP_RC_SEL__MASK	0x00000020
#define	PCIE_SW_BOOTSTRAP__PCIE2_EP_RC_SEL__INV_MASK	0xffffffdf
#define	PCIE_SW_BOOTSTRAP__PCIE3_EP_RC_SEL__MASK	0x00000010
#define	PCIE_SW_BOOTSTRAP__PCIE3_EP_RC_SEL__INV_MASK	0xffffffef
#define	PCIE_SW_BOOTSTRAP__PIPE_PORT_SEL__INV_MASK	0xfffffff0
#define	PCIE_SW_BOOTSTRAP__PIPE_PORT_SEL__SHIFT		0
#define	PCIE_PHY_CLK_CTRL__PHY_REF_USE_PAD__MASK	0x00000200
#define	PCIE_PHY_PIPE_CTRL__MAC0_PCLKREQ_N__INV_MASK	0xfffcffff
#define	PCIE_PHY_PIPE_CTRL__MAC1_PCLKREQ		0x000c0000
#define	PCIE_PHY_PIPE_CTRL__MAC1_PCLKREQ_N__INV_MASK	0xfff3ffff
#define	PCIE_PHY_PIPE_CTRL__MAC2_PCLKREQ		0x00300000
#define	PCIE_PHY_PIPE_STAT__PIPE0_PHYSTATUS__MASK	0x00000001
#define	PCIE_PHY_PIPE_STAT__PIPE1_PHYSTATUS__MASK	0x00000002
#define	PCIE_PHY_PIPE_STAT__PIPE2_PHYSTATUS__MASK	0x00000004
#define	PCIE1_MISC_STAT__GDA_PAB_DL_UP__MASK		0x00000001
#define	PCIE2_MISC_STAT__GDA_PAB_DL_UP__MASK		0x00000001
#define	PCIE3_MISC_STAT__GDA_PAB_DL_UP__MASK		0x00000001
#define	PCIE1_INT_EN__PERST_N_PIN__MASK			0x00000020
#define	PCIE1_INT_EN__GDA_PAB__MASK			0x00000001
#define	PCIE2_INT_EN__PERST_N_PIN__MASK			0x00000020
#define	PCIE2_INT_EN__GDA_PAB__MASK			0x00000001
#define	PCIE3_INT_EN__PERST_N_PIN__MASK			0x00000020
#define	PCIE3_INT_EN__GDA_PAB__MASK			0x00000001
#define	PCIE2_INT_CLR__PERST_N_PIN__MASK		0x00000020
#define	PCIE2_INT_CLR__GDA_PAB__MASK			0x00000001
#define	PCIE3_INT_CLR__PERST_N_PIN__MASK		0x00000020
#define	PCIE3_INT_CLR__GDA_PAB__MASK			0x00000001
#endif	/* SYNOPSYS_PCIE_H */
