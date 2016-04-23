#
# Makefile for the linux kernel.
#

obj-y					:= lm2.o usb.o
obj-$(CONFIG_SMP)			+= platsmp.o
obj-$(CONFIG_SATA_AHCI_PLATFORM)	+= sata.o
obj-$(CONFIG_MMC_SDHCI_PLTFM)		+= sdhci.o
obj-$(CONFIG_SPI_XSPI)			+= xspi.o
obj-$(CONFIG_LM2_GPDMA)			+= dma.o
