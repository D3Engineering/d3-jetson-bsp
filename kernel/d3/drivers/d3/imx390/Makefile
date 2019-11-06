# -*-makefile-*-
imx390-objs	:= \
	imx390-main.o \
	imx390-modes.o \
	imx390-mode-1936x1100-SP1L.o \
	imx390-mode-1936x1100-SP1H.o \
	imx390-mode-1936x1100-SP2.o

ifeq ($(CONFIG_D3_IMX390_HDR_ENABLE),y)
	imx390-objs += imx390-mode-1936x1100-HDR.o
endif

obj-$(CONFIG_D3_IMX390) += imx390.o

