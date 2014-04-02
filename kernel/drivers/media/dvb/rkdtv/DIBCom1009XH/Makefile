#
# Makefile for the kernel multimedia device drivers.
#
obj-$(CONFIG_DIBCOM1009XH_CONTROL)	+= Board/ Bridge/ Driver/  User/ Dibcom1009XH_app.o 
#obj-$(CONFIG_DIBCOM1009XH_CONTROL)	+= Sample/LinuxDriverLibrary/ Dibcom1009XH_app.o
#obj-$(CONFIG_DIBCOM1009XH_CONTROL)	+= Dibcom1009XH_app.o
USER 	= drivers/media/dvb/rkdtv/DIBCom1009XH/User
DRIVER  = drivers/media/dvb/rkdtv/DIBCom1009XH/Driver
DEMOD   = drivers/media/dvb/rkdtv/DIBCom1009XH/Board
FIRMWARE= drivers/media/dvb/rkdtv/DIBCom1009XH/Firmware
BRIDGE	= drivers/media/dvb/rkdtv/DIBCom1009XH/Bridge
DRIVER_CHIP_PATH 	= $(DRIVER)/Common/Dragonfly
BRIDGE_CHIP_PATH	= $(BRIDGE)/Common/Dragonfly

INCLUDE	= 	-I$(USER)/Common/ \
		-I$(DEMOD)/ \
		-I$(DRIVER)/Common/ \
		-I$(DRIVER)/Common/Messages/ \
		-I$(DRIVER)/Driver2Bridge/ \
		-I$(BRIDGE)/Common/ \
		-I$(BRIDGE)/LinuxKernelSpi/ \
		-I$(DRIVER_CHIP_PATH)/ \
		-I$(BRIDGE_CHIP_PATH)/ \




EXTRA_CFLAGS += -Idrivers/media/dvb/dvb-core/  -Idrivers/media/dvb/frontends -Idrivers/media/dvb/rkdtv/ -Idrivers/media/dvb/rkdtv/DIBCom1009XH/ $(INCLUDE) 


