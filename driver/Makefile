#-------------------------------------------------------------------------------
#   Makefile for MOXA UPort series drvier
#-------------------------------------------------------------------------------
DRV_VER  := 1.3.11
REL_DATE := 12/16/2014
KVER	 := $(shell uname -r)
KDISVER  := $(shell uname -a)
KDIR	 := /lib/modules/$(KVER)/build
PWD	 := $(shell pwd)
TARGET	 := mxu11x0

all:	module

install: module
	mkdir -p /lib/modules/$(KVER)/kernel/drivers/usb/serial
	cp -f ./$(TARGET).ko /lib/modules/$(KVER)/kernel/drivers/usb/serial
	mkdir -p /lib/modules/$(KVER)/misc/
	cp -f ./$(TARGET).ko /lib/modules/$(KVER)/misc/
	depmod -a

ifeq "$(findstring fc6, $(KDISVER))" "fc6"
CFLAGS += -D__FC6__
endif

ifneq ($(KERNELRELEASE),)
obj-m := $(TARGET).o
else
module: envchk
	@echo "  *******************************************************************"
	@echo "  # MOXA UPort 1110/1130/1150/1150I USB to Serial Hub Driver v$(DRV_VER) #"
	@echo "  #                for Linux Kernel 2.6.x & above                   #"
	@echo "  #                                                                 #"
	@echo "  #               release date : $(REL_DATE)                         #"
	@echo "  *******************************************************************"
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules 
endif

envchk:
	@./mxconf | tee build.log

remove:
	rm -f /lib/modules/$(KVER)/kernel/drivers/usb/serial/$(TARGET).ko
	rm -f /lib/modules/$(KVER)/misc/$(TARGET).ko
	depmod -a
	@echo ""
	@echo "**********************************************************************"
	@echo " MOXA UPort 11x0 series driver ver $(DRV_VER) removed successfully."
	@echo "**********************************************************************"

clean: clean_local remove

clean_local:
	@echo "Removing driver files..."
	rm -f *.o
	rm -f .*.o.cmd
	rm -f *.symvers
	rm -f *.order
	rm -f *.markers
	rm -rf ./.tmp_versions
	rm -f $(TARGET).mod*
	rm -f .$(TARGET)*
	rm -f $(TARGET).ko
