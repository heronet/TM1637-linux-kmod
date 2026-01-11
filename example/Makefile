# SPDX-License-Identifier: GPL-2.0
obj-m += tm1637.o

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a

load:
	sudo insmod tm1637.ko

unload:
	sudo rmmod tm1637

reload: unload load

dtbo:
	dtc -@ -I dts -O dtb -o tm1637.dtbo tm1637-overlay.dts
	sudo cp tm1637.dtbo /boot/overlays/

.PHONY: all clean install load unload reload dtbo