KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

obj-m += miniPLC_driver.o

all:
        $(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean:
        $(MAKE) -C $(KERNELDIR) M=$(PWD) clean
        rm -f *.mod.* *.o *.symvers *.order 