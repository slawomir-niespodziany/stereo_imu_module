obj-m += imu.o 
imu-objs := stereo_imu.o stereo_imu_mag.o stereo_imu_mem.o

PWD := $(CURDIR) 

all: 
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules 

clean: 
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean