obj-m := cmabuf.o

all:
	make -C /usr/src/kernel M=$(PWD) modules

clean:
	make -C /usr/src/kernel M=$(PWD) clean