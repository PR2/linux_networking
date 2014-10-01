include $(shell rospack find mk)/cmake.mk

all: src/multi_interface_roam/sigblock.so

clean: sigblock_clean

sigblock_clean:
	rm src/multi_interface_roam/sigblock.so

src/multi_interface_roam/sigblock.so: src/multi_interface_roam/sigblock.c
	cd src/multi_interface_roam; ./sigblock_build.py build_ext --inplace
