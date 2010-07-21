PACKAGE_DIR=$(shell pwd)
PACKAGE=$(notdir $(PACKAGE_DIR))

my_all: wpa_supplicant/wpa_supplicant/libwpa_supplicant.a 
	make all

clean: my_clean

my_clean:
	make -C wpa_supplicant/wpa_supplicant clean

wpa_supplicant/wpa_supplicant/libwpa_supplicant.a:
	make -C wpa_supplicant/wpa_supplicant libwpa_supplicant.a

include $(shell rospack find mk)/cmake.mk
