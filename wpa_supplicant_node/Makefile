PACKAGE_DIR=$(shell pwd)
PACKAGE=$(notdir $(PACKAGE_DIR))

include $(shell rospack find mk)/cmake.mk

all: libwpa_supplicant
clean: libwpa_supplicant_clean

libwpa_supplicant_clean:
	make -C wpa_supplicant/wpa_supplicant clean

libwpa_supplicant:
	make -C wpa_supplicant/wpa_supplicant libwpa_supplicant.so wpa_supplicant.pc

