include $(shell rospack find mk)/cmake.mk

all: src/multi_interface_roam/sigmask.so

clean: sigmask_clean

sigmask_clean:
	rm src/multi_interface_roam/sigmask.so

src/multi_interface_roam/sigmask.so:
	cd src/multi_interface_roam; ./sigmask_build.py build_ext --inplace
