VESC_TOOL ?= vesc_tool

all: clean balance.vescpkg

balance.vescpkg: balance
	$(VESC_TOOL) --buildPkg "balance.vescpkg:balance.lisp:ui.qml:0:README.md:Balance"

balance:
	$(MAKE) -C c_lib

clean:
	rm -f balance.vescpkg
	$(MAKE) -C c_lib clean

.PHONY: all clean balance
