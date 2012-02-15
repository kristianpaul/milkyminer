MMDIR?=../../..

TARGETS=soc.fpg
SERIAL?=/dev/ttyUSB0

all: $(TARGETS)
clean:
	rm -f $(TARGETS)

soc.fpg:
	$(MAKE) -C $(MMDIR)/tools
	$(MAKE) -C $(MMDIR)/boards/milkymist-one/synthesis -f Makefile.xst
	cp $(MMDIR)/boards/milkymist-one/synthesis/build/system.fpg soc.fpg

flash: all
	jtag -n flashall.batch

connect:
	stty -F $(SERIAL) raw 115200
	while : ; do cat $(SERIAL) ; done & cat > $(SERIAL)

.PHONY: flash clean connect
