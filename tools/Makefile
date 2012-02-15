TARGETS=byteswap
CC=gcc

all: $(TARGETS)

%: %.c
	$(CC) -O2 -Wall -I. -s -o $@ $<

.PHONY: clean install

clean:
	rm -f $(TARGETS) *.o
