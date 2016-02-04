#!/bin/sh

SRC=temphtest.c

AVRDUDEFLAGS=-v -c dapa -p atmega644 -P /dev/parport0 -b 115200
CFLAGS=-Wall -fexec-charset=wh1602b -Os -mcall-prologues -mmcu=atmega644
LDFLAGS=
LIBS=-static -Lwh1602b/ -Lds18b20/ -lwh1602b -lds18b20

PROG=$(basename $(SRC))

.PHONY : program clean

all: $(PROG).hex # generate hex

program: $(PROG).hex
	sudo avrdude $(AVRDUDEFLAGS) -U flash:w:$^

$(PROG).hex: .$(PROG).bin # flash
	avr-objcopy -j .text -j .data -O ihex $^ $(PROG).hex
	avr-size -d $^

.$(PROG).bin: .$(PROG).o # link
	avr-gcc $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

.$(PROG).o: $(SRC) # compile
	avr-gcc $(CFLAGS) -o $@ -c $^

clean: #clean
	- rm -f $(PROG).hex
	- rm -f .$(PROG).bin
	- rm -f .$(PROG).o
