SDCC ?= sdcc

ifeq ($(NMEA), 1)
STCCODESIZE ?= 8184
SDCCOPTS ?= --code-size $(STCCODESIZE) --stack-auto --iram-size 256 --xram-size 256 --data-loc 0x30 --disable-warning 126 --disable-warning 59 --disable-warning 283
else
STCCODESIZE ?= 4089
SDCCOPTS ?= --code-size $(STCCODESIZE) --xram-size 0 --data-loc 0x30 --disable-warning 126 --disable-warning 59 --disable-warning 283
endif

STCGAL ?= stcgal
STCGALOPTS ?= 
STCGALPORT ?= /dev/tty.usbmodem588B0417091
STCGALPROT ?= auto
FLASHFILE ?= main.hex
SYSCLK ?= 11059
CFLAGS ?= -DWITHOUT_LEDTABLE_RELOC

ifeq ($(NMEA), 1)
CFLAGS += -DWITH_NMEA
endif

ifeq ($(MOD6), 1)
CFLAGS += -DMOD6
else ifeq ($(MOD6_MIN), 1)
CFLAGS += -DMOD6_MIN
else ifeq ($(MOD6_V2), 1)
CFLAGS += -DMOD6_V2
else ifeq ($(MOD4), 1)
CFLAGS += -DMOD4
else ifeq ($(MOD4_MIN), 1)
CFLAGS += -DMOD4_MIN
else ifeq ($(MOD4_V2), 1)
CFLAGS += -DMOD4_V2
else ifeq ($(MOD_BCD), 1)
CFLAGS += -DMOD_BCD
endif

SRC = src/adc.c src/ds1302.c src/eeprom.c

OBJ=$(patsubst src%.c,build%.rel, $(SRC))

all: main

build/%.rel: src/%.c src/%.h
	mkdir -p $(dir $@)
	$(SDCC) $(SDCCOPTS) $(CFLAGS) -o $@ -c $<

main: $(OBJ)
	$(SDCC) -o build/ src/$@.c $(SDCCOPTS) $(CFLAGS) $^
	@ tail -n 5 build/main.mem | head -n 2
	@ tail -n 1 build/main.mem
	cp build/$@.ihx $@.hex

eeprom:
	sed -ne '/:..1/ { s/1/0/2; p }' main.hex > eeprom.hex

flash:
	$(STCGAL) -p $(STCGALPORT) -P $(STCGALPROT) -t $(SYSCLK) $(STCGALOPTS) $(FLASHFILE)

clean:
	rm -f *.ihx *.hex *.bin
	rm -rf build/*

cpp: SDCCOPTS+=-E
cpp: main
