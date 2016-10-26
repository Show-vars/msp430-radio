CC=msp430-elf-gcc -mmcu=msp430g2553
DBG=mspdebug


SOURCES=$(wildcard src/*.c)
OUTPUT=out.elf

STRIPARGS=-Os -ffunction-sections -Wl,--gc-sections -fno-asynchronous-unwind-tables -Wl,--strip-all
DBGARGS=rf2500
all: compile

compile:
	$(CC) $(SOURCES) -lm -o $(OUTPUT)

compile-stripped:
	$(CC) $(SOURCES) -lm $(STRIPARGS) -o $(OUTPUT)

prog: compile-stripped
	$(DBG) $(DBGARGS) "prog $(OUTPUT)"

clean:
	-@rm $(OUTPUT)
