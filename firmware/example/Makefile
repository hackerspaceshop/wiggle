
build: example.hex

example.hex: example.c i2cmaster.S
	avr-gcc -DF_CPU=9600000UL -mmcu=attiny13 -std=gnu99 -Os -c -o example.o example.c
	avr-gcc -DF_CPU=9600000UL -mmcu=attiny13 -c -o i2cmaster.o i2cmaster.S
	avr-gcc -mmcu=attiny13 -o example.elf example.o i2cmaster.o
	avr-objcopy -O ihex example.elf example.hex
	avr-size example.elf example.hex

prog:
	avrdude -p attiny13 -P usb  -c usbtiny -U flash:w:example.hex

clean:
	rm -f example.o i2cmaster.o example.elf example.hex

