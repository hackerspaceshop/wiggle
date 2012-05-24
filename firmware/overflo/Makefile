
build: wiggle.hex

wiggle.hex: wiggle.c i2cmaster.S
	avr-gcc -DF_CPU=9600000UL -mmcu=attiny13 -std=gnu99 -Os -c -o wiggle.o wiggle.c
	avr-gcc -DF_CPU=9600000UL -mmcu=attiny13 -c -o i2cmaster.o i2cmaster.S
	avr-gcc -mmcu=attiny13 -o wiggle.elf wiggle.o i2cmaster.o
	avr-objcopy -O ihex wiggle.elf wiggle.hex
	avr-size wiggle.elf wiggle.hex

prog:
	avrdude -p attiny13 -P usb  -c usbtiny -U flash:w:wiggle.hex

clean:
	rm -f wiggle.o i2cmaster.o wiggle.elf wiggle.hex

