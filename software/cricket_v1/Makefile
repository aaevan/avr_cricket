
default:
	# compile for attiny86 with warnings, optimizations, and 1 MHz clock frequency
	avr-gcc -Wall -Os -DF_CPU=1000000 -mmcu=attiny85 -o cricket_test.o cricket_test.c
	avr-objcopy -j .text -j .data -O ihex cricket_test.o cricket_test.hex
	avrdude -c usbtiny -p t85 -U flash:w:cricket_test.hex:i

clean: /dev/null
	rm -f cricket_test.o cricket_test.hex
