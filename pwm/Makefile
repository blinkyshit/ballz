CC=avr-gcc
CFLAGS=-g -Os -Wall -I../include -I../mixer -mcall-prologues -mmcu=atmega328p -DF_CPU=16000000U -Wl,-u,vfprintf -lprintf_flt
OBJ2HEX=avr-objcopy 
TARGET=pwm

program: $(TARGET).hex 
	sudo avrdude -p m328p -P usb -c avrispmkII -Uflash:w:$(TARGET).hex

$(TARGET).hex: $(TARGET).obj
	$(OBJ2HEX) -R .eeprom -O ihex $< $@

$(TARGET).obj: $(TARGET).o debug.o
	$(CC) -Wl,-u,vfprintf -lprintf_flt -lc -lm $(CFLAGS) -o $@ -Wl,-Map,$(TARGET).map $(TARGET).o debug.o -lc
#k$(CC) $(CFLAGS) -o $@ -Wl,-Map,$(TARGET).map $(TARGET).o debug.o

clean:
	rm -f *.hex *.obj *.o *.map
