CC=avr-gcc
CFLAGS=-g -Os -Wall -I../include -I../mixer -mcall-prologues -mmcu=atmega324a -DF_CPU=8000000U -Wl,-u,vfprintf -lprintf_flt
OBJ2HEX=avr-objcopy 
TARGET=ballz

program: $(TARGET).hex 
	sudo avrdude -p m324p -F -P usb -c avrispmkII -Uflash:w:$(TARGET).hex

$(TARGET).hex: $(TARGET).obj
	$(OBJ2HEX) -R .eeprom -O ihex $< $@

$(TARGET).obj: $(TARGET).o debug.o fsm.o
	$(CC) -Wl,-u,vfprintf -lprintf_flt -lc -lm $(CFLAGS) -o $@ -Wl,-Map,$(TARGET).map $(TARGET).o debug.o fsm.o -lc
#k$(CC) $(CFLAGS) -o $@ -Wl,-Map,$(TARGET).map $(TARGET).o debug.o

clean:
	rm -f *.hex *.obj *.o *.map
