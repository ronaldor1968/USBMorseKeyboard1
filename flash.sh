sudo /home/reboucas/Desenvolvimento/ide/arduino-1.8.13/hardware/tools/avr/bin/avrdude -c usbasp -P usb -p attiny45  -U flash:w:main.hex:i -C /home/reboucas/Desenvolvimento/ide/arduino/hardware/tools/avr/etc/avrdude.conf