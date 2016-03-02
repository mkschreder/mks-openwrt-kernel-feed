This repository contains extra hardware support for OpenWRT which I'm working
on. 

ssd1306fb: OLED display driver
 - implements framebuffer device for this display. This driver already exists
   in linux kernel tree as ssd1307fb however the other driver did not work with
   my display so I modified it so that it would work. 

ch341-i2c: ch341 i2c interface driver
 - implements i2c support for the ch341 controller chip. I have implemented
   proper usb packet splitting for this chip so that we can increase
   performance to the maximum that the chip can support. This driver has been
   tested with both the ssd1306 display and an at24 eeprom. 

About me

I'm a freelance consultant working with linux software development. If you
think that I can add value to your products then you should contact me:
mkschreder.uk@gmail.com

Github: https://github.com/mkschreder/
