### modifications 
- added eeprom upload functionality so fpga can read 1024 non-volatile user programmable bytes upon startup. Jupyter notebook to upload 1024 bytes [here](https://github.com/ruffner/lupa300/tree/master/tools/mojo_trigger).

### original
This is the base code using by the Mojo V3 to load the FPGA and act as a USB to serial port/ADC for the FPGA. 

This code is intended to be used with a modified version of the Arduino IDE. It will not work "out of the box."

See https://alchitry.com/blogs/tutorials/arduino-ide for details.
