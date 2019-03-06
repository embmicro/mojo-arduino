### Changes to Mojo V3 Firmware
- **Custom EEPROM** added eeprom upload functionality so fpga can read 1024 non-volatile user programmable bytes upon startup. 
- **To Upload** Jupyter notebook to upload 1024 bytes [here](https://github.com/ruffner/lupa300/tree/master/tools/mojo_trigger).
- **At startup** upon FPGA configuration at AVR startup, the 1024 bytes are transferred to the FPGA via SPI.
- **ADC functionality** removed to simplify the use of SPI. 

### original
This is the base code using by the Mojo V3 to load the FPGA and act as a USB to serial port/ADC for the FPGA. 

This code is intended to be used with a modified version of the Arduino IDE. It will not work "out of the box."

See https://alchitry.com/blogs/tutorials/arduino-ide for details.
