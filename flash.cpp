#include "flash.h"

void waitBusy() {
  SET(CS_FLASH, LOW);
  SPI.transfer(0x05); //read status register
  while (SPI.transfer(0) & 0x01)
    ;
  SET(CS_FLASH, HIGH);
}

void SPI_Setup() {
  OUT(SS); // prevent SPI from entering slave mode
  SET(SS, HIGH);
  IN(MISO);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
}

void eraseFlash() {
  SPI_Setup();

  SET(CS_FLASH, LOW);
  SPI.transfer(0x06); // Write mode
  SET(CS_FLASH, HIGH);

  SET(CS_FLASH, LOW);
  SPI.transfer(0x01); // Write status reg
  SPI.transfer(0x00); // Disable protection
  SET(CS_FLASH, HIGH);

  SET(CS_FLASH, LOW);
  SPI.transfer(0x06); // Write mode
  SET(CS_FLASH, HIGH);

  SET(CS_FLASH, LOW);
  SPI.transfer(0x60); // Full chip erase
  SET(CS_FLASH, HIGH);

  waitBusy(); // Wait for operation to finish

  SPI.end();
}

void writeByteFlash(uint32_t address, uint8_t b) {
  SPI_Setup();

  waitBusy(); // wait for other writes

  SET(CS_FLASH, LOW);
  SPI.transfer(0x06); // Write mode
  SET(CS_FLASH, HIGH);

  SET(CS_FLASH, LOW);
  SPI.transfer(0x02); // Single write
  SPI.transfer(address >> 16);
  SPI.transfer(address >> 8);
  SPI.transfer(address);
  SPI.transfer(b);
  SET(CS_FLASH, HIGH);

  SPI.end();
}

void waitHardwareBusy() {
  SET(CS_FLASH, LOW);
  delayMicroseconds(1); // change to 1
  while (digitalRead(MISO) == 0)
    ;
  SET(CS_FLASH, HIGH);
}

void writeFlash(uint32_t startAddress, uint8_t *data, uint16_t length) {
  if (length < 2) {
    if (length == 1)
      writeByteFlash(startAddress, data[0]);
    return;
  }

  uint16_t pos;

  SPI_Setup();

  waitBusy();

  SET(CS_FLASH, LOW);
  SPI.transfer(0x70); // Hardware EOW detection
  SET(CS_FLASH, HIGH);

  SET(CS_FLASH, LOW);
  SPI.transfer(0x06); // Write Enable
  SET(CS_FLASH, HIGH);

  SET(CS_FLASH, LOW);
  SPI.transfer(0xAD); // Auto-increment write
  SPI.transfer(startAddress >> 16);
  SPI.transfer(startAddress >> 8);
  SPI.transfer(startAddress);
  SPI.transfer(data[0]);
  SPI.transfer(data[1]);
  SET(CS_FLASH, HIGH);

  waitHardwareBusy();

  for (pos = 2; pos <= length - 2; pos += 2) {
    SET(CS_FLASH, LOW);
    SPI.transfer(0xAD); // Auto-increment write
    SPI.transfer(data[pos]);
    SPI.transfer(data[pos + 1]);
    SET(CS_FLASH, HIGH);

    waitHardwareBusy();
  }

  SET(CS_FLASH, LOW);
  SPI.transfer(0x04); // Write Disable
  SET(CS_FLASH, HIGH);

  SET(CS_FLASH, LOW);
  SPI.transfer(0x80); // Disable hardware EOW detection
  SET(CS_FLASH, HIGH);

  if (pos < length)
    writeByteFlash(startAddress + pos, data[pos]);

}

void readFlash(volatile uint8_t* buffer, uint32_t address, uint16_t count) {
  SPI_Setup();

  SET(CS_FLASH, LOW);
  SPI.transfer(0x03);
  SPI.transfer(address >> 16);
  SPI.transfer(address >> 8);
  SPI.transfer(address);

  for (uint16_t c = 0; c < count; c++)
    buffer[c] = SPI.transfer(0);
  SET(CS_FLASH, HIGH);

  SPI.end();
}




