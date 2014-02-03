inline void enableDataBus() {
  FPGA_BUS_DDR = 0xFF;
}

void initLoad() {
  SET(INIT, HIGH);
  SET(CCLK, LOW);
  SET(PROGRAM, LOW);
  enableDataBus();
} 

void startLoad() {
  SET(CCLK, LOW);
  SET(PROGRAM, LOW);
  delay(1);
  SET(PROGRAM, HIGH);
  while (!VALUE(INIT));
}

void sendByte(uint8_t b) {
  FPGA_BUS_PORT = b;
  SET(CCLK, HIGH);
  SET(CCLK, LOW);
}

void sendExtraClocks() {
  FPGA_BUS_PORT = 0xff;
  for (int i = 0; i < 10; i++) {
    SET(CCLK, HIGH);
    SET(CCLK, LOW);
  }
}

void loadFromFlash() {
  uint32_t lastAddr = 0;

  initLoad();
  startLoad();

  readFlash(loadBuffer, 0, 5);

  if (loadBuffer[0] != 0xaa){
    return;
  }

  for (uint8_t k = 0; k < 4; k++) {
    lastAddr |= (uint32_t) loadBuffer[k + 1] << (k * 8);
  }

  uint32_t curAddr = 5;

  while (curAddr + 256 < lastAddr) {
    readFlash(loadBuffer, curAddr, 256);
    enableDataBus();
    for (uint16_t i = 0; i < 256; i++) {
      sendByte(loadBuffer[i]);
    }
    curAddr += 256;
  }

  if (curAddr < lastAddr) {
    uint8_t rem = lastAddr - curAddr;
    readFlash(loadBuffer, curAddr, rem);
    enableDataBus();
    for (uint8_t i = 0; i < rem; i++) {
      sendByte(loadBuffer[i]);
    }
  }

  sendExtraClocks();
}
