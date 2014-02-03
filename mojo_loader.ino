#include "hardware.h"
#include "ring_buffer.h"
#include <SPI.h>
#include "flash.h"

typedef enum {
  IDLE,
  READ_SIZE,
  WRITE_TO_FLASH,
  WRITE_TO_FPGA,
  VERIFY_FLASH,
  LOAD_FROM_FLASH
} 
loaderState_t;

typedef enum {
  WAIT, START_LOAD, LOAD, SERVICE
} 
taskState_t;

uint8_t loadBuffer[256];
RingBuffer_t adcBuffer, serialBuffer;
volatile taskState_t taskState = SERVICE;

uint8_t adcPort = 0x0F;
volatile uint8_t convPort = 0x0F;

void userLoop() {
  uartTask();
  adcTask();
}

void configADC(uint8_t preScaler, uint8_t highPower, uint8_t refSelect, uint8_t port) {
  ADCSRA = (0 << ADEN); //disable
  ADMUX = (refSelect << REFS0) | (port & 0x07);

  if (port > 7)
    ADCSRB = (1 << MUX5) | (highPower << ADHSM);
  else
    ADCSRB = (highPower << ADHSM);

  convPort = port;
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIE)
    | (preScaler << ADPS0);
}


/* this is used to undo any setup you did in initPostLoad */
void disablePostLoad() {
  ADCSRA = 0; //disable ADC
  UCSR1B = 0; //disable serial port
  SPI.end();
  SET(CCLK, LOW);
  OUT(PROGRAM);
  SET(PROGRAM, LOW);
  IN(INIT);
  SET(INIT, HIGH);
}

/* Here you can do some setup before entering the userLoop loop */
void initPostLoad() {
  Serial.flush();

  RingBuffer_InitBuffer(&adcBuffer, loadBuffer, 128);
  RingBuffer_InitBuffer(&serialBuffer, loadBuffer+128, 128);

  adcPort = 0x0f;
  ADC_BUS_DDR &= ~ADC_BUS_MASK; // make inputs
  ADC_BUS_PORT &= ~ADC_BUS_MASK; // no pull ups

  UBRR1 = 1; // 0.5 M Baud

  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
  UCSR1A = (1 << U2X1);
  UCSR1B = (1 << TXEN1) | (1 << RXEN1) | (1 << RXCIE1);

  SET(CS_FLASH, HIGH);
  OUT(SS);
  SET(SS, HIGH);
  SPI_Setup(); // enable the SPI Port

  DDRD |= (1 << 3);
  DDRD &= ~(1 << 2);
  PORTD |= (1 << 2);

  OUT(TX_BUSY);
  SET(TX_BUSY, LOW);
  
  IN(PROGRAM);

  SET(CCLK, HIGH);
}

void setup() {
  /* Disable clock division */
  clock_prescale_set(clock_div_1);

  OUT(CS_FLASH);
  SET(CS_FLASH, HIGH);
  OUT(CCLK);
  OUT(PROGRAM);

  /* Disable digital inputs on analog pins */
  DIDR0 = 0xF3;
  DIDR2 = 0x03;

  Serial.begin(115200); // Baud rate does nothing

  sei();

  loadFromFlash();
  initPostLoad();
}

void loop() {
  static loaderState_t state = IDLE;
  static int8_t destination;
  static int8_t verify;
  static uint32_t byteCount;
  static uint32_t transferSize;

  int16_t w;
  uint8_t bt;
  uint8_t buffIdx;

  switch (taskState) {
  case WAIT:
    break;
  case START_LOAD:
    disablePostLoad();
    taskState = LOAD;
    state = IDLE;
    break;
  case LOAD:
    w = Serial.read();
    bt = (uint8_t) w;
    if (w >= 0) {
      switch (state) {
      case IDLE:
        byteCount = 0;
        transferSize = 0;
        if (bt == 'F') { // write to flash
          destination = 0;
          verify = 0;
          state = READ_SIZE;
          Serial.write('R');
        }
        if (bt == 'V') { // write to flash and verify
          destination = 0;
          verify = 1;
          state = READ_SIZE;
          Serial.write('R');
        }
        if (bt == 'R') { // write to RAM
          destination = 1;
          state = READ_SIZE;
          Serial.write('R');
        }
        if (bt == 'E') { //erase
          eraseFlash();
          Serial.write('D');
        }
        break;
      case READ_SIZE:
        transferSize |= ((uint32_t) bt << (byteCount++ * 8));
        if (byteCount > 3) {
          byteCount = 0;
          if (destination) {
            state = WRITE_TO_FPGA;
            initLoad();
            startLoad();
          } 
          else {
            state = WRITE_TO_FLASH;
            eraseFlash();
          }
          Serial.write('O');
        }
        break;
      case WRITE_TO_FLASH:
        // we can only use the batch write for even addresses
        // so address 5 is written as a single byte
        if (byteCount == 0)
          writeByteFlash(5, bt);

        buffIdx = (byteCount++ - 1) % 256;
        loadBuffer[buffIdx] = bt;

        if (buffIdx == 255 && byteCount != 0)
          writeFlash(byteCount + 5 - 256, loadBuffer, 256);

        if (byteCount == transferSize) {

          if (buffIdx != 255)
            writeFlash(byteCount + 5 - (buffIdx + 1), loadBuffer,
            buffIdx + 1);

          delayMicroseconds(50);
          uint32_t size = byteCount + 5;
          for (uint8_t k = 0; k < 4; k++) {
            writeByteFlash(k + 1, (size >> (k * 8)) & 0xFF);
            delayMicroseconds(50);
          }
          delayMicroseconds(50);
          writeByteFlash(0, 0xAA);
          Serial.write('D');
          Serial.flush();
          if (verify) {
            state = VERIFY_FLASH;
          } 
          else {
            state = LOAD_FROM_FLASH;
          }
        }
        break;
      case WRITE_TO_FPGA:
        sendByte(bt);
        if (++byteCount == transferSize) {
          sendExtraClocks();
          state = IDLE;
          taskState = SERVICE;
          initPostLoad();
          Serial.write('D');
        }
        break;
      case VERIFY_FLASH:
        if (bt == 'S') {
          byteCount += 5;
          for (uint32_t k = 0; k < byteCount; k += 256) {
            uint16_t s;
            if (k + 256 <= byteCount) {
              s = 256;
            } 
            else {
              s = byteCount - k;
            }
            readFlash(loadBuffer, k, s);
            uint16_t br = Serial.write((uint8_t*) loadBuffer, s);
            k -= (256 - br); // if all the bytes weren't sent, resend them next round
            delay(10); // needed to prevent errors in Windows
          }
          state = LOAD_FROM_FLASH;
        }
        break;
      case LOAD_FROM_FLASH:
        if (bt == 'L') {
          loadFromFlash();
          Serial.write('D');
          state = IDLE;
          taskState = SERVICE;
          initPostLoad();
        }
        break;
      }
    }

    break;
  case SERVICE:
    userLoop();
    break;
  } 
}

/* This is called when any control lines on the serial port are changed. 
 It requires a modification to the Arduino core code to work.          */
void lineStateEvent(unsigned char linestate)
{
  static unsigned long start = 0; 
  static uint8_t falling = 0;
  if (!(linestate & LINESTATE_DTR)) {
    if ((millis() - start) < 250) {
      if (++falling >= 5)
        taskState = START_LOAD;
    } 
    else {
      start = millis();
      falling = 1;
    }
  }
}

void adcTask() {
  static uint8_t preScaler = 0x05; // 128
  static uint8_t highPower = 1;
  static uint8_t refSelect = 0x01; // AVcc

  uint8_t adc_bus = (ADC_BUS_PIN & ADC_BUS_MASK) >> ADC_BUS_OFFSET;

  if (adc_bus != adcPort) {
    adcPort = adc_bus;
    if (adcPort < 2 || (adcPort < 10 && adcPort > 3)) { // 0,1,4,5,6,7,8,9
      configADC(preScaler, highPower, refSelect, adcPort);
    } 
    else {
      ADCSRA = (0 << ADEN); //disable
    }
  }

  while (!RingBuffer_IsEmpty(&adcBuffer)) {
    uint8_t byte1 = *adcBuffer.Out;
    if (++adcBuffer.Out == adcBuffer.End)
      adcBuffer.Out = adcBuffer.Start;
    uint8_t byte2 = *adcBuffer.Out;
    if (++adcBuffer.Out == adcBuffer.End)
      adcBuffer.Out = adcBuffer.Start;

    uint_reg_t CurrentGlobalInt = GetGlobalInterruptMask();
    GlobalInterruptDisable();

    adcBuffer.Count -= 2;

    SetGlobalInterruptMask(CurrentGlobalInt);

    SET(SS, LOW);
    uint8_t keyWord = SPI.transfer(byte1);
    uint8_t config = SPI.transfer(byte2);
    SET(SS, HIGH);
    if (keyWord == 0xAA) {
      preScaler = config & 0x07;
      highPower = (config >> 3) & 0x01;
      refSelect = (config >> 4) & 0x03;
      configADC(preScaler, highPower, refSelect, adcPort);
    }
  }
}

ISR(ADC_vect) {
  RingBuffer_Insert(&adcBuffer, ADCL );
  RingBuffer_Insert(&adcBuffer, (convPort << 4) | ADCH );
}

void serialRXEnable() {
  UCSR1B |= (1 << RXEN1);
}

void serialRXDisable() {
  UCSR1B &= ~(1 << RXEN1);
}

static inline void Serial_SendByte(const char DataByte)
{
  while (!(UCSR1A & (1 << UDRE1)));
  UDR1 = DataByte;
}

void uartTask() {
  if (Serial) { // does the data have somewhere to go?
    uint16_t ct = RingBuffer_GetCount(&serialBuffer);
    if (ct > 0) {
      if (serialBuffer.Out + ct <= serialBuffer.End) {
        serialBuffer.Out += Serial.write(serialBuffer.Out, ct);
        if (serialBuffer.Out == serialBuffer.End)
          serialBuffer.Out = serialBuffer.Start;
      } 
      else {
        uint8_t* end = serialBuffer.Out + ct;
        uint16_t ct2 = end - serialBuffer.End;
        uint16_t ct1 = ct - ct2;
        Serial.write(serialBuffer.Out, ct1);
        Serial.write(serialBuffer.Out, ct2);
        serialBuffer.Out = serialBuffer.Start + ct2;
      }

      uint_reg_t CurrentGlobalInt = GetGlobalInterruptMask();
      GlobalInterruptDisable();

      serialBuffer.Count -= ct;

      SetGlobalInterruptMask(CurrentGlobalInt);
    }

    if (RingBuffer_GetCount(&serialBuffer) < 10) {
      SET(TX_BUSY, LOW);
      serialRXEnable();
    }
  }

  int16_t w;
  while ((w = Serial.read()) >= 0) {
    Serial_SendByte(w);
  }
}

ISR(USART1_RX_vect) {
  RingBuffer_Insert(&serialBuffer, UDR1 );
  if (serialBuffer.Count > 110)
    SET(TX_BUSY, HIGH);
  else
    SET(TX_BUSY, LOW);
  if (serialBuffer.Count > 125)
    serialRXDisable();
}














