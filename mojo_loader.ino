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

uint8_t loadBuffer[384];
RingBuffer_t adcBuffer, serialBuffer;
volatile taskState_t taskState = SERVICE;

uint8_t adcPort = 0x0F;
volatile uint8_t convPort = 0x0F;

/* This is where you should add your own code! Feel free to edit anything here. 
   This function will work just like the Arduino loop() function in that it will
   be called over and over. You should try to not delay too long in the loop to 
   allow the Mojo to enter loading mode when requested. */
void userLoop() {
  uartTask();
  adcTask();
}

/* this is used to undo any setup you did in initPostLoad */
void disablePostLoad() {
  ADCSRA = 0; // disable ADC
  UCSR1B = 0; // disable serial port
  SPI.end();  // disable SPI
  SET(CCLK, LOW);
  OUT(PROGRAM);
  SET(PROGRAM, LOW); // reset the FPGA
  IN(INIT);
  SET(INIT, HIGH); // pullup on INIT
}

/* Here you can do some setup before entering the userLoop loop */
void initPostLoad() {
  Serial.flush();

  // These buffers are used by the demo ADC/Serial->USB code to prevent dropped samples
  RingBuffer_InitBuffer(&adcBuffer, loadBuffer, 128);
  RingBuffer_InitBuffer(&serialBuffer, loadBuffer+128, 256);

  adcPort = 0x0f; // disable the ADC by default
  ADC_BUS_DDR &= ~ADC_BUS_MASK; // make inputs
  ADC_BUS_PORT &= ~ADC_BUS_MASK; // no pull ups

  // Again, the Arduino libraries didn't offer the functionality we wanted
  // so we access the serial port directly. This sets up an interrupt
  // that is used with our own buffer to capture serial input from the FPGA
  UBRR1 = 1; // 0.5 M Baud

  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
  UCSR1A = (1 << U2X1);
  UCSR1B = (1 << TXEN1) | (1 << RXEN1) | (1 << RXCIE1);

  // Setup all the SPI pins
  SET(CS_FLASH, HIGH);
  OUT(SS);
  SET(SS, HIGH);
  SPI_Setup(); // enable the SPI Port

  DDRD |= (1 << 3);
  DDRD &= ~(1 << 2);
  PORTD |= (1 << 2);

  // This pin is used to signal the serial buffer is almost full
  OUT(TX_BUSY);
  SET(TX_BUSY, LOW);
  
  // set progam as an input so that it's possible to use a JTAG programmer with the Mojo
  IN(PROGRAM);

  // the FPGA looks for CCLK to be high to know the AVR is ready for data
  SET(CCLK, HIGH);
}

/* We needed more flexibility than the Arduino libraries provide. This sets 
   the ADC up to free-run and call an interrupt when each new sample is ready */
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

  sei(); // enable interrupts

  loadFromFlash(); // load on power up
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
  case START_LOAD: // command to enter loader mode
    disablePostLoad(); // setup peripherals
    taskState = LOAD; // enter loader mode
    state = IDLE; // in idle state
    break;
  case LOAD:
    w = Serial.read();
    bt = (uint8_t) w;
    if (w >= 0) { // if we have data
      switch (state) {
      case IDLE: // in IDLE we are waiting for a command from the PC
        byteCount = 0;
        transferSize = 0;
        if (bt == 'F') { // write to flash
          destination = 0; // flash
          verify = 0; // don't verify
          state = READ_SIZE;
          Serial.write('R'); // signal we are ready
        }
        if (bt == 'V') { // write to flash and verify
          destination = 0; // flash
          verify = 1; // verify
          state = READ_SIZE;
          Serial.write('R'); // signal we are ready
        }
        if (bt == 'R') { // write to RAM
          destination = 1; // ram
          state = READ_SIZE;
          Serial.write('R'); // signal we are ready
        }
        if (bt == 'E') { //erase
          eraseFlash();
          Serial.write('D'); // signal we are done
        }
        break;
      case READ_SIZE: // we need to read in how many bytes the config data is
        transferSize |= ((uint32_t) bt << (byteCount++ * 8));
        if (byteCount > 3) {
          byteCount = 0;
          if (destination) {
            state = WRITE_TO_FPGA;
            initLoad(); // get the FPGA read for a load
            startLoad(); // start the load
          } 
          else {
            state = WRITE_TO_FLASH;
            eraseFlash();
          }
          Serial.write('O'); // signal the size was read
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
          writeFlash(byteCount + 5 - 256, loadBuffer, 256); // write blocks of 256 bytes at a time for speed

        if (byteCount == transferSize) { // the last block to write

          if (buffIdx != 255) // finish the partial block write
            writeFlash(byteCount + 5 - (buffIdx + 1), loadBuffer,
            buffIdx + 1);

          delayMicroseconds(50); // these are necciary to get reliable writes
          uint32_t size = byteCount + 5;
          for (uint8_t k = 0; k < 4; k++) {
            writeByteFlash(k + 1, (size >> (k * 8)) & 0xFF); // write the size of the config data to the flash
            delayMicroseconds(50);
          }
          delayMicroseconds(50);
          writeByteFlash(0, 0xAA); // 0xAA is used to signal the flash has valid data
          Serial.write('D'); // signal we are done
          Serial.flush(); // make sure it sends
          if (verify) {
            state = VERIFY_FLASH;
          } 
          else {
            state = LOAD_FROM_FLASH;
          }
        }
        break;
      case WRITE_TO_FPGA:
        sendByte(bt); // just send the byte!
        if (++byteCount == transferSize) { // if we are done
          sendExtraClocks(); // send some extra clocks to make sure the FPGA is happy
          state = IDLE;
          taskState = SERVICE; // enter user mode
          initPostLoad();
          Serial.write('D'); // signal we are done
        }
        break;
      case VERIFY_FLASH:
        if (bt == 'S') {
          byteCount += 5;
          for (uint32_t k = 0; k < byteCount; k += 256) { // dump all the flash data
            uint16_t s;
            if (k + 256 <= byteCount) {
              s = 256;
            } 
            else {
              s = byteCount - k;
            }
            readFlash(loadBuffer, k, s); // read blocks of 256
            uint16_t br = Serial.write((uint8_t*) loadBuffer, s); // dump them to the serial port
            k -= (256 - br); // if all the bytes weren't sent, resend them next round
            delay(10); // needed to prevent errors in some computers running Windows (give it time to process the data?)
          }
          state = LOAD_FROM_FLASH;
        }
        break;
      case LOAD_FROM_FLASH:
        if (bt == 'L') {
          loadFromFlash(); // load 'er up!
          Serial.write('D'); // loading done
          state = IDLE;
          taskState = SERVICE;
          initPostLoad();
        }
        break;
      }
    }

    break;
  case SERVICE:
    userLoop(); // loop the user code
    break;
  } 
}

/* This is called when any control lines on the serial port are changed. 
 It requires a modification to the Arduino core code to work.         
 
 This looks for 5 pulses on the DTR line within 250ms. Checking for 5 
 makes sure that false triggers won't happen when the serial port is opened. */
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

/* This checks to see what port the FPGA is requesting. If it hasn't changed then
   no worries, but if it has the ADC needs to be stopped and set to the new port.
   
   It then empties the ADC buffer into the SPI bus so the FPGA can actually have the
   ADC data. */
void adcTask() {
  static uint8_t preScaler = 0x05; // 128
  static uint8_t highPower = 1;
  static uint8_t refSelect = 0x01; // AVcc

  uint8_t adc_bus = (ADC_BUS_PIN & ADC_BUS_MASK) >> ADC_BUS_OFFSET;

  if (adc_bus != adcPort) { // did the requested ADC pin change?
    adcPort = adc_bus;
    if (adcPort < 2 || (adcPort < 10 && adcPort > 3)) { // 0,1,4,5,6,7,8,9
      configADC(preScaler, highPower, refSelect, adcPort); // reconfigure ADC
    } 
    else { // pin is not valid
      ADCSRA = (0 << ADEN); //disable ADC
    }
  }

  while (!RingBuffer_IsEmpty(&adcBuffer)) { // for all the samples
    // Grab two bytes from the ring buffer, do it directly for a speed gain
    uint8_t byte1 = *adcBuffer.Out;
    if (++adcBuffer.Out == adcBuffer.End)
      adcBuffer.Out = adcBuffer.Start;
    uint8_t byte2 = *adcBuffer.Out;
    if (++adcBuffer.Out == adcBuffer.End)
      adcBuffer.Out = adcBuffer.Start;

    uint_reg_t CurrentGlobalInt = GetGlobalInterruptMask();
    GlobalInterruptDisable();

    adcBuffer.Count -= 2; // actually remove the two bytes from the buffer

    SetGlobalInterruptMask(CurrentGlobalInt);

    SET(SS, LOW);
    uint8_t keyWord = SPI.transfer(byte1); // each sample is two bytes
    uint8_t config = SPI.transfer(byte2);
    SET(SS, HIGH);
    if (keyWord == 0xAA) { // the keyWord is used by the FPGA to config the ADC
      preScaler = config & 0x07;
      highPower = (config >> 3) & 0x01;
      refSelect = (config >> 4) & 0x03;
      configADC(preScaler, highPower, refSelect, adcPort);
    }
  }
}

ISR(ADC_vect) { // new ADC sample, save it
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

/* This function handles all the serial to USB work. It works
   much the same way as the ADC task, but it just forwards data
   from one port to the other instead of the ADC to the FPGA. */
void uartTask() {
  if (Serial) { // does the data have somewhere to go?
    uint16_t ct = RingBuffer_GetCount(&serialBuffer);
    if (ct > 0) { // is there data to send?
      if (serialBuffer.Out + ct <= serialBuffer.End) { // does it loop in our buffer?
        ct = Serial.write(serialBuffer.Out, ct); // dump all the date
        serialBuffer.Out += ct;
        if (serialBuffer.Out == serialBuffer.End)
          serialBuffer.Out = serialBuffer.Start; // loop the buffer
      } 
      else { // it looped the ring buffer
        uint8_t* loopend = serialBuffer.Out + ct;
        uint16_t ct2 = loopend - serialBuffer.End;
        uint16_t ct1 = ct - ct2;
        uint16_t ct1s = Serial.write(serialBuffer.Out, ct1); // dump first block
        if (ct1s == ct1) {
          ct2 = Serial.write(serialBuffer.Start, ct2); // dump second block
          serialBuffer.Out = serialBuffer.Start + ct2; // update the pointers
          ct = ct1+ct2;
        } 
        else {
          ct = ct1s;
          serialBuffer.Out += ct;
        }
      }

      uint_reg_t CurrentGlobalInt = GetGlobalInterruptMask();
      GlobalInterruptDisable();

      serialBuffer.Count -= ct; // update the count
      
      SetGlobalInterruptMask(CurrentGlobalInt);
    }

    if (RingBuffer_GetCount(&serialBuffer) < 250) {
      SET(TX_BUSY, LOW); // re-enable the serial port
      serialRXEnable();
    }

    int16_t w;
    while ((w = Serial.read()) >= 0) {
      Serial_SendByte(w);
    }
  }
}

ISR(USART1_RX_vect) { // new serial data!
  *(serialBuffer.In) = UDR1;

  if (++serialBuffer.In == serialBuffer.End)
    serialBuffer.In = serialBuffer.Start;

  serialBuffer.Count++;

  if (serialBuffer.Count >= 250) { // are we almost out of space?
    SET(TX_BUSY, HIGH); // signal we can't take any more
    if (serialBuffer.Count > 254) 
      serialRXDisable(); // if our flag is ignored disable the serial port so it doesn't clog things up
  }
}














