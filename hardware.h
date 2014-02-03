#ifndef HARDWARE_H_
#define HARDWARE_H_

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>

#include "Arduino.h"
#include <SPI.h>

#define FPGA_BUS_PORT PORTB
#define FPGA_BUS_DDR DDRB
#define FPGA_BUS_PIN PINB
#define FPGA_BUS_MASK 0xFF
#define FPGA_BUS_OFFSET 0

#define ADC_BUS_PORT PORTB
#define ADC_BUS_DDR DDRB
#define ADC_BUS_PIN PINB
#define ADC_BUS_MASK 0xF0
#define ADC_BUS_OFFSET 4

#define CCLK_N 3
#define CS_FLASH_N 2
#define INIT_N 30
#define TX_BUSY_N 30
#define DONE_N 5
#define PROGRAM_N 13

#define CCLK_PORT PORTD
#define CCLK_DDR DDRD
#define CCLK_BIT PD0
#define CCLK_PIN PIND

#define CS_FLASH_PORT PORTD
#define CS_FLASH_DDR DDRD
#define CS_FLASH_BIT PD1
#define CS_FLASH_PIN PIND

#define INIT_PORT PORTD
#define INIT_DDR DDRD
#define INIT_BIT PD5
#define INIT_PIN PIND

#define TX_BUSY_PORT PORTD
#define TX_BUSY_DDR DDRD
#define TX_BUSY_BIT PD5
#define TX_BUSY_PIN PIND

#define DONE_PORT PORTC
#define DONE_DDR DDRC
#define DONE_BIT PC6
#define DONE_PIN PINC

#define PROGRAM_PORT PORTC
#define PROGRAM_DDR DDRC
#define PROGRAM_BIT PC7
#define PROGRAM_PIN PINC

#define MISO_PORT PORTB
#define MISO_DDR DDRB
#define MISO_BIT PB3
#define MISO_PIN PINB

#define SS_PORT PORTB
#define SS_DDR DDRB
#define SS_BIT PB0
#define SS_PIN PINB

#define SET(p, v) p ## _PORT = (p ## _PORT & ~(1 << p ## _BIT)) | (v << p ## _BIT)
#define VALUE(p) (p ## _PIN & (1 << p ## _BIT))
#define OUT(p) p ## _DDR |= (1 << p ## _BIT)
#define IN(p) p ## _DDR &= ~(1 << p ## _BIT)

#define LINESTATE_DTR  1

#endif /* HARDWARE_H_ */
