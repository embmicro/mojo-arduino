
/*
 * flash.h
 *
 *  Created on: Jan 17, 2014
 *      Author: Justin Rajewski
 */

#ifndef FLASH_H_
#define FLASH_H_

#include "ring_buffer.h"
#include "hardware.h"

void SPI_Setup(void);

/*
 * eraseFlash()
 * This will erase the entire flash
 * memory. This must be called before writing
 * to it to ensure it is in the erased state.
 */
void eraseFlash(void);

/*
 * writeByteFlash(address, byte)
 * This writes one byte to the address
 * specified.
 */
void writeByteFlash (uint32_t, uint8_t);

/*
 * writeFlash(address, data, length)
 * This will write a block of data of size length
 * to the flash starting at the address specified.
 *
 * The starting address MUST be an even number as
 * writes are performed in pairs. If an odd number
 * is given the actual start address will be the
 * address one earlier.
 */
void writeFlash(uint32_t, uint8_t*, uint16_t);

/*
 * readFlash(data, address, length)
 * This reads the flash memory and stores the
 * values into data. Data must be an array
 * of size length or more.
 */
void readFlash(volatile uint8_t*, uint32_t, uint16_t);

#endif /* FLASH_H_ */

