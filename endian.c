/*
 * File:	endian.c
 *
 * Author:	Stevie Alvarez
 *
 * Contributor:
 *
 * Description:	Endian conversion utilities module implementation.
 */

#include "endian.h"

uint16_t __switch_endianness(uint16_t v) {
	return (((v << 8) & 0xFF00) | ((v >> 8) & 0x00FF));
}
