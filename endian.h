/*
 * File:	endian.h
 *
 * Author:	Stevie Alvarez
 *
 * Contributor:
 *
 * Description:	Endian conversion utilities module declarations.
 */

#ifndef _ENDIAN_H_
#define _ENDIAN_H_

#include "common.h"

/*
 * PROTOTYPES
 */

/*
 * Switch endianness (byte order) of value.
 *
 * @param v	value to switch endianness of
 *
 * @return endian-swapped version of v
 */
uint16_t __switch_endianness(uint16_t v);

#endif
