/*
 * File:	pci.h
 *
 * Author:	Stevie Alvarez
 *
 * Contributor:
 *
 * Description:	PCI initialization module declarations.
 *
 * References:	https://wiki.osdev.org/PCI
 */

#ifndef _PCI_H_
#define _PCI_H_

#include "common.h"

#define PCI_OUTP	0xCF8
#define PCI_INP		0xCFC
#define PCI_CONF_ENABLE 0x80000000

#define PCI_MAX_N_BUS	256
#define PCI_MAX_N_DEV	32

#define PCI_VENID_OFFSET	0x0
#define PCI_DEVID_OFFSET	0x2
#define PCI_INT_LINE_OFFSET	0x3C

#define PCI_CC_AND_RID_OFFSET_U8	0x8

#define PCI_RID_MASK		0b0000000011111111
#define PCI_CC_MASK		0b00000000111111111111111111111111
#define PCI_RID_FROM_U32(v_)	((v_) & PCI_RID_MASK)
#define PCI_CC_FROM_U32(v_)	(((v_) >> 8) & PCI_CC_MASK)

#define PCI_U32_MASK	0xFFFFFFFF
#define PCI_U16_MASK	0xFFFF
#define PCI_U8_MASK	0xFF

/*
 * Types
 */

struct pci_device {
	uint8_t		bus;		// pci bus associated with this device
	uint8_t		slot;		// bus slot associated with this device
	uint8_t		fn;		// funciton number associated with this device
};

/*
 * Templates
 */

/*
 * Performs PCI enumeration to initialize devices.
 *
 * Iterates over all PCI buses and devices, and upon a vendor ID and device ID
 * match, calls the initialization function of each respective device that's found.

 */
void __pci_init(void);

/*
 * Read 16 bits from the provided pci address.
 *
 * @param bus		Bus number
 * @param slot		Slot/device number
 * @param func		Function number, from 0 to 8 (inclusive)
 * @param offset	Offset into configuration space
 *
 * @return uint16_t value read from the PCI bus at the indicated location.
 */
uint16_t __pci_read_u16(uint8_t bus, uint8_t slot, uint8_t func, uint8_t offset);

/*
 * Read 32 bits from the provided pci address.
 *
 * @param bus		Bus number
 * @param slot		Slot/device number
 * @param func		Function number, from 0 to 8 (inclusive)
 * @param offset	Offset into configuration space
 *
 * @return uint32_t value read from the PCI bus at the indicated location.
 */
uint32_t __pci_read_u32(uint8_t bus, uint8_t slot, uint8_t func, uint8_t offset);

/*
 * Fetch device's associated interrupt line.
 *
 * @param dev	information of the pci device in question
 *
 * @return uint8_t value specifying the interrupt line
 */
uint8_t __pci_device_get_interrupt_line(struct pci_device *dev);

#endif
