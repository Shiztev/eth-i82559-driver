/*
 * File:	pci.c
 *
 * Author:	Stevie Alvarez
 *
 * Contributor:
 *
 * Description:	PCI initialization module implementation.
 *
 * References:	https://wiki.osdev.org/PCI
 */

#include "pci.h"

#include "common.h"
#include "lib.h"
#include "cio.h"
#include "eth0.h"

/*
 * Parses out one byte from the given PCI address.
 *
 * Rounds the offset field down to the nearest multiple of 2. That is, it's as if
 * the two least significant bits are masked off (0b11111110).
 * This ensures that u8 accesses are properly aligned within the PCI
 * configuration space.
 *
 * NOTE: Modified version from https://wiki.osdev.org/PCI
 */
uint8_t __pci_read_u8(uint8_t bus, uint8_t slot, uint8_t func, uint8_t offset)
{
	uint32_t address;
	uint32_t lbus  = (uint32_t)bus;
	uint32_t lslot = (uint32_t)slot;
	uint32_t lfunc = (uint32_t)func;
	uint16_t tmp   = 0;

	// Create configuration address as per Figure 1
	// NOTE: ensures offset is a multiple of 4
	address = (uint32_t)((lbus << 16) | (lslot << 11) |
		  (lfunc << 8) | (offset & 0xFC) | ((uint32_t)PCI_CONF_ENABLE));

	// Write out the address
	__outl(PCI_OUTP, address);
	// Read in the data
	// offset % 2 = 0 -> (offset & 2) * 8) = 0 will choose the first word of the 32-bit register
	// offset % 2 = 1 -> (offset & 2) * 8) = 16 will choose the second word of the 32-bit register
	tmp = (uint16_t)((__inl(PCI_INP) >> ((offset & 2) * 8)) & PCI_U8_MASK);
	return tmp;
}

/*
 * Parses out two bytes from the given PCI address.
 *
 * Rounds the offset field down to the nearest multiple of 2. That is, it's as if
 * the two least significant bits are masked off (0b11111110).
 * This ensures that u16 accesses are properly aligned within the PCI
 * configuration space.
 *
 * NOTE: Modified version from https://wiki.osdev.org/PCI
 */
uint16_t __pci_read_u16(uint8_t bus, uint8_t slot, uint8_t func, uint8_t offset)
{
	uint32_t address;
	uint32_t lbus  = (uint32_t)bus;
	uint32_t lslot = (uint32_t)slot;
	uint32_t lfunc = (uint32_t)func;
	uint16_t tmp   = 0;

	// Create configuration address as per Figure 1
	// NOTE: ensures offset is a multiple of 4
	address = (uint32_t)((lbus << 16) | (lslot << 11) |
		  (lfunc << 8) | (offset & 0xFC) | ((uint32_t)PCI_CONF_ENABLE));

	// Write out the address
	__outl(PCI_OUTP, address);
	// Read in the data
	// offset % 2 = 0 -> (offset & 2) * 8) = 0 will choose the first word of the 32-bit register
	// offset % 2 = 1 -> (offset & 2) * 8) = 16 will choose the second word of the 32-bit register
	tmp = (uint16_t)((__inl(PCI_INP) >> ((offset & 2) * 8)) & PCI_U16_MASK);
	return tmp;
}

/*
 * Parses out four bytes from the given PCI address.
 *
 * Rounds the offset field down to the nearest multiple of four. That is, it's as
 * if the two least significant bits are masked off (0b11111100).
 * This ensures that u32 accesses are properly aligned within the PCI
 * configuration space.
 *
 * NOTE: Modified version from https://wiki.osdev.org/PCI
 */
uint32_t __pci_read_u32(uint8_t bus, uint8_t slot, uint8_t func, uint8_t offset)
{
	uint32_t address;
	uint32_t lbus  = (uint32_t)bus;
	uint32_t lslot = (uint32_t)slot;
	uint32_t lfunc = (uint32_t)func;
	uint32_t tmp   = 0;

	// Create configuration address as per Figure 1
	address = (uint32_t)((lbus << 16) | (lslot << 11) |
		  (lfunc << 8) | (offset & 0xFC) | ((uint32_t)PCI_CONF_ENABLE));

	// Write out the address
	__outl(PCI_OUTP, address);
	// Read in the data
	// Choose all 32 bits of the 32-bit register
	tmp = (uint32_t)(__inl(PCI_INP) & PCI_U32_MASK);
	return tmp;
}

/*
 * Determines which device corresponds to the provided (bus, slot) location.
 * Calls the respective initialization function for the said device.
 *
 * Expects bus and device to be valid (not 0xFFFF).
 */
void _check_device(uint8_t bus, uint8_t device, uint8_t fn)
{
	struct pci_device pci = {bus, device, fn};
	uint16_t vendor_id = __pci_read_u16(bus, device, fn, PCI_VENID_OFFSET);
	uint16_t device_id = __pci_read_u16(bus, device, fn, PCI_DEVID_OFFSET);
	//uint32_t mm_bar;
	//uint32_t im_bar;

	if (vendor_id == 0xFFFF)
		return;

	switch (vendor_id) {
		case ETH0_VID:
			if (device_id == ETH0_DID)	// i8255x Controller found!
				__eth_init(&pci);
			break;
		default:
			break;
	}
	//mm_bar = _pci_read_u32(bus, device, 0, 10);
	//im_bar = _pci_read_u32(bus, device, 0, 14);
}

void __pci_init(void)
{
	uint16_t bus;
	uint8_t device;

	__cio_printf("---PCI Init-------------------------------\n");

	for (bus = 0; bus < PCI_MAX_N_BUS; bus++) {
		for (device = 0; device < PCI_MAX_N_DEV; device++) {
			_check_device(bus, device, 0);
		}
	}
}

uint8_t __pci_device_get_interrupt_line(struct pci_device *dev) {
	return __pci_read_u8(dev->bus, dev->slot, dev->fn, PCI_INT_LINE_OFFSET);
}
