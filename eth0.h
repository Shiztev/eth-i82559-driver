/*
 * File:	eth0.h
 *
 * Author:	Stevie Alvarez
 *
 * Contributor:
 *
 * Description: Interface for the i82559 C-Step Intel ethernet card. While it
 *		ensures that the chip is in the i82559 series (see __eth_init()),
 *		this driver treats the chip like an i82557, excluding the
 *		configuration action command.
 *
 * References:	[1] Intel 8255x 10/100 Mbps Ethernet Controller Family
 *			https://www.intel.com/content/dam/doc/manual/8255x-10-100-mbps-ethernet-controller-software-dev-manual.pdf
 *		[2] Network Driver for Intel Ethernet Cards I217 and 82577LM
 *			https://osdev.org/Intel_Ethernet_i217
 */

#ifndef _ETH0_H_
#define _ETH0_H_

#include "common.h"
#include "kmem.h"
#include "pci.h"

/*
 * Intel 82557 Ethernet Controller identification for PCI Enumeration (section 4 [1])
 */

#define ETH0_CLASS_C	0x020000
#define ETH0_VID	0x8086
#define ETH0_DID	0x1229

#define CSR_PCI_MMIO_OFFSET	0x10
#define CSR_PCI_PIO_OFFSET	0x14

// PCI Revision register values for the i82559 A-step, B-step, and C-step (section 4.1.5 [1])
// Note: Most lab machines have an i82559 C-step, but differences between X-step's is negligible
#define ETH0_ASTEP	0x06
#define ETH0_BSTEP	0x07
#define ETH0_CSTEP	0x08

// Determine if memory mapped or port I/O supported (section 4.1.3 [1])
#define ETH0_MMIO_SUPPORTED(cmd_reg_)	(((cmd_reg_) & 0b010))
#define ETH0_PORTIO_SUPPORTED(cmd_reg_)	((cmd_reg_) & 0b01)

// Determine BAR for MMIO
#define ETH0_MMIO_BAR_MASK	0b11111111111111111111111111110000
#define ETH0_PIO_BAR_MASK	0b11111111111111111111111111111100

/*
 * Configuration values
 */

#define CLEAN_BASE_ADDR	0

#define PORT_SOFTWARE_RESET_CMD	0x00000000

// Configuration Action Command byte values (see seciton 6.4.2.3 [1])

#define CAC_SIZE	11
#define CAC_B00		CAC_SIZE

#define TRSMT_FIFO_LIM	0
#define RCV_FIFO_LIM	0b1000
#define CAC_B01		TRSMT_FIFO_LIM | RCV_FIFO_LIM

#define MIN_PCI_TIC	0
#define CAC_B02		MIN_PCI_TIC

#define MWI_ENABLE	0
#define CAC_B03		MWI_ENABLE

#define RCV_DMA_MIN	0
#define CAC_B04		RCV_DMA_MIN

#define TRSMT_DMA_MAX		0
#define DMA_MAX_BYTE_ENABLE	0
#define CAC_B05			TRSMT_DMA_MAX | DMA_MAX_BYTE_ENABLE

#define EXTENDED_STAT_COUNTERS_DISABLE	0b00100000
#define EXTENDED_TCB_DISABLE		0b00010000
#define CAC_BYTE_6_CONST		0b00000010
#define CAC_B06				EXTENDED_STAT_COUNTERS_DISABLE \
					| EXTENDED_TCB_DISABLE \
					| CAC_BYTE_6_CONST

#define UNDERRUN_RETRY			0b010
#define	DISCARD_SHORT_FRAMES_ENABLE	0b1
#define CAC_B07		UNDERRUN_RETRY | DISCARD_SHORT_FRAMES_ENABLE

#define ENABLE_MII	0b1
#define CAC_B08		ENABLE_MII

#define CAC_B09		0  // don't need any of the conf settings in byte 9

#define NO_LOOPBACK		0
#define PREAMBLE_7_BYTES	0b100000
#define NSAI			0b0000		// bit 3 = 1 to turn off
#define CAC_BYTE_10_CONST	0b110
#define CAC_B10			NO_LOOPBACK | PREAMBLE_7_BYTES	| NSAI \
				| CAC_BYTE_10_CONST

/*
 * REGISTERS
 */

// SCB Registers (offset from csr, see section 6.3.1 [1])
#define SCB_STATUS		0x0000  // csr + 0
#define SCB_CMD			0x0002  // csr + 16 (2 byte offset)
#define SCB_GENERAL_PTR		0x0004  // csr + 32 (4 byte offset)
#define CSR_PORT		0x0008  // csr + 64 (8 byte offset)

#define EEPROM_CTRL		0x000E  // csr + 112 (14 byte offset)

// Mask to preserve bottom byte of SCB Status register (see section 6.3.1 [1])
#define SCB_STATUS_PRESERVE_NON_INT_MASK	0b0000000011111111

/* SCB Status Register */

// Possible interrupts (check second byte of SCB status word, see section 6.3.2.1 [1])
# define SCB_CX		0b1000000000000000
# define SCB_FR		0b0100000000000000
# define SCB_CI		0b0010000000000000  // aka CNA
# define SCB_RNR	0b0001000000000000
# define SCB_MDI	0b0000100000000000
# define SCB_SWI	0b0000010000000000
// Reserved		0b0000001000000000
# define SCB_FCP	0b0000000100000000

// Possible RUS values
#define RUS_MASK	0b0000000000111100
#define RUS_IDLE	0b0000000000000000  // RU in idle state
#define RUS_SUSP	0b0000000000000100  // RU in suspend state
#define RUS_NO_RSRCS	0b0000000000001000  // RU in No Resources state
#define RUS_READY	0b0000000000010000  // RU in Ready state

// Possible CUS values
#define CUS_MASK	0b0000000011000000
#define CUS_IDLE	0b0000000000000000  // CU in idle state
#define CUS_SUSP	0b0000000001000000  // CU in suspend state
#define CUS_LPQA	0b0000000010000000
#define CUS_HQPA	0b0000000011000000

// SCB Status Register Masks
#define SCB_STAT_MASK	0b1111111100000000
#define SCB_RUS		0b0000000000111100
#define SCB_CUS		0b0000000011000000

// Fetch SCB Status Register values
#define RUS(v_)		((v_) & SCB_RUS)
#define CUS(v_)		((v_) & SCB_CUS)

/* SCB Command Register */

// SCB cmd value when an issued cmd is accepted (see second paragraph of section 6.3.2.2[1])
#define SCB_GOT_CMD	0

// CUC and RUC Values (see section 6.3.2.2 [1])
#define CUC_NOP		0
#define CUC_START	0b0000000000010000
#define CUC_RESUME	0b0000000000100000
#define CUC_LOAD_BA	0b0000000001100000

#define RUC_NOP		0
#define RUC_START	0b0000000000000001
#define RUC_RESUME	0b0000000000000010
#define RUC_LOAD_BA	0b0000000000000110

// Interrupt masks for SCB commands (CUC and RUC)
#define SCB_CMD_NO_INT	0b0000001000000000

/* Command Block List entries (see section 6.4 [1]) */

// CBL Command and Status indicies
#define CBL_REGS	0  // index of cmd and status registers in CBL
#define CBL_LA		1  // index of link address in CBL

// CBL Header options
#define CBL_CMD_EL_BIT	0b10000000000000000000000000000000
#define CBL_CMD_S_BIT	0b01000000000000000000000000000000
#define CBL_CMD_I_BIT	0b00100000000000000000000000000000
#define CBL_STATUS_INIT 0
#define CBL_NO_LA	0

// Types of action commands
#define CBL_CMD_NOP	0
#define CBL_CMD_IAS	0b00000000000000010000000000000000
#define CBL_CMD_CONF	0b00000000000000100000000000000000
#define CBL_CMD_MAS	0b00000000000000110000000000000000
#define CBL_CMD_TCB	0b00000000000001000000000000000000

// Mask over a CBL entries status field for the C and OK bits (see section 6.4.2 [1])
#define CBL_STATUS_C_MASK	0b00000000000000001000000000000000
#define CBL_STATUS_OK_MASK	0b00000000000000000010000000000000

// Mask over a TCB entries status field for the U bit (see section 6.4.2.5 [1])
#define TCB_STATUS_U_MASK	0b00000000000000000001000000000000

/* Bounding Constants */

#define BYTE_MASK		0b11111111

// CBL command sizes
#define CBL_CONF_SIZE		13
#define CBL_IAS_SIZE		8

// Size of data to transmit must be below the size of a slice of memory 
#define U8_PER_U32		4			// bytes per 32-bits
#define TCB_H_SIZE		16			// TCB header size (bytes)
#define MTU			(SZ_SLICE - TCB_H_SIZE)	// MTU is size of a slice (not 1500 Ethernet MTU)
#define TCB_SIZE		SZ_SLICE		// net size of TCB (bytes) (NOTE: TCB_SIZE =  MTU + TCB_H_SIZE)
#define ETH_HDR_SIZE		((sizeof(struct eth_hdr)) + MAC_ADDR_LEN)
#define MAX_DATA_SIZE		((MTU) - (ETH_HDR_SIZE))	// maximum size of a TCB
#define TCB_U32_SIZE		((TCB_SIZE) / U8_PER_U32)		// size of TCB (32-bit words)
#define IP_MAX_DATA_SIZE	(MAX_DATA_SIZE - (sizeof(struct ip_hdr)))



// TCB utilities (see section 6.4.2.5 [1])
#define INVALID_ADDR		0xFFFFFFFF
#define TCB_SIZE_MASK		0b0011111111111111			// mask down to TCB byte count field
// NOTE: if NC bit is set, you are responsible for calculating CRC. I've observed that it inserts src address after the dest addr, and shifts all data following backwards. Intel makes scuffed NICs.
#define TCB_NC_BIT		0b00000000000100000000000000000000	// Determines where source address comes from
#define TCB_H_DEFAULT		CBL_CMD_TCB  // | TCB_NC_BIT // | CBL_CMD_EL_BIT
#define TCB_TBD_N		0
#define TCB_THRESH		0
#define TCB_NO_TBD_ADDR		INVALID_ADDR		// no tbd -> tbd addr = 32 bits set to 1
#define TCB_EOF_FLAG		0b1000000000000000
#define TCB_SIZE_UPPER_BYTE(s_) \
	(((s_) >> 8) & 0b00000111)  // want the 3 least significant bits of the upper byte of size
				    // to guarantee the net size (see eth0.c __eth_transmit) is
				    // 11 bits since MTU (1500 bytes) is between 2^10 and 2^11
				    // and because we're right shifting (I don't trust gcc to
				    // right logical shift instead of right arithmatic shift)

// Miscilanous Constants
#define	MAC_ADDR_LEN	6	// length of MAC addresses, in bytes
#define ETH0_DELAY	20	// delay value

/*
 * TYPES
 */

struct eth_hdr {
	uint8_t		dst[MAC_ADDR_LEN];
	//uint8_t		src[MAC_ADDR_LEN];  NSAI is off -> NIC jankily inserts SA
	uint16_t	eth_type;
} __attribute__((__packed__));

struct eth_frame {
	struct eth_hdr	header;
	uint8_t		buf[MAX_DATA_SIZE];
} __attribute__((__packed__));

struct tcb {
	uint32_t		header;		// status and cmd registers
	uint32_t		la;		// link address of next TCB
	uint32_t		tbd_aa;		// TBD array address
	uint16_t		tcb_cnt_eof;	// TCB byte count and EOF fields
	uint8_t			thresh;		// transmit threshold field
	uint8_t			tbd_num;	// TBD number field
	struct eth_frame	data;		// data byte buffer
} __attribute__((__packed__));

struct nop {
	uint32_t	header;
	uint32_t	la;
} __attribute__((__packed__));

/*
 * GLOBALS
 */

/*
 * PROTOTYPES
 */

/* Read values from the NIC. Adopted from [2]. */

/*
 * Read a uint32_t from the NIC.
 *
 * @param reg	the register to read from; the offset into the csr
 *
 * @return uint32_t value stored in the designated register.
 */
uint32_t __eth_read_u32(uint16_t reg);

/*
 * Read a uint16_t from the NIC.
 *
 * @param reg	the register to read from; the offset into the csr
 *
 * @return uint16_t value stored in the designated register.
 */
uint16_t __eth_read_u16(uint16_t reg);

/* Write values from the NIC. Adopted from [2]. */

/*
 * Write a uint32_t to the NIC.
 *
 * @param reg		the register to read from; the offset into the csr
 * @param value		the value to write to the register
 */
void __eth_write_u32(uint16_t reg, uint32_t value);

/*
 * Write a uint16_t to the NIC.
 *
 * @param reg		the register to read from; the offset into the csr
 * @param value		the value to write to the register
 */
void __eth_write_u16(uint16_t reg, uint16_t value);

/*
 * Initialize the driver for the i82559 controller.
 *
 * Configures and sets up the NIC. This function must be called to set up this
 * module, and no other function defined for the NIC will work before this
 * function completes successfully.
 *
 * @param pci_info	information pertaining to the pci address of the device
 */
void __eth_init(struct pci_device *pci_info);

/*
 * ISR for interrupts originating from the i82557 controller.
 */
void __eth_isr(int vector, int vode);

/*
 * Transmit provided data to broadcast address.
 *
 * Treats data as an array of specified size (measured in bytes).
 * The size is expected to be less than MAX_DATA_SIZE.
 *
 * @param data	array of bytes to transmit
 * @param size	size of data in bytes (< MAX_DATA_SIZE)
 */
void __eth_transmit(uint8_t *data, uint16_t size);

#endif
