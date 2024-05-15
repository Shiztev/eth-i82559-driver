/*
 * File:	eth0.c
 *
 * Author:	Stevie Alvarez
 *
 * Contributor:
 *
 * Description: Implimentation of the interface for the i82559 Intel
 *		ethernet card.
 *
 * References:	[1] Intel 8255x 10/100 Mbps Ethernet Controller Family
 *			https://www.intel.com/content/dam/doc/manual/8255x-10-100-mbps-ethernet-controller-software-dev-manual.pdf
 *		[2] Network Driver for Intel Ethernet Cards I217 and 82577LM
 *			https://osdev.org/Intel_Ethernet_i217
 */

#define	SP_KERNEL_SRC

#include "eth0.h"

#include "support.h"
#include "common.h"
#include "endian.h"
#include "x86pic.h"
#include "cio.h"
#include "lib.h"

static uint32_t csr_mmio;
static uint32_t csr_pio;
static uint32_t cbl;
// eth.src == 70:4d:7b:84:39:b1 (wireshark filter for testing)
static uint8_t src_mac_addr[] = {0x70, 0x4D, 0x7B, 0x84, 0x39, 0xB1};
// eth.dst == ff:ff:ff:ff:ff:ff (wireshark filter for testing)
// NOTE: cannot receive eth frames w/ broadcast as dest addr (unless in promiscuous mode, see seciton 6.4.3.3 [1])
static uint8_t broadcast_addr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// eth.src == 70:4d:7b:84:39:b1 or eth.dst == ff:ff:ff:ff:ff:ff (wireshark filter for testing)
static bool_t mmio_avail = true;

static void _eth_trace(char *string)
{
	__cio_printf("%s\n", string);
	__cio_printf("-------------------------------\n");
}

static void _eth_print_debug(uint32_t addr, uint32_t num_bytes)
{
	uint8_t* addr_to_print = (uint8_t*)addr;
	for (int i = 0; i < num_bytes; i++) {
		__cio_printf("%02x ", addr_to_print[i]);
		if ((i + 1) % 8 == 0) {
		    __cio_printf("  ");
		}
		if ((i + 1) % 16 == 0) {
		    __cio_printf("\n");
		}
	}
}

uint32_t __eth_read_u32(uint16_t reg)
{
	if (!mmio_avail) {
		return __inl(csr_pio + reg);
	}
	return (*(volatile uint32_t *)(csr_mmio + ((uint32_t)reg)));
}

uint16_t __eth_read_u16(uint16_t reg)
{
	if (!mmio_avail) {
		return __inw(csr_pio + reg);
	}
	return (*(volatile uint16_t *)(csr_mmio + ((uint32_t)reg)));
}

void __eth_write_u32(uint16_t reg, uint32_t value)
{
	if (!mmio_avail) {
		__outl(csr_pio + reg, value);
	} else {
		(*(volatile uint32_t *)(csr_mmio + ((uint32_t)reg))) = value;
	}
}

void __eth_write_u16(uint16_t reg, uint16_t value)
{
	if (!mmio_avail) {
		__outw(csr_pio + reg, value);
	} else {
		(*(volatile uint16_t *)(csr_mmio + ((uint32_t)reg))) = value;
	}
}

/* Handle a CI/CNA interrupt from an i82559. */
static uint16_t _eth_ci_isr()
{
	__cio_printf("ETH0 INT: got CI int\n");

	struct tcb *tcb = (struct tcb *)cbl;

	if ((tcb->header & CBL_STATUS_C_MASK) && (tcb->header & CBL_STATUS_OK_MASK)) {
		__cio_printf("ETH0 INT: tcb is C and OK!\n");
	} else {
		__cio_printf("ETH0 INT: tcb NOT C *or* NOT OK!!!!! This is very bad.\n");
	}

	// free cbl data
	// note: all transmits are one cuc transmit cmd followed by nop
	_km_slice_free((void *)(tcb->la));
	_km_slice_free((void *)tcb);
	return SCB_CI;
}

/* Handle interrupts triggered by an i82559 Ethernet Controller. */
void __eth_isr(int vector, int code)
{
	uint16_t scb_status = __eth_read_u16(SCB_STATUS);
	uint16_t scb_stat_ack = (scb_status & SCB_STATUS_PRESERVE_NON_INT_MASK);

	_eth_trace("ETH0 ISR Received");

	// Any number of the status interrupt bits can be set for a given interrupt
	// therefore, we must check and ACK each *expected* interrupt bit individually.
	
	if (scb_status & SCB_CI) {
		scb_stat_ack |= _eth_ci_isr();
	}

	if ((scb_status & (~SCB_CI)) & 0b1111111100000000) {
		// Other interrupts occured, not handled.
		__cio_printf("ETH0 INT: got non-CI int\n");
	}

	// TODO: handle other interrupts

	__cio_printf("ETH0 INT: acking only CI int (if occured)...\n");
	__eth_write_u16(SCB_STATUS, scb_stat_ack);

	// ACK interrupt through PIC
	if (vector > PIC_PRI_MAX) {
		__outb(PIC_SEC_CMD_PORT, PIC_EOI);
	}
	__outb(PIC_PRI_CMD_PORT, PIC_EOI);
}

/*
 * Format tcb buffer with provided data, with respect to size.
 * 
 * Assumes size < MAX_DATA_SIZE.
 *
 * Returns address of tcb. This could be extended to chain transmit commands
 * together, by setting the link address field of previous transmit command
 * to the address returned here.
 */
static uint32_t _eth_format_tcb(uint8_t *data, uint16_t size, uint8_t dest_addr[MAC_ADDR_LEN])
{
	struct tcb *tcb_buf;
	struct nop *nop;

	// allocate TCB
	tcb_buf = (struct tcb*)_km_slice_alloc();

	// set up TCB header
	tcb_buf->header = TCB_H_DEFAULT;
	tcb_buf->tbd_aa = TCB_NO_TBD_ADDR;
	tcb_buf->tcb_cnt_eof = ((size + ETH_HDR_SIZE) & TCB_SIZE_MASK) | TCB_EOF_FLAG;
	tcb_buf->thresh = TCB_THRESH;
	tcb_buf->tbd_num = TCB_TBD_N;

	// set up ethernet header
	__memcpy(tcb_buf->data.header.dst, dest_addr, MAC_ADDR_LEN);
	//__memcpy(tcb_buf->data.header.src, src_mac_addr, MAC_ADDR_LEN);

	// type field of eth header indicates packet length, see https://en.wikipedia.org/wiki/Ethernet_frame#Header
	// TODO: do I need to swap byte order (for wire shark)? very unclear.
	tcb_buf->data.header.eth_type = __switch_endianness(size);  

	// copy data over
	__memcpy(tcb_buf->data.buf, data, size);

	// create terminating nop
	nop = (struct nop*)_km_slice_alloc();
	nop->header = CBL_CMD_EL_BIT;
	nop->la = CBL_NO_LA;

	// terminate with a nop
	tcb_buf->la = (uint32_t)nop;

	return (uint32_t)tcb_buf;
}

/*
 * Transmit provided data of a given size.
 *
 * Will not transmit data if size >= MAX_DATA_SIZE.
 *
 * Sends the provided data in an ethernet frame, addressed from the static
 * src_mac_addr MAC address to the broadcast MAC address.
 */
void __eth_transmit(uint8_t *data, uint16_t size)
{
	_eth_trace("ETH0 TRSMT");

	if (size >= MAX_DATA_SIZE) {
		__cio_printf("ETH0 ERR: data size of %d bytes is greater than "
				"max data size of %d\n", size, MAX_DATA_SIZE);
		__cio_printf("-------------------------------\n");
		return;
	}


	// create CBL constituted by a tcb, followed by a delineating, terminating nop
	cbl = _eth_format_tcb(data, size, broadcast_addr);

	// pass the CBL to the CU, issue a start CUC (moves CU from idle to active state)
	__eth_write_u32(SCB_GENERAL_PTR, cbl);
	__eth_write_u16(SCB_CMD, (CUC_START | RUC_NOP));

	_eth_trace("ETH0: Transmitting...");
}

static bool_t _get_mmio_bar(uint32_t bar)
{
	// if bit in position 0 is set to 1, port I/O bar not mmio
	if (bar & 1) {
		return false;
	}

	csr_mmio = (bar & ETH0_MMIO_BAR_MASK);
	return true;
}

static bool_t _get_pio_bar(uint32_t bar)
{
	// if bit in position 0 is set to 1, mmio bar not port I/O
	if (!(bar & 1)) {
		return false;
	}

	csr_pio = (bar & ETH0_PIO_BAR_MASK);
	return true;
}

/* Issue a software reset to the NIC. */
static void _software_reset(void)
{
	__eth_write_u32(CSR_PORT, PORT_SOFTWARE_RESET_CMD);
	// must wait 10 microseconds after software reset (see section 6.3.3.1 [1])
	// this is probably > 10 microseconds, __delay() is not accurate
	__delay(100);
}

/*
 * Write byte to byte offset of buffer, with respect to CBL structure of
 * little-endian byte structure over 32-bit dwords.
 */
static void _write_action_cmd_byte(uint32_t byte_num, uint8_t v, uint32_t *buf)
{
	uint32_t *dest;
	uint32_t offset;
	uint32_t preserved;

	// calc bitshift over a 4 byte chunk
	offset = (byte_num % 4) * 8;
	// adjust dest addr to a rounded down offset of multiples of 4 (4 byte chunks)
	dest = (uint32_t *)((uint32_t)buf + ((byte_num / 4) * 4));
	// preserve data not within byte to write to
	preserved = (*dest & (~(0x000000FF << offset)));

	*dest = preserved | (((uint32_t)v & 0x000000FF) << offset);
}

/*
 * Sets the base of the CU or RU to 0.
 *
 * See section 6.2.1 [1].
 *
 * The cuc and ruc arguments specify commands for the CU and RU, respectively.
 * To set the base of the CU, pass CUC_LOAD_BA and RUC_NOP as the arguments.
 * To set the base of the RU, pass CUC_NOP and RUC_LOAD_BA as the arguments.
 */
static void _reset_base_addr(uint16_t cuc, uint16_t ruc) {
	__eth_write_u32(SCB_GENERAL_PTR, CLEAN_BASE_ADDR);
	__eth_write_u16(SCB_CMD, (cuc | ruc));
	do {
		__delay(ETH0_DELAY);
	} while(__eth_read_u16(SCB_CMD) != SCB_GOT_CMD);
}

/*
 * Configure the i82559.
 *
 * Sets the base address of the CU and RU, and issues a configuration action
 * command.
 *
 * The configuration action command issued is based on the specification in
 * section 6.4.2.3 [1]
 */
static void _eth_configure(void)
{
	uint32_t cbl[CBL_CONF_SIZE];
	uint32_t *config_data;
	uint32_t i = 0;
	uint16_t curr_status;

	_eth_trace("ETH0 CONFIG");

	// Set base address for CU and RU to 0
	_reset_base_addr(CUC_LOAD_BA, RUC_NOP);
	_reset_base_addr(CUC_NOP, RUC_LOAD_BA);

	// set up config action cmd header
	// status and cmd registers
	cbl[CBL_REGS] = CBL_CMD_EL_BIT | CBL_CMD_CONF;
	cbl[CBL_LA] = CBL_NO_LA;

	// offset cbl pointer to data section of config action command
	config_data = (uint32_t *)((uint32_t)cbl + (2 * sizeof(uint32_t)));

	// insert config cmd data
	_write_action_cmd_byte(i++, CAC_B00, config_data);
	_write_action_cmd_byte(i++, CAC_B01, config_data);
	_write_action_cmd_byte(i++, CAC_B02, config_data);
	_write_action_cmd_byte(i++, CAC_B03, config_data);
	_write_action_cmd_byte(i++, CAC_B04, config_data);
	_write_action_cmd_byte(i++, CAC_B05, config_data);
	_write_action_cmd_byte(i++, CAC_B06, config_data);
	_write_action_cmd_byte(i++, CAC_B07, config_data);
	_write_action_cmd_byte(i++, CAC_B08, config_data);
	_write_action_cmd_byte(i++, CAC_B09, config_data);
	_write_action_cmd_byte(i++, CAC_B10, config_data);

	// give cbl to CU, then run start CUC
	__eth_write_u32(SCB_GENERAL_PTR, (uint32_t)cbl);
	__eth_write_u16(SCB_CMD, CUC_START | RUC_NOP);

	// interrupts not set up, poll till cmd completed
	do {
		__delay(ETH0_DELAY);
		curr_status = __eth_read_u16(SCB_STATUS);
	} while(!((curr_status & SCB_CI) && (cbl[CBL_REGS] & CBL_STATUS_C_MASK)));

	// ensure command succeeded (see bottom of section 6.4.2.3 [1])
	if (!(cbl[CBL_REGS] & CBL_STATUS_OK_MASK)) {
		__panic("ETH0 CONFIGURE: cbl complete (C) bit set, but OK bit not set");
	}

	// ack interrupts
	__eth_write_u16(SCB_STATUS, curr_status);
	_eth_trace("ETH0 CONFIG Complete!");
}

/* Set the MAC address of the NIC. */
void _eth_set_addr(void)
{
	uint32_t cbl[CBL_IAS_SIZE];
	uint32_t *cmd_data;
	uint32_t i = 0;
	uint16_t curr_status;

	_eth_trace("ETH0 SET MAC");

	// set up individual address cmd header
	cbl[CBL_REGS] = CBL_CMD_EL_BIT | CBL_CMD_IAS;
	cbl[CBL_LA] = CBL_NO_LA;

	// offset cbl pointer to data section of IAS action command
	cmd_data = (uint32_t *)((uint32_t)cbl + (2 * sizeof(uint32_t)));

	// insert IAS cmd data
	_write_action_cmd_byte(i, src_mac_addr[i], cmd_data);
	i++;
	_write_action_cmd_byte(i, src_mac_addr[i], cmd_data);
	i++;
	_write_action_cmd_byte(i, src_mac_addr[i], cmd_data);
	i++;
	_write_action_cmd_byte(i, src_mac_addr[i], cmd_data);
	i++;
	_write_action_cmd_byte(i, src_mac_addr[i], cmd_data);
	i++;
	_write_action_cmd_byte(i, src_mac_addr[i], cmd_data);
	i++;

	// give cbl to CU, then run start CUC
	__eth_write_u32(SCB_GENERAL_PTR, (uint32_t)cbl);
	__eth_write_u16(SCB_CMD, CUC_START | RUC_NOP);

	do {
		__delay(ETH0_DELAY);
		curr_status = __eth_read_u16(SCB_STATUS);
	} while(!((curr_status & SCB_CI) && (cbl[CBL_REGS] & CBL_STATUS_C_MASK)));

	// ensure command succeeded (see section 6.4.2.2 [1])
	if (!(cbl[CBL_REGS] & CBL_STATUS_OK_MASK)) {
		__panic("ETH0 SET MAC: cbl complete (C) bit set, but OK bit not set");
	}

	// ack interrupts
	__eth_write_u16(SCB_STATUS, curr_status);
	_eth_trace("ETH0 SET MAC Complete!");
}

/* Initialize the NIC. Ensures that it's a i82559 series controller. */
void __eth_init(struct pci_device *pci)
{
	_eth_trace("ETH0 INIT");

	uint32_t cc_and_rid = __pci_read_u32(pci->bus, pci->slot, pci->fn, PCI_CC_AND_RID_OFFSET_U8);
	uint32_t class_code = PCI_CC_FROM_U32(cc_and_rid);
	uint8_t rev_id = PCI_RID_FROM_U32(cc_and_rid);
	uint8_t int_line;

	if (class_code != ETH0_CLASS_C) {
		__panic("ETH0 INIT: provided pci address has invalid class code for i82559");
	}

	switch (rev_id) {
		case ETH0_ASTEP:
		case ETH0_BSTEP:
		case ETH0_CSTEP:
			break;
		default:
			__panic("ETH0 INIT: expected PCI revision ID to identify "
				"device as i82559 A, B, or C step, but it was "
				"identified as a non-i82559 i8255x chip");
	}

	if (!(_get_mmio_bar(__pci_read_u32(pci->bus, pci->slot, pci->fn, CSR_PCI_MMIO_OFFSET)))) {
		mmio_avail = false;
		// TODO: could do funky pointer stuff instead
			// __eth_read/write function, change it's pointer to a port IO based function if no mmio
			// that way you don't have to do a conditional check each read/write
	}

	if (!(_get_pio_bar(__pci_read_u32(pci->bus, pci->slot, pci->fn, CSR_PCI_PIO_OFFSET)))) {
		__panic("ETH0 INIT: invalid pio address");
	}

	// reset and configure the NIC
	_software_reset();
	_eth_configure();
	_eth_set_addr();

	// install ISR
	int_line = __pci_device_get_interrupt_line(pci);
	__install_isr((int_line + 0x20), __eth_isr);

	_eth_trace("ETH0 INIT Complete!");
}
