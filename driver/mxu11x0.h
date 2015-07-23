/*
 *
 * MOXA UPort 11x0 USB to Serial Hub Driver Header
 *
 * Copyright (C) 2007 MOXA Technology Co. Ltd.
 *
 * This driver is based on the Linux io_ti driver, which is
 *   Copyright (C) 2000-2002 Inside Out Networks
 *   Copyright (C) 2001-2002 Greg Kroah-Hartman
 *   Copyright (C) 2004-2006 Al Borchers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _MXU11X0_H_
#define _MXU11X0_H_

#undef dbg
#define dbg(format, arg...) while (0)
//#define dbg(format, arg...) do { if (debug) printk(KERN_DEBUG "%s: " format "\n" , __FILE__ , ## arg); } while (0)
//#define dbg(format, args...) printk(KERN_DEBUG "%s: " format "\n", __FILE__, args)
//#define dbg(format, args...) printk(KERN_DEBUG "(%04d-%s)" format "\n", __LINE__, __FUNCTION__, args)

/* Configuration ids */
#define MXU1_BOOT_CONFIG			1
#define MXU1_ACTIVE_CONFIG			2

/* Vendor and product ids */
#define MXU1_VENDOR_ID				0x110a	
#define MXU1_1110_PRODUCT_ID			0x1110
#define MXU1_1130_PRODUCT_ID			0x1130
#define MXU1_1150_PRODUCT_ID			0x1150
#define MXU1_1151_PRODUCT_ID			0x1151
#define MXU1_1131_PRODUCT_ID			0x1131

/* Model Name */
#define MXU1_MODEL_1110				0x01
#define MXU1_MODEL_1130				0x02
#define MXU1_MODEL_1150				0x03
#define MXU1_MODEL_1151				0x04
#define MXU1_MODEL_1131				0x05
/* Commands */
#define MXU1_GET_VERSION			0x01
#define MXU1_GET_PORT_STATUS			0x02
#define MXU1_GET_PORT_DEV_INFO			0x03
#define MXU1_GET_CONFIG				0x04
#define MXU1_SET_CONFIG				0x05
#define MXU1_OPEN_PORT				0x06
#define MXU1_CLOSE_PORT				0x07
#define MXU1_START_PORT				0x08
#define MXU1_STOP_PORT				0x09
#define MXU1_TEST_PORT				0x0A
#define MXU1_PURGE_PORT				0x0B
#define MXU1_RESET_EXT_DEVICE			0x0C
#define MXU1_GET_OUTQUEUE			0x0D
#define MXU1_WRITE_DATA				0x80
#define MXU1_READ_DATA				0x81
#define MXU1_REQ_TYPE_CLASS			0x82

/* Module identifiers */
#define MXU1_I2C_PORT				0x01
#define MXU1_IEEE1284_PORT			0x02
#define MXU1_UART1_PORT				0x03
#define MXU1_UART2_PORT				0x04
#define MXU1_RAM_PORT				0x05

/* Modem status */
#define MXU1_MSR_DELTA_CTS			0x01
#define MXU1_MSR_DELTA_DSR			0x02
#define MXU1_MSR_DELTA_RI			0x04
#define MXU1_MSR_DELTA_CD			0x08
#define MXU1_MSR_CTS				0x10
#define MXU1_MSR_DSR				0x20
#define MXU1_MSR_RI				0x40
#define MXU1_MSR_CD				0x80
#define MXU1_MSR_DELTA_MASK			0x0F
#define MXU1_MSR_MASK				0xF0

/* Line status */
#define MXU1_LSR_OVERRUN_ERROR			0x01
#define MXU1_LSR_PARITY_ERROR			0x02
#define MXU1_LSR_FRAMING_ERROR			0x04
#define MXU1_LSR_BREAK				0x08
#define MXU1_LSR_ERROR				0x0F
#define MXU1_LSR_RX_FULL			0x10
#define MXU1_LSR_TX_EMPTY			0x20

/* Line control */
#define MXU1_LCR_BREAK				0x40

/* Modem control */
#define MXU1_MCR_LOOP				0x04
#define MXU1_MCR_DTR				0x10
#define MXU1_MCR_RTS				0x20

/* Mask settings */
#define MXU1_UART_ENABLE_RTS_IN			0x0001
#define MXU1_UART_DISABLE_RTS			0x0002
#define MXU1_UART_ENABLE_PARITY_CHECKING	0x0008
#define MXU1_UART_ENABLE_DSR_OUT		0x0010
#define MXU1_UART_ENABLE_CTS_OUT		0x0020
#define MXU1_UART_ENABLE_X_OUT			0x0040
#define MXU1_UART_ENABLE_XA_OUT			0x0080
#define MXU1_UART_ENABLE_X_IN			0x0100
#define MXU1_UART_ENABLE_DTR_IN			0x0800
#define MXU1_UART_DISABLE_DTR			0x1000
#define MXU1_UART_ENABLE_MS_INTS		0x2000
#define MXU1_UART_ENABLE_AUTO_START_DMA		0x4000
#define MXU1_UART_SEND_BREAK_SIGNAL		0x8000

/* Parity */
#define MXU1_UART_NO_PARITY			0x00
#define MXU1_UART_ODD_PARITY			0x01
#define MXU1_UART_EVEN_PARITY			0x02
#define MXU1_UART_MARK_PARITY			0x03
#define MXU1_UART_SPACE_PARITY			0x04

/* Stop bits */
#define MXU1_UART_1_STOP_BITS			0x00
#define MXU1_UART_1_5_STOP_BITS			0x01
#define MXU1_UART_2_STOP_BITS			0x02

/* Bits per character */
#define MXU1_UART_5_DATA_BITS			0x00
#define MXU1_UART_6_DATA_BITS			0x01
#define MXU1_UART_7_DATA_BITS			0x02
#define MXU1_UART_8_DATA_BITS			0x03

/* Operation modes */
#define MXU1_UART_232				0x00
#define MXU1_UART_485_RECEIVER_DISABLED		0x01
#define MXU1_UART_485_RECEIVER_ENABLED		0x02
#define MXU1_RS232				0
#define MXU1_RS4852W				1
#define MXU1_RS422				2
#define MXU1_RS4854W				3

/* Pipe transfer mode and timeout */
#define MXU1_PIPE_MODE_CONTINOUS		0x01
#define MXU1_PIPE_MODE_MASK			0x03
#define MXU1_PIPE_TIMEOUT_MASK			0x7C
#define MXU1_PIPE_TIMEOUT_ENABLE		0x80

/* User define ioctl */
#define MOXA					404
#define MOXA_SET_INTERFACE			(MOXA + 1)


/* Config struct */
struct mxu1_uart_config {
	__u16	wBaudRate;
	__u16	wFlags;
	__u8	bDataBits;
	__u8	bParity;
	__u8	bStopBits;
	char	cXon;
	char	cXoff;
	__u8	bUartMode;
} __attribute__((packed));

/* Get port status */
struct mxu1_port_status {
	__u8	bCmdCode;
	__u8	bModuleId;
	__u8	bErrorCode;
	__u8	bMSR;
	__u8	bLSR;
} __attribute__((packed));

/* Get outqueue length */
struct mxu1_port_outqueue {
	__u8	bCmdCode;
	__u8	bModuleId;
	__u8	bErrorCode;
	__u8	bLength;
} __attribute__((packed));

/* Purge modes */
#define MXU1_PURGE_OUTPUT			0x00
#define MXU1_PURGE_INPUT			0x80

/* Read/Write data */
#define MXU1_RW_DATA_ADDR_SFR			0x10
#define MXU1_RW_DATA_ADDR_IDATA			0x20
#define MXU1_RW_DATA_ADDR_XDATA			0x30
#define MXU1_RW_DATA_ADDR_CODE			0x40
#define MXU1_RW_DATA_ADDR_GPIO			0x50
#define MXU1_RW_DATA_ADDR_I2C			0x60
#define MXU1_RW_DATA_ADDR_FLASH			0x70
#define MXU1_RW_DATA_ADDR_DSP			0x80

#define MXU1_RW_DATA_UNSPECIFIED		0x00
#define MXU1_RW_DATA_BYTE			0x01
#define MXU1_RW_DATA_WORD			0x02
#define MXU1_RW_DATA_DOUBLE_WORD		0x04

struct mxu1_write_data_bytes {
	__u8	bAddrType;
	__u8	bDataType;
	__u8	bDataCounter;
#ifdef __CHECK_ENDIAN__
	__be16	wBaseAddrHi;
	__be16	wBaseAddrLo;
#else
	__u16	wBaseAddrHi;
	__u16	wBaseAddrLo;
#endif
	__u8	bData[0];
} __attribute__((packed));

struct mxu1_read_data_bytes {
	__u8	bCmdCode;
	__u8	bModuleId;
	__u8	bErrorCode;
	__u8	bData[0];
} __attribute__((packed));

/* Interrupt struct */
struct mxu1_interrupt {
	__u8	bICode;
	__u8	bIInfo;
} __attribute__((packed));

/* Interrupt codes */
#define MXU1_GET_PORT_FROM_CODE(c)		(((c) >> 4) - 3)
#define MXU1_GET_FUNC_FROM_CODE(c)		((c) & 0x0f)
#define MXU1_CODE_HARDWARE_ERROR		0xFF
#define MXU1_CODE_DATA_ERROR			0x03
#define MXU1_CODE_MODEM_STATUS			0x04

/* Download firmware max packet size */
#define MXU1_DOWNLOAD_MAX_PACKET_SIZE		64

/* Firmware image header */
struct mxu1_firmware_header {
#ifdef __CHECK_ENDIAN__
	__le16	wLength;
#else
	__u16 	wLength;
#endif
	__u8	bCheckSum;
} __attribute__((packed));

/* UART addresses */
#define MXU1_UART1_BASE_ADDR			0xFFA0	/*UART 1 base address*/
#define MXU1_UART2_BASE_ADDR			0xFFB0	/*UART 2 base address*/
#define MXU1_UART_OFFSET_LCR			0x0002	/*UART MCR register offset */
#define MXU1_UART_OFFSET_MCR			0x0004	/*UART MCR register offset */

#endif /* _MXU11X0_H_ */
