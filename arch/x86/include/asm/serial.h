/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_SERIAL_H
#define _ASM_X86_SERIAL_H

/*
 * GEN3 has high clock frequency
 */
#ifdef CONFIG_X86_INTEL_CE_GEN3
#define BASE_BAUD (14745600/16)
#else
/*
 * This assumes you have a 1.8432 MHz clock for your UART.
 *
 * It'd be nice if someone built a serial card with a 24.576 MHz
 * clock, since the 16550A is capable of handling a top speed of 1.5
 * megabits/second; but this requires a faster clock.
 */
#define BASE_BAUD (1843200/16)
#endif

/* Standard COM flags (except for COM4, because of the 8514 problem) */
#ifdef CONFIG_SERIAL_8250_DETECT_IRQ
# define STD_COMX_FLAGS	(UPF_BOOT_AUTOCONF |	UPF_SKIP_TEST	| UPF_AUTO_IRQ)
# define STD_COM4_FLAGS	(UPF_BOOT_AUTOCONF |	0		| UPF_AUTO_IRQ)
#else
# define STD_COMX_FLAGS	(UPF_BOOT_AUTOCONF |	UPF_SKIP_TEST	| 0		)
# define STD_COM4_FLAGS	(UPF_BOOT_AUTOCONF |	0		| 0		)
#endif

/*
 * The following code is for Intel Media SOC Gen3 base support.
*/
#ifdef CONFIG_X86_INTEL_CE_GEN3
/*
 * Definition of the serial port for Intel Media SOC Gen3.
*/
#ifdef CONFIG_SMP
#define SERIAL_PORT_DFNS			\
	/* UART CLK   PORT IRQ     FLAGS        */			\
	{ 0, BASE_BAUD, 0x3F8, 38, STD_COMX_FLAGS },	/* ttyS0 */	\
	{ 0, BASE_BAUD, 0x2F8, 38, STD_COMX_FLAGS },	/* ttyS1 */	\
	{ 0, BASE_BAUD, 0x3E8, 38, STD_COMX_FLAGS },	/* ttyS2 */
#else
#define SERIAL_PORT_DFNS			\
	/* UART CLK   PORT IRQ     FLAGS        */			\
	{ 0, BASE_BAUD, 0x3F8, 4, STD_COMX_FLAGS },	/* ttyS0 */	\
	{ 0, BASE_BAUD, 0x2F8, 4, STD_COMX_FLAGS },	/* ttyS1 */	\
	{ 0, BASE_BAUD, 0x3E8, 4, STD_COMX_FLAGS },	/* ttyS2 */
#endif
#else
#define SERIAL_PORT_DFNS								\
	/* UART		CLK		PORT	IRQ	FLAGS			    */	\
	{ .uart = 0,	BASE_BAUD,	0x3F8,	4,	STD_COMX_FLAGS	}, /* ttyS0 */	\
	{ .uart = 0,	BASE_BAUD,	0x2F8,	3,	STD_COMX_FLAGS	}, /* ttyS1 */	\
	{ .uart = 0,	BASE_BAUD,	0x3E8,	4,	STD_COMX_FLAGS	}, /* ttyS2 */	\
	{ .uart = 0,	BASE_BAUD,	0x2E8,	3,	STD_COM4_FLAGS	}, /* ttyS3 */
#endif

#endif /* _ASM_X86_SERIAL_H */
