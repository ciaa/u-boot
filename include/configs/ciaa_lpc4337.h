/*
 * (C) Copyright 2014 - VanguardiaSur - www.vanguardiasur.com.ar
 *
 * Derived from Hitex LPC4350 config file by
 *   Alexander Potashev, Emcraft Systems, aspotashev@emcraft.com
 *   Pavel Boldin, Emcraft Systems, paboldin@emcraft.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/* Define if you wanna see debug stuff */
#undef DEBUG

/* Cortex-M4 CPU core, but also uses the common Cortex-M3 code */
#define CONFIG_SYS_ARMCORTEXM3
#define CONFIG_SYS_ARMCORTEXM4

/* Enable RAMCODE to be loaded into separated section */
#define CONFIG_ARMCORTEXM3_RAMCODE
#define CONFIG_MEM_RAMCODE_BASE		0x10000000
#define CONFIG_MEM_RAMCODE_LEN		(32 * 1024)

#define CONFIG_SYS_LPC18XX

/*
 * Except it have eNVM
 */
#define CONFIG_LPC43XX_ENVM

/*
 * Add header to the U-Boot image to pass necessary information
 * to the Boot ROM bootloader.
 */
#undef CONFIG_LPC18XX_BOOTHEADER

/* Compute checksum */
#define CONFIG_LPC178X_FCG

/*
 * Enable GPIO driver. This is used for pinmux, so it's rather
 * needed.
 */
#define CONFIG_LPC18XX_GPIO

/* Display CPU and Board information */
#define CONFIG_DISPLAY_CPUINFO		1
#define CONFIG_DISPLAY_BOARDINFO	1
#define CONFIG_SYS_BOARD_REV_STR	"1"

/* Monitor prompt */
#define CONFIG_SYS_PROMPT		"ciaa > "

/* We want to call the CPU specific initialization */
#define CONFIG_ARCH_CPU_INIT

/* We do not use cortex_m3_soc_init() yet */
#undef CONFIG_ARMCORTEXM3_SOC_INIT

/*
 * Clock configuration (see cpu/arm_cortexm3/lpc18xx/clock.c for details)
 */
/*
 * This should be setup to the board specific rate for the external oscillator
 */
#define CONFIG_LPC18XX_EXTOSC_RATE	12000000

/*
 * PLL1 multiplier value (1..256)
 */
#define CONFIG_LPC18XX_PLL1_M		12	/* 12 MHz * 12 = 144 MHz */

/* Number of clock ticks in 1 sec */
#define CONFIG_SYS_HZ			1000

/* Use internal clock (CPU clock) for the Cortex-M4 SysTick timer */
#define CONFIG_ARMCORTEXM3_SYSTICK_CPU

/* No watchdog for now */
#undef CONFIG_HW_WATCHDOG

/* No interrupts */
#undef CONFIG_USE_IRQ

/*
 * Memory layout configuration
 */
/*
 * Internal flash on the NXP LPC4337 MCU. There are two banks.
 */
#define CONFIG_MEM_NVM_BASE		0x1a000000
#define CONFIG_MEM_NVM_LEN		(512 * 1024)
#define CONFIG_MEM_NVM_SECTORS		15

#define CONFIG_MEM_NVM2_BASE		0x1b000000
#define CONFIG_MEM_NVM2_LEN		(512 * 1024)
#define CONFIG_MEM_NVM2_SECTORS		15

/* TODO: Add some comments with the SRAM layout */
#define CONFIG_MEM_RAM_BASE		0x10080000
#define CONFIG_MEM_RAM_LEN		(24 * 1024)

#define CONFIG_MEM_RAM_BUF_BASE		(CONFIG_MEM_RAM_BASE + CONFIG_MEM_RAM_LEN)
#define CONFIG_MEM_RAM_BUF_LEN		(1 * 1024)

#define CONFIG_MEM_MALLOC_BASE		(CONFIG_MEM_RAM_BUF_BASE + CONFIG_MEM_RAM_BUF_LEN)
#define CONFIG_MEM_MALLOC_LEN		(11 * 1024)

#define CONFIG_MEM_STACK_BASE		(CONFIG_MEM_MALLOC_BASE + CONFIG_MEM_MALLOC_LEN)
#define CONFIG_MEM_STACK_LEN		(4 * 1024)

#define CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_SIZE			(4 * 1024)
#define CONFIG_ENV_ADDR			0

/* malloc() pool size */
#define CONFIG_SYS_MALLOC_LEN		CONFIG_MEM_MALLOC_LEN

/* Configuration of the external DRAM memory */
#define CONFIG_NR_DRAM_BANKS		1
#define CONFIG_SYS_RAM_CS		0	/* 0 .. 3 */
#define CONFIG_SYS_RAM_BASE		0x28000000
#define CONFIG_SYS_RAM_SIZE		(8 * 1024 * 1024)

/* Buffers for Ethernet DMA (cannot be in the internal System RAM) */
#define CONFIG_MEM_ETH_DMA_BUF_BASE	0x20000000	/* Region of SRAM */

/* Use the CPU_CLOCK/2 for EMC */
#define CONFIG_LPC18XX_EMC_HALFCPU

#define CONFIG_SYS_NO_FLASH
#define CONFIG_SPIFI
#define CONFIG_SPIFI_BASE		0x14000000
#define CONFIG_SPIFI_SIZE		(16*1024*1024)
#define CONFIG_SPIFILIB_IN_ENVM		/* Place SPIFI lib into ENVM */

/* Serial console configuration */
#define CONFIG_SYS_NS16550		1
#undef CONFIG_NS16550_MIN_FUNCTIONS
#define CONFIG_SYS_NS16550_SERIAL	1
/*
 * Registers are 32-bit. The negative value tells the ns16550 driver that
 * registers should be post-padded with zeroes (because the CPU is in
 * little-endian mode.)
 */
#define CONFIG_SYS_NS16550_REG_SIZE	(-4)
#define CONFIG_CONS_INDEX		1

/*
 * USART0 registers base: 0x40081000
 * UART1 registers base:  0x40082000
 * USART2 registers base: 0x400C1000
 * USART3 registers base: 0x400C2000
 */

/*
 * We could choose the USART3 because it's the one that's powered
 * by the host. USART2 is available, but through the FTDI
 * chip so it's powered by the board.
 *
 * However, most mortals will prefer to connect just one cable (the USB)
 * and get console from there.
 */
#define CIAA_CONSOLE_ON_USART2

#ifdef CIAA_CONSOLE_ON_USART3
#define CONFIG_SYS_NS16550_COM1		0x400c2000
#define CONFIG_SYS_NS16550_CLK		clock_get(CLOCK_UART3)
/*
 * Pin configuration for USART3
 */
#define CONFIG_LPC18XX_UART_TX_IO_GROUP		2       /* P2 */
#define CONFIG_LPC18XX_UART_TX_IO_PIN		3       /* P2.3 = USART3 TXD */
#define CONFIG_LPC18XX_UART_TX_IO_FUNC		2
#define CONFIG_LPC18XX_UART_RX_IO_GROUP		2       /* P2 */
#define CONFIG_LPC18XX_UART_RX_IO_PIN		4       /* P2.4 = USART3 RXD */
#define CONFIG_LPC18XX_UART_RX_IO_FUNC		2
#endif

#ifdef CIAA_CONSOLE_ON_USART2
#define CONFIG_SYS_NS16550_COM1		0x400c1000
#define CONFIG_SYS_NS16550_CLK		clock_get(CLOCK_UART2)
/*
 * Pin configuration for USART2
 */
#define CONFIG_LPC18XX_UART_TX_IO_GROUP		7       /* P7 */
#define CONFIG_LPC18XX_UART_TX_IO_PIN		1       /* P7.1 = USART2 TXD */
#define CONFIG_LPC18XX_UART_TX_IO_FUNC		6
#define CONFIG_LPC18XX_UART_RX_IO_GROUP		7       /* P7 */
#define CONFIG_LPC18XX_UART_RX_IO_PIN		2       /* P7.2 = USART2 RXD */
#define CONFIG_LPC18XX_UART_RX_IO_FUNC		6
#endif

#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }

/*
 * Ethernet configuration
 */
#define CONFIG_NET_MULTI
#define CONFIG_LPC18XX_ETH
#define CONFIG_LPC18XX_ETH_DIV_SEL	4	/* 150-250 MHz */
#define CONFIG_LPC18XX_ENET_USE_PHY_RMII

/*
 * Ethernet RX buffers are malloced from the internal SRAM (more precisely,
 * from CONFIG_SYS_MALLOC_LEN part of it). Each RX buffer has size of 1536B.
 * So, keep this in mind when changing the value of the following config,
 * which determines the number of ethernet RX buffers (number of frames which
 * may be received without processing until overflow happens).
 */
#define CONFIG_SYS_RX_ETH_BUFFER	2
#define CONFIG_SYS_TX_ETH_BUFFER	2

/* Console I/O buffer size */
#define CONFIG_SYS_CBSIZE		256

/* Print buffer size */
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_SYS_MEMTEST_START	CONFIG_SYS_RAM_BASE
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_RAM_BASE + CONFIG_SYS_RAM_SIZE)

/* Needed by "loadb" */
#define CONFIG_SYS_LOAD_ADDR		CONFIG_SYS_RAM_BASE

/*
 * Monitor is actually in eNVM. In terms of U-Boot, it is neither "flash",
 * not RAM, but CONFIG_SYS_MONITOR_BASE must be defined.
 */
#define CONFIG_SYS_MONITOR_BASE		0x0

/*
 * Monitor is not in flash. Needs to define this to prevent
 * U-Boot from running flash_protect() on the monitor code.
 */
#define CONFIG_MONITOR_IS_IN_RAM	1

/*
 * Enable all those monitor commands that are needed
 */
#include <config_cmd_default.h>
#undef CONFIG_CMD_BOOTD
#undef CONFIG_CMD_CONSOLE
#undef CONFIG_CMD_ECHO
#undef CONFIG_CMD_EDITENV
#undef CONFIG_CMD_FPGA
#undef CONFIG_CMD_IMI
#undef CONFIG_CMD_ITEST
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_LOADS
#undef CONFIG_CMD_MISC
#define CONFIG_CMD_NET	/* Obligatory for the Ethernet driver to build */
#undef CONFIG_CMD_NFS
#undef CONFIG_CMD_SOURCE
#undef CONFIG_CMD_XIMG

/* To save memory disable long help */
#undef CONFIG_SYS_LONGHELP

/* Max number of command args */
#define CONFIG_SYS_MAXARGS		16

/* Auto-boot sequence configuration */
#define CONFIG_BOOTDELAY		3
#define CONFIG_ZERO_BOOTDELAY_CHECK
#define CONFIG_HOSTNAME			ciaa-nxp
#define CONFIG_BOOTARGS			"console=ttyS0,115200 earlyprintk"
#define CONFIG_BOOTCOMMAND		"run flashboot"

/* Ensure that the board-specific misc_init_r() gets invoked. */
#define CONFIG_MISC_INIT_R

/*
 * Short-cuts to some useful commands (macros)
 */
#define CONFIG_EXTRA_ENV_SETTINGS				\
	"uboot_image=u-boot.bin\0"				\
	"image=uImage-ciaa-nxp\0"				\
	"loadaddr=0x28000000\0"					\
	"addip=setenv bootargs ${bootargs} ip=${ipaddr}:${serverip}:${gatewayip}:${netmask}:${hostname}:eth0:off\0" \
	"flashaddr=0x14000000\0"				\
	"flashboot=run addip;bootm ${flashaddr}\0"		\
	"ipaddr=192.168.0.103\0"				\
	"serverip=192.168.0.45\0"				\
	"ethaddr=C0:B1:3C:88:78:93\0"                           \
	"netboot=tftp ${image};run addip;bootm\0"		\
	"update=tftp ${image};cp.b ${loadaddr} ${flashaddr} ${filesize}\0" \
	"uboot_update=tftp ${uboot_image};cptf 0x1a000000 ${loadaddr} ${filesize}\0"

/* Linux kernel boot parameters configuration */
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_CMDLINE_TAG

#endif /* __CONFIG_H */
