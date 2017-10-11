/* ---------------------------------------------------------------------------- */
/*                  Atmel Microcontroller Software Support                      */
/*                       SAM Software Package License                           */
/* ---------------------------------------------------------------------------- */
/* Copyright (c) 2015, Atmel Corporation                                        */
/*                                                                              */
/* All rights reserved.                                                         */
/*                                                                              */
/* Redistribution and use in source and binary forms, with or without           */
/* modification, are permitted provided that the following condition is met:    */
/*                                                                              */
/* - Redistributions of source code must retain the above copyright notice,     */
/* this list of conditions and the disclaimer below.                            */
/*                                                                              */
/* Atmel's name may not be used to endorse or promote products derived from     */
/* this software without specific prior written permission.                     */
/*                                                                              */
/* DISCLAIMER:  THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR   */
/* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE   */
/* DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,      */
/* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT */
/* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,  */
/* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    */
/* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING         */
/* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, */
/* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                           */
/* ---------------------------------------------------------------------------- */

/**
 *  \page gmac_uip_telnetd GMAC Telnetd Example
 *
 *  \section Purpose
 *
 *  This project implements a telnet server example of the uIP TCP/IP stack.
 *  It enables the device to act as a simple telnetd server.
 *
 *  \section Requirements
 *
 * This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 *  \section Description
 *
 * Please refer to the uIP documentation for more information
 * about the TCP/IP stack, the telnetd example.
 *
 * By default, the example does not use DHCP.
 * If you want to use DHCP, please:
 * - Open file uip-conf.h and don't comment the line "#define UIP_DHCP_on".
 * - Include uip/apps/dhcps to compile.
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the board.
 *     Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 baud rate
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Connect an Ethernet cable between the evaluation board and the network.
 *      The board may be connected directly to a computer; in this case,
 *      make sure to use a cross/twisted wired cable such as the one provided
 *      with the evaluation kit.
 *  -# Start the application. It will display the following message on the terminal:
 *    \code
 *    -- GMAC uIP Telnetd Example xxx --
 *    -- xxxxxx-xx
 *    -- Compiled: xxx xx xxxx xx:xx:xx --
 *    - MAC 3a:1f:34:08:54:05
 *    - Host IP 10.217.12.223
 *    - Router IP 10.217.12.1
 *    - Net Mask 255.255.255.0
 *    \endcode
 *  -# Connect to the %device IP address using telnet on port 23:
 *    \code
 *    telnet 10.217.12.223 23
 *    \endcode
 *    A telnet terminal will appear:
 *    \code
 *    uIP command shell
 *    Type '?' and return for help
 *    uIP 1.0>
 *    \endcode
 * \note
 * Make sure the IP address of the device(EK board) and the computer are in the same network.
 *
 *  \section References
 *  - gmac_uip_telnetd/main.c
 *  - gmacb.h
 *  - gmacd.h
 *  - gmac.h
 *  - uip.h
 */

/** \file
 *
 *  This file contains all the specific code for the gmac_uip_telnetd example.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include "uip.h"
#include "uip_arp.h"
#include "gmac_tapdev.h"
#include "timer.h"

/*---------------------------------------------------------------------------
 *         Variables
 *---------------------------------------------------------------------------*/

/** TWI clock frequency in Hz. */
#define TWCK            400000
/** Slave address of twi_eeprom AT24MAC.*/
#define AT24MAC_SERIAL_NUM_ADD  0x5F
/** Page size of an AT24MAC402 chip (in bytes)*/
#define PAGE_SIZE       16
/** Page numbers of an AT24MAC402 chip */
#define EEPROM_PAGES    16
/** EEPROM Pins definition */
#define BOARD_PINS_TWI_EEPROM PINS_TWI0
/** TWI0 peripheral ID for EEPROM device*/
#define BOARD_ID_TWI_EEPROM   ID_TWIHS0
/** TWI0 base address for EEPROM device */
#define BOARD_BASE_TWI_EEPROM TWIHS0

/* uIP buffer : The ETH header */
#define BUF ((struct uip_eth_hdr *)&uip_buf[0])

/** TWI driver instance.*/
static Twid twid;

/** The PINs for TWI*/
static const Pin twiPins[]      = BOARD_PINS_TWI_EEPROM;

/* The MAC address used for demo */
static struct uip_eth_addr GMacAddress = {{0x3a, 0x1f, 0x34, 0x08, 0x54, 0x54}};

/* The IP address used for demo (ping ...) */
static uint8_t HostIpAddress[4] = {192, 168, 1, 3 };

/* Set the default router's IP address. */
static const uint8_t RoutIpAddress[4] = {192, 168, 1, 2 };

/* The NetMask address */
static const uint8_t NetMask[4] = {255, 255, 255, 0};

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * Initialize demo application
 */
static void _app_init(void)
{
	printf("P: telnetd application init\n\r");
	telnetd_init();

#ifdef __DHCPC_H__
	printf("P: DHCPC Init\n\r");
	dhcpc_init(MacAddress.addr, 6);
#endif
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 * uip_log: Global function for uIP to use.
 * \param m Pointer to string that logged
 */
void uip_log(char *m)
{
	TRACE_INFO("-uIP log- %s\n\r", m);
}

#ifdef __DHCPC_H__
/**
 * dhcpc_configured: Global function for uIP DHCPC to use,
 * notification of DHCP configuration.
 * \param s Pointer to DHCP state instance
 */
void dhcpc_configured(const struct dhcpc_state *s)
{
	u8_t *pAddr;

	printf("\n\r");
	printf("=== DHCP Configurations ===\n\r");
	pAddr = (u8_t *)s->ipaddr;
	printf("- IP   : %d.%d.%d.%d\n\r",
			pAddr[0], pAddr[1], pAddr[2], pAddr[3]);
	pAddr = (u8_t *)s->netmask;
	printf("- Mask : %d.%d.%d.%d\n\r",
			pAddr[0], pAddr[1], pAddr[2], pAddr[3]);
	pAddr = (u8_t *)s->default_router;
	printf("- GW   : %d.%d.%d.%d\n\r",
			pAddr[0], pAddr[1], pAddr[2], pAddr[3]);
	pAddr = (u8_t *)s->dnsaddr;
	printf("- DNS  : %d.%d.%d.%d\n\r",
			pAddr[0], pAddr[1], pAddr[2], pAddr[3]);
	printf("===========================\n\r\n");
	uip_sethostaddr(s->ipaddr);
	uip_setnetmask(s->netmask);
	uip_setdraddr(s->default_router);

#ifdef __RESOLV_H__
	resolv_conf(s->dnsaddr);
#else
	printf("DNS NOT enabled in the demo\n\r");
#endif
}
#endif

/**
 *  \brief gmac_uip_telnetd example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uip_ipaddr_t ipaddr;
	struct timer periodic_timer, arp_timer;
	uint32_t i;
	struct uip_eth_addr OrigiGMacAddr;

	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	TimeTick_Configure();
	printf("-- GMAC uIP Telnetd Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ ,
			COMPILER_NAME);

	/* Configure systick for 1 ms. */
	TimeTick_Configure();
	/* Configure TWI pins. */
	PIO_Configure(twiPins, PIO_LISTSIZE(twiPins));
	/* Enable TWI */
	PMC_EnablePeripheral(BOARD_ID_TWI_EEPROM);
	TWI_ConfigureMaster(BOARD_BASE_TWI_EEPROM, TWCK, BOARD_MCK);
	TWID_Initialize(&twid, BOARD_BASE_TWI_EEPROM);
	/* Display MAC & IP settings */
	TWID_Read(&twid, AT24MAC_SERIAL_NUM_ADD, 0x9A, 1, OrigiGMacAddr.addr, PAGE_SIZE,
				0);

	if ((OrigiGMacAddr.addr[0] == 0xFC) && (OrigiGMacAddr.addr[1] == 0xC2)
		&& (OrigiGMacAddr.addr[2] == 0x3D)) {
		for (i = 0; i < 6; i++)
			GMacAddress.addr[i] = OrigiGMacAddr.addr[i];
	}

	printf("-- MAC %x:%x:%x:%x:%x:%x\n\r",
			GMacAddress.addr[0], GMacAddress.addr[1], GMacAddress.addr[2],
			GMacAddress.addr[3], GMacAddress.addr[4], GMacAddress.addr[5]);

#ifndef __DHCPC_H__
	printf(" - Host IP  %d.%d.%d.%d\n\r",
			HostIpAddress[0], HostIpAddress[1], HostIpAddress[2], HostIpAddress[3]);
	printf(" - Router IP  %d.%d.%d.%d\n\r",
			RoutIpAddress[0], RoutIpAddress[1], RoutIpAddress[2], RoutIpAddress[3]);
	printf(" - Net Mask  %d.%d.%d.%d\n\r",
			NetMask[0], NetMask[1], NetMask[2], NetMask[3]);
#endif

	/* System devices initialize */
	gmac_tapdev_setmac((uint8_t *)GMacAddress.addr);
	gmac_tapdev_init();
	clock_init();
	timer_set(&periodic_timer, CLOCK_SECOND / 2);
	timer_set(&arp_timer, CLOCK_SECOND * 10);

	/* Init uIP */
	uip_init();

#ifdef __DHCPC_H__
	printf("P: DHCP Supported\n\r");
	uip_ipaddr(ipaddr, 0, 0, 0, 0);
	uip_sethostaddr(ipaddr);
	uip_ipaddr(ipaddr, 0, 0, 0, 0);
	uip_setdraddr(ipaddr);
	uip_ipaddr(ipaddr, 0, 0, 0, 0);
	uip_setnetmask(ipaddr);
#else
	/* Set the IP address of this host */
	uip_ipaddr(ipaddr, HostIpAddress[0], HostIpAddress[1],
				HostIpAddress[2], HostIpAddress[3]);
	uip_sethostaddr(ipaddr);

	uip_ipaddr(ipaddr, RoutIpAddress[0], RoutIpAddress[1],
				RoutIpAddress[2], RoutIpAddress[3]);
	uip_setdraddr(ipaddr);

	uip_ipaddr(ipaddr, NetMask[0], NetMask[1], NetMask[2], NetMask[3]);
	uip_setnetmask(ipaddr);
#endif
	uip_setethaddr(GMacAddress);
	_app_init();

	while (1) {
		uip_len = gmac_tapdev_read();

		if (uip_len > 0) {
			if (BUF->type == htons(UIP_ETHTYPE_IP)) {
				uip_arp_ipin();
				uip_input();

				/* If the above function invocation resulted in data that
					should be sent out on the network, the global variable
					uip_len is set to a value > 0. */
				if (uip_len > 0) {
					uip_arp_out();
					gmac_tapdev_send();
				}
			} else if (BUF->type == htons(UIP_ETHTYPE_ARP)) {
					uip_arp_arpin();

				/* If the above function invocation resulted in data that
					should be sent out on the network, the global variable
					uip_len is set to a value > 0. */
				if (uip_len > 0)
					gmac_tapdev_send();
			}
		} else if (timer_expired(&periodic_timer)) {
			timer_reset(&periodic_timer);

			for (i = 0; i < UIP_CONNS; i++) {
				uip_periodic(i);

				/* If the above function invocation resulted in data that
					should be sent out on the network, the global variable
					uip_len is set to a value > 0. */
				if (uip_len > 0) {
					uip_arp_out();
					gmac_tapdev_send();
				}
			}

#if UIP_UDP

			for (i = 0; i < UIP_UDP_CONNS; i++) {
				uip_udp_periodic(i);

				/* If the above function invocation resulted in data that
					should be sent out on the network, the global variable
					uip_len is set to a value > 0. */
				if (uip_len > 0) {
					uip_arp_out();
					gmac_tapdev_send();
				}
			}

#endif /* UIP_UDP */

			/* Call the ARP timer function every 10 seconds. */
			if (timer_expired(&arp_timer)) {
				timer_reset(&arp_timer);
				uip_arp_timer();
			}
		}
	}
}

