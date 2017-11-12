/*
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the
 *	  distribution.
 *
 *	* Neither the name of Texas Instruments Incorporated nor the names of
 *	  its contributors may be used to endorse or promote products derived
 *	  from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <stdio.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include <rsc_types.h>
#include <pru_rpmsg.h>
#include "PRU1_resource_table.h"

volatile register uint32_t __R31;

/* Host-0 Interrupt sets bit 30 in register R31 */
#define HOST_INT			((uint32_t) 1 << 30)

/* The PRU-ICSS system events used for RPMsg are defined in the Linux device tree
 * PRU0 uses system event 16 (To ARM) and 17 (From ARM)
 * PRU1 uses system event 18 (To ARM) and 19 (From ARM)
 */
#define TO_ARM_HOST			18
#define FROM_ARM_HOST			19

/*
 * Using the name 'rpmsg-pru' will probe the rpmsg_pru driver found
 * at linux-x.y.z/drivers/rpmsg/rpmsg_pru.c
 */
#define CHAN_NAME			"rpmsg-pru"
#define CHAN_DESC			"Channel 30"
#define CHAN_PORT			30

/*
 * Used to make sure the Linux drivers are ready for RPMsg communication
 * Found at linux-x.y.z/include/uapi/linux/virtio_config.h
 */
#define VIRTIO_CONFIG_S_DRIVER_OK	4

uint8_t payload[RPMSG_BUF_SIZE];


static unsigned char lookup[16] = {
	0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
	0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf, };

uint8_t reverse(uint8_t n) {
	// Reverse the top and bottom nibble then swap them.
	return (lookup[n&0b1111] << 4) | lookup[n>>4];
}

// Detailed breakdown of the math
//  + lookup reverse of bottom nibble
//  |       + grab bottom nibble
//  |       |        + move bottom result into top nibble
//  |       |        |     + combine the bottom and top results 
//  |       |        |     | + lookup reverse of top nibble
//  |       |        |     | |       + grab top nibble
//  V       V        V     V V       V
// (lookup[n&0b1111] << 4) | lookup[n>>4]

/*
 * main.c
 */
void main(void)
{
	CT_CFG.GPCFG1_bit.PRU1_GPI_MODE = 0x0; // direct in

	struct pru_rpmsg_transport transport;
	uint16_t src, dst, len;
	volatile uint8_t *status;

	/* Allow OCP master port access by the PRU so the PRU can read external memories */
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	/* Clear the status of the PRU-ICSS system event that the ARM will use to 'kick' us */
	CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;

	/* Make sure the Linux drivers are ready for RPMsg communication */
	status = &resourceTable.rpmsg_vdev.status;
	while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

	/* Initialize the RPMsg transport structure */
	pru_rpmsg_init(&transport, &resourceTable.rpmsg_vring0, &resourceTable.rpmsg_vring1, TO_ARM_HOST, FROM_ARM_HOST);

	/* Create the RPMsg channel between the PRU and ARM user space using the transport structure. */
	while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, CHAN_NAME, CHAN_DESC, CHAN_PORT) != PRU_RPMSG_SUCCESS);
	int cond = 0;
	// while (cond) {
	// 	/* Check bit 30 of register R31 to see if the ARM has kicked us */
	// 	if (__R31 & HOST_INT) {
	// 		/* Clear the event status */
	// 		CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;
	// 		/* Receive all available messages, multiple messages can be sent per kick */
	// 		while (pru_rpmsg_receive(&transport, &src, &dst, payload, &len) == PRU_RPMSG_SUCCESS) {
	// 			/* Echo the message back to the same address from which we just received */
	// 			pru_rpmsg_send(&transport, dst, src, payload, len);
	// 			cond = 0;
	// 		}
	// 	}
	// }

	CT_CFG.GPCFG1_bit.PRU1_GPI_MODE = 0x2; // 28-bit shift in

	CT_CFG.GPCFG1_bit.PRU1_GPI_DIV0 = 0x17; // 200MHz / 12.5 -> 16MHz
	//CT_CFG.GPCFG1_bit.PRU1_GPI_DIV1 = 0x1E; // -> 1MHz
	CT_CFG.GPCFG1_bit.PRU1_GPI_DIV1 = 0x0E; // -> 2MHz
	//CT_CFG.GPCFG1_bit.PRU1_GPI_DIV1 = 0x06; // -> 4MHz



	src = 1024; // found out to be working ok
	dst = 30;
	while(1) {

			memcpy( payload, "                                                                 \n", 67 );

			cond = 31;

			while( ~__R31 & 1<<28 ) { };  // wait for cnt16 (replaceable by system event)

			while( ~__R31 & 1<<19 ) { };  // wait for exactly 10 bits of data

			int s = __R31;

			if( ~s & 1<<18 ) {
				memcpy( payload+58, "broken?", 7 );
				s>>=1;
			}

			// 2MHz sampling of inversed 'x' byte: 1100000000011111111000000001100

			payload[47] = '>';
			payload[48] = 0xFF & ~reverse(s>>1); // true for 1MHz sampling and transmission

			int a = s>>2;
			a = ~reverse( (1 & a>>0)<<0 | (1 & a>>2)<<1 | (1 & a>>4)<<2 | (1 & a>>6)<<3 | (1 & a>>8)<<4 | (1 & a>>10)<<5 | (1 & a>>12)<<6 | (1 & a>>14)<<7 );

			payload[50] = '>';
			payload[51] = a; // true for 2MHz sampling, 1MHz transmission

			//int b = s>>3;
			//b = ~reverse( (1 & b>>0)<<0 | (1 & b>>2)<<1 | (1 & b>>4)<<2 | (1 & b>>6)<<3 | (1 & b>>8)<<4 | (1 & b>>10)<<5 | (1 & b>>12)<<6 | (1 & b>>14)<<7 );

			//payload[51] = b; // true for 2MHz sampling, 1MHz transmission

			CT_CFG.GPCFG1_bit.PRU1_GPI_SB = 1; // clear start bit
			
			while( s ) {
				payload[cond--] = '0' + s % 2;
				s /= 2;
			}

			// char help[] = "expected: 1 11100001 0 for x, 1 01100001 0 for y, 1 11111111 0 for \\0\n";

			pru_rpmsg_send(&transport, dst, src, payload, 67);

	}
}
