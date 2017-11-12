/*
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
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
#include <pru_cfg.h>
#include "PRU0_resource_table.h"
#include <types.h>

volatile register uint32_t __R30;
volatile register uint32_t __R31;

void main(void)
{
	volatile uint32_t gpio;

	/* Clear SYSCFG[STANDBY_INIT] to enable OCP master port */
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	__delay_cycles(2000); // 1ms

	gpio = 1; /* pr1_pru0_pru_r30_0, in order to use it, configure P9_31 as pruout (mode 5) */

	/*
	 * 1st example: 
	 * 	repeatedly transmit 'U' character
	 *
	 * char test[] = {1, 0, 1, 0, 1, 0, 1, 0, 1, 0};
	 * int i = 0;
	 * __R30 = gpio;
	 * __delay_cycles(1000000);
	 * while (1) {
	 * 	__R30 = test[9-i%10];
	 * 	__delay_cycles(1000);
	 * 	i++;
	 * 	if( i%10 == 0 ) 
	 * 		__delay_cycles(4000);
	 * }
	*/

	CT_CFG.GPCFG0_bit.PRU0_GPO_MODE = 0x0; // direct out

	__R30 = 0;


	// 0x00 - 1;
	// 0x01 - 1.5;
	// 0x02 - 2;
	// 0x03 - 2.5;
	// 0x04 - 3;
	// 0x05 - 3.5;
	// 0x06 - 4;
	// 0x07 - 4.5;
	// 0x08 - 5;
	// 0x09 - 5.5;
	// 0x0a - 6;
	// 0x0b - 6.5;
	// 0x0c - 7;
	// 0x0d - 7.5;
	// 0x0e - 8;
	// 0x0f - 8.5;
	// 0x10 - 9;
	// 0x11 - 9.5;
	// 0x12 - 10;
	// 0x13 - 10.5;
	// 0x14 - 11;
	// 0x15 - 11.5;
	// 0x16 - 12;
	// 0x17 - 12.5;
	// 0x18 - 13;
	// 0x19 - 13.5;
	// 0x1a - 14;
	// 0x1b - 14.5;
	// 0x1c - 15;
	// 0x1d - 15.5;
	// 0x1e - 16;

	// x = div*2-2;

	// CT_CFG.GPCFG0_bit.PRU0_GPO_DIV0 = 0x1e; // 200 / 16 = 12.5
	// CT_CFG.GPCFG0_bit.PRU0_GPO_DIV1 = 0x1e; // 12.5 / 16 = 0.78125

	__delay_cycles(400000000); // 2s?

	__R30 = 1<<5;

	CT_CFG.GPCFG0_bit.PRU0_GPO_MODE = 0x1; // shift out

	CT_CFG.GPCFG0_bit.PRU0_GPO_DIV0 = 0x17; // 200 / 12.5 = 16
	CT_CFG.GPCFG0_bit.PRU0_GPO_DIV1 = 0x1e; // 16 / 16 = 1

	unsigned char msg[] = "xy";
	//unsigned char msg[] = "lubie Martyne\r\n";
	int i = 0;

	__R30 |= 1<<29; 	/* set LOAD_GPO_SH0 */
	__R30 &= 0xFFFF0000;
	__R30 &= ~(1<<29); 	/* reset LOAD_GPO_SH0 */

	__R30 |= 1<<31;		/* set ENABLE_SHIFT */

	while(1) {

		while( CT_CFG.GPCFG0_bit.PRU0_GPO_SH_SEL == 1 ) {}; // wait until it's zero

		__R30 |= 1<<30; 				/* set LOAD_GPO_SH1 */
		__R30 &= ~0xFFFF; 				/* reset R30[0:15] */
		__R30 |= ( 0x01FF & ~(msg[(i+0)%sizeof(msg)]<<1) ) | ( 0xFC00 & ~(msg[(i+1)%sizeof(msg)]<<11 ) );
		__R30 &= ~(1<<30); 				/* reset LOAD_GPO_SH1 */

		while( CT_CFG.GPCFG0_bit.PRU0_GPO_SH_SEL == 0 ) {}; // wait until it's one

		__R30 |= 1<<29; 	/* set LOAD_GPO_SH0 */
		__R30 &= 0xFFFF0000;
		__R30 |= ( 0x0007 & ~(msg[(i+1)%sizeof(msg)]>>5) ) | ( 0x1FF0 & ~(msg[(i+2)%sizeof(msg)]<<5 ) );
		__R30 &= ~(1<<29); 	/* reset LOAD_GPO_SH0 */

		i+=3;


		while( CT_CFG.GPCFG0_bit.PRU0_GPO_SH_SEL == 1 ) {}; // wait until it's zero

		__R30 = 0;

		while( CT_CFG.GPCFG0_bit.PRU0_GPO_SH_SEL == 0 ) {}; // wait until it's zero
		
		CT_CFG.GPCFG0_bit.PRU0_GPO_MODE = 0x0; // direct out


		__delay_cycles(200000000);

		CT_CFG.GPCFG0_bit.PRU0_GPO_MODE = 0x1; // shift out

		__R30 |= 1<<29 | 1<<30 ; 	/* set LOAD_GPO_SH0/1 */
		__R30 &= 0xFFFF0000;
		__R30 &= ~(1<<29 | 1<<30); 	/* reset LOAD_GPO_SH0/1 */

		__R30 |= 1<<31;

	}

	__R30 &= ~(1<<31);	/* reset ENABLE_SHIFT */
}

