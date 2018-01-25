#include "common.h"
#include <stdint.h>
#include <types.h>
#include <pru_cfg.h>

volatile register uint32_t __R30;
volatile register uint32_t __R31;

// #define UART_3M
#define UART_4_16M

#ifdef UART_3M
#warn "UART_3M used!"
#endif

#ifdef UART_4M
#warn "UART_4M used!"
#endif

void uart_init( uart_mode_t mode ) 
{
	/* NOTE: TX hardcoded to run on PRU0, RX on PRU1 */
	switch(mode) {
		case TX:
			CT_CFG.GPCFG0_bit.PRU0_GPO_MODE = 0x0; /* direct out */
			__delay_cycles( 100 );
#ifdef UART_3M
			CT_CFG.GPCFG0_bit.PRU0_GPO_DIV0 = 0x14; /*   200 /   11 = 18.18 MHz */
			CT_CFG.GPCFG0_bit.PRU0_GPO_DIV1 = 0x0a; /* 18.18 /    6 =  3.03 MHz */
#elif defined UART_4M
#warn "4M"
			CT_CFG.GPCFG0_bit.PRU0_GPO_DIV0 = 0x17; /*   200 / 12.5 =    16 MHz */
			CT_CFG.GPCFG0_bit.PRU0_GPO_DIV1 = 0x06; /*    16 /    4 =     4 MHz */
#elif defined UART_4_16M
#warn "4_16M"
			CT_CFG.GPCFG0_bit.PRU0_GPO_DIV0 = 0x1e; /*   200 /   16 =  12.5 MHz */
			CT_CFG.GPCFG0_bit.PRU0_GPO_DIV1 = 0x04; /*  12.5 /    3 = 4_167 MHz */
#else
			CT_CFG.GPCFG0_bit.PRU0_GPO_DIV0 = 0x17; /*   200 / 12.5 =    16 MHz */
			CT_CFG.GPCFG0_bit.PRU0_GPO_DIV1 = 0x0e; /*    16 /    8 =     2 MHz */
#endif
			__delay_cycles( 100 );
			CT_CFG.GPCFG0_bit.PRU0_GPO_MODE = 0x1; /* shift out */

			break;
		case RX:
			CT_CFG.GPCFG1_bit.PRU1_GPI_MODE = 0x0; /* direct in */
			__delay_cycles( 100 );
#ifdef UART_3M
			CT_CFG.GPCFG1_bit.PRU1_GPI_DIV0 = 0x9;  /*   200 / 5.5  = 36.36 MHz */
			CT_CFG.GPCFG1_bit.PRU1_GPI_DIV1 = 0x1;  /* 36.36 / 1.5  = 24.24 MHz */
#elif defined UART_4M
#warn "4M"
			CT_CFG.GPCFG1_bit.PRU1_GPI_DIV0 = 0x03; /*   200 /  2.5 =    80 MHz */
			CT_CFG.GPCFG1_bit.PRU1_GPI_DIV1 = 0x03; /*    80 /  2.5 =    32 MHz */
#elif defined UART_4_16M
#warn "4_16M"
			CT_CFG.GPCFG1_bit.PRU1_GPI_DIV0 = 0x0a; /*   200 /    6 = 33.33 MHz */
			CT_CFG.GPCFG1_bit.PRU1_GPI_DIV1 = 0x00; /* 33.33 /    1 = 33.33 MHz */
#else
			CT_CFG.GPCFG1_bit.PRU1_GPI_DIV0 = 0x17; /*   200 / 12.5 =    16 MHz */
			CT_CFG.GPCFG1_bit.PRU1_GPI_DIV1 = 0x00; /*    16 /    1 =    16 MHz */
#endif
			__delay_cycles( 100 );
			CT_CFG.GPCFG1_bit.PRU1_GPI_MODE = 0x2; /* 28-bit shift in */
			__delay_cycles( 100 );

#ifdef UART_3M
			CT_CFG.GPCFG1_bit.PRU1_GPI_DIV0 = 0x9;  /*   200 / 5.5  = 36.36 MHz */
			CT_CFG.GPCFG1_bit.PRU1_GPI_DIV1 = 0x1;  /* 36.36 / 1.5  = 24.24 MHz */
#elif defined UART_4M
#warn "4M"
			CT_CFG.GPCFG1_bit.PRU1_GPI_DIV0 = 0x03; /*   200 /  2.5 =    80 MHz */
			CT_CFG.GPCFG1_bit.PRU1_GPI_DIV1 = 0x03; /*    80 /  2.5 =    32 MHz */
#elif defined UART_4_16M
#warn "4_16M"
			CT_CFG.GPCFG1_bit.PRU1_GPI_DIV0 = 0x0a; /*   200 /    6 = 33.33 MHz */
			CT_CFG.GPCFG1_bit.PRU1_GPI_DIV1 = 0x00; /* 33.33 /    1 = 33.33 MHz */
#else
			CT_CFG.GPCFG1_bit.PRU1_GPI_DIV0 = 0x17; /*   200 / 12.5 =    16 MHz */
			CT_CFG.GPCFG1_bit.PRU1_GPI_DIV1 = 0x00; /*    16 /    1 =    16 MHz */
#endif
			__delay_cycles( 100 );
			CT_CFG.GPCFG1_bit.PRU1_GPI_MODE = 0x2; /* 28-bit shift in */
			break;
	}
}

#ifdef BITBANG_TX
#error "not implemented!"
void uart_tx( uint8_t *payload, int len )
{
}
#else
void uart_tx( uint8_t *payload, int len )
{
	while(len > 0) {
		char msg[3] = {0};
		memcpy( msg, payload, (len < 3 ? len : 3) );
		len -= 3;
		payload+=3;

		__R30 |= 1<<29; 				/* set LOAD_GPO_SH0 */
		__R30 &= ~0xFFFF; 				/* reset R30[0:15] */
		__R30 |= ( 0x01FF & ~(msg[0]<<1) ) | ( (!msg[1] ? 0 : 0xFC00) & ~(msg[1]<<11 ) );
		__R30 &= ~(1<<29); 				/* reset LOAD_GPO_SH0 */

		__R30 |= 1u<<31; 				/* set ENABLE_SHIFT */

		while( CT_CFG.GPCFG0_bit.PRU0_GPO_SH_SEL == 1 ) {}; /* wait until it's zero */

		__R30 |= 1<<30; 	/* set LOAD_GPO_SH1 */
		__R30 &= 0xFFFF0000;
		__R30 |= ( (!msg[1] ? 0 : 0x0007) & ~(msg[1]>>5) ) | ( (!msg[2] ? 0 : 0x1FF0) & ~(msg[2]<<5 ) );
		__R30 &= ~(1<<30); 	/* reset LOAD_GPO_SH1 */

		/* NOTE: two bits are wasted every 32 bits for the sake of simplicity */
		/* NOTE: this could be simplified even more? */

		while( CT_CFG.GPCFG0_bit.PRU0_GPO_SH_SEL == 0 ) {}; /* wait until it's one */
	}

	__R30 = 1<<29; /* reset ENABLE_SHIFT and SH0 register */
}
#endif
