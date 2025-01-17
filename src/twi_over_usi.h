/*
 * Header file for the USI TWI Slave driver.
 *
 * Created: 1/8/2025 1:15:00 PM
 * Author : Mikhail
 * ------------------------------------------------------------------------------

 * Created from Atmel source files for Application Note AVR312: Using the USI 
 * Module as an I2C slave.
 */

#ifndef _TWI_OVER_USI_SLAVE_H_
#define _TWI_OVER_USI_SLAVE_H_

#include <stdbool.h>

/*void    usiTwiSlaveInit( uint8_t );*/
void usi_twi_slave_init(uint8_t);
bool if_tx_buffer_is_not_empty(void);
/*void    usiTwiTransmitByte( uint8_t );*/
void usi_twi_transmit_byte(uint8_t);
/*uint8_t usiTwiReceiveByte( void ); */
uint8_t usi_twi_receive_byte(void);
/*bool    usiTwiDataInReceiveBuffer( void );*/
bool if_rx_buffer_is_not_empty(void);
/*void    (*_onTwiDataRequest)(void); */
/*bool    usiTwiDataInTransmitBuffer(void);*/

/* TWI Buffers  */
/* RX buffer sizes could be: 1, 2, 4, 8, 16, 32, 64, 128 or 256 */
#define TWI_RX_BUFFER_SIZE  ( 16 )
#define TWI_RX_BUFFER_MASK (0x0f)

/* TX buffer sizes could be: 1, 2, 4, 8, 16, 32, 64, 128 or 256 */
#define TWI_TX_BUFFER_SIZE ( 16 )
#define TWI_TX_BUFFER_MASK (0x0f)

#endif  // ifndef _TWI_OVER_USI_SLAVE_H_
