/*
 * Stepper_Motor_Driver.c
 *
 * Created: 9/16/2024 12:51:18 AM
 * Author : Mikhail
 */ 

/* For util/delay.h define F_CPU macro. */
/* For some reason 0x1000000UL worked just fine here */
#define F_CPU 0x1000000UL
/*#define F_CPU 4000000UL*/
/*#define F_CPU 8000000UL*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "twi_over_usi.h"

#define CLOCKWISE_DIRECTION_HALF_STEPS
#define COUNTERCLOCKWISE_DIRECTION_HALF_STEPS

#define CLOCKWISE_DIRECTION_FULL_STEPS
#define COUNTERCLOCKWISE_DIRECTION_FULL_STEPS

#define HALF_STEP_CLOCKWISE
#define HALF_STEP_COUNTERCLOCKWISE
/* There are two (sets of) coils in bipolar stepper motor */
/* And there are 2 H bridges for bipolar stepper motor */
/* One H bridge per coil - call it H bridge 1 and H bridge 2 */
/* Each H bridge has "two sides" - "left side" and "right side" */
/* Each side has two half-s - "upper half" and "lower half" */
/* "Upper half" provides connection to DC positive pole */
/* "Upper half" activates by negative value ("0") */
/* "Lower half" provides connection to DC negative pole */
/* "Lower half" activates by positive value ("1") */
/* One coil will be active if the current in it's H bridge */
/* flows from one side DC positive pole through the coil */
/* to another side DC negative pole (from upper half to lower half) */

/* Both coils are inactive, all sides (right and left) are inactive, */
/* all halves of H bridges are inactive */
#define EVERYTHING_IS_INACTIVE 0x33
#define H_BRIDGE_1_INACTIVE 0x03
#define H_BRIDGE_2_INACTIVE 0x30

/*-----------------------------------*/
#define H_BRIDGE_1_UPPER_LEFT_WR 0x01
#define HB1_U_L_ACTIVE 0xFE
/* Z = Z & HB1_U_L_ACTIVE - Activate H bridge 1 upper left */
/* Z = EVERYTHING_IS_INACTIVE & HB1_U_L_ACTIVE == 0x33 & 0xFE == (0x32)  */
#define HB1_U_L_INACTIVE 0x01
/* Z = Z | HB1_U_L_INACTIVE - Deactivate H bridge 1 upper left */
/* Z = EVERYTHING_IS_INACTIVE | HB1_U_L_INACTIVE == 0x33 | 0x01 == (0x33)  */

#define H_BRIDGE_1_UPPER_RIGHT_GR 0x02
#define HB1_U_R_ACTIVE 0xFD
/* Z = Z & HB1_U_R_ACTIVE - Activate H bridge 1 upper right */
/* Z = EVERYTHING_IS_INACTIVE & HB1_U_R_ACTIVE == 0x33 & 0xFD == (0x31)  */
#define HB1_U_R_INACTIVE 0x02
/* Z = Z & HB1_U_R_INACTIVE - Deactivate H bridge 1 upper right */
/* Z = EVERYTHING_IS_INACTIVE | HB1_U_R_INACTIVE == 0x33 | 0x02 == (0x33)  */
			 
#define H_BRIDGE_1_LOWER_LEFT_WB 0x04
#define HB1_L_L_ACTIVE 0x04
/* Z = Z | HB1_L_L_ACTIVE - Activate H bridge 1 lower left */
/* Z = EVERYTHING_IS_INACTIVE | HB1_L_L_ACTIVE == 0x33 | 0x04 == (0x37)  */
#define HB1_L_L_INACTIVE 0xFB
/* Z = Z & HB1_L_L_INACTIVE - Deactivate H bridge 1 lower left */
/* Z = EVERYTHING_IS_INACTIVE & HB1_L_L_INACTIVE == 0x33 & 0xFB == (0x33)  */

#define H_BRIDGE_1_LOWER_RIGHT_GB 0x08
#define HB1_L_R_ACTIVE 0x08
/* Z = Z | HB1_L_R_ACTIVE - Activate H bridge 1 lower right */
/* Z = EVERYTHING_IS_INACTIVE | HB1_L_R_ACTIVE == 0x33 | 0x08 == (0x3B)  */
#define HB1_L_R_INACTIVE 0xF7
/* Z = Z & HB1_L_R_INACTIVE - Deactivate H bridge 1 lower right */
/* Z = EVERYTHING_IS_INACTIVE & HB1_L_R_INACTIVE == 0x33 & 0xF7 == (0x33)  */

/* Coil 1 active in "positive" direction if HB1_U_L_ACTIVE and HB1_L_R_ACTIVE */
/* Z = (EVERYTHING_IS_INACTIVE & HB1_U_L_ACTIVE ) | HB1_L_R_ACTIVE == (0x33 & 0xFE) | 0x08 == 0x32 | 0x08 == 0x3A */
#define COIL_1_ACTIVE_POSITIVE_DIRECTION 0x0A

/* Coil 1 active in "negative" direction if HB1_U_R_ACTIVE and HB1_L_L_ACTIVE */
/* Z = (EVERYTHING_IS_INACTIVE & HB1_U_R_ACTIVE ) | HB1_L_L_ACTIVE == (0x33 & 0xFD) | 0x04 == 0x31 | 0x04 == 0x35 */
#define COIL_1_ACTIVE_NEGATIVE_DIRECTION 0x05


/*---------------------------*/
#define H_BRIDGE_2_UPPER_LEFT_RR 0x10
#define HB2_U_L_ACTIVE 0xEF
/* Z = Z & HB2_U_L_ACTIVE - Activate H bridge 2 upper left */
/* Z = EVERYTHING_IS_INACTIVE & HB2_U_L_ACTIVE == 0x33 & 0xEF == (0x23)  */
#define HB2_U_L_INACTIVE 0x10
/* Z = Z | HB2_U_L_INACTIVE - Deactivate H bridge 2 upper left */
/* Z = EVERYTHING_IS_INACTIVE | HB2_U_L_INACTIVE == 0x33 | 0x10 == (0x33)  */

#define H_BRIDGE_2_UPPER_RIGHT_YR 0x20
#define HB2_U_R_ACTIVE 0xDF
/* Z = Z & HB2_U_R_ACTIVE - Activate H bridge 2 upper right */
/* Z = EVERYTHING_IS_INACTIVE & HB2_U_R_ACTIVE == 0x33 & 0xDF == (0x13)  */
#define HB2_U_R_INACTIVE 0x20
/* Z = Z | HB2_U_R_INACTIVE - Deactivate H bridge 2 upper right */
/* Z = EVERYTHING_IS_INACTIVE | HB2_U_R_INACTIVE == 0x33 | 0x20 == (0x33)  */

#define H_BRIDGE_2_LOWER_LEFT_RB 0x40
#define HB2_L_L_ACTIVE 0x40
/* Z = Z | HB2_L_L_ACTIVE - Activate H bridge 2 lower left */
/* Z = EVERYTHING_IS_INACTIVE | HB2_L_L_ACTIVE == 0x33 | 0x40 == (0x73)  */
#define HB2_L_L_INACTIVE 0xBF
/* Z = Z & HB2_L_L_INACTIVE - Deactivate H bridge 2 lower left */
/* Z = EVERYTHING_IS_INACTIVE & HB2_L_L_INACTIVE == 0x33 & 0xBF == (0x33)  */

#define H_BRIDGE_2_LOWER_RIGHT_YB 0x80
#define HB2_L_R_ACTIVE 0x80
/* Z = Z | HB2_L_R_ACTIVE - Activate H bridge 2 lower right */
/* Z = EVERYTHING_IS_INACTIVE | HB2_L_R_ACTIVE == 0x33 | 0x80 == (0xB3)  */
#define HB2_L_R_INACTIVE 0x7F
/* Z = Z & HB2_L_R_INACTIVE - Deactivate H bridge 2 lower right */
/* Z = EVERYTHING_IS_INACTIVE & HB2_L_R_INACTIVE == 0x33 & 0x7F == (0x33)  */

/* Coil 2 active in "positive" direction if HB2_U_L_ACTIVE and HB2_L_R_ACTIVE */
/* Z = (EVERYTHING_IS_INACTIVE & HB2_U_L_ACTIVE ) | HB2_L_R_ACTIVE == (0x33 & 0xEF) | 0x80 == 0x23 | 0x80 == 0xA3 */
#define COIL_2_ACTIVE_POSITIVE_DIRECTION 0xA0

/* Coil 2 active in "negative" direction if HB2_U_R_ACTIVE and HB2_L_L_ACTIVE */
/* Z = (EVERYTHING_IS_INACTIVE & HB2_U_R_ACTIVE ) | HB2_L_L_ACTIVE == (0x33 & 0xDF) | 0x40 == 0x13 | 0x40 == 0x53 */
#define COIL_2_ACTIVE_NEGATIVE_DIRECTION 0x50

/*--------------------------------------------*/
/* Full step sequence (clockwise direction):  */
/* 1. Coil 1 activate in "positive" direction, coil 2 deactivate */
#define COIL_1_ACTIVE_POSITIVE_DIRECTION_ONLY 0x3A
/* 2. Coil 2 activate in "positive" direction, coil 1 deactivate */
#define COIL_2_ACTIVE_POSITIVE_DIRECTION_ONLY 0xA3
/* 3. Coil 1 activate in "negative" direction, coil 2 deactivate */
#define COIL_1_ACTIVE_NEGATIVE_DIRECTION_ONLY 0x35
/* 4. Coil 2 activate in "negative" direction, coil 1 deactivate */
#define COIL_2_ACTIVE_NEGATIVE_DIRECTION_ONLY 0x53

/* Full step sequence (counterclockwise direction):  */
/* 1. Coil 2 activate in "negative" direction, coil 1 deactivate */
#define COIL_2_ACTIVE_NEGATIVE_DIRECTION_ONLY 0x53
/* 2. Coil 1 activate in "negative" direction, coil 2 deactivate */
#define COIL_1_ACTIVE_NEGATIVE_DIRECTION_ONLY 0x35
/* 3. Coil 2 activate in "positive" direction, coil 1 deactivate */
#define COIL_2_ACTIVE_POSITIVE_DIRECTION_ONLY 0xA3
/* 4. Coil 1 activate in "positive" direction, coil 2 deactivate */
#define COIL_1_ACTIVE_POSITIVE_DIRECTION_ONLY 0x3A

/*--------------------------------------------*/
/* Half step sequence (clockwise direction):  */
/* 1. Coil 1 activate in "positive" direction, coil 2 deactivate */
#define COIL_1_ACTIVE_POSITIVE_DIRECTION_ONLY 0x3A
/* 2. Coil 1 active in "positive" direction, */
/*    coil 2 activate in "positive" direction */
#define COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION 0xAA
/* 3. Coil 1 deactivate, coil 2 active in "positive" direction */
#define COIL_2_ACTIVE_POSITIVE_DIRECTION_ONLY 0xA3
/* 4. Coil 1 activate in "negative" direction, */
/*    Coil 2 active in "positive" direction */
#define COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION 0xA5
/* 5. Coil 1 active in "negative" direction, coil 2 deactivate */
#define COIL_1_ACTIVE_NEGATIVE_DIRECTION_ONLY 0x35
/* 6. Coil 1 active in "negative" direction */
/*    coil 2 activate in "negative" direction */
#define COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION 0x55
/* 7. Coil 1 deactivate, coil 2 active in "negative" direction */
#define COIL_2_ACTIVE_NEGATIVE_DIRECTION_ONLY 0x53
/* 8. Coil 1 activate in "positive" direction */
/*    coil 2 active in "negative" direction */
#define COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION 0x5A

/* Half step sequence (counterclockwise direction):  */
/* 1. Coil 1 deactivate, coil 2 activate in "negative" direction */
#define COIL_2_ACTIVE_NEGATIVE_DIRECTION_ONLY 0x53
/* 2. Coil 1 activate in "negative" direction */ 
/*    coil 2 active in "negative" direction */
#define COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION 0x55
/* 3. Coil 1 active in "negative" direction, coil 2 deactivate */
#define COIL_1_ACTIVE_NEGATIVE_DIRECTION_ONLY 0x35
/* 4. Coil 1 active in "negative" direction */
/*    coil 2 activate in "positive" direction */
#define COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION 0xA5
/* 5. Coil 1 deactivate, coil 2 active in "positive" direction */
#define COIL_2_ACTIVE_POSITIVE_DIRECTION_ONLY 0xA3
/* 6. Coil 1 activate in "positive" direction */
/*    coil 2 active in "positive" direction */
#define COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION 0xAA
/* 7. Coil 1 active in "positive" direction, coil 2 deactivate */
#define COIL_1_ACTIVE_POSITIVE_DIRECTION_ONLY 0x3A
/* 8. Coil 1 active in "positive" direction */
/*    coil 2 activate in "negative" direction */
#define COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION 0x5A

/*##############################################################*/
/* H-Bridge wire color code. (SMM - The color code is mine). */
/* The color code is presented in C1C2 pairs */
/* (for example, red-yellow - RY). */
/* The first color is the color of the wire, */
/* the second color is the color of the heat shrink tube. */ 
/* First color meanings: */
/* H-Bridge 1: */
/* White - left side */
/* Green - right side */
/* H-Bridge 2: */
/* Red - left side */
/* Yellow - right side */
/* Second color meanings (both H-Bridges): */
/* X-Red - connected to DC positive pole (upper half) */
/* X-Black - connected to DC negative pole (lower half) */
/* H-Bridge 1: */
/* WR: White-Red, GR: Green-Red, WB: White-Black, GB: Green-Black */
/* H-Bridge 2: */
/* RR: Red-Red, YR: Yellow-Red, RB: Red-Black, YB: Yellow-Black */

/* In each H-Bridge, current can flow either from */
/* the upper left side to the lower right side, or */
/* from the upper right side to the lower left side. */
/* It means active pairs could be: */
/* Coil 1 active in "positive" direction */
/* H_BRIDGE_1_UPPER_LEFT is 0 and H_BRIDGE_1_LOWER_RIGHT is 1 */
/* activates coil 1 into "positive" direction  */
/* H_BRIDGE_1_UPPER_RIGHT & H_BRIDGE_1_LOWER_LEFT */
/* activates coil 1 into "negative" direction  */
/* H_BRIDGE_2_UPPER_LEFT & H_BRIDGE_2_LOWER_RIGHT */
/* activates coil 2 into "positive" direction  */
/* H_BRIDGE_2_UPPER_RIGHT & H_BRIDGE_2_LOWER_LEFT */
/* activates coil 2 into "negative" direction  */

#define MAX_STEPS 1000
#define FULL_HALF_STEPS_MODE_BIT 0
#define FULL_HALF_STEPS_MODE_MASK 0x01
/* HALF_STEPS_MODE_MASK was added for convenience. It is */
/* same as FULL_HALF_STEPS_MODE_MASK */
#define HALF_STEPS_MODE_MASK 0x01
#define HALF_STEPS_MODE 1
#define FULL_STEPS_MODE 0
#define DIRECTION_BIT 1
#define DIRECTION_MASK 0x02
#define CLOCKWISE_DIRECTION 1
#define COUNTERCLOCKWISE_DIRECTION 0
#define EEPROM_ADDRESS_DIRECTION_FLAG 0x0
#define INIT_WORKING_MODE_BIT 2
#define INIT_WORKING_MODE_MASK 0x04
#define INIT_MODE 1
#define WORKING_MODE 0
#define PHOTOINTERRUPTER1_MASK (1<<PIND2)
#define PHOTOINTERRUPTER1_ACTIVE (0<<PIND2)
#define PHOTOINTERRUPTER2_MASK (1<<PIND3)
#define PHOTOINTERRUPTER2_ACTIVE (0<<PIND3)


/* Attiny-2313 macros/defines for communication */
/* through Two-wires USI using TWI (AKA I2C) protocol */
#define DDR_USI DDRB
#define PORT_USI PORTB
#define PIN_USI PINB
/*#define PUSI_SDA PB5*/
/*#define PUSI_SCL PB7 */
#define PINUSI_SDA PINB5
#define PINUSI_SCL PINB7
#define PINUSI_SDA_MASK (0x01<<PINUSI_SDA)
#define PINUSI_SCL_MASK (0x01<<PINUSI_SCL)
#define PINUSI_MASK ((0x01<<PINUSI_SDA)|(0x01<<PINUSI_SCL))
#define PORT_USI_SDA PB5
#define PORT_USI_SCL PB7
#define PORT_USI_SCL_MASK (0x01<<PORT_USI_SCL)
#define PORT_USI_SDA_MASK (0x01<<PORT_USI_SDA)
#define PORT_USI_MASK (PORT_USI_SDA_MASK|PORT_USI_SCL_MASK)
/*#define PIN_USI_SDA PINB5*/
/*#define PIN_USI_SCL PINB7*/
#define USI_START_COND_INT USISIF
/*#define USI_START_VECTOR USI_START_vect*/
/*#define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect*/
typedef enum
{
	USI_SLAVE_CHECK_ADDRESS                = 0x00,
	USI_SLAVE_SEND_DATA                    = 0x01,
	USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA = 0x02,
	USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA   = 0x03,
	USI_SLAVE_REQUEST_DATA                 = 0x04,
	USI_SLAVE_GET_DATA_AND_SEND_ACK        = 0x05
} overflow_state_t;


#define SET_USI_TO_SEND_ACK( ) \
{ \
	/* prepare ACK */ \
	USIDR = 0; \
	/* set SDA as output */ \
	DDR_USI |= ( 1 << PORT_USI_SDA ); \
	/* clear all interrupt flags, except Start Cond */ \
	USISR = \
	( 0 << USISIF ) | \
	( 1 << USIOIF ) | ( 1 << USIPF ) | \
	( 1 << USIDC )| \
	/* set USI counter to shift 1 bit */ \
	( 0x0E << USICNT0 ); \
}

#define SET_USI_TO_READ_ACK()  \
{  \
	DDR_USI &=  ~(1<<PORT_USI_SDA);  /* Set SDA as intput */ \
	USIDR    =  0;                   /* Prepare ACK       */ \
	/* Clear all flags, except Start Cond  */ \
	USISR    =  (0<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|\
	(0x0E<<USICNT0);    /* set USI counter to shift 1 bit. */ \
}

#define SET_USI_TO_TWI_START_CONDITION_MODE()  \
{ \
	/* Enable Start Condition Interrupt. Disable Overflow Interrupt.*/  \
	USICR    =  (1<<USISIE)|(0<<USIOIE)| \
	/* Set USI in Two-wire mode. No USI Counter overflow hold.      */  \
	(1<<USIWM1)|(0<<USIWM0)| \
	/* Shift Register Clock Source = External, positive edge        */  \
	(1<<USICS1)|(0<<USICS0)|(0<<USICLK)| \
	(0<<USITC);                          \
	/* Clear all flags, except Start Cond                            */ \
	USISR    =  (0<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)| \
	(0x0<<USICNT0);  \
}

#define SET_USI_TO_SEND_DATA()  \
{ \
	DDR_USI |=  (1<<PORT_USI_SDA);   /* Set SDA as output */ \
	/* Clear all flags, except Start Cond */ \
	USISR    =  (0<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)| \
	/* set USI to shift out 8 bits        */ \
	(0x0<<USICNT0); \
}

#define SET_USI_TO_READ_DATA()  \
{  \
	DDR_USI &= ~(1<<PORT_USI_SDA);  /* Set SDA as input */ \
	/* Clear all flags, except Start Cond */ \
	USISR    =  (0<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)| \
	/* set USI to shift out 8 bits        */ \
	(0x0<<USICNT0); \
}


/* This is speed in msecs per timer ticks  */
/* Initially set it 100 msecs */
volatile uint16_t stepping_speed=100;
volatile uint16_t steps_remaining=0;
volatile uint8_t mode=0x0;
volatile uint8_t direction_to_initial_position=CLOCKWISE_DIRECTION;
/*uint16_t timer_divider_value=390;*/


/* Attiny-2313 variables for communication */
/* through Two-wires USI using TWI (AKA I2C) protocol */
uint8_t slave_addr;
volatile overflow_state_t overflow_state;

uint8_t rx_buf[TWI_RX_BUFFER_SIZE];
volatile uint8_t rx_head;
volatile uint8_t rx_tail;

uint8_t tx_buf[TWI_TX_BUFFER_SIZE];
volatile uint8_t tx_head;
volatile uint8_t tx_tail;

/* data requested callback  */
void (*_onTwiDataRequest)(void);


/* Attiny-2313 functions for communication */
/* through Two-wires USI using TWI (AKA I2C) protocol */

// flushes the TWI buffers

/***************************/
/* clear_twi_buffers(void) */
/*************************************/
/* This function clears twi buffers  */
/*************************************/
/* No arguments                      */
/* Returns nothgin                   */
/*************************************/
void clear_twi_buffers(void)
{
  rx_tail=0;
  rx_head=0;
  tx_tail=0;
  tx_head=0;
} 

/********************************************/
/* usi_twi_slave_init(uint8_t twi_address)  */
/********************************************/
/* initialize USI for TWI/I2C slave mode    */
/**********************************************************/
/*  uint8_t twi_address - this is TWI/I2C slave address   */
/*                        this unit address. Normally the */
/*                        TWI/I2C slave address should be */
/*                        acquired from EEPROM memory     */ 
/* returns nothing                                        */
/**********************************************************/
void usi_twi_slave_init(uint8_t twi_address)
{
  clear_twi_buffers();

  slave_addr=twi_address;

/* In Two Wire mode (USIWM1, USIWM0 = 1X), the slave USI will pull SCL */
/* low when a start condition is detected or a counter overflow (only */
/* for USIWM1, USIWM0 = 11). This inserts a wait state. SCL is released */
/* by the ISRs (USI_START_vect and USI_OVERFLOW_vect).  */

/* Set SCL and SDA as output */
  DDR_USI|=(1<<PORT_USI_SCL)|(1<<PORT_USI_SDA);

/* set SCL high and set SDA high */
  PORT_USI|=(0x01<<PORT_USI_SCL)|(1<<PORT_USI_SDA);

/* Set SDA as input */
  DDR_USI&=~(0x01<< PORT_USI_SDA);

/* enable Start Condition Interrupt -  1 << USISIE */
/* disable Overflow Interrupt - 0 << USIOIE */
/* set USI in Two-wire mode, no USI Counter overflow hold - */
/* 1 << USIWM1 and 0 << USIWM0 */
/* Shift Register Clock Source = external, positive edge */
/* 4-Bit Counter Source = external, both edges - */
/* 1 << USICS1 and 0 << USICS0 and 0 << USICLK */
/* no toggle clock-port pin - 0 << USITC */
  USICR=(1<<USISIE) | (0<<USIOIE) | (1<<USIWM1)|(0<<USIWM0) |
	    (1<<USICS1)|(0<<USICS0)|(0<<USICLK) | (0<<USITC);

/*  clear all interrupt flags and reset overflow counter  */
  USISR=(1<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC);
/*  USISR=(1<<USI_START_COND_INT)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC);  */
} // end usiTwiSlaveInit


/***********************************/
/* if_tx_buffer_is_not_empty(void) */
/******************************************/ 
/* Has no input parameters                */
/* Returns true if tx buffer is not empty */
/*         false if tx buffer is empty    */
/*******************************************/
bool if_tx_buffer_is_not_empty(void)
{
/* return 0 (false) if the tx buffer is empty  */
  return tx_head != tx_tail;
} 

/*  put data in the transmission buffer, wait if buffer is full  */
void usi_twi_transmit_byte(uint8_t data_byte)
{
  uint8_t tmphead;

/* calculate buffer index  */
  tmphead=(tx_head+1)&TWI_TX_BUFFER_MASK;

/*  wait for free space in buffer  */
  while(tmphead==tx_tail);

/* store data in buffer */
  tx_buf[tmphead]=data_byte;

/* store new index */
  tx_head = tmphead;
} 

/***********************************/
/* if_rx_buffer_is_not_empty(void) */
/******************************************/
/* Has no input parameters                */
/* Returns true if rx buffer is not empty */
/*         false if rx buffer is empty    */
/******************************************/
bool if_rx_buffer_is_not_empty(void)
{
/* return 0 (false) if the rx buffer is empty  */
  return rx_head != rx_tail;
} 

// return a byte from the receive buffer, wait if buffer is empty
uint8_t usi_twi_receive_byte(void)
{
/* wait for Rx data */
  while(rx_head==rx_tail);

/* calculate buffer index */
 rx_tail=(rx_tail+1)&TWI_RX_BUFFER_MASK;

/* return data from the buffer. */
  return rx_buf[rx_tail];
}



/***********************/
/* ISR(USI_START_vect) */
/**********************************************/
/* This function handles interrupt requests   */
/* upon start condition                       */
/**********************************************/
ISR(USI_START_vect)
{
/* set default starting conditions for new TWI package */
  overflow_state=USI_SLAVE_CHECK_ADDRESS;

/* set SDA as input */
  DDR_USI&=~(0x01<<PORT_USI_SDA);

/* wait for SCL to go low to ensure the Start Condition has completed */
/* (the start detector will hold SCL low ) - if a Stop Condition arises */
/* then leave the interrupt to prevent waiting forever - don't use USISR */
/* to test for Stop Condition as in Application Note AVR312 because the */
/* Stop Condition Flag is going to be set from the last TWI sequence */
/* Wait while SCL his high and SDA is low */
  while((PIN_USI&(0x01<<PINUSI_SCL))&&!((PIN_USI&(0x01<<PINUSI_SDA))))
  {};

/* a Stop Condition did not occur  */
  if(!(PIN_USI&(0x01<<PINUSI_SDA)))
  {
/* keep Start Condition Interrupt enabled to detect RESTART */
/* enable Overflow Interrupt */
/* set USI in Two-wire mode, hold SCL low on USI Counter overflow */
/* Shift Register Clock Source = External, positive edge */
/* 4-Bit Counter Source = external, both edges */
/* no toggle clock-port pin */
    USICR=(0x01<<USISIE)|(0x01<<USIOIE)|
          (0x01<<USIWM1)|(0x01<<USIWM0)|
          (0x01<<USICS1)|(0x0<<USICS0)|(0x0<<USICLK)|
          (0x0<<USITC);
  }
  else  /* if(!(PIN_USI&(0x01<<PINUSI_SDA))) */
  {
/* a Stop Condition did occur */
/* enable Start Condition Interrupt */
/* disable Overflow Interrupt */
/* set USI in Two-wire mode, no USI Counter overflow hold */
/* Shift Register Clock Source = external, positive edge */
/* 4-Bit Counter Source = external, both edges */
/* no toggle clock-port pin */
    USICR=(0x01<<USISIE)|(0x0<<USIOIE)|
          (0x01<<USIWM1)|(0x0<<USIWM0)|
          (0x01<<USICS1)|(0x0<<USICS0)|(0x0<<USICLK)|
          (0x0<<USITC);
  } /* if(!(PIN_USI&(0x01<<PINUSI_SDA))) */

/* clear interrupt flags - resetting the Start Condition Flag will */
/* release SCL */
/* set USI to sample 8 bits (count 16 external SCL pin toggles) */
/*  USISR=(0x01<<USI_START_COND_INT)|(0x01<<USIOIF)|(0x01<<USIPF)|(0x01<<USIDC)| */
  USISR=(0x01<<USISIF)|(0x01<<USIOIF)|(0x01<<USIPF)|(0x01<<USIDC)|
        (0x0<<USICNT0);
} 

/**************************/
/* ISR(USI_OVERFLOW_vect) */
/***********************************************/
/* This function handles interrupt requests    */
/* upon overflow condition. It handles all the */
/* communication.                              */
/* Only disabled when waiting for a new Start  */
/* condition                                   */
/***********************************************/
ISR(USI_OVERFLOW_vect)
{
  switch(overflow_state)
  {
/* Address mode: check address and send ACK (and next USI_SLAVE_SEND_DATA) if OK, */
/* else reset USI */
    case USI_SLAVE_CHECK_ADDRESS:
    if((USIDR==0x0)||((USIDR>>1)==slave_addr))
    {
/* callback */
      if(_onTwiDataRequest) 
        _onTwiDataRequest();
      if(USIDR&0x01)
      {  overflow_state=USI_SLAVE_SEND_DATA;  }
      else  {  overflow_state=USI_SLAVE_REQUEST_DATA;  } 
      SET_USI_TO_SEND_ACK();
    }
    else  {  SET_USI_TO_TWI_START_CONDITION_MODE();  }
    break;

/* Master write data mode: check reply and goto USI_SLAVE_SEND_DATA if OK, */
/* else reset USI */
    case USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA:
      if(USIDR!=0x0)
      {
/* if NACK, the master does not want more data */
        SET_USI_TO_TWI_START_CONDITION_MODE();
        return;
      }
/* from here we just drop straight into USI_SLAVE_SEND_DATA if the */
/* master sent an ACK  */

/* copy data from buffer to USIDR and set USI to shift byte */
/* next USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA  */
    case USI_SLAVE_SEND_DATA:
/* Get data from Buffer */
      if(tx_head!=tx_tail)
      {
        tx_tail=(tx_tail+1)&TWI_TX_BUFFER_MASK;
        USIDR=tx_buf[tx_tail];
      }
      else
      {
/* the buffer is empty */
        SET_USI_TO_TWI_START_CONDITION_MODE( );
        return;
      } 
      overflow_state=USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA;
      SET_USI_TO_SEND_DATA( );
      break;

/* set USI to sample reply from master  */
/* next USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA  */
    case USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA:
      overflow_state=USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA;
      SET_USI_TO_READ_ACK( );
      break;

/* Master read data mode: set USI to sample data from master  */
/* next USI_SLAVE_GET_DATA_AND_SEND_ACK  */
    case USI_SLAVE_REQUEST_DATA:
      overflow_state=USI_SLAVE_GET_DATA_AND_SEND_ACK;
      SET_USI_TO_READ_DATA( );
      break;

/* copy data from USIDR and send ACK */
/* next USI_SLAVE_REQUEST_DATA */
    case USI_SLAVE_GET_DATA_AND_SEND_ACK:
/* put data into buffer */
/* Not necessary, but prevents warnings */
      rx_head=(rx_head+1)&TWI_RX_BUFFER_MASK;
      rx_buf[rx_head]=USIDR;
/* next USI_SLAVE_REQUEST_DATA */
      overflow_state=USI_SLAVE_REQUEST_DATA;
      SET_USI_TO_SEND_ACK( );
      break;
  }  /* switch(overflow_state)  */
} 







/*****************************************************************************/
/* set_direction_to_initial_position(uint8_t direction_to_initial_position_) */
/*****************************************************************************/
/* Not sure if this function is needed.  */
/*****************************************/
inline void set_direction_to_initial_position(uint8_t direction_to_initial_position_)
{
  direction_to_initial_position=direction_to_initial_position_;
}

/*************************************************/
/* set_stepping_speed(uint16_t stepping_speed_)  */
/**********************************************************/
/* This function sets second timer divider (for TIMER1)   */             
/* registers assuming first timer divider is 1024 (MAX)   */
/*--------------------------------------------------------*/
/* uint16_t stepping_speed_ - is stepping speed in msecs  */
/*                            between ticks               */
/**********************************************************/ 
void set_stepping_speed(uint16_t msecs_between_ticks)
{
/* It appears that Timer1 rate = 8,000,000/1024 ~ 7813.  */
/* (1024 is "first" divider selected by TCCR1B register: */
/* bits CS12, CS11 and CS10 - 101) */
/* It means that it should tick ~7813 times per second */
/* If the desired time between ticks is x seconds, */
/* then we need to count ~(7813*x) ticks and trigger an */
/* interrupt. This value defines the “second” divider */
/* and is stored in the OCR1A register. */
/* Function has an integer parameter in msecs so it should */
/* be divided by 1000 internally. */
/* So when timer_divider_value_ will be set in the OCR1A register */
/* than Timer1 will count exactly this number of ticks with specified */
/* (in TCCT1B) rate. Then Timer1 will stop counting and it will */
/* request an interrupt after approximately msecs_between_ticks msecs. */
  uint16_t timer_divider_value_ = (uint16_t)(((uint32_t)7813*msecs_between_ticks)/1000);
  cli();
  OCR1A=timer_divider_value_;
/* This is same as above, just split into two explicit steps
  OCR1AH=(uint8_t)((timer_divider_value_>>8)&0x00ff);
  OCR1AL=(uint8_t)(timer_divider_value_&0x00ff);
*/
  stepping_speed=msecs_between_ticks;
  sei();
}

/*************************/
/* stop_movements(void)  */
/**********************************************************/
/* Function simply clears counter of the remaining steps  */
/* and clears TIMER interrupt mask                        */
/**********************************************************/
void stop_movements(void)
{
  cli();
  steps_remaining=0;
  TIMSK=0x00; /* Interrupts mask */
              /* Clear all bits - no timer interrupts needed.  */
  sei();
}

/*************************************************/
/* set_number_of_steps(uint8_t number_of_steps_) */
/*************************************************/
/* Function sets number of steps to be done  */
/* It should stop all previous movements     */
/* set counter of the remaining steps        */
/* and enable timer TIMER1 interrupts        */ 
/*********************************************/
void set_number_of_steps(uint16_t number_of_steps_)
{
/* Starting new sequence of steps, so stop/cancel all previous */
/* movements */
  stop_movements();

  cli();
  steps_remaining=number_of_steps_;
/* Enable timer interrupts  */
  TIMSK = (1 << OCIE1A);  /* Interrupt Mask  */
                          /* OICE1A bit is set to 1 means Timer/Counter1 */
                          /* Output Compare A Match Interrupt Enable */
  sei();
}

/*******************************************/
/* set_direction_mode(uint8_t direction_)  */
/*******************************************/
void set_direction_mode(uint8_t direction_)
{
/* First if some movement is not finished - stop any movements  */
  stop_movements();
  cli();
/* Next - clear direction bit in the mode variable */
  mode&=~DIRECTION_MASK;
/* If requested direction is not counterclockwise  */
/* set the proper value into the direction bit  */
  if(direction_!=COUNTERCLOCKWISE_DIRECTION)
  {  mode|=CLOCKWISE_DIRECTION<<DIRECTION_BIT;  }
  else { mode|=COUNTERCLOCKWISE_DIRECTION<<DIRECTION_BIT;  }
  sei();
}

/**********************************************************/
/* set_full_half_step_mode(uint8_t full_half_step_mode_)  */
/**********************************************************/
void set_full_half_step_mode(uint8_t full_half_step_mode_)
{
/* First if some movement is not finished - stop any movements  */
  stop_movements();
  cli();
/* Next - clear half/full-step mode bit in the mode variable */
  mode&=~FULL_HALF_STEPS_MODE_MASK;
/* If requested mode is not half-step mode  */
/* set the proper value into the half/full-step mode bit  */
  if(full_half_step_mode_!=HALF_STEPS_MODE)
  {  mode|=FULL_STEPS_MODE<<FULL_HALF_STEPS_MODE_BIT;  }
  else { mode|=HALF_STEPS_MODE<<FULL_HALF_STEPS_MODE_BIT;  }
  sei();
}

/*************************************/
/* read_eeprom_byte(uint8_t address) */
/********************************************************************/
/* This function reads a byte from EEPROM memory                    */
/* (taken from the datasheet)                                       */
/* uint8_t address - is address of requested byte in EEPROM memory  */
/* returns requested EEPROM byte as uint8_t                         */
/********************************************************************/
uint8_t read_eeprom_byte(uint8_t address_)
{
/* Wait for completion of previous write */
  while(EECR & (1<<EEPE))
  {};
/* Set up address register */
  EEAR = address_;
/* Start EEPROM read by writing EERE */
  EECR |= (1<<EERE);
/* Return data from data register */
  return EEDR;
}

/*****************************************************/
/* write_eeprom_byte(uint8_t address, uint8_t value) */
/********************************************************************/
/* This function writes a byte to EEPROM memory                     */
/* (taken from the datasheet)                                       */
/* uint8_t address - is address of the byte in EEPROM memory        */
/* uint8_t value - is a value should be written to EEPROM memory    */
/* returns nothing                                                  */
/********************************************************************/
void write_eeprom_byte(uint8_t address_, uint8_t value)
{
/* Wait for completion of previous write */
  while(EECR & (1<<EEPE))
  {};
/* Set up address and data registers */
  EEAR = address_;
  EEDR = value;
/* Write logical one to EEMPE */
  EECR |= (1<<EEMPE);
/* Start EEPROM write by setting EEPE */
  EECR |= (1<<EEPE);
}

/**************************/
/* ISR(TIMER1_COMPA_vect) */
/**********************************************/
/* This function handles interrupt requests   */
/* from timer TIMER1                          */
/**********************************************/
/* Timer TIMER1 assigned to serve for the     */
/* movements only. Timer TIMER1 will be       */
/* deactivated if no movement assumed.        */
/* This handler should check current state of */
/* all coils, direction of the movement in    */
/* the mode variable and half/full step mode  */
/* in the same mode variable                  */
/* It will set new state for both coils       */
/* (PORTB value) and reduce counter of        */
/* remaining steps.                           */
/* If remaining steps are 0 - do nothing and  */
/* switch off timer TIMER1 interrupts.        */ 
/**********************************************/
ISR(TIMER1_COMPA_vect)
{
#if 1
  uint8_t pind_=PIND;

  if((pind_&PHOTOINTERRUPTER1_MASK)==PHOTOINTERRUPTER1_ACTIVE)
  {
/* Photointerrupter1 is active, which means the table */
/* is in the initial position. */
/* Movement towards the initial position is not possible. */
/* Movement towards the final position is possible.*/

/* -- Regardless of the value of the initialization flag, */
/*    the movement direction flag is set in the opposite */
/*    direction to direction_to_initial_position.  */
    /*mode&=~DIRECTION_MASK;  *//* Clear Direction bit  */
/*
    uint8_t direction_opposite_to_initial_position=
      (direction_to_initial_position==CLOCKWISE_DIRECTION) ?
    COUNTERCLOCKWISE_DIRECTION : CLOCKWISE_DIRECTION;
    mode|=(direction_opposite_to_initial_position<<DIRECTION_BIT);
*/

    if((mode&=INIT_WORKING_MODE_MASK)==
        (INIT_MODE<<INIT_WORKING_MODE_BIT))
    {
/* Photointerrupter1 is active and the initialization flag */
/* is set to INIT_MODE, it means the table was in the initial */
/* position, and either voltage was applied or a soft/hard reset */
/* occurred. Also, the INT0 interrupt handler, which is executed */
/* when the state of photointerrupter1 changes, was not executed. */
/* In all other cases, the INIT_MODE flag will either change while */
/* processing the current situation or while processing the */
/* INT0 interrupt (caused by activating Photointerrupter1). */
/* In this case: */
/* -- Since this is an initialization process the */
/*    direction_to_initial_position variable must be read from */
/*    EEPROM memory. And the movement direction flag is set in the */
/*    opposite direction to direction_to_initial_position.  */
      direction_to_initial_position=read_eeprom_byte(EEPROM_ADDRESS_DIRECTION_FLAG);
/* set_direction_mode() function could not be used here, while it has  */
/* cli()-sei() pair inside. Those are wrong inside ISR() function.  */
      mode&=~DIRECTION_MASK;  /* Clear Direction bit  */
/* Set direction mode into opposite to initial position */
      uint8_t direction_to_final_position=
                (direction_to_initial_position==CLOCKWISE_DIRECTION) ?
                  COUNTERCLOCKWISE_DIRECTION : CLOCKWISE_DIRECTION;
      mode|=(direction_to_final_position<<DIRECTION_BIT);
/* -- The initialization flag is changed from INIT_MODE to */
/* WORKING_MODE, */
      mode&=~INIT_WORKING_MODE_MASK;  /* Clear Initialization flag bit  */
      mode|=(WORKING_MODE<<INIT_WORKING_MODE_BIT);
/* -- steps_remaining is set to 0. */
	  steps_remaining=0;
/* Clear all bits - no timer interrupts needed.  */
      TIMSK=0x00; /* Interrupts mask */
/* No need to do anything else here - just exit from the ISR  */
      return;
    }
/* Photointerrupter1 is active and the initialization flag */
/* is set to WORKING_MODE */
/* This means that initialization has occurred and movement */
/* towards the final position is taking place. */
/* In this case, no other flags are changed. */
/* And the planned step continues. */
  }

  if((pind_&PHOTOINTERRUPTER2_MASK)==PHOTOINTERRUPTER2_ACTIVE)
  {
/* Photointerrupter2 is active, which means the table */
/* is in the final position. */
/* Movement towards the final position is not possible. */
/* Movement towards the initial position is possible.*/

    if((mode&=INIT_WORKING_MODE_MASK)==
       (INIT_MODE<<INIT_WORKING_MODE_BIT))
    {
/* Photointerrupter2 is active and the initialization flag */
/* is set to INIT_MODE, this means either the table was in */
/* the final position, and either voltage was applied, or a */
/* soft/hard reset occurred. In this case, the INT1 interrupt */
/* handler, which is triggered when the photointerrupter2 */
/* state changes, was not executed. Or the table entered the */
/* final position during initialization. In this case, the INT1 */
/* interrupt handler, which is triggered when the photointerrupter2 */
/* state changes, was executed. In all these cases: */  
/* -- Since this is an initialization process, the variable */
/*    direction_to_initial_position must be read from EEPROM memory. */
/*    And the movement flag must be set in the direction of */
/*    direction_to_initial_position.  */
/* -- The steps_remaining variable is set to MAX_STEPS. */
/* -- Other variables are not changed. Continue executing steps. */
      direction_to_initial_position=read_eeprom_byte(EEPROM_ADDRESS_DIRECTION_FLAG);
/* set_direction_mode() function could not be used here, while it has  */
/* cli()-sei() pair inside. Those are wrong inside ISR() function.  */
      mode&=~DIRECTION_MASK;  /* Clear Direction bit  */
      mode|=(direction_to_initial_position<<DIRECTION_BIT);
      steps_remaining=MAX_STEPS;
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/* Warning! During initialization, the variable */
/* direction_to_initial_position may be set to a value that does */
/* not correspond to the current configuration of the equipment */
/* connections. For example, this is possible when changing the */
/* poles of the H-bridge outputs. In this case, it was assumed */
/* that the table was moving in the direction of the initial */
/* position, but in reality the table was moving in the direction */
/* of the final position. Here it is necessary to change the variable */
/* direction_to_initial_position and DIRECTION mode to the opposite. */
/* If the INT1 interrupt handler is triggered, this situation is */
/* automatically corrected, with a simultaneous correction of the */
/* value in the EEPROM memory. If the table was in the final position, */
/* and voltage was applied, then... it is necessary to develop a */
/* method for detecting this error!!! TBDL !!! */
    }
/* If the initialization flag is set to WORKING_MODE, then the INT1 */
/* interrupt handler has already run and set all the necessary */
/* parameters, so there is no need to change any variables. */
  }

/* If number of remaining steps is not 0 - */
/* perform the step according to the algorithm */
/* Number of remaining steps could become 0 if  */
/* current state could not be recognized */
  if(steps_remaining>0)
  {
/* While Two-wired USI occupies two pins on PORTB: */
/* PINB7 (SCL) and PINB5 (SDA) */
/* those pins should be reassigned on PORTD */
/* PIND1 and PIND0 where selected for substitute */
/* PINB7 (SCL) and PINB5 (SDA) respectively */
/* (d1)b(d0)bb bbbb - PORTB and ddd dd(b7)(b5) - PORTD  */
    uint8_t portb_value=PORTB;
    uint8_t portd_value=PORTD;
    uint8_t port_value=(portb_value&(~PORT_USI_MASK))+
                       ((portd_value&0x02)<<(PORT_USI_SCL-1))+
                       ((portd_value&0x01)<<PORT_USI_SDA);
    uint8_t port_var=0xFF;
    uint8_t port_value_array[8] =
    {
        COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION,
        COIL_1_ACTIVE_POSITIVE_DIRECTION_ONLY,
        COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION,
        COIL_2_ACTIVE_POSITIVE_DIRECTION_ONLY,
        COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION,
        COIL_1_ACTIVE_NEGATIVE_DIRECTION_ONLY,
        COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION,
        COIL_2_ACTIVE_NEGATIVE_DIRECTION_ONLY
    };

    if(port_value==EVERYTHING_IS_INACTIVE)
    {  port_var=0;  }
    else
	{
      for(int8_t t=0; t<8; t++)
      {
        if(port_value==port_value_array[t])
        {
          port_var=t;
          break;
        }
      }
    }

	if(port_var>=8)
    {
/* This is "impossible state" for coils.  */
/* Reset port_value and do not do any more steps. */
      port_var=0xFF;
      port_value=EVERYTHING_IS_INACTIVE;
      steps_remaining=0;
    }

#if 0
    switch(port_value)
    {
      case EVERYTHING_IS_INACTIVE:
      case COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION:
        port_var=0;
        break;
      case COIL_1_ACTIVE_POSITIVE_DIRECTION_ONLY:
        port_var=1;
        break;
      case COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION:
        port_var=2;
        break;
      case COIL_2_ACTIVE_POSITIVE_DIRECTION_ONLY:
        port_var=3;
        break;
      case COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION:
        port_var=4;
        break;
      case COIL_1_ACTIVE_NEGATIVE_DIRECTION_ONLY:
        port_var=5;
        break;
      case COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION:
        port_var=6;
        break;
      case COIL_2_ACTIVE_NEGATIVE_DIRECTION_ONLY:
        port_var=7;
        break;
      default:
/* This is "impossible state" for coils.  */
/* Reset port_value and do not do any more steps. */
        port_var=0xFF;
        port_value=EVERYTHING_IS_INACTIVE;
        steps_remaining=0;
        break;
    }  /* switch(port_value)  */
#endif /* #if 0 */

/* Again: if number of remaining steps is not 0 - */
/* perform the step according to the algorithm */
/* Number of remaining steps could become 0 if  */
/* current state could not be recognized (see above) */
	if(steps_remaining>0)
	{
/* By default, the movement is assumed to be clockwise in half-step mode. */
      int8_t step_=1;

/* Assume CLOCKWISE_DIRECTION is "positive" direction: */
/* Moving clockwise will increment the value of the port_var variable. */
/* COUNTERCLOCKWISE_DIRECTION is "negative" direction */
/* Moving counterclockwise will decrement the value of the */
/* port_var variable. */
      if(((mode&DIRECTION_MASK)>>DIRECTION_BIT)==COUNTERCLOCKWISE_DIRECTION)
        step_*=-1;

/* Half-step mode movement passes each half-step position. */
/* Full-step mode movement skips exactly one half-step position. */
      if(((mode&HALF_STEPS_MODE_MASK)>>FULL_HALF_STEPS_MODE_BIT)==FULL_STEPS_MODE)
        step_*=2;

      port_var=(uint8_t)(port_var+step_);
/* port_var variable can have only 8 possible values */
/* (from 0 to 7 inclusive), representing 8 half-step positions. */
      port_var=(port_var&0x07);
	}  /* if(steps_remaining>0) */

# if 1
    if(port_var<8)
    {  port_value=port_value_array[port_var];  }
    else
    {
/* This is "impossible state" for coils.  */
/* Reset port_value and do not do any more steps. */
      port_value=EVERYTHING_IS_INACTIVE;
      steps_remaining=0;
    }
#endif /* #if 0 */

#if 0
    switch(port_var)
    {
      case 0:
        port_value=COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION;
        break;
      case 1:
        port_value=COIL_1_ACTIVE_POSITIVE_DIRECTION_ONLY;
        break;
      case 2:
        port_value=COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION;
        break;
      case 3:
        port_value=COIL_2_ACTIVE_POSITIVE_DIRECTION_ONLY;
        break;
      case 4:
        port_value=COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION;
        break;
      case 5:
        port_value=COIL_1_ACTIVE_NEGATIVE_DIRECTION_ONLY;
        break;
      case 6:
        port_value=COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION;
        break;
      case 7:
        port_value=COIL_2_ACTIVE_NEGATIVE_DIRECTION_ONLY;
        break;
      default:
/* This is "impossible state" for coils.  */
/* Reset port_value and do not do any more steps. */
        port_value=EVERYTHING_IS_INACTIVE;
        steps_remaining=0;
        break;
    }  /* switch(port_var) */
#endif /* #if 0 */

/* While Two-wired USI occupies two pins on PORTB: */
/* PINB7 (SCL) and PINB5 (SDA) */
/* those pins should be reassigned on PORTD */
/* PIND1 and PIND0 where selected for substitute */
/*  PINB7 (SCL) and PINB5 (SDA) respectively */
/*  (d1)b(d0)bb bbbb - PORTB and ddd dd(b7)(b5) - PORTD  */
    portb_value=(portb_value&PORT_USI_MASK)+(port_value&(~(PORT_USI_MASK)));
	portd_value=((port_value&PORT_USI_SCL_MASK)>>(PORT_USI_SCL-1))+
                ((port_value&PORT_USI_SDA_MASK)>>PORT_USI_SDA); 
    PORTB=portb_value;
	PORTD=portd_value;
    if((steps_remaining>0)&&
       ((pind_&PHOTOINTERRUPTER1_MASK)!=PHOTOINTERRUPTER1_ACTIVE)&&
       ((pind_&PHOTOINTERRUPTER2_MASK)!=PHOTOINTERRUPTER2_ACTIVE))
      steps_remaining--;
  }  /* if(steps_remaining>0) */

/* if number of remaining steps became 0 */
/* clear all timer interrupts - no timer */
/* interrupts needed */
  if(steps_remaining<=0)
  {
    TIMSK=0x00; /* Interrupts mask */
                /* Clear all bits - no timer interrupts */
                /* needed.  */
  }  /* if(steps_remaining>0) */

#endif  /* #if 0 */
}

/*******************/
/* ISR(INT0_vect)  */
/*******************************************************/
/* This is an interrupt handler.                       */
/* It handles external interrupt requests on pin       */
/* INT0/PIND2                                          */
/* It assumed any level change on the INT0/PIND2 pin   */
/* triggers this interrupt request.                    */
/*-----------------------------------------------------*/
/* INTO pin must be assigned to the photointerrupter1  */
/* residing on "Initial position".                     */
/* It is assumed that a low level on the INT0/PIND2    */
/* pin (value equal to 0) means that photointerrupter1 */
/* is enabled. The table is in its initial position.   */
/* High level on the INT0/PIND2 pin (value equal to 1) */
/* means that photointerrupter1 is disabled. The table */
/* is one full-/half-step away from the initial        */
/* position.                                           */
/*******************************************************/
ISR(INT0_vect)  
{
/* In both cases, when the table is at the initial */
/* position (INT0/PIND2 is 0) or one full-/half-step from */
/* the initial position (INT0/PIND2 is 1):  */
/*   1. Initialization mode is off  */
/*   2. Movement movement from the initial position */
/*      is possible only towards the final position. */
/*   3. No need to change full-/half-step mode */
  mode&=~INIT_WORKING_MODE_MASK;  /* Clear Initialization flag bit  */
/* Set Initialization flag into Working mode  */
  mode|=(WORKING_MODE<<INIT_WORKING_MODE_BIT);  
                                                 
  mode&=~DIRECTION_MASK;  /* Clear Direction bit  */
/* Set direction mode into opposite to initial position */
  uint8_t direction_to_final_position=
    (direction_to_initial_position==CLOCKWISE_DIRECTION) ?
      COUNTERCLOCKWISE_DIRECTION : CLOCKWISE_DIRECTION;
  mode|=(direction_to_final_position<<DIRECTION_BIT);

/* If INT0 is 0 - the table just reached initial position  */
/* and photointerrupter1 became active  */
/*   if it was initialization mode - it should cease */
/*   initialization movements.  */
/*   if it was not initialization mode - it should cease  */
/*   any movements anyway.  */
/*   --Set steps_remaining into 0  */
/*   (--should timer be off too??, set mask to 0??)  */
/*   (--make all coils inactive?? and waiting for  */
/*   the next move??)  */
  uint8_t pind_=PIND;

/*??????????????????????????????????????*/
/*??  if((PORTD&(1<<PIND2))!=0)??*/
/*    if((pind_&(1<<PIND2))==0) */
/*    if((PIND & 0x04) == 0 )*/
  if((pind_&PHOTOINTERRUPTER1_MASK)==PHOTOINTERRUPTER1_ACTIVE)
  {
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*!!! Not sure about reseting coils in PORTB  !!!*/
/*      uint8_t portb_value = EVERYTHING_IS_INACTIVE;  */
/*      PORTB=portb_value;  */
    steps_remaining=0;
    TIMSK=0x00; /* Interrupts mask */
                /* Clear all bits - no timer interrupts */
                /* needed.  */
  }
/* If INT0/PIND2 is not 0 - the table just exited initial  */
/* position and photointerrupter1 became inactive.  */
/*   do nothing with number of remaining steps.  */
/*   do nothing with current coils state (PORTB output value) */
}

/*******************/
/* ISR(INT1_vect)  */
/*******************************************************/
/* This is an interrupt handler.                       */
/* It handles external interrupt requests on pin       */
/* INT1/PIND3                                          */
/* It assumed any level change on the INT1/PIND3 pin   */
/* triggers this interrupt request.                    */
/*-----------------------------------------------------*/
/* INT1 pin must be assigned to the photointerrupter2  */
/* residing on "Final position".                       */
/* It is assumed that a low level on the INT1/PIND3    */
/* pin (value equal to 0) means that photointerrupter2 */
/* is enabled. The table is in its final position.     */
/* High level on the INT1/PIND3 pin (value equal to 1) */
/* means that photointerrupter2 is disabled. The table */
/* is one full-/half-step away from the final          */
/* position.                                           */
/*******************************************************/
ISR(INT1_vect)  
{
  uint8_t pind_=PIND;

  if((pind_&PHOTOINTERRUPTER2_MASK)==PHOTOINTERRUPTER2_ACTIVE)
  {
/* In the case when the table is in the final */
/* position (INT1/PIND3 pin is 0 and photointerrupter2 is */
/* active). There are two possible options:  */
    if((mode&=INIT_WORKING_MODE_MASK)==
        (INIT_MODE<<INIT_WORKING_MODE_BIT))
    {
/* 1) The INIT_MODE flag is set. This is possible when */
/* voltage was applied, or when a soft/hard reset */
/* occurred and the table was between the initial and */
/* final positions. (The case when the table was in the */
/* initial or final position was handled in the */
/* ISR(TIMER1_COMPA_vect) function). In this case, the */
/* initialization process occurs and it is assumed that */
/* the table moved towards the initial position, but */
/* got to the final position. That is, the physical */
/* configuration of the system contradicts the program */
/* settings. This means that the program settings must be adjusted: */
/* -- change the value of the direction_to_initial_position */
/*    variable to the opposite. This value must also be */
/*    written to the EEPROM memory. */
/* -- write the new value from direction_to_initial_position */
/*    to the direction flag.  */
/* -- enter the MAX_STEPS value into the steps_remaining variable */
/*    (so that the table is guaranteed to reach the initial */
/*     position). */
/* -- TIMER1_ACTIVE is set in TIMSK */
/* -- INIT_MODE flag is not touched. */
/* -- return, scheduled step continues to execute. */
      direction_to_initial_position = 
        (direction_to_initial_position==CLOCKWISE_DIRECTION) ?
          COUNTERCLOCKWISE_DIRECTION : CLOCKWISE_DIRECTION;
      write_eeprom_byte(EEPROM_ADDRESS_DIRECTION_FLAG, direction_to_initial_position);
      mode&=~DIRECTION_MASK;  /* Clear Direction bit  */
      mode|=(direction_to_initial_position<<DIRECTION_BIT);
      steps_remaining=MAX_STEPS;
      TIMSK = (1 << OCIE1A);  /* Interrupt Mask  */
                              /* OICE1A bit is set to 1 means Timer/Counter1 */
                              /* Output Compare A Match Interrupt Enable */
    }
    else
    {
/* 2) INIT_MODE flag is cleared (WORKING_MODE flag is set). */
/* This is possible when the table goes beyond the working */
/* area during the working process. In this case, all */
/* movements are stopped until the next request: */
/* -- steps_remaining variable is set to 0. */
/* -- TIMSK is set to 0. */
/* -- the direction_to_initial_position variable is not changed. */
/* - the direction flag is changed to direction_to_initial_position. */
/* - the WORKING_MODE flag is not changed. */
/* - return, nothing else needs to be done. */
      steps_remaining=0;
      TIMSK = 0;  /* Interrupt Mask is cleared - no timer interrupt needed */
      mode&=~DIRECTION_MASK;  /* Clear Direction bit  */
      mode|=(direction_to_initial_position<<DIRECTION_BIT);
    }
  }

/* In case the table is one full-/half-step away from the final */
/* position (INT1/PIND3 pin is 1 and photointerrupter2 is not active): */
/* Regardless of the INIT_MODE flag, no variables are touched */
/* and the scheduled step continues to execute. Just return here.  */
}

/*******************/
/* int main(void)  */
/*******************/
int main(void)
{
/* Initialization */
  CLKPR=0x80;  /* Switch off clock divider in system generator */
  CLKPR=0x00;

  PORTA=0x00; /* Port A Initialization. */
  DDRA=0x00;  /* All three bits of the Port A are inputs. */

/* Attiny2313 has a USI interface, where there is a */
/* Two-wire communication mode. In this mode it is */
/* possible to implement the TWI protocol (AKA I2C). */
/* For Two-wire communication mode, two bits/pins are */
/* reserved in the Port B: */
/* PB7 - this is SCL: Two-wire mode Serial Clock for USI Two-wire */
/* mode. */
/* PB5 - this is SDA: Two-wire mode Serial Interface Data. */
/* Therefore, for h-bridge to work, you need to allocate two wires */
/* in another port. Let these be bits/pins PD0 and PD1 in Port D. */
/* Alternative functions for these bits/pins - RxD and TxD work when */
/* updating the firmware. */
  PORTB = 0xFF;  /* Port B Initialization  */
  DDRB = 0x5F;  /* Bit 5 and bit 7 of the port B are inputs */
                /* All other bits of the port B are outputs */

/* There are two bit/pins in PORTB reserved for Two-wire mode */
/* communication: */
/* PB7 - this is SCL: Two-wire mode Serial Clock for USI Two-wire */
/* mode. */
/* PB5 - this is SDA: Two-wire mode Serial Interface Data. */
  PORTD=0x7F; /* Port D Initialization */
  DDRD=0x03;  /* Bit 0 and bit 1 of the Port D are outputs */
              /* All other bits of the Port D are inputs. */

  TCCR0A = 0x00;   /* Timer 0 initialization */
  TCCR0B = 0x00;
  OCR0A = 0x00;
  TCNT0=0x00;

  TCCR1A=0x00; /* Timer 1 initialization */
  TCCR1B=(1<<WGM12)|(1<<CS12)|(1<<CS10);
/*  TCCR1B=0x0D;  *//* ICNC1 ICES1 - WGM13 WGM12 CS12 CS11 CS10  */
                    /*   0     0   0   0     1     1   0    1    */
                    /*  ICNC1 = 0 - no noise canceler - not needed here      */
                    /*  ICES1 = 0 - falling edge is used... not needed here  */
                    /*  WGM13 WGM12 WGM11 WGM10 - 0100 CTC - "Clear Timer on */
                    /*                                       Compare Match"  */
                    /*                                       mode            */
                    /*                                       Top is OCR1A    */
                    /*              WGM11 WGM10 - are from TCCR1A register   */
/*  OCR1A=977; */  /* About 100 msecs  */
/*  OCR1AH=0x03;
  OCR1AL=0x0C;*/
/*  OCR1A=1954;  *//*  About 200 msecs  */
  OCR1A=0x00;
  OCR1B=0x00;

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/* For now slave address should be something simple */ 
/* Assume 0x01 was selected as a slave address */
/* Later more sophisticated algorithm will be implemented */
/* probably reading this from the EEPROM */
  uint8_t sl_addr=0x01;
  usi_twi_slave_init(sl_addr);

  TCNT1H=0x00;  /* Reset Timer 1 counter  */
  TCNT1L=0x00;

  ICR1L=0x00;
  ICR1H=0x00;

  TIMSK = (1 << OCIE1A);
/*  TIMSK=0x40; */ /* Interrupts mask */
                   /* OICE1A bit is set to 1 means Timer/Counter1 */
                   /* Output Compare A Match Interrupt Enable */
/*  TIMSK = 0x00;  */


/* External interrupts initialization */
/* GIMSK=0x00; */ /* 0x0 value means no listener initialized  */
  GIMSK|=(1 << INT0);  /* This will listen on PORTD-PIN2 for INT0 */
  GIMSK|=(1 << INT1);  /* This will listen on PORTD-PIN3 for INT1 */
/* MCUCR = 0x00; */  /* 0x0 value means the low level of INT0  */
                     /* generates an interrupt request */
/* MCUCR=(1 << ISC01) | (1 << ISC00); */ /* This was presented as */
                                         /* "generate an interrupt */
                                         /* request on any logical */
                                         /* change" in some example */
                                         /* code but according to the */
                                         /* data sheet it suppose  */
                                         /* to be "the rising edge"  */
                                         /* of INT0 generates an */
                                         /* interrupt request" */
  MCUCR=((1 << ISC00) | (1 << ISC10));  /* 0x01 value in ISC00 and */
                                        /* 0x01 value in ISC10 */
                                        /* means any logical change */
										/* on INT0 and/or INT1 */
                                        /* generates an interrupt */
                                        /* request */

  USISR=0x00; /* Universal serial interface initialization */

  ACSR=0x80;  /* Analog comparator initialization */
              /* ACD bit set to 1 - Analog comparator is disabled */
              /* This will reduce power consumption in Active and */
              /* Idle mode. Also ACIE bit is set to 0 - AC interrupt */
              /* is disabled */

/* Initially H-bridge driver will be set to: */
/* Half-Step mode */
  mode=(HALF_STEPS_MODE<<FULL_HALF_STEPS_MODE_BIT); 
/*  mode=(FULL_STEPS_MODE<<FULL_HALF_STEPS_MODE_BIT);   */

/* Check if the table is not in the initial position  */

/* Set mode into "Initialization" mode*/ 
/* During initialization, if the table is not in the initial */
/* position, then it should reach the initial position */
/* and trigger an external interrupt INT0. */

/* The H-Bridge driver receives information */
/* about direction to the initial position from the */
/* EEPROM memory. This may not match the current physical */
/* configuration of the system. In this case, the table */
/* will be in its final position and will trigger an external */
/* interrupt EXT1, where the direction to the initial position */
/* will be reversed and written to the EEPROM memory. */

/* Set number of steps to MAX  */
/* Not changing stepping speed (100 ms per half-step) */
/*  direction_to_initial_position=read_eeprom_byte(EEPROM_ADDRESS_DIRECTION_FLAG);*/
  set_direction_to_initial_position(COUNTERCLOCKWISE_DIRECTION);
/*  set_direction_to_initial_position(CLOCKWISE_DIRECTION);   */
/*??!!! Not sure if set_direction_to_initial_position(...) function is needed  !!!??*/
/* set_direction_mode() function could not be used here, while it has  */
/* cli()-sei() pair inside. */
  mode&=~DIRECTION_MASK;
  if((PORTD&(1<<PIND2))!=0)
  {
/* Table is not in the initial position */
/* set number of remaining steps (MAX_STEPS) outside */
/* of cli() - sei () sequence */
/*    steps_remaining=MAX_STEPS;  */  
    steps_remaining=0;
    mode|=direction_to_initial_position<<DIRECTION_BIT;
    mode|=(INIT_MODE<<INIT_WORKING_MODE_BIT);
  }
  else
  {
    uint8_t direction_opposite_to_initial_position=
      (direction_to_initial_position==CLOCKWISE_DIRECTION) ?
         COUNTERCLOCKWISE_DIRECTION : CLOCKWISE_DIRECTION;    
/* Table is in the initial position - assume initialization */
/* has been finished. So - no remaining steps. The direction */
/* of movement should be changed to the direction to the */
/* final position. And clear initialization bit.  */
    steps_remaining=0;
    mode|=direction_opposite_to_initial_position<<DIRECTION_BIT;
    mode&=(~(INIT_MODE<<INIT_WORKING_MODE_BIT));
  }

/* Set PORTB pins into initial configuration: all coils are inactive  */
  PORTB=EVERYTHING_IS_INACTIVE;
  sei();  
/*  TCCR0B |= (0x01<<CS02)|(0x01<<CS00); */  /* This code came from */
                                             /* some example */
                                             /* It shows that timer */
											 /* could start counting  */
											 /* AFTER sei()  */

/* set_stepping_speed() function contains cli() - sei() pair. */
/* So call it outside of any cli() - sei() sequence  */
  set_stepping_speed(10); 

/* set_number_of_steps() function contains cli() - sei() pair. */
/* So call it outside of any cli() - sei() sequence  */
  if((PORTD&(1<<PIND2))!=0)
	  set_number_of_steps(MAX_STEPS);

#if 0
/* Just for tests - perform some steps and stop.  */
  set_direction_mode(COUNTERCLOCKWISE_DIRECTION);
  set_full_half_step_mode(FULL_STEPS_MODE);
/* Set number of steps should be called AFTER */
/* direction is set and half/full-step mode is set */
/* while both set_direction_mode(...) and */
/* set_full_half_step_mode(...) reset step counter  */

  set_number_of_steps(10);
  _delay_ms(200);
  set_stepping_speed(10);
  _delay_ms(10);
  set_number_of_steps(10);
  _delay_ms(200);
#endif /* #if 0 */

/* Just for tests (until I2C will be implemented)  */
#define GENERAL_TEST_MODE

#ifdef GENERAL_TEST_MODE
  uint8_t direction_=COUNTERCLOCKWISE_DIRECTION;
  uint8_t steps_=60;
/*  uint8_t half_full_step_mode = FULL_STEPS_MODE; */
#endif  /* #ifdef GENERAL_TEST_MODE  */

/* Main (endless) loop starts here */
  while(1)
  {
#ifdef GENERAL_TEST_MODE
    if(steps_remaining==0x00)
	{
      _delay_ms(500);  
      set_stepping_speed(5);
	  if(direction_==COUNTERCLOCKWISE_DIRECTION)
	    direction_=CLOCKWISE_DIRECTION;
      else 
	    direction_=COUNTERCLOCKWISE_DIRECTION;
      set_direction_mode(direction_);

/* Test if half/full mode changes  */
/* 
      if(half_full_step_mode==HALF_STEPS_MODE)
        set_full_half_step_mode(FULL_STEPS_MODE);  
      else
        set_full_half_step_mode(HALF_STEPS_MODE);
*/
      set_number_of_steps(steps_); 
      if(steps_>=40)
        steps_=9;
      steps_++;

/* More tests - some POTB pins got to be changed here, some not... */
/* Not to be used with actual scheme - will be damaged.  */
/* Just use some LEDs  */
/*
	    uint8_t portb_value_=PORTB;

	    if((portb_value_&0x04)!=0)
	    {  portb_value_&=0xFB;  }
	    else
	    {  portb_value_|=0x04;  }
	    PORTB=portb_value_;
*/
/*    _delay_ms(1000);*/
	}
#endif /* #ifdef GENERAL_TEST_MODE */
  };

  return 0;
}




