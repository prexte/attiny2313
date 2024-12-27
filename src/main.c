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


/* This is speed in msecs per timer ticks  */
/* Initially set it 100 msecs */
volatile uint16_t stepping_speed=100;
volatile uint16_t steps_remaining=0;
volatile uint8_t mode=0x0;
volatile uint8_t direction_to_initial_position=CLOCKWISE_DIRECTION;
/*uint16_t timer_divider_value=390;*/

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
  uint8_t portb_value=PORTB;
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

  if(steps_remaining>0)
  {
#ifdef CLOCKWISE_DIRECTION_HALF_STEPS
    if(((mode&DIRECTION_MASK)>0)&&
	   ((mode&HALF_STEPS_MODE_MASK)>0))
    {
/* mode is half steps and clockwise direction */
      switch(portb_value)
      {
        case EVERYTHING_IS_INACTIVE:
        case COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION:
          portb_value=COIL_1_ACTIVE_POSITIVE_DIRECTION_ONLY;
          break;
        case COIL_1_ACTIVE_POSITIVE_DIRECTION_ONLY:
          portb_value=COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION;
          break;
        case COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION:
          portb_value=COIL_2_ACTIVE_POSITIVE_DIRECTION_ONLY;
          break;
        case COIL_2_ACTIVE_POSITIVE_DIRECTION_ONLY:
          portb_value=COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION;
          break;
        case COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION:
          portb_value=COIL_1_ACTIVE_NEGATIVE_DIRECTION_ONLY;
          break;
        case COIL_1_ACTIVE_NEGATIVE_DIRECTION_ONLY:
          portb_value=COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION;
          break;
        case COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION:
          portb_value=COIL_2_ACTIVE_NEGATIVE_DIRECTION_ONLY;
          break;
        case COIL_2_ACTIVE_NEGATIVE_DIRECTION_ONLY:
          portb_value=COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION;
          break;
        default:
/* This is "impossible state" for coils.  */
/* Reset PORTB and do not do any more steps. */
          portb_value=EVERYTHING_IS_INACTIVE;
		  steps_remaining=0;
          break;
	  }
	}
#endif   /*  #ifdef CLOCKWISE_DIRECTION_HALF_STEPS  */
  
#ifdef COUNTERCLOCKWISE_DIRECTION_HALF_STEPS
    if(((mode&DIRECTION_MASK)==0)&&
       ((mode&HALF_STEPS_MODE_MASK)>0))
    {
/* mode is half steps and counterclockwise direction */
      switch(portb_value)
      {
        case EVERYTHING_IS_INACTIVE:
        case COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION:
          portb_value=COIL_2_ACTIVE_NEGATIVE_DIRECTION_ONLY;
          break;
        case COIL_2_ACTIVE_NEGATIVE_DIRECTION_ONLY:
          portb_value=COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION;
          break;
        case COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION:
          portb_value=COIL_1_ACTIVE_NEGATIVE_DIRECTION_ONLY;
          break;
        case COIL_1_ACTIVE_NEGATIVE_DIRECTION_ONLY:
          portb_value=COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION;
          break;
        case COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION:
          portb_value=COIL_2_ACTIVE_POSITIVE_DIRECTION_ONLY;
          break;
        case COIL_2_ACTIVE_POSITIVE_DIRECTION_ONLY:
          portb_value=COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION;
          break;
        case COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION:
          portb_value=COIL_1_ACTIVE_POSITIVE_DIRECTION_ONLY;
          break;
        case COIL_1_ACTIVE_POSITIVE_DIRECTION_ONLY:
          portb_value=COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION;
          break;
        default:
/* This is "impossible state" for coils.  */
/* Reset PORTB and do not do any more steps. */
          portb_value=EVERYTHING_IS_INACTIVE;
          steps_remaining=0;
          break;
      }
    }
#endif  /* #ifdef COUNTERCLOCKWISE_DIRECTION_HALF_STEPS  */

#ifdef CLOCKWISE_DIRECTION_FULL_STEPS
    if(((mode&DIRECTION_MASK)>0)&&
       ((mode&HALF_STEPS_MODE_MASK)==0))
    {
/* mode is full steps and clockwise direction */
      switch(portb_value) 
      {
        case EVERYTHING_IS_INACTIVE:
        case COIL_2_ACTIVE_NEGATIVE_DIRECTION_ONLY:
          portb_value=COIL_1_ACTIVE_POSITIVE_DIRECTION_ONLY;
          break;
        case COIL_1_ACTIVE_POSITIVE_DIRECTION_ONLY:
          portb_value=COIL_2_ACTIVE_POSITIVE_DIRECTION_ONLY;
          break;
        case COIL_2_ACTIVE_POSITIVE_DIRECTION_ONLY:
          portb_value=COIL_1_ACTIVE_NEGATIVE_DIRECTION_ONLY;
          break;
        case COIL_1_ACTIVE_NEGATIVE_DIRECTION_ONLY:
          portb_value=COIL_2_ACTIVE_NEGATIVE_DIRECTION_ONLY;
          break;

        case COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION:
          portb_value=COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION;
          break;
        case COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION:
          portb_value=COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION;
          break;
        case COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION:
          portb_value=COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION;
          break;
        case COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION:
          portb_value=COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION;
          break;

        default:
/* This is "impossible state" for coils.  */
/* Reset PORTB and do not do any more steps. */
          portb_value=EVERYTHING_IS_INACTIVE;
          steps_remaining=0;
          break;
      }
    }
#endif  /* #ifdef CLOCKWISE_DIRECTION_FULL_STEPS  */

#ifdef COUNTERCLOCKWISE_DIRECTION_FULL_STEPS
    if(((mode&DIRECTION_MASK)==0)&&
       ((mode&HALF_STEPS_MODE_MASK)==0))
    {
/* mode is full steps and counterclockwise direction */
      switch(portb_value) 
      {
        case EVERYTHING_IS_INACTIVE:
        case COIL_2_ACTIVE_NEGATIVE_DIRECTION_ONLY:
          portb_value=COIL_1_ACTIVE_NEGATIVE_DIRECTION_ONLY;
          break;
        case COIL_1_ACTIVE_NEGATIVE_DIRECTION_ONLY:
          portb_value=COIL_2_ACTIVE_POSITIVE_DIRECTION_ONLY;
          break;
        case COIL_2_ACTIVE_POSITIVE_DIRECTION_ONLY:
          portb_value=COIL_1_ACTIVE_POSITIVE_DIRECTION_ONLY;
          break;
        case COIL_1_ACTIVE_POSITIVE_DIRECTION_ONLY:
          portb_value=COIL_2_ACTIVE_NEGATIVE_DIRECTION_ONLY;
          break;

        case COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION:
          portb_value=COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION;
          break;
        case COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION:
          portb_value=COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION;
          break;
        case COIL_1_ACTIVE_NEGATIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION:
          portb_value=COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION;
          break;
        case COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_POSITIVE_DIRECTION:
          portb_value=COIL_1_ACTIVE_POSITIVE_DIRECTION_COIL_2_ACTIVE_NEGATIVE_DIRECTION;
          break;

        default:
/* This is "impossible state" for coils.  */
/* Reset PORTB and do not do any more steps. */
          portb_value=EVERYTHING_IS_INACTIVE;
          steps_remaining=0;
          break;
      }
    }
#endif /* #ifdef COUNTERCLOCKWISE_DIRECTION_FULL_STEPS  */
    PORTB=portb_value;
    if((steps_remaining>0)&&
       ((pind_&PHOTOINTERRUPTER1_MASK)!=PHOTOINTERRUPTER1_ACTIVE)&&
       ((pind_&PHOTOINTERRUPTER2_MASK)!=PHOTOINTERRUPTER2_ACTIVE))
      steps_remaining--;
  }
  else
  {
    TIMSK=0x00; /* Interrupts mask */
                /* Clear all bits - no timer interrupts */
                /* needed.  */
  }

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

  PORTB = 0xFF;  /* Port B Initialization  */
  DDRB = 0xFF;

  PORTD=0x7F; /* Port D Initialization */
  DDRD=0x00;  /* All seven bits of the Port D are inputs. */

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




