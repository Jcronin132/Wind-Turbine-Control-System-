/*
 * Lab 4 interrupts.c
 *
 * Created: 10/25/2017 2:54:18 PM
 * Author : jcronin1
 */ 

// STEPS TO USING INTERRUPTS     
// 1) include avr/interrupts.h
// 2) include Interrupts handler routine
// 3) enables globally   sei();
// 4) enable locally 

   
# include <avr/io.h>
# include <avr/interrupt.h>//				
# include <avr/pgmspace.h>
# define _BV(bit)(1 << (bit))

// makes two char arrays					0	1     2   3   4     5    6     7   8     9    A    B    C	D    E    F 
const unsigned char decode[16] PROGMEM = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x67,0x77,0x7C,0x39,0x5E,0x79,0x71};
const unsigned char digilut[4] PROGMEM = {0x01, 0x02, 0x04, 0x08};  // 0001, 0010, 0100, 1000
volatile uint16_t NUM_2_DISPLAY= 0x5A6F;
volatile uint8_t currentDecode;
uint8_t i = 0;
uint16_t j;


int main(void)
{
	// Setting up an interrupt on Timer0  every 4ms
	OCR0A = 250;   // this is for a second @ 8mhz. what number do i need for 4ms?
	TCCR0A |= (1<<WGM01);      // configuring WGMx[2:0] to 010 which is CTC mode.     what about the COM bits?
	TCCR0B |= (1<<CS02);   //   // what prescaler do we need?  this current configuration has 256 prescaler.                     
	TIMSK0 |= (1<<OCIE0A);      // enables Timer0 interrupts                                                          
							
	// Setting up a Pin change interrupt on the 328p button     
	DDRB &= ~(1 << DDB7);     // makes pin 7 an input 
	PORTB |= (1 << PORTB7);		// pull up resistor 
	PCICR |= (1 << PCIE0);    //  enables pin change interrupts.... 
	PCMSK0 |= (1 << PCINT7);  //  ....on PORTB7 only 

	// Setting up the Interrupt on ADC completion
	ADMUX = (1 << REFS0);           // Vcc for now. right justified by default, using ADC0  
	ADCSRA  |=  (1 << ADEN)|(1 << ADIE)|(7 << ADPS0);          // 128 prescaler because bigger is better
	
	DDRB = 0xFF;	// make Port B an output  // this will be the ADC port
	DDRD = 0xFF;	// make Port d an output
	
 	sei();	     // enable global interrupts    this should be at the bottom in case an interrupt fires unintentionally  
	
    while (1)	    // loop to wait for interrupt
    {	/* do nothing */   }
}

ISR (TIMER0_COMPA_vect)     // Timer Interrupt Routine              
{
	if (i>3)
	{
		i = 0;   //to make sure not go outside the array size
	}
	j = (NUM_2_DISPLAY>>(4*i));
	j &= 0x000F;
	PORTB =  pgm_read_byte(&(digilut[i]));	
	currentDecode = pgm_read_byte(&(decode[j]));
	PORTD = currentDecode; 
	i++;
	
}

ISR ( PCINT0_vect)      // External Interrupt Routine
{	
	if( ( PINB & (1 << PINB7)))  // check to see if pin 7 is high
	{
		// *LOW to HIGH pin change *
	}
	else
	{	
		//ADCSRA |= (1 << ADSC); // starts an ADC conversion
		NUM_2_DISPLAY++;
		// pass num_2_display to ADC and start conversion
		// *HIGH to LOW pin change*
	}
}

ISR ( ADC_vect)      // Analog Digital Conversion
{
	 //Overwrite NUM_2_DISPLAY with new value. 
	 NUM_2_DISPLAY = ADC;     
	// ADCH?
	  
}


ISR(BADISR_vect)
{
	while(1);
}

  