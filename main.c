
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

uint16_t range=0;


void drive_left(int8_t a)
{
	if(a == 0)
	{
		OCR0B = 0;
		PORTC &= ~(1 << PINC0);
		PORTC &= ~(1 << PINC1);
	}
	else if(a < 0) 
	{
		PORTC |= (1 << PINC0);
		PORTC &= ~(1 << PINC1);
		OCR0B = (-a) << 2;
	}
	else
	{
		PORTC |= (1 << PINC1);
		PORTC &= ~(1 << PINC0);
		OCR0B = a << 2;
	}

	//if(a)
		//PORTD |= (1 << PIND5);	

	
}

void drive_right(int8_t a)
{
	if(a == 0)
	{
		OCR0A = 0;
		PORTC &= ~(1 << PINC3);
		PORTC &= ~(1 << PINC2);
	}
	else if(a < 0) 
	{
		PORTC |= (1 << PINC2);
		PORTC &= ~(1 << PINC3);
		OCR0A = (-a) << 2;
	}
	else
	{
		PORTC |= (1 << PINC3);
		PORTC &= ~(1 << PINC2);
		OCR0A = a << 2;
	}

}

ISR(TIMER1_CAPT_vect)
{
	if(TCCR1B&(1<<ICES1))	//interrupt came from a rising edge
	{
		TCNT1=0;	//clear timer
		TCCR1B&=(0<<ICES1);	//set interrupt for falling edge
	}
	else
	{
		range=ICR1;//store sonar value
	}
}

void sonar()
{
	TCCR1B|=(1<<ICES1);	//set interrupt for rising edge

	//keep PB1 high for a while
	PORTB|=(1<<1);	
	_delay_us(10);
	PORTB&=(0<<1);
}

int main(void)
{
	//setup for sonar
	TCCR1B|=(1<<CS11)|(1<<CS10);	//prescale
	TIMSK1|=(1<<ICIE1);	//icp1 (PB0) triggers the counter (interrupt)

	DDRB|=(1<<1);	//PB1 triggers the sonar
	PORTB&=(0<<1);

	DDRB&=(0<<0);	//PB0 inputs range data
	PORTB&=(0<<0);

	//zidit's setups :D
	TCCR0A = 0b10100001;
	TCCR0B = 0b00000011;	

	DDRC |= (1<<PINC0) | (1<<PINC1);
	DDRD |= (1<<PIND5) | (1<<PIND6);

	OCR0A = 100;
	drive_right(0);
	drive_right(0);

	while(1){
		drive_right(100);
		_delay_ms(500);
		drive_right(-100);
		_delay_ms(500);
		drive_right(0);

		drive_left(100);
		_delay_ms(500);
		drive_left(-100);
		_delay_ms(500);	
		drive_left(0);
	}

}
