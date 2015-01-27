
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


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




int main(void)
{
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
