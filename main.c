
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>


typedef struct 
{
	volatile int8_t dir;
	volatile int32_t position;
	volatile int32_t destination;
	volatile int16_t current_speed;
	volatile int16_t target_speed; 
	volatile uint8_t* pwm_hw;
	volatile uint8_t* dir_port;
	uint8_t  dir_pin_a;
	uint8_t  dir_pin_b;
}motor_t;

motor_t left, right;

int32_t left_lead;
volatile float range=0;
volatile uint8_t direction;

void put_char( const unsigned char data){
 
 while(!(UCSR0A & (1<<UDRE0)));
 UDR0 = data;
 
}


void init_motor(motor_t* const motor, volatile uint8_t* pwm, volatile uint8_t* port, uint8_t pin_a, uint8_t pin_b)
{
	motor->dir = 0;
	motor->position = 0;
	motor->destination = 0;

	motor->pwm_hw = pwm;
	motor->dir_port = port;
	motor->dir_pin_a = pin_a;
	motor->dir_pin_b = pin_b;
}

void motor_set_dir(motor_t* const motor, const int16_t dir)
{
	if(dir == 0)
	{
		*(motor->dir_port) &= ~motor->dir_pin_a;
		*(motor->dir_port) &= ~motor->dir_pin_b;
	}
	else if(dir < 0) 
	{
		*(motor->dir_port) |= motor->dir_pin_a;
		*(motor->dir_port) &= ~motor->dir_pin_b;
		motor->dir = -1;
	}
	else
	{
		*(motor->dir_port) |= motor->dir_pin_b;
		*(motor->dir_port) &= ~motor->dir_pin_a;
		motor->dir = 1;
	}
}

void motor_break(motor_t* const motor)
{
	*(motor->pwm_hw) = 0;
	*(motor->dir_port) |= motor->dir_pin_a;
	*(motor->dir_port) |= motor->dir_pin_b;

	for(uint8_t i = 0; i < 255; i++)
		*(motor->pwm_hw) = i;

}

void motor_set_speed(motor_t* const motor, const int16_t speed)
{
	motor_set_dir(motor, speed);
	*(motor->pwm_hw) = abs(speed) >> 7;
}



void motor_set_destination(motor_t* const motor, int32_t dest)
{
	motor->destination = dest;

}

void motor_step(motor_t* const motor)
{
	motor->position += motor->dir; 
}

uint8_t motor_move(motor_t* const motor)
{
	int32_t distance = motor->destination - motor->position;
	int16_t speed;
	
	if(abs(distance) < 3){
		motor_set_speed(motor,0);
		_delay_ms(10);
		return 0; // close enought
	}

	if(abs(distance) < 20)
		speed = 16000;
	else speed = 32700;
	
	if(distance < 0)
		motor_set_speed(motor,-speed);
	else 
		motor_set_speed(motor,speed);

	return 1;


}

void sonar()	//sends a high bit via PB1 and starts to listen to PB0
{
	TCCR1B|=(1<<ICES1);	//set interrupt for rising edge

	//keep PB1 high for a while
	PORTB|=(1<<1);	
	_delay_us(100);
	PORTB&=~(1<<1);
}

void move_sonar(uint16_t len)
{

	motor_set_speed(&right, 22000);
	motor_set_speed(&left, 22000);
	_delay_ms(500);

	while (1){

		sonar();
		_delay_ms(100);
		float dist = range - len;
		if(dist <= 0)
		{
			motor_set_speed(&left, 0);
			motor_set_speed(&right, 0);
			return;
		}
		else if(dist < 10)
		{
			motor_set_speed(&left, 14000);
			motor_set_speed(&right, 14000);
		}
		else if(dist < 20)
		{
			motor_set_speed(&left, 20000);
			motor_set_speed(&right, 20000);
		}			
		else
		{
			motor_set_speed(&left, 32000);	//32000
			motor_set_speed(&right, 32700); //32700
		}
	}
}

ISR(INT0_vect)
{
	motor_step(&left);
	//put_char('L');
}

ISR(INT1_vect)
{
	motor_step(&right);
	//put_char('R');
}

ISR(TIMER1_CAPT_vect)	//sonar is talking via PB0
{
	if(TCCR1B&(1<<ICES1))	//interrupt came from a rising edge
	{
		TCNT1=0;	//clear timer
		TCCR1B&=~(1<<ICES1);	//set interrupt for falling edge
	}
	else
	{
		range=(float)ICR1* (8.0/100.0);//store sonar value
	}
}

void move(const int16_t left_mm, const int16_t right_mm)
{

	float right_steps = right_mm / 5.340;
	float left_steps = left_mm / 5.340;

	motor_set_destination(&left, left.position + left_steps);
	motor_set_destination(&right, right.position + right_steps);

	uint8_t i;

	do
	{
		i = motor_move(&left);
		i += motor_move(&right);
		//_delay_us(100);

	} while(i); 	
}

int16_t get_diff(uint8_t d, uint8_t c)
{
	int16_t diff = d - c;
	if(diff > 128)
		return  diff - 256;
	if(diff < -128)
		return diff + 256;
	return diff;
}

void rotate_gyro_right(int16_t steps, int16_t speed)
{
	uint8_t direction_dest = direction + steps;
	
	{
		if(get_diff(direction_dest, direction) > 0)
		{
				motor_set_speed(&left, speed);
	}
		else
		{
				motor_set_speed(&left, -speed);
			
		}
	}

	while (abs(get_diff(direction_dest, direction)) > 1);
	motor_set_speed(&left,0);

}

void rotate_gyro_right_fix(int16_t steps)
{
	uint8_t direction_dest = direction + steps;
	int16_t speed = 26000;

	while(abs(get_diff(direction_dest, direction)) > 3)
	{
		rotate_gyro_right(get_diff(direction_dest, direction), speed);
		speed -= 1000;
		_delay_ms(500);
	}

}


void rotate(motor_t* const motor, int16_t steps)
{
	
	int32_t dest = motor->position + steps;
	while(abs(dest - motor->position) > 1)
	{
		while(abs(dest - motor->position) > 1)
		{
			if(dest > motor->position)
				motor_set_speed(motor, 32000);
			else
				motor_set_speed(motor, -32000);
		}
		motor_set_speed(motor,0);
		_delay_ms(100);
	}	

}

void move_turn_right(int16_t dist)
{
//	printf("move turn R:  ");
//	printf("%i", right.position);
//	printf(" %i\n", left.position);
 	move_sonar(dist);
	_delay_ms(100);


	rotate(&left, 41);
//	printf("%i", right.position);
//	printf(" %i\n", left.position);
	_delay_ms(100);
}

void move_turn_left(int16_t dist)
{
//	printf("move turn L:  ");
//	printf("%i", right.position);
//	printf(" %i\n", left.position);
 	move_sonar(dist);
	_delay_ms(100);

	rotate(&right, 41);
//	printf("%i", right.position);
//	printf(" %i\n", left.position);
	_delay_ms(100);
}

void back()
{
	motor_set_speed(&left, -32000);
	motor_set_speed(&right, -32000);
	_delay_ms(1000);
	motor_set_speed(&left, 0);
	motor_set_speed(&right, 0);
}


void lab()
{
	move_turn_right(280);
	back();
	move_turn_right(280);
	back();
	move_turn_right(280);
	back();

	move_turn_left(280);
	back();
	move_turn_left(200);
	back();

	move_turn_right(280);
	back();
	move_turn_right(200);
	back();
	move_turn_right(280);
	back();
}

ISR(USART_RX_vect)
{
   direction= UDR0; 
//	motor_set_speed(&left, 24000);
}




FILE uart_str = FDEV_SETUP_STREAM(put_char, NULL, _FDEV_SETUP_RW);

int main(void)
{
	//setup for sonar
	TCCR1B|=(1<<CS11);	// prescale clk/8 overflows in 4ms if clock is divided by 8 
	TIMSK1|=(1<<ICIE1);	//icp1 (PB0) triggers the counter (interrupt)

	DDRB|=(1<<1);	//PB1 triggers the sonar
	PORTB&=~(1<<1);

	DDRB&=~(1<<0);	//PB0 inputs range data
	PORTB&=~(1<<0);

	//zidit's setups :D
	TCCR0A = 0b10100001;
	TCCR0B = 0b00000011;	

	DDRC |= (1<<PINC0) | (1<<PINC1) | (1<<PINC2) | (1<<PINC3);
	DDRD |= (1<<PIND5) | (1<<PIND6);
	DDRB |= (1<<PINB5);


	init_motor(&left, &OCR0B, &PORTC, (1<<PINC0), (1<<PINC1));
	init_motor(&right, &OCR0A, &PORTC, (1<<PINC2), (1<<PINC3));

	EICRA = 0b00000101;
	EIMSK = 0b00000011;
	sei();


	#define BAUDRATE 9600        //The baudrate that we want to use
	#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)    //The formula that does all the required maths
	 
	 UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
	 UBRR0L = (uint8_t)(BAUD_PRESCALLER);
	 UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	 UCSR0C = ((1<<UCSZ00)|(1<<UCSZ01));
	
   	UCSR0B |= (1 << RXCIE0); // Enable the USART Recieve Complete interrupt (USART_RXC)
	
	//stdout = &uart_str;

//	printf("test");


	_delay_ms(1000);
	//lab();
	//rotate_right(40);
	//	rotate_gyro_right_fix(64);
		//_delay_ms(2000);
	while(1){
		lab();
		//rotate_gyro_right_fix(64);
		//_delay_ms(2000);
		//lab();
		//rotate_gyro_right_fix(64);
		//_delay_ms(2000);
		//rotate_gyro_right_fix(-64);
		//_delay_ms(2000);
		/*if(direction < 128)
			motor_set_speed(&left, 24000);
		else 
			motor_set_speed(&left, 0);*/
		//rotate_right(42);
		//_delay_ms(1000);
		//rotate_right(-40);
		//_delay_ms(1000);
	}

}




