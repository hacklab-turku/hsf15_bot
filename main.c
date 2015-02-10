
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>


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

volatile uint16_t range=0;


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

	/*int32_t distance = motor->destination - motor->position;
	
	if(abs(distance) < 2)
		return 0; // close enought
	else if(abs(distance) < 5) //ramp down
		motor->target_speed = 0;
	else if(abs(distance) < 10) //ramp down
		motor->target_speed = 10000;
	else if(abs(distance) < 20) //ramp down
		motor->target_speed = 20000;

	else				// ramp up
	{
		if(distance < 0)
			motor->target_speed = -32700;
		else 
			motor->target_speed = 32700;
	}

	int16_t inc = 16; //must be <64

	if(motor->current_speed <= motor->target_speed - inc)
		motor->current_speed += inc;
	else if(motor->current_speed >= motor->target_speed + inc)
		motor->current_speed -= inc;
	
	motor_set_speed(motor, motor->current_speed);
	return 1;*/

}

void move_sonar(uint16_t len)
{
	sonar();
	_delay_ms(100);
	if(range  > 100)
		motor_set_speed(&left, 25000);
	else
		motor_set_speed(&left, 0);
	
	/*if(abs(distance) < 3){
		motor_set_speed(motor,0);
		_delay_ms(10);
		return 0; // close enought
	}

	if(abs(distance) < 20)
		speed = 13000*/

}

ISR(INT0_vect)
{
	motor_step(&left);
}

ISR(INT1_vect)
{
	motor_step(&right);
}

ISR(TIMER1_CAPT_vect)
{
	if(TCCR1B&(1<<ICES1))	//interrupt came from a rising edge
	{
		TCNT1=0;	//clear timer
		TCCR1B&=~(1<<ICES1);	//set interrupt for falling edge
	}
	else
	{
		range=ICR1;//store sonar value
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

void sonar()
{
	TCCR1B|=(1<<ICES1);	//set interrupt for rising edge

	//keep PB1 high for a while
	PORTB|=(1<<1);	
	_delay_us(100);
	PORTB&=~(1<<1);
}




int main(void)
{
	//setup for sonar
	TCCR1B|=(1<<CS11)|(1<<CS10);	//prescale
	TIMSK1|=(1<<ICIE1);	//icp1 (PB0) triggers the counter (interrupt)

	DDRB|=(1<<1);	//PB1 triggers the sonar
	PORTB&=~(1<<1);

	DDRB&=~(1<<0);	//PB0 inputs range data
	PORTB&=~(0<<0);

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


	/*move(1000, 1000);
	move(190, 0);
	move(400, 400);
	move(190, 0);
	move(200, 200);*/
	//move(300, 300);
	//move(300, 300);
	//_delay_ms(500);
	//move(0, 40);
	//_delay_ms(500);
	//move(-40, 0);
	//_delay_ms(500);
	//move(-80, -80);
	while(1){
		move_sonar(0);
		
	}

}




