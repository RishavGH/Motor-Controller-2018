/*
 * atmega_328.c
 *
 * Created: 25-Sep-16 19:26:58
 *  Author: Shashank more
 */ 


#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define GL_A 1   //PB1
#define GL_B 3  //PB3	
#define GL_C 6   //PD6
#define GH_A 2   //PB2
#define GH_B 3   //PD3
#define GH_C 5   //PD5
#define EN_GATE	2  //PD2
#define nFAULT 4  //PD4
#define nOCTW 0  //PD0
#define nPWRGD 7  //PD7
#define VDD_SPI 3  //PC3
#define HALL_B 0  //PC0
#define HALL_G 1  //PC1
#define HALL_Y 2  //PC2
#define zero_throttle 5 //PC5
#define throttle_led 0  //PB0
#define MOSI 3
#define SCK 5
#define n_SCS 1  //PD1

void pin_change_interrupt_init(void);
void ADC_init(void);
void GH_A_PWM(void);
void GL_A_PWM(void);
void GH_B_PWM(void);
void GL_B_PWM(void);
void GH_C_PWM(void);
void GL_C_PWM(void);

uint8_t throttle;
char i=0;
int ADC_i;

int adc_val = 0, offset = 0;

ISR(PCINT1_vect)
{
	if(PINC & (1<<HALL_Y) && !(PINC & (1<<HALL_G)) && !(PINC & (1<<HALL_B)))         // Commutation
	{
		// GH_B & GL_C
		OCR0A=0;  //<--
		OCR0B=0;
		OCR2A=255;  
		OCR2B=throttle;  //<--
		OCR1A=255;
		OCR1B=0;
	}
	else if(!(PINC & (1<<HALL_Y)) && PINC & (1<<HALL_G) && PINC & (1<<HALL_B))
	{
		// GH_C & GL_B
		OCR0A=255;
		OCR0B=throttle;  //<-
		OCR2A=0;  //<--
		OCR2B=0;
		OCR1A=255;
		OCR1B=0;
	}
	else if(!(PINC & (1<<HALL_Y)) && PINC & (1<<HALL_G) && !(PINC & (1<<HALL_B)))
	{
		// GH_A & GL_B
		OCR0A=255;
		OCR0B=0;
		OCR2A=0;  //<--
		OCR2B=0;
		OCR1A=255;
		OCR1B=throttle;  //<--
	}
	else if(!(PINC & (1<<HALL_Y)) && !(PINC & (1<<HALL_G)) && PINC & (1<<HALL_B))
	{
		// GH_C & GL_A
		OCR0A=255;
		OCR0B=throttle;  //<--
		OCR2A=255;
		OCR2B=0;
		OCR1A=0;  //<--
		OCR1B=0;
	}
	else if(PINC & (1<<HALL_Y) && !(PINC & (1<<HALL_G)) && PINC & (1<<HALL_B))
	{
		// GH_B & GL_A
		OCR0A=255;
		OCR0B=0;
		OCR2A=255;
		OCR2B=throttle;  //<--
		OCR1A=0;  //<--
		OCR1B=0;
	}
	else if(PINC & (1<<HALL_Y) && PINC & (1<<HALL_G) && !(PINC & (1<<HALL_B)))
	{
		// GH_A & GL_C
		OCR0A=0;  //<--
		OCR0B=0;
		OCR2A=255;
		OCR2B=0;
		OCR1A=255;
		OCR1B=throttle;  //<--
	}
	else
	{
		//comm_port=0x00;
		OCR0A=255;
		OCR0B=0;
		OCR2A=255;
		OCR2B=0;
		OCR1A=255;
		OCR1B=0;
	}
}

ISR(ADC_vect)
{
	adc_val = ADC;
	
	if (adc_val < 198)
	{
		throttle = 0;
	}
	else if (adc_val <= 812)
	{//adc_val = 875;
		
		//int scaled_adc = ((float)adc_val/1024)*255; //41.828
		//OCR2A = (255.0/174.5)*(scaled_adc - 42);
		offset = (((float)adc_val/1023) * 255) - 42.115;
		throttle = ((float)offset/174.7353) * 255;		//176.382
	}
	else
	throttle = 255;
	//throttle=fabs((int)((float)ADC*255/750-60));
	//throttle=fabs((int)((float)ADC*255/870-52));
	//throttle=(ADC*255.0)/1023.0;
}

int main(void)
{
	DDRD|=(1<<GH_B)|(1<<GH_C)|(1<<GL_C)|(1<<EN_GATE);
	DDRB|=(1<<GL_A)|(1<<GL_B)|(1<<GH_A)|(1<<throttle_led);
	DDRC|=(1<<zero_throttle);
	sei();
	pin_change_interrupt_init();
	GH_A_PWM();
	GL_A_PWM();
	GH_B_PWM();
	GL_B_PWM();
	GH_C_PWM();
	GL_C_PWM();
	ADC_init();
	_delay_ms(1000);
	PORTD|=(1<<EN_GATE);
	_delay_ms(100);
	PORTB=(1<<throttle_led);
	throttle = 0;
	while(1)
	{
		ADCSRA|=(1<<ADSC);
		while (ADCSRA & (1<<ADSC));
		if (throttle<300)
		{
			PORTC|=(1<<zero_throttle);
			PORTC&= ~(1<<zero_throttle);
		}
		/*if (!(PIND & (1<<nFAULT)))
		{
			//_delay_us(64);
			//spi read command to now at which mosfets OC event occur
			//spi gate reset command
		}*/
	}
}

void pin_change_interrupt_init(void)
{
	PCMSK1=(1<<PCINT8)|(1<<PCINT9)|(1<<PCINT10)|(1<<PCINT13);
	PCICR=(1<<PCIE1);
}
void ADC_init(void)
{
	ADMUX=(1<<REFS0)|(1<<MUX2);
	ADCSRA=(1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(ADPS0);
}
void GH_A_PWM(void)
{
	TCCR1A|=(1<<COM1B1)|(1<<WGM10);
	TCCR1B|=(1<<WGM12)|(1<<CS10);
}
void GL_A_PWM(void)
{
	TCCR1A|=(1<<COM1A1)|(1<<COM1A0)|(1<<WGM10);
	TCCR1B|=(1<<WGM12)|(1<<CS10);
}
void GH_B_PWM(void)
{
	TCCR2A|=(1<<COM2B1)|(1<<WGM21)|(1<<WGM20);
	TCCR2B|=(1<<CS20);
}
void GL_B_PWM(void)
{
	TCCR2A|=(1<<COM2A1)|(1<<COM2A0)|(1<<WGM21)|(1<<WGM20);
	TCCR2B|=(1<<CS20);
}
void GH_C_PWM(void)
{
	TCCR0A|=(1<<COM0B1)|(1<<WGM01)|(1<<WGM00);
	TCCR0B|=(1<<CS00);
}
void GL_C_PWM(void)
{
	TCCR0A|=(1<<COM0A1)|(1<<COM0A0)|(1<<WGM01)|(1<<WGM00);
	TCCR0B|=(1<<CS00);
}