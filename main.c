#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#define F_CPU 16000000
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

void USART_Init(unsigned int ubrr)
{
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);
}

void USART_Transmit(volatile char data)
{
	/* Wait for empty transmit buffer */
	while (!(UCSR0A & (1<<UDRE0)));
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

void USART_put(volatile char *string) 
{
	while(*string != 0x00)
		USART_Transmit(*string++);
}

unsigned char USART_Receive(void)
{
	/* Wait for data to be received */
	while (!(UCSR0A & (1<<RXC0)));
	/* Get and return received data from buffer */
	return UDR0;
}

ISR(TIMER2_COMPA_vect) // LED ON
{
	PORTD |= _BV(PORTD2);
}

ISR(TIMER2_COMPB_vect) // LED OFF
{
	PORTD &= ~_BV(PORTD2);
}

ISR(TIMER1_COMPA_vect) // Buzz ON
{
	PORTD |= _BV(PORTD3);
}

ISR(TIMER1_COMPB_vect) // Buzz OFF
{
	PORTD &= ~_BV(PORTD3);
}

void Init1KhzSquareWave(void) // this is for the LED
{
	DDRD |= _BV(PORTD2); // set output pin
	TCCR2A |= _BV(WGM21); // set CTC
	TIMSK2 |= _BV(OCIE2A) | _BV(OCIE2B); // set compare interrupts on timer2 COMPA & B
	TCCR2B |= _BV(CS21) |_BV(CS20); // set prescalar 32
	OCR2A = F_CPU/(1000 * 2 * 32); // 250 end
	OCR2B = OCR2A / 2; // 50% duty cycle
}

void InitVariablePWMFrequencyClk(void) // this is for the piezo
{
	DDRD |= _BV(PORTD3);
	TCCR1B |= _BV(WGM12); // set CTC
	TIMSK1 |= _BV(OCIE1A) | _BV(OCIE1B); // set compare interrupts on time1 COMPA & B
	TCCR1B |= _BV(CS10); // no prescaling / start timer
	OCR1A = 15296; // 1046hz 
	OCR1B = OCR1A / 2; // 50% duty cycle
	// PWM_fequency = clock_speed / [Prescaller_value * (1 + TOP_Value) ]
}

void InitADC(void)
{
	ADMUX |= 1; // Start with ADC0
	ADMUX |= _BV(REFS0); // Set ADC reference to AVCC -- needs 5 volts on
	ADMUX &= ~_BV(ADLAR); // clear for 10bit mode, 8 bit mode when being set
	
	ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // Set ADC prescaler to 128 - 125KHz sample rate @ 16MHz
	ADCSRA |= _BV(ADATE); // ADC Auto trigger

	ADCSRB = 0;  // Set ADC to Free-Running Mode
	
	ADCSRA |= _BV(ADEN);  // Enable ADC
	ADCSRA |= _BV(ADIE);  // Enable ADC Interrupt
	ADCSRA |= _BV(ADSC);  // Start ADC Conversions
}

volatile uint16_t ADCValue = 0;
ISR(ADC_vect)
{	
	ADCValue = ADCL;
	ADCValue = (ADCH << 8) + ADCValue;
	
	if ((ADMUX & 0b00000001) == 0) // LED
	{
		// adjust PWM duty cycle
		OCR2B = ADCValue >> 2; // sensor resolution 0 - 250 due to 8bit timer @ 1khz
	}
	else // potentiometer
	{
		// adjust frequency of the clock
		OCR1A = 7644 + ADCValue * 7; // 7644 -> 2093 hz, 15296 -> 1043 hz
		OCR1B = OCR1A / 10000; // 50% duty cycle
	
	}	
}

void setup(void) 
{
	sei(); // enable global interrupts
	InitADC();
	Init1KhzSquareWave();
	InitVariablePWMFrequencyClk();
	USART_Init(MYUBRR);
}

int main(void)
{
	setup();
	volatile char uart_out[5] = {0};
	unsigned short pot_bias = 0;
    while (1) 
    {
		// switches between the states
		if ((ADMUX & 0x1) == 0) // LED
		{
			pot_bias = 0;
			ADMUX |= _BV(0);
			itoa(ADCValue, uart_out, 10);
			USART_put(uart_out);
			USART_put("\n\r");
		}
		else // potentiometer
		{
			if (pot_bias++ > 13)
				ADMUX &= ~(_BV(0));
				
			itoa(7644 + ADCValue * 7 + 490, uart_out, 10);
			USART_put(uart_out);
			USART_put("\n\r");
		}
    }
	return 0;
}

