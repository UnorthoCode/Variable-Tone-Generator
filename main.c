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

ISR(TIMER1_COMPA_vect) // Buzz ON
{
	PORTD |= _BV(PORTD3);
}

ISR(TIMER1_COMPB_vect) // Buzz OFF
{
	PORTD &= ~_BV(PORTD3);
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
	ADMUX |= 1; // Start with ADC1
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
	// adjust frequency of the clock
	OCR1A = 7644 + ADCValue * 7; // 7644 -> 2093 hz, 15296 -> 1043 hz
	OCR1B = OCR1A / 10000; // 50% duty cycle
}

void setup(void) 
{
	sei(); // enable global interrupts
	InitADC();
	InitVariablePWMFrequencyClk();
	USART_Init(MYUBRR);
}

int main(void)
{
	setup();
	volatile char uart_out[5] = {0};
    while (1) 
    {
	itoa(7644 + ADCValue * 7 + 490, uart_out, 10);
	USART_put(uart_out);
	USART_put("\n\r");
    }
	return 0;
}

