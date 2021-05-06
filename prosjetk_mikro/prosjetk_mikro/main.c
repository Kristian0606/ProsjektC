/*
 * prosjetk_mikro.c
 *
 * Created: 16.04.2021 12:52:25
 * Author : Victor Moland,Birger Mælen og Kristian Finnerud Nilsen
 * Styre en RGB diode ved hjelp av 3 potmeter(R, G og B) 
 *
 */ 

#define F_CPU 16000000UL //System klokken
#define BAUD 9600 // Ønsket baudrate
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL)))-1) //UBRR value
#define __DELAY_BACKWARD_COMPATIBLE__

//Bibloteker
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "USART.h"

//Globale variabler
uint8_t Red_value;
uint8_t Green_value;
uint8_t Blue_value;

int main(void)
{
	//Konfigurer kontrolleren
	DDRD = (1<<PORTD6) | (1<<PIND3);// RØD LED og internal-pullup på knapp (PD3)
	DDRB = (1<<PORTB1) | (1<<PORTB2); // Setter GRØNN OG BLÅ LED til output.
	
	//ADC
    ADMUX = (1<<REFS0) | (1<<ADLAR); //leftshift på ADC så vi kan lese av ADCH 8-bits verdier.
	DIDR0 = (1<<ADC0D) | (1<<ADC1D) |(1<<ADC2D); // Tre ulike ADC potmeter
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //
	
	//PWM TIMER 0
	TCCR0A = (1<<COM0A1) | (1<<WGM00) | (1<<WGM01); //Fast PWM og OCR0A 
	TCCR0B = (1<<CS02); //prescale 256
	
	//PWM TIMER 1
	TCCR1A = (1<<COM1A1) |(1<<COM1B1) |  (1<<WGM10);
	TCCR1B = (1<<CS12) | (1<<WGM12); //prescale 256
	
	//Interupt
	PCICR = (1<<PCIE2);
	PCMSK2 = (1 << PCINT19);
	sei();
	
	//Terminal
	initUSART(); //Starter terminalen
	
	
    do{
		//Rød verdi:
		ADMUX = 0b01100000; //Måler over den Rød.
	    ADCSRA |= (1<<ADSC);
	    do {} while(ADCSRA & (1<<ADSC));
		Red_value = ADCH; //lagrer verdien til Rød på ADCH
		printString("Rød:");
	    printByte(Red_value); //printer verdien til R.
		printString("  ---  ");
		
		//Grønn verdi:
		 ADMUX = 0b01100001; //Måler over den Grønne
		 ADCSRA |= (1<<ADSC);
		 do {} while(ADCSRA & (1<<ADSC));
		 Green_value = ADCH;
		 printString("Grønn:");
		 printByte(Green_value);
		 printString("  ---  ");
		 
		 //Blå verdi:
		  ADMUX = 0b01100010; //Måler over den Blå
		  ADCSRA |= (1<<ADSC);
		  do {} while(ADCSRA & (1<<ADSC));
		  Blue_value = ADCH;
		  printString("Blå:");
		  printByte(Blue_value);
		  printString("\r\n");
		 
		 _delay_ms(1000);
		 
    } while(1);
    }
	
    ISR(PCINT2_vect){
		//RGB til farger:
		OCR0A = Red_value;
		OCR1A = Green_value;
		OCR1B = Blue_value;
		
		_delay_ms(100);
	}


