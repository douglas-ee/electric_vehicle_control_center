#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "nokia5110.h"
#include <avr/eeprom.h>

unsigned char cont = '0';
//void EEPROM_write(unsigned int uiAddress, unsigned char ucData);
//unsigned char EEPROM_read(unsigned int uiAddress);

ISR(PCINT2_vect)
{
	if((PIND & 0b00000001) == 0)
		eeprom_write_byte(0, cont);
	if((PIND & 0b00000010) == 0)
		cont = eeprom_read_byte(0);
	if((PIND & 0b00000100) == 0)
		cont++;
}

int main(void)
{
	DDRD &= 0b11111000;
	PORTD = 0b00000111;
	PCICR = 0b00000100;
	PCMSK2 = 0b00000111;
	
	sei();
	nokia_lcd_init();

	while(1)
	{
		char cont_string[2];
		cont_string[0] = cont;
		cont_string[1] = '\0';
		
		nokia_lcd_clear();
		nokia_lcd_set_cursor(0, 20);
		nokia_lcd_write_string("Cont: ", 1);
		nokia_lcd_write_string(cont_string, 2);
		nokia_lcd_render(); // resetando o lcd
		_delay_ms(200);
	}
}

/*
void EEPROM_write(unsigned int uiAddress, unsigned char ucData)
{
	while(EECR & ( 1 << EEPE));
	EEAR = uiAddress;
	EEDR = ucData;
	EECR |= (1 << EEMPE);
	EECR |= (1 << EEPE);
}

unsigned char EEPROM_read(unsigned int uiAddress)
{
	while(EECR & ( 1 << EEPE));
	EEAR = uiAddress;
	EECR |= (1 << EERE);
	return EEDR;
}
*/