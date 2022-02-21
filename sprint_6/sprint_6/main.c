#define F_CPU 16000000UL
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
#include <stdio.h>
#include "nokia5110.h"
#include "configuracao.h"
#include <avr/eeprom.h>

// declarando variaveis globais
uint8_t flag_5ms = 0, flag_500ms = 0;
uint16_t Velocidade_carro_kmH = 0, RPM_motor = 0, Diametro_pneu_cm = 65, leitura_ADC = 0, Distancia_hodometro_km = 0,  RPM_motor_pass = 0;
uint32_t tempo_ms = 0, acelerador = 0, acelerador_sub = 0;
uint64_t Distancia_hodometro_km_pass = 0;
const float PI = 3.14161;

ISR(TIMER0_COMPA_vect);
ISR(INT0_vect);
ISR(PCINT2_vect);
void anima_velocidade(uint16_t velocidade_carro, uint8_t *flag_disparo);
void anima_LCD(uint16_t diametro_pneu_cm, uint16_t rpm_motor, uint16_t distancia_hodometro_km, uint8_t *flag_disparo);

int main(void)
{
	// Variáveis locais
	unsigned char aux_eeprom[8];
	unsigned char aux_2_eeprom[8];

	configuracao();		// chamando funcao que configura GPIO, interruptores, timers, ADC, PWM
	nokia_lcd_init();	// inicia o display nokia

	// Gravando byte a byte na EEPROM
	for (int i = 0; i < 4; i++)
	{
		aux_eeprom[i] = eeprom_read_byte(i);
		Diametro_pneu_cm = atoi(aux_eeprom); // Converter de char para int
	}

	// Gravando byte a byte na EEPROM
	for (int i = 4; i < 9; i++)
	{
		aux_eeprom[i - 4] = eeprom_read_byte(i);
		Distancia_hodometro_km = atoi(aux_eeprom);
	}
	Distancia_hodometro_km_pass = Distancia_hodometro_km * 100000;

	while (1)
	{
		OCR2B = acelerador_sub; // PWM
		anima_velocidade(Velocidade_carro_kmH, &flag_5ms);		// funcao responsavel por retornar a velocidade no display 7-Seg
		anima_LCD(Diametro_pneu_cm, RPM_motor, Distancia_hodometro_km, &flag_500ms);	// funcao responsavel por retornar variaveis para o display nokia

		itoa(Diametro_pneu_cm, aux_2_eeprom, 10);
		for (int i = 0; i < 4; i++)
		{
			eeprom_write_byte(i, aux_2_eeprom[i]);
		}
		itoa(Distancia_hodometro_km, aux_2_eeprom, 10);
		for (int i = 4; i < 8; i++)
		{
			eeprom_write_byte(i, aux_2_eeprom[i - 4]);
		}
	}
}

ISR(TIMER0_COMPA_vect)
{
	tempo_ms++;
	//PORTD ^= 0b01000000;
	if((tempo_ms % 1) == 0)		//true a cada 5ms
	flag_5ms = 1;
	if((tempo_ms % 100) == 0)	//true a cada 500ms
	flag_500ms = 1;
}

ISR(INT0_vect) // Tacômetro
{
	static uint8_t cont_5voltas = 0;
	static uint32_t tempo_ms_anterior = 0;
	uint16_t delta_t_ms = 0;

	if(cont_5voltas == 5)
	{
		delta_t_ms = tempo_ms - tempo_ms_anterior;
		tempo_ms_anterior = tempo_ms;
		RPM_motor = 300000 / (delta_t_ms);
		Velocidade_carro_kmH = ((uint32_t)Diametro_pneu_cm * 565) / delta_t_ms;
		tempo_ms_anterior = tempo_ms;
		cont_5voltas = 0;
		acelerador = ADC;
		acelerador_sub = (acelerador * 255) / 1022;
	}
	cont_5voltas++;
	Distancia_hodometro_km_pass += (Diametro_pneu_cm * PI);
	Distancia_hodometro_km = Distancia_hodometro_km_pass / 100000;
}

ISR(PCINT2_vect)  //Interrupção externa para os pinos D
{
	if(!(PIND & 0b00010000)) // botao [+] - PD4
	{
		if(Diametro_pneu_cm < 200)	// limitando o diametro max em 200cm
		{
			Diametro_pneu_cm++;
		}
	}
	if(!(PIND & 0b00100000)) // botao [-] - PD5
	{
		if(Diametro_pneu_cm > 1)	// limitando o diametro min em 1cm
		{
			Diametro_pneu_cm--;
		}
	}
	if(!(PIND & 0b00000001)) // botao [+] - PD4
	{
		Diametro_pneu_cm = 0;
		Distancia_hodometro_km_pass = 0;
	}
}

void anima_velocidade(uint16_t velocidade_carro, uint8_t *flag_disparo)
{
	static int8_t cont_dig = 0; // contador
	if(*flag_disparo)			// *flag_disparo = &flag_5ms (sempre que &flag_5ms for true ele dispara
	{
		switch(cont_dig)
		{
			case 0:
			PORTB &= 0b10000000; // resetando PB0 - PB6
			PORTB |= 0b01100000; // resetando PB4, pino que habilita o display das unidades
			PORTB |= (((velocidade_carro / 1) % 10) & 0b00001111); // separa o digito das unidades e coloca em PB0 - PB3
			break;

			case 1:
			PORTB &= 0b10000000; // resetando PB0 - PB6
			PORTB |= 0b01010000; // resetando PB5, pino que habilita o display das dezenas
			PORTB |= (((velocidade_carro / 10) % 10) & 0b00001111); // separa o digito das unidades e coloca em PB0 - PB3
			break;

			case 2:
			PORTB &= 0b10000000; // resetando PB0 - PB6
			PORTB |= 0b00110000; // resetando PB6, pino que habilita o display das centenas
			PORTB |= (((velocidade_carro / 100) % 10) & 0b00001111); // separa o digito das unidades e coloca em PB0 - PB3
			cont_dig = -1;
			break;
		}
		cont_dig++;
		*flag_disparo = 0;
	}
}

void anima_LCD(uint16_t diametro_pneu_cm, uint16_t rpm_motor, uint16_t distancia_hodometro_km, uint8_t *flag_disparo)
{
	if(*flag_disparo)	// *flag_disparo = &flag_500ms (sempre que &flag_500ms for true ele dispara
	{
		// variaveis locais para receber a conversao para string
		unsigned char diametro_pneu_cm_string[4];
		unsigned char rpm_motor_string[6];
		unsigned char distancia_hodometro_km_string[4];

		// sprintf recebe o valor uint16_t e converte para uma string
		sprintf(diametro_pneu_cm_string, "%u", Diametro_pneu_cm);
		sprintf(rpm_motor_string, "%u", RPM_motor);
		sprintf(distancia_hodometro_km_string, "%u", Distancia_hodometro_km);

		// configurandoo display nokia para mostrar as informacoes necessarias ao codigo, diametro, rpm, quilometragem...
		nokia_lcd_clear();		// limpa o display nokia
		nokia_lcd_set_cursor(0, 0);
		nokia_lcd_write_string("Comp. Bordo", 1);
		nokia_lcd_draw_Hline(10, 0 , 80);

		nokia_lcd_set_cursor(0, 15);
		nokia_lcd_write_string("Diam(cm): ", 1);
		nokia_lcd_write_string(diametro_pneu_cm_string, 1);

		nokia_lcd_set_cursor(0, 25);
		nokia_lcd_write_string("RPM: ", 1);
		nokia_lcd_write_string(rpm_motor_string, 1);

		nokia_lcd_set_cursor(40, 35);
		nokia_lcd_write_string(distancia_hodometro_km_string, 2);
		nokia_lcd_write_string("Km", 1);

		// condicao responsavel por identificar se o motor_dc se encontra P - parado, ou girando para D - direita, R - esquerda
		if(!(PIND & 0b10000000))
		{
			nokia_lcd_set_cursor(0, 35);
			nokia_lcd_write_string("[P]", 2);	// P - parado
		}
		if((PIND & 0b10000000))
		{
			if(!(PIND & 0b01000000))
			{
				nokia_lcd_set_cursor(0, 35);
				nokia_lcd_write_string("[D]", 2);	// D - direita
			}
			if((PIND & 0b01000000))
			{
				nokia_lcd_set_cursor(0, 35);
				nokia_lcd_write_string("[R]", 2);	// R - esquerda
			}
		}
		nokia_lcd_render(); // resetando o lcd
		*flag_disparo = 0;
	}
}
