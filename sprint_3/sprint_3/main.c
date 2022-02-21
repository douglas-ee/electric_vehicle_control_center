/*
 * sprint_4_tacometro_LCD.c
 * Author : Douglas dos Santos Gomes
 * Matricula : 118.111.102
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "nokia5110.h"
#include "configuracao.h"

// declarando variaveis globais
uint8_t flag_5ms = 0, flag_500ms = 0;
uint16_t Velocidade_carro_kmH = 123, RPM_motor = 0, Diametro_pneu_cm = 65;
uint32_t tempo_ms = 0;
float Distancia_hodometro_km = 0;
const float PI = 3.14161;

// chamando o prototipo das funcoes
void anima_velocidade(uint16_t velocidade_carro, uint8_t *flag_disparo);
void anima_LCD(uint16_t diametro_pneu_cm, uint16_t rpm_motor, float distancia_hodometro_km, uint8_t *flag_disparo);

ISR(TIMER0_COMPA_vect);
ISR(INT0_vect);
ISR(PCINT2_vect);

int main(void)
{
	configuracao();		// chamando funcao que configura
	nokia_lcd_init();	// inicia o displau nokia

	while (1)
	{
		anima_velocidade(Velocidade_carro_kmH, &flag_5ms);
		anima_LCD(Diametro_pneu_cm, RPM_motor, Distancia_hodometro_km, &flag_500ms);
	}
}

void anima_velocidade(uint16_t velocidade_carro, uint8_t *flag_disparo)
{
	static int8_t cont_dig = 0;
	if(*flag_disparo)
	{
		switch(cont_dig)
		{
		case 0:
			PORTB &= 0b10000000; //resetando PB0 - PB6
			PORTB |= 0b01100000; //resetando PB4, pino que habilita o display das unidades
			PORTB |= (((velocidade_carro / 1) % 10) & 0b00001111); // separa o digito das unidades e coloca em PB0 - PB3
			break;

		case 1:
			PORTB &= 0b10000000; //resetando PB0 - PB6
			PORTB |= 0b01010000; //resetando PB5, pino que habilita o display das dezenas
			PORTB |= (((velocidade_carro / 10) % 10) & 0b00001111); // separa o digito das unidades e coloca em PB0 - PB3
			break;

		case 2:
			PORTB &= 0b10000000; //resetando PB0 - PB6
			PORTB |= 0b00110000; //resetando PB6, pino que habilita o display das centenas
			PORTB |= (((velocidade_carro / 100) % 10) & 0b00001111); // separa o digito das unidades e coloca em PB0 - PB3
			cont_dig = -1;
			break;
		}
		cont_dig++;
		*flag_disparo = 0;
	}
}

void anima_LCD(uint16_t diametro_pneu_cm, uint16_t rpm_motor, float distancia_hodometro_km, uint8_t *flag_disparo)
{
	if(*flag_disparo)
	{
		unsigned char diametro_pneu_cm_string[4];
		unsigned char rpm_motor_string[6];
		unsigned char distancia_hodometro_km_string[8];

		sprintf(diametro_pneu_cm_string, "%u", diametro_pneu_cm);
		sprintf(rpm_motor_string, "%u", rpm_motor);
		sprintf(distancia_hodometro_km_string, "%u", (uint16_t)distancia_hodometro_km);

		nokia_lcd_clear();	// limpa o display nokia

		nokia_lcd_set_cursor(0, 0);
		nokia_lcd_write_string("Comp. Bordo: ", 1);
		nokia_lcd_draw_Hline(10, 0 , 80);

		nokia_lcd_set_cursor(0, 14);
		nokia_lcd_write_string("Dim(cm): ", 1);
		nokia_lcd_write_string(diametro_pneu_cm_string, 1);

		nokia_lcd_set_cursor(0, 24);
		nokia_lcd_write_string("RPM: ", 1);
		nokia_lcd_write_string(rpm_motor_string, 1);

		nokia_lcd_set_cursor(25, 34);
		nokia_lcd_write_string(distancia_hodometro_km_string, 2);
		nokia_lcd_write_string("Km: ", 1);

		nokia_lcd_render();
		*flag_disparo = 0;
	}
}

// incremendo do tempo
ISR(TIMER0_COMPA_vect)
{
	tempo_ms++;
	//PORTD ^= 0b01000000;
	if((tempo_ms % 5) == 0) //true a cada 5ms
		flag_5ms = 1;
	if((tempo_ms % 500) == 0) //true a cada 500ms
		flag_500ms = 1;
}

ISR(INT0_vect)
{
	static uint8_t cont_5voltas = 0;
	static uint32_t tempo_ms_anterior = 0;
	uint16_t delta_t_ms = 0;

	if(cont_5voltas == 5)
	{
		delta_t_ms = tempo_ms - tempo_ms_anterior;
		RPM_motor = 300000 / (delta_t_ms); //(5voltas*60min*1000ms)/delta
		Velocidade_carro_kmH = ((uint32_t)Diametro_pneu_cm * 565) / delta_t_ms; //(PI*3.6)/(60*100)
		tempo_ms_anterior = tempo_ms;
		cont_5voltas = 0;
	}
	cont_5voltas++;
	Distancia_hodometro_km += ((float)Diametro_pneu_cm * 3.1415) / 100000;
}

ISR(PCINT2_vect) // interrupção por mudança de pino na porta D (PD3 e PD4) - BOTÂO + ou -
{
	if(!(PIND & 0b00010000) == 0) // BOTÃO + PD3
	{
		if(Diametro_pneu_cm < 200)
			Diametro_pneu_cm++;
	}

	if(!(PIND & 0b00001000) == 0) // BOTÃO - PD4
	{
		if(Diametro_pneu_cm > 0)
			Diametro_pneu_cm--;
	}
}
