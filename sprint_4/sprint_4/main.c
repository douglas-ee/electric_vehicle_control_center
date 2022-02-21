/*
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
uint16_t Velocidade_carro_kmH = 0, RPM_motor = 0, Diametro_pneu_cm = 65, leitura_ADC = 0;
uint32_t tempo_ms = 0, acelerador = 0, acelerador_sub = 0;
float Distancia_hodometro_km = 0;
const float PI = 3.14161;

// chamando o prototipo das funcoes
void anima_velocidade(uint16_t velocidade_carro, uint8_t *flag_disparo);
void anima_LCD(uint16_t diametro_pneu_cm, uint16_t rpm_motor, float distancia_hodometro_km, uint8_t *flag_disparo);

// declarando prototipo das funcoes
ISR(TIMER0_COMPA_vect);
ISR(INT0_vect);
ISR(PCINT2_vect);
ISR(ADC_vect);
void anima_velocidade(uint16_t velocidade_carro, uint8_t *flag_disparo);
void anima_LCD(uint16_t diametro_pneu_cm, uint16_t rpm_motor, float distancia_hodometro_km, uint8_t *flag_disparo);

int main(void)
{
	configuracao();		// chamando funcao que configura GPIO, interruptores, timers, ADC, PWM
	nokia_lcd_init();	// inicia o display nokia

	while (1)
	{
		OCR2B = acelerador_sub;		// ajustando o comparador para TC2 contar ate acelerador_sub
		anima_velocidade(Velocidade_carro_kmH, &flag_5ms);		// funcao responsavel por retornar a velocidade no display 7-Seg
		anima_LCD(Diametro_pneu_cm, RPM_motor, Distancia_hodometro_km, &flag_500ms);	// funcao responsavel por retornar variaveis para o display nokia
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

void anima_LCD(uint16_t diametro_pneu_cm, uint16_t rpm_motor, float distancia_hodometro_km, uint8_t *flag_disparo)
{
	if(*flag_disparo)	// *flag_disparo = &flag_500ms (sempre que &flag_500ms for true ele dispara
	{
		// variaveis locais para receber a conversao para string
		unsigned char diametro_pneu_cm_string[4];
		unsigned char rpm_motor_string[6];
		unsigned char distancia_hodometro_km_string[8];

		// sprintf recebe o valor uint16_t e converte para uma string
		sprintf(diametro_pneu_cm_string, "%u", diametro_pneu_cm);
		sprintf(rpm_motor_string, "%u", rpm_motor);
		sprintf(distancia_hodometro_km_string, "%u", (uint16_t)distancia_hodometro_km);

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

// incremendo do tempo com flags
ISR(TIMER0_COMPA_vect)
{
	tempo_ms++;
	//PORTD ^= 0b01000000;
	if((tempo_ms % 5) == 0)		//true a cada 5ms
		flag_5ms = 1;
	if((tempo_ms % 500) == 0)	//true a cada 500ms
		flag_500ms = 1;
}

// bloco de equacoes que devem rodam disparo INT0 na borda de descida
ISR(INT0_vect)
{
	// variaveis locais necessarias para o programa
	static uint8_t cont_5voltas = 0;
	static uint32_t tempo_ms_anterior = 0;
	uint16_t delta_t_ms = 0;

	//true em cont_5voltas == 5
	if(cont_5voltas == 5)
	{
		delta_t_ms = tempo_ms - tempo_ms_anterior;									// equacao delta_t = t1 - t0
		RPM_motor = 300000 / (delta_t_ms);											// equacao rpm = (5voltas*60min*1000ms)/delta_t
		Velocidade_carro_kmH = ((uint32_t)Diametro_pneu_cm * 565) / delta_t_ms;		// equacao velocidade = (PI*3.6)/(60*100)
		tempo_ms_anterior = tempo_ms;												// armazenando valor do tempo
		cont_5voltas = 0;
		acelerador = ADC;															// armazenando valor da aceleracao que recebeu do ADC
		acelerador_sub = (acelerador * 255) / 1022;									// equacao de conversao para o potenciometro
	}
	cont_5voltas++;
	Distancia_hodometro_km += ((float)Diametro_pneu_cm * PI) / 100000;				// equacao para achar a distancia percorrida
}

// interrupcao por mudanca de pino na porta D -(PD4 e PD5)
ISR(PCINT2_vect)
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
}

ISR(ADC_vect)
{
	leitura_ADC = ADC;
}
