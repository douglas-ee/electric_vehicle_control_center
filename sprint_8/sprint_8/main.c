/*
	Aluno: Douglas dos Santos Gomes
	Matricula: 118.111.102
*/

#define F_CPU 16000000UL
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>
#include "SSD1306.h"
#include "Font5x8.h"
#include "configuracao.h"
#include <avr/eeprom.h>

// declarando variaveis globais
uint8_t flag_5ms = 0, flag_500ms = 0, flag_over_flow = 0;
uint16_t Velocidade_carro_kmH = 0, RPM_motor = 0, Diametro_pneu_cm = 65, leitura_ADC = 0, Distancia_hodometro_km = 0,  RPM_motor_pass = 0, Distancia_objeto_cm = 0, bateria_per = 0, temperatura_C = 0;
uint32_t tempo_up = 0, tempo_down = 0, tempo_ms = 0, acelerador = 0, acelerador_sub = 0, Distancia_hodometro_km_pass = 0;
const float PI = 3.14161;

// declarando os prototipos das funcoes
ISR(TIMER0_COMPA_vect);
ISR(TIMER1_OVF_vect);
ISR(TIMER1_CAPT_vect);
ISR(INT0_vect);
ISR(PCINT2_vect);
void anima_velocidade(uint16_t velocidade_carro, uint8_t *flag_disparo);
void anima_LCD(uint16_t diametro_pneu_cm, uint16_t rpm_motor, uint16_t distancia_hodometro_km, uint16_t distancia_objeto_cm, uint8_t *flag_disparo);
void anima_ADC(uint16_t *bateria, uint16_t *temperatura, uint8_t *flag_disparo);

int main(void)
{	
	// variaveis locais
	unsigned char aux_eeprom[8];
	unsigned char aux_2_eeprom[8];

	configuracao();		// chamando funcao que configura GPIO, interruptores, timers, ADC, PWM
	
	// loop responsavel por separar e ler os dados do diametro que serao armazenados na eeprom, nos espacos 0 ate 3
	for (int i = 0; i < 4; i++)
	{
		aux_eeprom[i] = eeprom_read_byte(i);
		Diametro_pneu_cm = atoi(aux_eeprom); // convertendo de char para int
	}

	// loop responsavel por separar e ler os dados do distancia que serao armazenados na eeprom, nos espacos 4 ate 8
	for (int i = 4; i < 9; i++)
	{
		aux_eeprom[i - 4] = eeprom_read_byte(i);
		Distancia_hodometro_km = atoi(aux_eeprom); // convertendo de char para int
	}
	Distancia_hodometro_km_pass = Distancia_hodometro_km * 100000;

	while (1)
	{
		anima_velocidade(Velocidade_carro_kmH, &flag_5ms);		// funcao responsavel por retornar a velocidade no display 7-Seg
		anima_LCD(Diametro_pneu_cm, RPM_motor, Distancia_hodometro_km, Distancia_objeto_cm, &flag_500ms);	// funcao responsavel por retornar variaveis para o display nokia

		itoa(Diametro_pneu_cm, aux_2_eeprom, 10); // convertendo de int para char
		// loop responsavel por separar e inserir os dados do diametro na eeprom, nos espacos 0 ate 3
		for (int i = 0; i < 4; i++)
		{
			eeprom_write_byte(i, aux_2_eeprom[i]);
		}
		itoa(Distancia_hodometro_km, aux_2_eeprom, 10); // convertendo de int para char
		// loop responsavel por separar e inserir os dados da distancia na eeprom, nos espacos 4 ate 8
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
	if((tempo_ms % 5) == 0)		//true a cada 5ms
		flag_5ms = 1;
	if((tempo_ms % 500) == 0)	//true a cada 500ms
		flag_500ms = 1;
}

ISR(TIMER1_OVF_vect)
{
	flag_over_flow = 1;
}

ISR(TIMER1_CAPT_vect)
{
	if(TCCR1B & (1 << ICES1))
		tempo_up = ICR1;
	else
		tempo_down = ((ICR1 - tempo_up) * 16);

	TCCR1B ^= (1 << ICES1);

	if(flag_over_flow == 0)
		Distancia_objeto_cm = (tempo_down / 58);
	else
		flag_over_flow = 0;
}

ISR(INT0_vect) // tacometro
{
	// variaveis locais
	static uint8_t cont_5voltas = 0;
	static uint32_t tempo_ms_anterior = 0;
	uint16_t delta_t_ms = 0;

	if(cont_5voltas == 5) // se cont_5voltas == 5, faca true
	{
		delta_t_ms = tempo_ms - tempo_ms_anterior;									// equacao delta_t = t1 - t0
		tempo_ms_anterior = tempo_ms;
		RPM_motor = 300000 / (delta_t_ms);											// equacao rpm = (5voltas*60min*1000ms)/delta_t
		Velocidade_carro_kmH = ((uint32_t)Diametro_pneu_cm * 565) / delta_t_ms;		// equacao velocidade = (PI*3.6)/(60*100)
		tempo_ms_anterior = tempo_ms;												// armazenando valor do tempo
		cont_5voltas = 0;
	}
	if(Velocidade_carro_kmH > 20 && Distancia_objeto_cm < 300)
		acelerador_sub = (acelerador * 255) / 10230;								// equacao de conversao para o potenciometro
	else
		acelerador_sub = (acelerador * 255) / 1023;									// equacao de conversao para o potenciometro

	cont_5voltas++;
	Distancia_hodometro_km_pass += (Diametro_pneu_cm * PI);							// equacao da distancia
	Distancia_hodometro_km = Distancia_hodometro_km_pass / 100000;					// equacao de distancia passando valores
}

ISR(PCINT2_vect)  // interrupcao externa para os pinos D
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
	// condicao para zerar os valores, apenas teste
	if(!(PIND & 0b00000001)) // botao [0] - PD0
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
			PORTB &= 0b00000001; // resetando PB1 - PB7
			PORTB |= 0b11000000; // resetando PB4, pino que habilita o display das unidades
			PORTB |= ((((velocidade_carro / 1) % 10) & 0b00001111) << 1); // separa o digito das unidades e coloca em PB0 - PB3
			break;

		case 1:
			PORTB &= 0b00000001; // resetando PB1 - PB7
			PORTB |= 0b10100000; // resetando PB5 e PB7, pino que habilita o display das dezenas
			PORTB |= ((((velocidade_carro / 10) % 10) & 0b00001111) << 1); // separa o digito das unidades e coloca em PB0 - PB3
			break;

		case 2:
			PORTB &= 0b00000001; // resetando PB1 - PB7
			PORTB |= 0b01100000; // resetando PB5 e PB7, pino que habilita o display das centenas
			PORTB |= ((((velocidade_carro / 100) % 10) & 0b00001111) << 1); // separa o digito das unidades e coloca em PB0 - PB3
			cont_dig = -1;
			break;
		}
		cont_dig++;
		*flag_disparo = 0;
	}
}

void anima_LCD(uint16_t diametro_pneu_cm, uint16_t rpm_motor, uint16_t distancia_hodometro_km, uint16_t distancia_objeto_cm, uint8_t *flag_disparo)
{
	// chamada do prototipo da funcao
	anima_ADC(&bateria_per, &temperatura_C, &flag_5ms);

	if(*flag_disparo)	// *flag_disparo para normalizar no tempo preciso desejado
	{

		// iniciando o SSD1306 e configurando o display 
		GLCD_Setup();
		GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
		GLCD_InvertScreen();

		GLCD_Clear();

		GLCD_GotoXY(3, 2);
		GLCD_PrintString("LASD Car");
		GLCD_DrawRectangle(1, 1, 50, 10 ,GLCD_Black);

		GLCD_GotoXY(2, 16);
		GLCD_PrintInteger(rpm_motor);
		GLCD_PrintString(" rpm");
	
		GLCD_GotoXY(2, 27);
		GLCD_PrintString("Sonar: ");
		GLCD_PrintInteger(distancia_objeto_cm);
		GLCD_PrintString("cm");

		GLCD_GotoXY(2, 40);
		GLCD_PrintString("D. Pneu: ");
		GLCD_PrintInteger(diametro_pneu_cm);
		GLCD_PrintString("cm");
						
		GLCD_GotoXY(20, 53);
		GLCD_PrintInteger(distancia_hodometro_km);
		GLCD_PrintString("km");
		GLCD_DrawRectangle(10, 50, 58, 62 ,GLCD_Black);

		GLCD_DrawRectangle(90, 2, 125, 32 ,GLCD_Black);
		GLCD_GotoXY(93, 8);
		GLCD_PrintInteger(bateria_per);
		GLCD_PrintString(" %");

		GLCD_GotoXY(93, 21);
		GLCD_PrintInteger(temperatura_C);
		GLCD_PrintString(" C");

		GLCD_DrawRectangle(90, 35, 125, 60 ,GLCD_Black);
		GLCD_DrawRectangle(94, 39, 121, 56 ,GLCD_Black);

		// condicao responsavel por identificar se o motor_dc se encontra P - parado, ou girando para D - direita, E - esquerda
		if(!(PIND & 0b10000000))
		{
			GLCD_GotoXY(106, 45);
			GLCD_PrintChar('P');	// P - parado
		}
		if((PIND & 0b10000000))
		{
			if(!(PIND & 0b01000000))
			{
				GLCD_GotoXY(106, 45);
				GLCD_PrintChar('D');	// D - direita
			}
			if((PIND & 0b01000000))
			{
				GLCD_GotoXY(106, 45);
				GLCD_PrintChar('E');	// E - esquerda
			}
		}
		*flag_disparo = 0;
		GLCD_Render();
	}
}

void anima_ADC(uint16_t *bateria, uint16_t *temperatura, uint8_t *flag_disparo)
{
	static uint8_t cont_dig = 0;
	if (*flag_disparo)
	{
		switch(cont_dig)
		{
		case 0: // canal 0
			ADMUX = 0b01000000; // ADMUX 0
			acelerador = ADC;
			OCR2B = acelerador_sub;
			break;

		case 1: // canal 1
			ADMUX = 0b01000001; // ADMUX 0
			*bateria = (((double)100/1023) * ADC);
			break;

		case 2: // canal 2
			ADMUX = 0b01000010; // ADMUX 0
			// equacao de tensao		[Vt = (5/1023)*ADC]
			// equacao de resistencia	[Rt = (1000*Vt)/(5 - Vt)]
			// equacao de temperatura	[T = 2,6*Rt - 260]
			*temperatura = (((double)2.6*(((double)1000*(((double)5/1023)*ADC))/(5 - (((double)5/1023)*ADC)))) - 260);
			cont_dig = -1;
			break;
		}
		cont_dig++;
		*flag_disparo = 0;
	}
}

