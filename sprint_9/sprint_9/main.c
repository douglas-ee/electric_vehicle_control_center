/*
	Aluno: Douglas dos Santos Gomes
	Matricula: 118.111.102
*/

// declaracoes das bibliotecas e defines necessarios
#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "SSD1306.h"
#include "Font5x8.h"
#include "configuracao.h"
#include <avr/eeprom.h>

// declarando variaveis globais
uint8_t flag_5ms = 0, flag_500ms = 0, flag_over_flow = 0;
uint16_t Velocidade_carro_kmH = 0, RPM_motor = 0, Diametro_pneu_cm = 65, leitura_ADC = 0, Distancia_hodometro_km = 0,  RPM_motor_pass = 0, Distancia_objeto_cm = 0, Bateria_per = 0, Temperatura_C = 0, temperatura_pass = 0;
uint32_t tempo_up = 0, tempo_down = 0, tempo_ms = 0, acelerador = 0, acelerador_sub = 0, Distancia_hodometro_km_pass = 0;
const float PI = 3.14161;
unsigned static char aux_eeprom[18];
unsigned static char aux_2_eeprom[18];

// declarando os prototipos das funcoes
ISR(USART_RX_vect);
ISR(TIMER0_COMPA_vect);
ISR(TIMER1_OVF_vect);
ISR(TIMER1_CAPT_vect);
ISR(INT0_vect);
ISR(PCINT2_vect);
void usart_init(unsigned int ubrr);
void usart_transmit(unsigned char data);
unsigned char usart_receive(void);
void anima_velocidade(uint16_t velocidade_carro, uint8_t *flag_disparo);
void anima_LCD(uint16_t diametro_pneu_cm, uint16_t rpm_motor, uint16_t distancia_hodometro_km, uint16_t distancia_objeto_cm, uint8_t *flag_disparo);
void anima_ADC(uint16_t *bateria, uint16_t *temperatura, uint8_t *flag_disparo);

int main(void)
{
	// chamada da funcao usart de inicializacao
	usart_init(MYUBRR);

	// chamando funcao que configura GPIO, interruptores, timers, ADC, PWM
	configuracao();

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

	// loop responsavel por separar e ler os dados da temperatura que serao armazenados na eeprom, nos espacos 9 ate 12
	for (int i = 9; i < 13; i++)
	{
		aux_eeprom[i - 9] = eeprom_read_byte(i);
		Temperatura_C = atoi(aux_eeprom); // convertendo de char para int
	}

	// loop responsavel por separar e ler os dados da bateria que serao armazenados na eeprom, nos espacos 13 ate 16
	for (int i = 13; i < 17; i++)
	{
		aux_eeprom[i - 13] = eeprom_read_byte(i);
		Bateria_per = atoi(aux_eeprom); // convertendo de char para int
	}

	while (1)
	{
		// chamada da funcao responsavel por retornar a velocidade no display 7-Seg com flag_5ms
		anima_velocidade(Velocidade_carro_kmH, &flag_5ms);
		
		// funcao responsavel por retornar variaveis para o display nokia com flag_500ms
		anima_LCD(Diametro_pneu_cm, RPM_motor, Distancia_hodometro_km, Distancia_objeto_cm, &flag_500ms);
		
		itoa(Diametro_pneu_cm, aux_2_eeprom, 10);	// convertendo de int para char
		// loop responsavel por separar e inserir os dados do diametro na eeprom, nos espacos 0 ate 3
		for (int i = 0; i < 4; i++)
		{
			eeprom_write_byte(i, aux_2_eeprom[i]);
		}
		itoa(Distancia_hodometro_km, aux_2_eeprom, 10); // convertendo de int para char
		// loop responsavel por separar e inserir os dados da distancia na eeprom, nos espacos 4 ate 8
		for (int i = 4; i < 9; i++)
		{
			eeprom_write_byte(i, aux_2_eeprom[i - 4]);
		}
		
		// condicao responsavel por limitar a isercao dos dados da temperatura na eeprom quando forem maiores que o anterior
		if(Temperatura_C > temperatura_pass)
		{
			itoa(Temperatura_C, aux_2_eeprom, 10); // convertendo de int para char
			// loop responsavel por separar e inserir os dados da temperatura na eeprom, nos espacos 9 ate 12
			for (int i = 9; i < 13; i++)
			{
				eeprom_write_byte(i, aux_2_eeprom[i - 9]);
			}
		}
		temperatura_pass = (unsigned int)Temperatura_C;

		itoa(Bateria_per, aux_2_eeprom, 10); // convertendo de int para char
		// loop responsavel por separar e inserir os dados da bateria na eeprom, nos espacos 13 ate 16
		for (int i = 13; i < 17; i++)
		{
			eeprom_write_byte(i, aux_2_eeprom[i - 13]);
		}
	}
}

ISR(USART_RX_vect)
{
	// variavel responsavel por receber os dados transmitidos ao UDR0
	char recebido;
	recebido = UDR0;
	
	// condicao responsavel por retornar os valores da eeprom
	if(recebido == 'l')		// se recebido == l, zera o valor da temperatura maxima na eeprom
	{
		//usart_transmit(Bateria_per);
		for (int i = 9; i < 13; i++)
		{
			eeprom_write_byte(i, aux_2_eeprom[i] = 0);
		}
	}
	if(recebido == 'd')		// se recebido == d, sera transmitido pela usart o valor da temperatura
	{
		for (int i = 9; i < 13; i++)
		{
			usart_transmit(eeprom_read_byte(i));
		}
	}
}

// chamada da funcao responsavel por criar as flags de tempo
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

// chamada da funcao responsavel pelos calculos do tacometro no INT0 na borda de descida
ISR(INT0_vect)
{
	// variaveis locais
	static uint8_t cont_5voltas = 0;
	static uint32_t tempo_ms_anterior = 0;
	uint16_t delta_t_ms = 0;

	// condicao responsavel pelo acionamento das equacoes sempre que o cont_5voltas == 5
	if(cont_5voltas == 5)
	{
		delta_t_ms = tempo_ms - tempo_ms_anterior;									// equacao delta_t = t1 - t0
		tempo_ms_anterior = tempo_ms;												// atribuindo valores entre variaveis
		RPM_motor = 300000 / (delta_t_ms);											// equacao rpm = (5voltas*60min*1000ms)/delta_t
		Velocidade_carro_kmH = ((uint32_t)Diametro_pneu_cm * 565) / delta_t_ms;		// equacao velocidade = (PI*3.6)/(60*100)
		tempo_ms_anterior = tempo_ms;												// armazenando valor do tempo
		cont_5voltas = 0;															// zerando o contador
	}
	
	// condicao responsavel por limitar o potenciometro em um intervalo projetado
	if(Velocidade_carro_kmH > 20 && Distancia_objeto_cm < 300)
		acelerador_sub = (acelerador * 255) / 10230;								// equacao de conversao para o potenciometro
	else
		acelerador_sub = (acelerador * 255) / 1023;									// equacao de conversao para o potenciometro

	cont_5voltas++;																	// contador
	Distancia_hodometro_km_pass += (Diametro_pneu_cm * PI);							// equacao da distancia
	Distancia_hodometro_km = Distancia_hodometro_km_pass / 100000;					// equacao de distancia passando valores
}

// interrupcao externa para os pinos D - PD0, PD4 e PD5
ISR(PCINT2_vect)
{
	 // condicao do botao [+] - PD4, ativada sempre que for true
	if(!(PIND & 0b00010000))
	{
		if(Diametro_pneu_cm < 200)		// limitando o diametro max em 200cm
		{
			Diametro_pneu_cm++;			// adicionando 1 no contador Diametro_pneu_cm
		}
	}
	 // condicao do botao [-] - PD5, ativada sempre que for true
	if(!(PIND & 0b00100000))
	{
		if(Diametro_pneu_cm > 1)		// limitando o diametro min em 1cm
		{
			Diametro_pneu_cm--;			// reduzindo 1 no contador Diametro_pneu_cm
		}
	}
	 // condicao do botao [0] - PD0, ativada sempre que for true
	if(!(PIND & 0b00000001))
	{
		// zerando os valores salvos na eeprom no display
		Diametro_pneu_cm = 0;
		Distancia_hodometro_km_pass = 0;
		Temperatura_C = 0;
		Bateria_per = 0;
	}
}

// funcao responsavel pela inicializacao da USART
void usart_init(unsigned int ubrr)
{
	UBRR0H = (unsigned char)(ubrr >> 8);						// ajusta a taxa de transmissao
	UBRR0L = (unsigned char)ubrr;								// atribuindo ubrr tipo unsigned char em UBRR0L
	//UCSR0B = (1<<RXEN0)|(1<<TXEN0);			// habilita o transmissor e o receptor
	UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);		// habilita o transmissor e o receptor
	UCSR0C = (1 << USBS0) | (3 << UCSZ00);						// ajusta o formato do frame: 8 bits de dados e 2 de parada
}

// funcao responsavel pelo envio de um frame de 5 a 8 bits
void usart_transmit(unsigned char data)
{
	while(!(UCSR0A & (1 << UDRE0)));							// espera a limpeza do registrador de transmissao
	UDR0 = data;												// coloca o dado no registrador e envia
}

// funcao responsavel pela recepção de um frame de 5 a 8 bits
unsigned char usart_receive(void)
{
	while(!(UCSR0A & (1 << RXC0)));								// espera o dado ser recxebido
	return UDR0;												// le o dado recebido e retorna
}

// funcao responsavel por configurar e mostrar o display de 7-Seg na simulacao
void anima_velocidade(uint16_t velocidade_carro, uint8_t *flag_disparo)
{
	// declarando variavel local
	static int8_t cont_dig = 0;
	// condicao com *flag_disparo = &flag_5ms (sempre que &flag_5ms for true ele dispara
	if(*flag_disparo)
	{
		// switch case responsavel por dividir o acionamento em 3 partes
		switch(cont_dig)
		{
		case 0:
			PORTB &= 0b00000001;												// resetando PB1 - PB7
			PORTB |= 0b11000000;												// resetando PB4, pino que habilita o display das unidades
			PORTB |= ((((velocidade_carro / 1) % 10) & 0b00001111) << 1);		// separa o digito das unidades e coloca em PB0 - PB3
			break;

		case 1:
			PORTB &= 0b00000001;												// resetando PB1 - PB7
			PORTB |= 0b10100000;												// resetando PB5 e PB7, pino que habilita o display das dezenas
			PORTB |= ((((velocidade_carro / 10) % 10) & 0b00001111) << 1);		// separa o digito das unidades e coloca em PB0 - PB3
			break;

		case 2:
			PORTB &= 0b00000001;												// resetando PB1 - PB7
			PORTB |= 0b01100000;												// resetando PB5 e PB7, pino que habilita o display das centenas
			PORTB |= ((((velocidade_carro / 100) % 10) & 0b00001111) << 1);		// separa o digito das unidades e coloca em PB0 - PB3
			cont_dig = -1;
			break;
		}
		cont_dig++;
		*flag_disparo = 0;
	}
}

// funcao responsavel por configurar e mostrar o LCD SSD1306
void anima_LCD(uint16_t diametro_pneu_cm, uint16_t rpm_motor, uint16_t distancia_hodometro_km, uint16_t distancia_objeto_cm, uint8_t *flag_disparo)
{
	// chamada do prototipo da funcao ADC
	anima_ADC(&Bateria_per, &Temperatura_C, &flag_5ms);

	// condicao com *flag_disparo = &flag_500ms (sempre que &flag_5ms for true ele dispara
	if(*flag_disparo)
	{
		// iniciando o SSD1306 e configurando o display
		GLCD_Setup();											// chamada do setup() para inicializar o display
		GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);			// chamada do setFont para chamar o tamanho, cor e tipo de fonte
		GLCD_InvertScreen();
		GLCD_Clear();											// limpando o display
		GLCD_GotoXY(3, 2);										// demarcando a posicao da string seguinte
		GLCD_PrintString("LASD Car");							// printando uma string no display
		GLCD_DrawRectangle(1, 1, 50, 10 , GLCD_Black);			// printando um retagulo no display de borda preta em dada posicao desejada
		GLCD_GotoXY(2, 16);										// demarcando a posicao da string seguinte
		GLCD_PrintInteger(rpm_motor);							// printando uma variavel no display
		GLCD_PrintString(" rpm");								// printando uma string no display
		GLCD_GotoXY(2, 27);										// demarcando a posicao da string seguinte
		GLCD_PrintString("Sonar: ");							// printando uma string no display
		GLCD_PrintInteger(distancia_objeto_cm);					// printando uma variavel no display
		GLCD_PrintString("cm");									// printando uma string no display
		GLCD_GotoXY(2, 40);										// demarcando a posicao da string seguinte
		GLCD_PrintString("D. Pneu: ");							// printando uma string no display
		GLCD_PrintInteger(diametro_pneu_cm);					// printando uma variavel no display
		GLCD_PrintString("cm");									// printando uma string no display
		GLCD_GotoXY(20, 53);									// demarcando a posicao da string seguinte
		GLCD_PrintInteger(distancia_hodometro_km);				// printando uma variavel no display
		GLCD_PrintString("km");									// printando uma string no display
		GLCD_DrawRectangle(10, 50, 58, 62 , GLCD_Black);		// printando um retagulo no display de borda preta em dada posicao desejada
		GLCD_DrawRectangle(90, 2, 125, 32 , GLCD_Black);		// printando um retagulo no display de borda preta em dada posicao desejada
		GLCD_GotoXY(93, 8);										// demarcando a posicao da string seguinte
		GLCD_PrintInteger(Bateria_per);							// printando uma variavel no display
		GLCD_PrintString(" %");									// printando uma string no display
		GLCD_GotoXY(93, 21);									// demarcando a posicao da string seguinte
		GLCD_PrintInteger(Temperatura_C);						// printando uma variavel no display
		GLCD_PrintString(" C");									// printando uma string no display
		GLCD_DrawRectangle(90, 35, 125, 60 , GLCD_Black);		// printando um retagulo no display de borda preta em dada posicao desejada
		GLCD_DrawRectangle(94, 39, 121, 56 , GLCD_Black);		// printando um retagulo no display de borda preta em dada posicao desejada

		// condicao responsavel por identificar se o motor_dc se encontra [P] - parado, ou girando para [D] - direita, [E] - esquerda
		if(!(PIND & 0b10000000))
		{
			GLCD_GotoXY(106, 45);								// demarcando a posicao da string seguinte
			GLCD_PrintChar('P');								// printando uma variavel [P] no display
		}
		if((PIND & 0b10000000))
		{
			if(!(PIND & 0b01000000))
			{
				GLCD_GotoXY(106, 45);							// demarcando a posicao da string seguinte
				GLCD_PrintChar('D');							// printando uma variavel [D] no display
			}
			if((PIND & 0b01000000))
			{
				GLCD_GotoXY(106, 45);							// demarcando a posicao da string seguinte
				GLCD_PrintChar('E');							// printando uma variavel [E] no display
			}
		}
		*flag_disparo = 0;
		GLCD_Render();
	}
}

// funcao responsavel por configurar o ADC da aceleracao, temperatura e bateria
void anima_ADC(uint16_t *bateria, uint16_t *temperatura, uint8_t *flag_disparo)
{
	// declarando variavel local
	static uint8_t cont_dig = 0;
	
	// condicao com *flag_disparo = &flag_5ms (sempre que &flag_5ms for true ele dispara
	if (*flag_disparo)
	{
		// switch case responsavel por dividir o acionamento em 3 partes
		switch(cont_dig)
		{
		case 0: // canal 0 - ADMUX funciona como acelerador 
			ADMUX = 0b01000000; // ADMUX 0
			acelerador = ADC;
			OCR2B = acelerador_sub;
			break;

		case 1: // canal 1 - ADMUX funciona como bateria
			ADMUX = 0b01000001; // ADMUX 1
			*bateria = (((double)100 / 1023) * ADC);
			break;

		case 2: // canal 2 - ADMUX funciona como acelerador
			ADMUX = 0b01000010; // ADMUX 2
			// equacao de tensao		[Vt = (5/1023)*ADC]
			// equacao de resistencia	[Rt = (1000*Vt)/(5 - Vt)]
			// equacao de temperatura	[T = 2,6*Rt - 260]
			*temperatura = (((double)2.6 * (((double)1000 * (((double)5 / 1023) * ADC)) / (5 - (((double)5 / 1023) * ADC)))) - 260);
			cont_dig = -1;
			break;
		}
		cont_dig++;
		*flag_disparo = 0;
	}
}