/*
Aluno: Douglas dos Santos Gomes
Matricula: 118.111.102
*/

#define F_CPU 1600000UL
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
	//variaveis necessarias para o display e contagens das dezenas e centenas
	uint8_t velocidade_veiculo[10] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09}, cont_1 = 0, cont_2 = 0;

	//habilitando portas de entrada e saida para o display e botoes
	DDRC = 0b00000011;
	DDRB = 0b00001111;
	PORTD |= (1 << 0);
	PORTD |= (1 << 1);

	//laco para contagem do velocimetro
	while (1)
	{
		//testa o botao (+) e incrementa as dezenas e centenas em cont_1 e cont_2 respectivamente
		if(!(PIND & (1 << 0)))
		{
			if(cont_2 <= 2)
			cont_1++;
			if(cont_1 == 10)
			{
				cont_1 = 0;
				cont_2++;
			}
		}

		//testa o botao (-) e decrementa as dezenas e centenas em cont_1 e cont_2 respectivamente
		if(!(PIND & (1 << 1)))
		{
			if(cont_1 == 0 && cont_2 != 0)
			{
				cont_2--;
				cont_1 = 10;
			}
			else
			cont_1--;
		}

		_delay_ms(1000);
		PORTC  = velocidade_veiculo [cont_2];
		PORTB  = velocidade_veiculo [cont_1];
		PORTD |= velocidade_veiculo[0];
	}
}

