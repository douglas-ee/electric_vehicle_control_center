/*
 * Author : Douglas dos Santos Gomes
 * Matricula: 118.111.102
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//variavel global
uint16_t velocidade_carro = 0;

//interrupcao externa INT0
ISR(INT0_vect)
{
	if(velocidade_carro < 300)
		velocidade_carro += 10;
}
//interrupcao externa INT1
ISR(INT1_vect)
{
	if(velocidade_carro > 0)
		velocidade_carro -= 10;
}
//prototipos
void anima_velocidade(uint16_t velocidade_carro);

int main(void)
{
	//GPIO
	DDRB |= 0b11111111;	//habilita os pinos
	DDRD &= 0b11110011; //PD2 e PD3 como entrada
	PORTD = 0b00001100; //PullUp PD2 e PD3
	//configuracao das interrupcoes
	EICRA = 0b00001010;
	EIMSK = 0b00000011;
	sei();
	//chamada da funcao
	while (1)
	{
		anima_velocidade(velocidade_carro);
	}
}

void anima_velocidade(uint16_t velocidade_carro)
{
	PORTB &= 0b00000000;
	PORTB = ((velocidade_carro / 1) % 10) | (0b110 << 4); //carrega a unidade
	_delay_ms(1);
	PORTB = ((velocidade_carro / 10) % 10) | (0b101 << 4); //carrega a dezena
	_delay_ms(1);
	PORTB = (velocidade_carro / 100) | (0b011 << 4); //carrega a centena
	_delay_ms(1);
}
