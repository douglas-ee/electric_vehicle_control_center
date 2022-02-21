#ifndef _CONFIGURACAO_H_
#define _CONFIGURACAO_H_

void configuracao()
{
	//DEFINIÇÃO DA GPIO
	
	//direção dos pinos
	DDRB	|= 0b01111111;	// habilita os pinos PD0..6 como saidas
	DDRD	&= 0b11100011;	// habilita os pinos PD2 e PD3 como entradas
	
	//pull ups das entradas
	PORTD	= 0b00011100;	// habilita o resistor de Pull-up de PD2..3

	// configuracao das interrupções externas
	EICRA	= 0b00000010; // interrupçõ eternsa INT0 na bord de descida
	EIMSK	= 0b00000001; // habilita a interrupcao externa INT0
	PCICR	= 0b00000100; // habilita interrupção PIN change 2 (porta D)
	PCMSK2	= 0b00011000; // habilita interrupção PIN chance PD3 e PD4
	
	//configuração do timer 0
	TCCR0A	= 0b00000010;	// habilita modo CTC do TC0
	TCCR0B	= 0b00000011;	// liga TC0 com prescaler = 64
	OCR0A	= 249;			// ajusta o comparador para TC0 contar ate 249.
	TIMSK0	= 0b00000010;	// habilita a iinterrupcao na igualdade de comparacao com OCR0A a cada 1ms = (249 + 1)*64/16MHz

	sei();
}

#endif
