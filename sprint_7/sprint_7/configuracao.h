#ifndef _CONFIGURACAO_H_
#define _CONFIGURACAO_H_

void configuracao()
{
	// definicao da GPIO
	DDRC	|= 0b00100000;	// Habilitando o PC5 para o potenci?metro e restante como saida para o display
	DDRB	|= 0b11111110;	// habilita os pinos PD0..6 como saidas
	DDRD	&= 0b11000010;	// habilita os pinos PD2 ate PD5 como entradas

	// pull ups das entradas
	PORTD	= 0b00111101;	// habilita o resistor de Pull-up de PD2 ate PD5

	// configuracao das interrupoes externas
	EICRA	= 0b10000010;	// interrupao externa INT0 e INT1 na borda de descida
	EIMSK	= 0b00000001;	// habilita a interrupcao externa INT0
	PCICR	= 0b00000100;	// habilita interrupcao PIN change 2 (porta D)
	PCMSK2	= 0b00110001;	// habilita interrupcao PIN chance PD4 e PD5

	EICRA |= 0b10000010;	//Dispara o INT0 na borda de descida

	//configuracao do timer 0
	TCCR0A	= 0b00000010;	// habilita modo CTC do TC0
	TCCR0B	= 0b00000011;	// liga TC0 com prescaler = 64
	OCR0A	= 249;			// ajusta o comparador para TC0 contar ate 249.
	TIMSK0	= 0b00000010;	// habilita a interrupcao na igualdade de comparacao com OCR0A a cada 1ms = (249 + 1)*64/16MHz

	// configuracao do ADC
	ADMUX   = 0b01000101;	// Vcc - com referencia do canal PC5
	ADCSRA  = 0b11100111;	// habilita o AD / habilita interrupcao / modo de conversao continua / prescaler - 128
	ADCSRB  = 0b00000000;	// modo de conversao continua
	DIDR0   = 0b00000000;	// habilita o pino PC5 como entradas digitais

	// configuracao do timer T2 para o modo PWM
	TCCR2A = 0b00100011;	// habilita o PWM para o pino PD3
	TCCR2B = 0b00000100;	// prescaler - 64
	TCCR1B = (1 << ICES1) | (1 << CS12); // captura - borda de subida, TC1 com prescaler = 256
	TIMSK1 = (1 << ICIE1) | (1 << TOIE1); // habilita interrupcao por captura

	sei();
}

#endif
