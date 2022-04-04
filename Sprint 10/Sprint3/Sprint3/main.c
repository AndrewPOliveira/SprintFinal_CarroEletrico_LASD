/*
 * Sprint10.c
 *
 * Created: 19/12/2021 13:40:52
 * Author : José Andrew Pessoa de Oliveira
 */ 


#include "SSD1306.h"
#include "Font5x8.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

uint16_t velocidade_veiculo = 0;
uint16_t diametro_pneu = 50;
uint16_t distancia_percorrida_dm = 0;//variavel para guardar distância em dm
uint16_t distancia_percorrida_km = 0;//variavel para auxiliar conversão da distância em Km
uint8_t contador_auxiliar = 0;
uint16_t RPM = 0;
uint16_t sensor_pneu = 0;
uint32_t tempo_borda_subida_SR04, tempo_delta_SR04=0,distancia_obstaculo;
unsigned char Marcha = 0;
uint16_t temp_bateria = 25;
uint16_t temp_bateria_max =0;
uint16_t carga_bateria = 50;
uint8_t contador_AD = 0;
uint16_t ultima_manut =0;


ISR(INT0_vect)
{
	sensor_pneu++;  //sensor que controla quantidade de voltas do pneu por intervalo de medição
}
ISR(PCINT2_vect)
{
	if ((PIND&0b01000000)==0)
		Marcha = 68;//D em ascii
	else
		Marcha = 82;//R em ascii
	if ((PIND&0b10000000)==0)
		Marcha = 80;//P em ascii
	
	if ((PIND&0b00001000)==0)
	{
		if(diametro_pneu < 200) //Incremento apenas se diâmetro não exceder o máximo
			diametro_pneu++;
		eeprom_write_word (5, diametro_pneu); //altera valor na memória quando diametro for alterado
	}
	if ((PIND&0b00010000)==0)
	{
		if(diametro_pneu > 30) //Decremento apenas se diâmetro for maior que o mínimo
			diametro_pneu--;
		eeprom_write_word (5, diametro_pneu); //altera valor na memória quando diametro for alterado
	}
	
}
ISR(TIMER2_COMPA_vect) //interrupção do TC2 para calculo da velocidade media a cada 1ms
{
	contador_auxiliar++;//incremento no contador auxiliar de tempo
	
	if (contador_auxiliar == 250) //calculo da velocidade média a cada 100*tempo_do_timer = 250*1ms = 250ms
	{
		velocidade_veiculo = (3.1415*diametro_pneu*sensor_pneu/(0.25/0.36))/20; //calculo da velocidade média no intervalo de tempo tomado em Km/h
		RPM = sensor_pneu/(0.25/(60/2));//calculo de voltas completas dividido pelo tempo gasto
		distancia_percorrida_dm += 3.1415*diametro_pneu*sensor_pneu/10;//em dm, porém será apresentado em Km
		contador_auxiliar = 0;//zera o contador
		sensor_pneu =0; //zera contador do sensor
		
		if (distancia_percorrida_dm > 10000)// teste para incrementar distância no hodômetro quando completar 1km
		{
			distancia_percorrida_km ++;
			distancia_percorrida_dm -= 10000;
			eeprom_write_dword (0, distancia_percorrida_km); //altera valor na memória quando 1km for percorrido
		}
	}
}
ISR(TIMER1_CAPT_vect)
{
	if(TCCR1B&(1<<ICES1))//testa configuração para borda de subida
		tempo_borda_subida_SR04 = ICR1; //guarda posição do tempo de subida no disparo de PB0
	else
		tempo_delta_SR04 = (ICR1 - tempo_borda_subida_SR04)*16; //calcula tempo em nível alto
	
	TCCR1B ^=(1<<ICES1);//inverte borda de captura
	distancia_obstaculo = tempo_delta_SR04/58;//converte valor medido de tempo para distancia do obstaculo em cm
}

ISR(USART_RX_vect)//interrupção para interação com UART
{
	char recebido, string1[8];
	uint8_t i=0;
	recebido = UDR0;
	if(recebido == 'd') //retorna temp_max da bateria
	{
		sprintf(string1, "%d", temp_bateria_max); 
		while(string1[i]!='\0')
		{
			UART_Transmit(string1[i]);
			i++;
		}
	}
	if(recebido == 'l') //limpa a locação de memória para temp_max
	{
		temp_bateria_max = 0;
		eeprom_write_word(10,temp_bateria_max);
	}
	if(recebido == 'u') //apresenta última manutenção
	{
		sprintf(string1, "%d", ultima_manut); //gera string para transmissão e processamento
		while(string1[i]!='\0')
		{
			UART_Transmit(string1[i]);
			i++;
		}
	}
	if(recebido == 'm') //foi realizada nova manutenção
	{
		ultima_manut = distancia_percorrida_km;
		eeprom_write_word(15,ultima_manut);
	}
}

ISR(ADC_vect) //Interrupção do ADC
{
	switch(contador_AD)
	{
		case 0://controle do motor
			ADMUX = 0b00000000; //seleção de PC0 com tensão de referência Aref = 5V
			if ((PIND&0b10000000)==0)
				OCR0B = 0; // zera o PWM se P for desabilitada
			else
			{
				if ((velocidade_veiculo>20)&&(distancia_obstaculo<300)&&(ADC/4>25))//freada brusca apenas se exceder o limite de velocidade ou distancia de obstaculo
				{
					OCR0B = 25;//~10% da velocidade máxima
				}
				else
				{
					OCR0B = ADC/4; //leitura do ADC varia de 0-1023, assim OCR0B varia de 0-255, onde Dpwm = OCR0B/256 (variação de 0 - 100%)
				}
			}
			contador_AD++;//incremento para percorrer toda a função switch
		break;
			
		case 1://controle da carga da bateria
			ADMUX = 0b00000001; //seleção de PC1 com tensão de referência Aref = 5V
			if (ADC<665) //ajuste para apresentação correta no display
				carga_bateria = (ADC*100)/1024; //carga da bateria em %
			else
				carga_bateria = (ADC*100)/1024+65; //carga da bateria em %
			contador_AD++;
		break;
		
		case 2://controle da temperatura da bateria
			ADMUX = 0b00000011; //seleção de PC3 com tensão de referência Aref = 5V
			temp_bateria = ((ADC*100/(1024-ADC))-100)/0.385; //temperatura da bateria
			if(temp_bateria > 200) //ajuste para erro quando sensor marca 0°C
				temp_bateria = 0;
			if(temp_bateria>temp_bateria_max)
			{
				temp_bateria_max = temp_bateria;
				eeprom_write_dword (10, temp_bateria_max); //altera valor na memória quando nova máxima for detectada
			}
			contador_AD = 0;
		break;
		
		default:
			ADMUX = 0b00000000; //seleção de PC2 com tensão de referência Aref = 5V
			contador_AD=0;
		break;
	}
	
}
void Mostrar_velocidade(void); //Função para mostrar velocidade no display de 7 segmentos
void carregar_dados(void); //Função para carregar dados da EEPROM
void anima_display(void); //código para uso do display I2C
void Alertas(void); //Emite mensagem sonora e visual se houver problema no veículo
void Inicia_UART(unsigned int); //função para inicializar a USART
void UART_Transmit(unsigned char data); //função para envio de um frame de 5 a 8bits

int main(void)
{	
	Inicia_UART(MYUBRR);
	GLCD_Setup();//INICIALIZAÇÃO DO DISPLAY I2C
	GLCD_SetFont(Font5x8,5,8,GLCD_Overwrite);
	GLCD_Clear();
	GLCD_InvertScreen();
	GLCD_Render();
	
	if ((PIND&0b01000000)==0)//inicialização da variável que guarda a marcha
		Marcha = 68;//D em ascii
	else
		Marcha = 82;//R em ascii
	if ((PIND&0b10000000)==0)
		Marcha = 80;//P em ascii
	
    DDRB = 0b11111110;//todos os pinos de B configurados como saídas, exceto PB0
    DDRD = 0b00000000;//pinos de D configurados como entrada
    PORTD |= 0b00011100;//habilitação dos botões de pull-up 3 e 4 em D, 2 recebe os dados do tacômetro
	DDRC |=0b0000100;//habilita PC2 como saída

	TCCR2A = 0b00000010; //habilita modo CTC do TC2
	TCCR2B = 0b00000101; //liga TC2, prescaler = 64
	OCR2A = 249; //ajusta o comparador para o TC2 contar até 249
	TIMSK2 = 0b00000010; //habilita a interrupção na igualdade de comparação com OCR2A. A interrupção ocorre a cada 1ms = (64*(249+1))/16MHz
	
	TCCR1B = (1<<ICES1)|(1<<CS12);//Captura na borda de subida, TC1 com prescaler = 256. Estouro a cada 256*(2^16)/16MHz = (2^16)*16us = 1,04s
	TIMSK1 = 1<<ICIE1; //habilita a interrupção por captura
	
	TCCR0A = 0b10100011;//habilita modo PWM rápido do TC0
	TCCR0B = 0b00000101; //liga TC0, prescaler = 1024, fpwm = 16M/(256*1024) = 61Hz
	OCR0B = 0; //controle do Ton do duty cycle (Duty Cycle = OCR0B/256) e o valor inicial é 0
	
	ADMUX = 0b00000000; //tensão de referência no pino C0 é Aref = 5V;
	ADCSRA = 0b11101111; //habilita o AD, habilita interrupção, modo de conversão contínua, prescaler = 128
	ADCSRB = 0x00;//modo de conversão contínua
	DIDR0 = 0b00110100;//habilita pino PC0, PC1 e PC3 como entrada do ADC0
	
	PCICR |= (1 << PCIE2); //habilitar interrupção PCINT2 na porta D
	PCMSK2 |= (1 << PCINT19);//habilitar interrupção PCINT20 correspondente ao pino D3
	PCMSK2 |= (1 << PCINT20);//habilitar interrupção PCINT21 correspondente ao pino D4
	PCMSK2 |= (1 << PCINT22);//habilitar interrupção PCINT22 correspondente ao pino D6
	PCMSK2 |= (1 << PCINT23);//habilitar interrupção PCINT23 correspondente ao pino D7
	
	EICRA = 0b00000010;//INT0 configurada para interrupção na descida do sensor do pneu
	EIMSK = 0b00000001;//habilitar interrupção externa INT0
	sei();//Habilitar interrupções globais
    
    while (1)
    {
		carregar_dados(); //carrega dados do tacometro e hodometro
		Mostrar_velocidade();//ativa display de velocidade
		anima_display();//ativa display I2C
    }

}

void Inicia_UART(unsigned int ubrr) //função para inicializar a USART
{
	UBRR0H = (unsigned char)(ubrr>>8);//ajusta taxa de transmissão
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);//Habilita o transmissor e o receptor
	UCSR0C = (1<<USBS0)|(1<<UCSZ00);//Ajusta o formato do frame: 8bits de dados e 2 de parada
}

void UART_Transmit(unsigned char data) //função para envio de um frame de 5 a 8bits
{
	while(!( UCSR0A & (1<<UDRE0)));//Espera a limpeza do registr. de transmissão
	UDR0 = data; //Coloca o dado no registrador e o envia
}

void anima_display(void)//código para o display I2C
{
	GLCD_Clear();
	GLCD_GotoXY(0,0);
	GLCD_PrintString("Comp. Bordo");
	GLCD_GotoX(90);
	GLCD_PrintInteger(carga_bateria);
	GLCD_PrintString(" %");
	GLCD_GotoXY(0,9);
	GLCD_PrintString("------------");
	GLCD_GotoX(90);
	GLCD_PrintInteger(temp_bateria);
	GLCD_PrintString(" C");
	GLCD_GotoXY(0,16);
	GLCD_PrintString("Diam(cm): ");
	GLCD_PrintInteger(diametro_pneu);
	GLCD_GotoXY(0,25);
	GLCD_PrintInteger(RPM);
	GLCD_PrintString(" rpm");
	GLCD_GotoXY(0,34);
	GLCD_PrintString("Sonar: ");
	GLCD_PrintInteger(distancia_obstaculo);
	GLCD_PrintString(" cm");
	GLCD_GotoXY(5,48);
	GLCD_PrintChar(Marcha);
	GLCD_GotoX(20);
	GLCD_PrintInteger(distancia_percorrida_km);
	GLCD_PrintString(" Km");
	Alertas(); //verifica se temperatura está aceitável
	GLCD_Render();
}

void Alertas(void) //Emite mensagem sonora e visual se houver problema no veículo
{
	if (temp_bateria > 80)//Printa mensagem de alerta se a temperatura da bateria for maior que 80°C
	{
		GLCD_DrawTriangle(96, 60, 120, 60, 108, 45, GLCD_Black);
		GLCD_GotoXY(106,50);
		GLCD_PrintString("!");
		PORTC |= 0b0000100; //ativa alerta sonoro para chamar atenção do motorista
	}
	else
		PORTC &= 0b1111011;
	
	if (distancia_percorrida_km > ultima_manut+2)//Printa mensagem de alerta se manutenção expirar
	{
		GLCD_DrawRectangle(98, 44, 118, 30,GLCD_Black);
		GLCD_GotoXY(106,35);
		GLCD_PrintString("M");
	}
	
		
}

void Mostrar_velocidade(void) //Função para mostrar velocidade no display de 7 segmentos
{

	PORTB &=0b00000001; //reseta pinos B de 7 a 1
	PORTB |=0b01100000; //seta 1 no pino de controle B6 e B7
	PORTB |= ((velocidade_veiculo/100)<<1)&0b00011110; //mostra a centena
	_delay_ms(70);

	PORTB &=0b11100001; //reseta pinos 4 a 1 de B
	PORTB ^=0b11000000; //inverte os pinos 7 e 6 de B
	PORTB |= (((velocidade_veiculo/10)%10)<<1)&0b00011110; //mostra a dezena
	_delay_ms(70);
	
	PORTB &=0b11100001; //reseta pinos 4 a 1 de B
	PORTB ^=0b01100000; //inverte os pinos 6 e 5 de B
	PORTB |= (((velocidade_veiculo)%10)<<1)&0b00011110; //mostra a unidade
	_delay_ms(70);

}

void carregar_dados(void)
{
	if(eeprom_read_dword (0)!=-1)//teste para verificar se algo está escrito na memória alocada
	{
		if(distancia_percorrida_km == 0)
		distancia_percorrida_km = eeprom_read_dword (0);//lê valor da distância da memória quando programa iniciar
	}
	if(eeprom_read_word (5)!=-1)//teste para verificar se algo está escrito na memória alocada
	{
		if (diametro_pneu != eeprom_read_word (5))//substituição do valor apenas quando houver diferença
			diametro_pneu = eeprom_read_word (5);//lê valor do diâmetro da memória quando programa iniciar
	}
	if(eeprom_read_word (10)!=-1)//teste para verificar se algo está escrito na memória alocada
	{
		if (temp_bateria_max != eeprom_read_word (10))//substituição do valor apenas quando houver diferença
			temp_bateria_max = eeprom_read_word (10);//lê valor do diâmetro da memória quando programa iniciar
	}
	if(eeprom_read_word (15)!=-1)//teste para verificar se algo está escrito na memória alocada
	{
		if (ultima_manut != eeprom_read_word (15))//substituição do valor apenas quando houver diferença
			ultima_manut = eeprom_read_word (15);//lê valor do diâmetro da memória quando programa iniciar
	}
}

