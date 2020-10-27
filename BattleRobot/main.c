/*
 * BattleRobot.c
 *
 * Created: 2020-07-23 오전 1:45:09
 * Author : 이재원
 */ 

#define F_CPU 16000000UL //ATmega128
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int i; //반복문을 위한 전역변수

//Switch Resister 세팅
void IntSet(){

	EICRA =	(1<<ISC01)|(0<<ISC00)|(1<<ISC11)|(0<<ISC10); //어떤 방식으로 인터럽트를 일으킬것인지, EICRA가 1번(ISC11,ISC10), 0번(ISC01,ISC00) 순인데 1,0,1,0으로 설정함으로써 Falling Edge ,1,1은 Rising Edge
	EIMSK = (1<<INT1)|(1<<INT0); //인터럽트 0번~7번을 어디에 사용할것인지, INT0,INT1 허용

}

//Motor Resister 세팅
void MotorSet(){
	DDRB = 0xFF; //B포트 출력설정(모터드라이브) -> 0b11111111
	DDRE = 0x0F; //E포트 출력(모터드라이브) -> 0b00001111

	TCCR1A = (1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(1<<WGM11)|(0<<WGM10);
	TCCR1B = (1<<WGM13)|(1<<WGM12)|(0<<CS02)|(0<<CS01)|(1<<CS00);
}
//UART
void UART1_INIT() {
	DDRD = 0b00001000;
	
	UCSR1A = 0x00;
	UCSR1B = (1<<RXEN1) | (1<<TXEN1) | (0<<UCSZ12);
	UCSR1C = (1<<UCSZ11) | (1<<UCSZ10);
	
	UBRR1H = 0;
	UBRR1L = 103;  // Baud Rate 9600
}
//UART
void UART1_Transmit(unsigned char cData) {
	while (!(UCSR1A & (1<<UDRE1)));
	
	UDR1 = cData;
}
//UART
unsigned char UART1_Receive() {
	while (!(UCSR1A & (1<<RXC1)));
	
	return UDR1;
}
//UART
void UART1_Transmit_16Int(int num)
{
	if (num<0) {
		UART1_Transmit('-');
		num = -num;                        // 값이 음수면 -부호 문자를 보내고 양수로 변환
	}
	
	UART1_Transmit((num / 10000) + 48);            // 만의 자리 전송
	UART1_Transmit(((num % 10000) / 1000) + 48);   // 천의 자리 전송
	UART1_Transmit(((num % 1000) / 100) + 48);      // 백의 자리 전송
	UART1_Transmit(((num % 100) / 10) + 48);      // 십의 자리 전송
	UART1_Transmit((num % 10) + 48);            // 일의 자리 전송
}
//ADC Set
void ADCSet(){

	DDRF = 0b00000000; // ADC 입력설정
	ADCSRA = 0x87; //세부설정
	
}


//ADC Resister 세팅
double adc[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; //adc 전역변수
double adc_max[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; //adc 최댓값 저장
double adc_min[8] = {500.0,500.0,500.0,500.0,500.0,500.0,500.0,500.0}; //adc 최솟값 저장

int Normal[8] = {0,}; //Normalization
int flag = 0; //mode flag 1 or 2


//최대값, 최솟값 설정 //0번 스위치 인터럽트
ISR(INT0_vect){

	//PORTE = 0b00000101; // 앞 뒤
	//ICR1 = 799;
	//OCR1A = 799; //OCR 설정 50% duty radio
	//OCR1B = 799;
	//_delay_ms(500); //확인용

	PORTA = 0xf0; //0번 스위치시 확인용

	flag = 1; //flag 1번 모드
}

int cnt;


//Normalization & Battle Mode //1번 스위치 인터럽트
ISR(INT1_vect){
	PORTA = 0x0f;
	flag = 2; //스위치 1번 시 flag 2번 모드
}

ISR(TIMER0_OVF_vect){ // T/C

	//flag 가 2일때 -> 1번 스위치 인터럽트
	if(flag == 2){  //1번 스위치 //모터 제어

		cnt++ ;
		TCNT0 = 131; //TCNT 초기화 //2ms

		if(cnt == 30){ // 60ms, 0.06초
			
			//(좌) 7 6 5 4 3 2 1 0 (우)//

			if((Normal[0] < 60) || (Normal[1] < 60) || (Normal[2] < 60) || (Normal[3] < 60)){ //0번,1번, 2번,3번 ADC가 검정선을 만났을 때 _좌회전

					PORTA = 0b11111100;  //좌회전


					PORTE = 0x09; //0b00001001(모터 방향설정) 뒤 뒤 원래거
					ICR1 = 799;
					OCR1A = 799; //OCR 설정 100% duty radio
					OCR1B = 799;
					_delay_ms(570); // 0.55 sec 후진
				
					

					PORTE = 0x0A; //0b00001010 (모터 방향설정) 뒤 앞
					ICR1 = 799;
					OCR1A = 700; //OCR 설정 80% duty radio
					OCR1B = 700;
					_delay_ms(480); //0.4 sec 좌회전
					

					cnt = 0; //cnt값 초기화
			

			}

			else if ((Normal[7] <= 60) || (Normal[6] < 60) || (Normal[5] < 60) || (Normal[4] < 60)){ //4, 5번 혹은 6번 혹은 7번 ADC가 검정선을 만났을 때 _우회전

				
					PORTA = 0b00111111;

					PORTE = 0x09; //0b00001001(모터 방향설정) 뒤 뒤
					ICR1 = 799;
					OCR1A = 799; //OCR 설정 100% duty radio
					OCR1B = 799;
					_delay_ms(570); //1sec 후진

					PORTE = 0x05; //0b00000101(모터 방향설정) 앞 뒤
					ICR1 = 799;
					OCR1A = 700; //OCR 설정 100% duty radio
					OCR1B = 700;
					_delay_ms(480);

					cnt = 0; //cnt값 초기화
				
				
			}
			else{
				PORTA = 0b10111101; //배틀모드의 기본 상태 LED

				PORTE = 0x06;
				ICR1 = 799;
				OCR1A = 759; //OCR 설정 85% duty radio
				OCR1B =	799; // 90%
				
				cnt = 0; //cnt 값 초기화
			}	
			
		}

	}
	
	
	
}

//main문
int main(void)
{
	DDRA = 0xff; //LED 출력설정
	PORTA = 0b01111110; // 기본 상태에서의 LED
	DDRD = 0x00; //switch 입력설정

	//기본 모터 출력
    PORTE = 0x06; //0b00000110 (모터 방향설정) 앞, 앞
    ICR1 = 799; //MAX
    OCR1A = 639; //OCR 설정 80% duty radio
    OCR1B = 639; 

	//레지스터 활성화
	MotorSet(); //motor
	IntSet(); //Int0, Int1 설정
	ADCSet(); //ADC
	UART1_INIT(); //UART

	//Timer/Counter
	TCCR0 = (0<<WGM01)|(0<<WGM00)|(0<<COM01)|(0<<COM00)|(1<<CS02)|(1<<CS01)|(0<<CS00); //(WGM01,WGM00)->0,0(Normal Mode) /(COM01,COM00)->0,0(Normal Mode)(WGM01,WGM00)->0,0(Normal Mode) ,CS는 256분주비 설정
	TIMSK = (1<<TOIE0); //EIMSK처럼 Interrupt Enable									//(WGM01,WGM00)->0,0(Normal Mode) ,CS는 256분주비 설정
	TCNT0 = 131; //131로 TCNT설정 //T(desired) = 2ms
	

	sei(); //전역 인터럽트 활성화

	//while문
    while (1){
	

		if(flag == 1){ //0번 스위치 인터럽트 //max min 값 받기

			

			for(i = 0; i < 8; i++)
			{
				ADCSRA |= (1<<ADSC);
				ADMUX =(0x40 | i); //0b010 00000 | i //ADC 0번 ~ 7번
				
				while(!(ADCSRA & (1<<ADIF)));
				adc[i] = ADC;
			}

			//adc 값 계속 받기 //최대, 최소 fix
			for(int i = 0; i < 8; i++)
			{
				if(adc[i] >= adc_max[i]) adc_max[i] = adc[i]; //ADC[i] 값이 max값 이상이면 max에다가 다시 저장_반복
				if(adc[i] <= adc_min[i]) adc_min[i] = adc[i];
			}

			cnt = 0; //cnt 값 초기화
		}

		else if(flag == 2) //1번 스위치 //정규화
		{
			
			
			for(i = 0; i < 8; i++)
			{
				ADCSRA |= (1<<ADSC);
				ADMUX =(0x40 | i); //0b010 00000 | i //ADC 0번 ~ 7번
				
				while(!(ADCSRA & (1<<ADIF)));
				adc[i] = ADC;
			}
			
			for(int i = 0; i < 8; i++){
				Normal[i] = (int)((adc[i] - adc_min[i])/(adc_max[i] - adc_min[i])*100.0); //정규화 식

			}
		
		}
		////ADC값 UART 통신
		for(int j = 0; j < 8; j++)
		{
			
			//정규화 값 통신
			UART1_Transmit_16Int((unsigned int)(Normal[j]));
			UART1_Transmit(' ');
			
			//MAX, MIN값 통신
				//UART1_Transmit('m');
				//UART1_Transmit('a');
				//UART1_Transmit('x');
				//UART1_Transmit(':');
				//UART1_Transmit_16Int((int)(adc_max[j]));
				//UART1_Transmit(' ');
				//UART1_Transmit('m');
				//UART1_Transmit('i');
				//UART1_Transmit('n');
				//UART1_Transmit(':');
				//UART1_Transmit_16Int((int)(adc_min[j]));
				//UART1_Transmit(' ');
			
		}
		UART1_Transmit(13); //ASCII /enter/

		

	}
}








