/*
 * RemoteControl.c
 *
 * Created: 28.12.2016 12:06:02
 *  Author: Vadim Kulakov, vad7@yahoo.com
 *
 * ATtiny13A
 *
 * 
 */ 
#define F_CPU 2400000UL
// Fuses: BODLEVEL = 4.3V (BODLEVEL[1:0] = 11), RSTDISBL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <util/atomic.h>

#define LED1PORT			PORTB
#define LED1				(1<<PORTB5)
#define LED1_ON				LED1PORT |= LED1
#define LED1_OFF			LED1PORT &= ~LED1
#define LED1_INIT			DDRB |= LED1

#define KEYS_PORT			PORTB
#define KEYS_PIN			PINB
#define KEYS_DDR			DDRB
#define SENS				(1<<PORTB4) // low pulses ~8us, period ~76ms
#define KEYS_MODE_FAN		(1<<PORTB2) // to left side R25, FAN - low <= 9us, otherwise MODE
#define KEYS_PWR_MINUS		(1<<PORTB1) // to left side R23, MINUS - low <= 9us, otherwise PWR
#define KEYS__PLUS			(1<<PORTB0) // to left side R22, PLUS - low <= 9us
#define KEYS_INTR_INIT		GIMSK |= (1<<PCIE); PCMSK |= (1<<PCINT2) | (1<<PCINT1) | (1<<PCINT0)
#define KEY_MODE			(KEYS_MODE_FAN * 2)
#define KEY_FAN				((KEYS_MODE_FAN * 2) + 1)
#define KEY_PWR				(KEYS_PWR_MINUS * 2)
#define KEY_MINUS			((KEYS_PWR_MINUS * 2) + 1)
#define KEY_PLUS			((KEYS__PLUS * 2) + 1)

#define POWER_ON_TEMP		24  // C
#define CHANGE_TEMP_TIMEOUT 22  // =2sec (*0.085sec)

uint8_t keys				= 0;
uint8_t KeyPaused			= 0;
uint8_t PowerOn				= 0;
uint8_t ActiveMode			= 0; // 0 - fan, 1 - cooling, 2 - heating
uint8_t ActiveTemp			= POWER_ON_TEMP;
uint8_t ActiveFan			= 0;
uint8_t NeedSave			= 0;
uint8_t ChangeTempTime		= 12; // *0.085sec

#define EPROM_OFFSET				0x00 // (address record - 1)
#define EPROM_PowerOn				0x01 // 0..1
#define EPROM_Mode					0x02 // 0 - fan, 1 - cooling, 2 - heating 
#define EPROM_Temp					0x03 // 17..30
#define EPROM_Fan					0x04 // 0..2
#define EPROM_RECORD_SIZE			4

register uint8_t key asm("10");
ISR(PCINT0_vect, ISR_NAKED)
{
	//asm(code : output operand list : input operand list : clobber list);
	asm volatile("in R10, %0" : : "I" (_SFR_IO_ADDR(KEYS_PIN)) : );  // key = KEYS_PIN
	asm volatile("push R1");
	asm volatile("push R0");
	asm volatile("in R0, 0x3F");
	asm volatile("push R0");
	asm volatile("push R24");
	asm volatile("push R25");
	
//	asm volatile("in R10, %0" : : "I" (_SFR_IO_ADDR(GIFR)) : );
	
	if(KeyPaused == 0) {
		key = (~key) & (KEYS_MODE_FAN | KEYS_PWR_MINUS | KEYS__PLUS);
		if(key) { // Pressed
			_delay_us(2);
			keys = key * 2 + ((KEYS_PIN & key) != 0);
			KeyPaused = 1; // skip all pulses until timer overflow
		}
	}
	TCNT0 = 0;
	
	asm volatile("pop R25");
	asm volatile("pop R24");
	asm volatile("pop R0");
	asm volatile("out 0x3F, R0");
	asm volatile("pop R0");
	asm volatile("pop R1");
	asm volatile("reti");
}

#if(1)
void Delay100ms(unsigned int ms) {
	while(ms-- > 0) {
		_delay_ms(100); wdt_reset();
	}
}

void FlashLED(uint8_t num, uint8_t toff, uint8_t ton) {
	while (num-- > 0) {
		LED1_OFF;
		Delay100ms(toff);
		LED1_ON;
		Delay100ms(ton);
	}
	LED1_OFF;
}

void FlashNumber(uint8_t Number)
{ // HEX
	FlashLED(Number / 16, 5, 15);
	Delay100ms(20);
	FlashLED(Number % 16, 5, 5);
	Delay100ms(20);
}

uint8_t EEPROM_read(uint8_t ucAddress) // max 256 bytes EEPROM only!
{
	while(EECR & (1<<EEWE)) ; // EEPE
	EEAR = ucAddress;
	EECR |= (1<<EERE);
	return EEDR;
}

void EEPROM_update(uint8_t ucAddress, uint8_t ucData) // max 256 bytes EEPROM only!
{
	if(EEPROM_read(ucAddress) == ucData) return;
	while(EECR & (1<<EEWE)) ; // EEPE
	cli();
	EECR = (0<<EEPM1)|(0<<EEPM0);
	EEAR = ucAddress;
	EEDR = ucData;
	EECR |= (1<<EEMWE); //(1<<EEMPE);
	EECR |= (1<<EEWE); //(1<<EEPE);
	sei();
}

uint8_t EEPROM_read_record(uint8_t ucAddress) // max 256 bytes EEPROM only!
{
	return EEPROM_read(EEPROM_read(EPROM_OFFSET) + ucAddress);
}

void EEPROM_update_record(uint8_t ucAddress, uint8_t ucData) // max 256 bytes EEPROM only!
{
	EEPROM_update(EEPROM_read(EPROM_OFFSET) + ucAddress, ucData);
	if(EEPROM_read_record(ucAddress) != ucData) { // flash cell broken
		EEPROM_update(EPROM_OFFSET, EEPROM_read(EPROM_OFFSET) + EPROM_RECORD_SIZE);
		NeedSave = 2;
	}
}

#endif

#define SETUP_WATCHDOG WDTCR = (1<<WDCE) | (1<<WDE); WDTCR = (1<<WDE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0); //  Watchdog reset 2 s
uint8_t LED_Warning = 1, LED_WarningOnCnt = 0, LED_WarningOffCnt = 0;

ISR(TIM0_OVF_vect) // 85 ms
{
	KeyPaused = 0;
	if(ChangeTempTime) ChangeTempTime--;
	if(NeedSave) NeedSave--;
	if(LED_WarningOnCnt) {
		LED1_ON;
		LED_WarningOnCnt--;
	} else if(LED_WarningOffCnt) {
		LED1_OFF;
		LED_WarningOffCnt--;
	} else if(LED_Warning) { // short flashes
		LED_WarningOffCnt = 2;
		LED_WarningOnCnt = 2;
		if(--LED_Warning == 0) LED_WarningOffCnt = 5;
	}
}

// dc: 0 - 7.6us pulse, 1 - 50ms pulse (76.4); 100ms pause
void PressKey(uint8_t port, uint8_t cnt, uint8_t dc) 
{
	while(cnt-- > 0) {
		uint8_t ddr = KEYS_DDR | port;
		if(dc) {
			while(KEYS_PIN & SENS) ;
			while((KEYS_PIN & SENS) == 0) ;
			KEYS_DDR = ddr; // set low
			_delay_ms(70);
		} else {
			while(KEYS_PIN & SENS) ;
			KEYS_DDR = ddr; // set low
			_delay_us(6); // while((SENS_PIN & SENS) == 0) ;
		}
		KEYS_DDR = ddr ^ port; // release
		wdt_reset();
		_delay_ms(100);
	}
}

int main(void)
{
	CLKPR = (1<<CLKPCE); CLKPR = (0<<CLKPS3) | (0<<CLKPS2) | (1<<CLKPS1) | (0<<CLKPS0); // Clock prescaler: 4
	SETUP_WATCHDOG;
	MCUCR = (1<<SE) | (0<<SM1) | (0<<SM0); // Idle sleep enable
	KEYS_DDR &= ~(KEYS_MODE_FAN | KEYS_PWR_MINUS | KEYS__PLUS); // In
	KEYS_PORT &= ~(KEYS_MODE_FAN | KEYS_PWR_MINUS | KEYS__PLUS); // = 0
	PORTB = (1<<PORTB3); // pullup not used pins
	LED1_INIT;
//	// Timer 8 bit
 	TCCR0A = (1<<WGM01) | (1<<WGM00);  // Timer0: Fast PWM OCRA
 	TCCR0B = (1<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00); // Timer0 prescaller: 1024
 	TIMSK0 |= (1<<TOIE0); // Timer/Counter0 Overflow Interrupt Enable
 	OCR0A = 200; // OC0A(TOP)=Fclk/prescaller/Freq - 1; Freq=Fclk/(prescaller*(1+TOP))
 	//OCR0B = 0; // 0..OCR0A, Half Duty cycle = ((TOP+1)/2-1)
	//TCCR0A |= (1<<COM0B1); // Start PWM out
	//OSCCAL = EEPROM_read(EPROM_OSCCAL);
	sei();
	while(ChangeTempTime) {	__asm__ volatile ("" ::: "memory"); } // delay 1 sec
	if(EEPROM_read(EPROM_OFFSET) == 0xFF) {
		EEPROM_update(EPROM_OFFSET, 0);
	} else {
		PowerOn = EEPROM_read_record(EPROM_PowerOn);
		if(PowerOn == 0xFF) PowerOn = 0;
		else {
			ActiveMode = EEPROM_read_record(EPROM_Mode);
			ActiveTemp = EEPROM_read_record(EPROM_Temp);
			ActiveFan = EEPROM_read_record(EPROM_Fan);
		}
	}
	if((MCUSR & (1<<WDRF)) == 0) { // No watchdog reset
		if(ActiveTemp > POWER_ON_TEMP) PressKey(KEYS__PLUS, ActiveTemp - POWER_ON_TEMP, 0); 
		else if(ActiveTemp < POWER_ON_TEMP) PressKey(KEYS_PWR_MINUS, POWER_ON_TEMP - ActiveTemp, 0);
		if(ActiveMode) PressKey(KEYS_MODE_FAN, ActiveMode, 1);
		if(ActiveFan) PressKey(KEYS_MODE_FAN, ActiveFan, 0);
		if(PowerOn) PressKey(KEYS_PWR_MINUS, 1, 1);
	}
	KEYS_INTR_INIT;
	while(1)
	{
		__asm__ volatile ("" ::: "memory"); // Need memory barrier
		sleep_cpu();
		wdt_reset();
		if(keys == KEY_MINUS) {
			if(ChangeTempTime) { // sleeping?
				if(ActiveTemp > 17) {
					ActiveTemp--;
					LED_Warning = 1;
				}
			}
			ChangeTempTime = CHANGE_TEMP_TIMEOUT;
			goto xEndKeys;
		} else if(keys == KEY_PLUS) {
			if(ChangeTempTime) { // sleeping?
				if(ActiveTemp < 30) {
					ActiveTemp++;
					LED_Warning = 1;
				}
			}
			ChangeTempTime = CHANGE_TEMP_TIMEOUT;
			goto xEndKeys;
		} else if(keys == KEY_MODE) {
			if(++ActiveMode > 2) ActiveMode = 0;
			LED_Warning = ActiveMode + 1;
			goto xEndKeys;
		} else if(keys == KEY_PWR) {
			PowerOn ^= 1;
			LED_Warning = PowerOn + 1;
			goto xEndKeys;
		} else if(keys == KEY_FAN) {
			if(++ActiveFan > 2) ActiveFan = 0;
			LED_Warning = ActiveFan + 1;
xEndKeys:
			NeedSave = 255;
			keys = 0;
		}
		if(NeedSave == 1) {
			NeedSave = 0;
			EEPROM_update_record(EPROM_PowerOn, PowerOn);
			EEPROM_update_record(EPROM_Mode, ActiveMode);
			EEPROM_update_record(EPROM_Temp, ActiveTemp);
			EEPROM_update_record(EPROM_Fan, ActiveFan);
			LED_Warning = 7;
		}
	}
}
