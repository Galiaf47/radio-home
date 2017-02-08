/**
 * MCU = atmega8a
 * Fuse bits:
 * Low 0xe4 int
 * Low 0xff ext
 * High 0xd9
 * Ext 0xff
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define CSN_PORT PORTA
#define CSN_PIN PORTA1
#define CE_PORT PORTA
#define CE_PIN PORTA0

#define BITRATE 9600
#define BAUD ((F_CPU / (2 * BITRATE)) - 1)
#define NRF_DATA_LENGTH 5
#define SET_REGISTER_DELAY 200
#define NRF_LISTEN_DELAY 20

#define SETBIT(x, y) x |= (1 << y)
#define CLEARBIT(x, y) x &= (~(1 << y))

#define CSN_LOW() CLEARBIT(CSN_PORT, CSN_PIN)
#define CSN_HIGH() SETBIT(CSN_PORT, CSN_PIN)
#define CE_LOW() CLEARBIT(CE_PORT, CE_PIN)
#define CE_HIGH() SETBIT(CE_PORT, CE_PIN)

void initPWM() {
    // DDRB |= (1 << DDB3);
    // Fast PWM; non-inverting mode; no prescaling
    // TCCR2 = 0b01101001;
    // 11 - Fast PWM
    TCCR2 |= (1<<WGM21) | (1<<WGM20);
    // 001 - no prescaling
    TCCR2 |= (1<<CS20);
    
    // Enable ovf interrupt
    TIMSK |= (1<<TOIE2);
}

void initADC() {
    PORTC = 0xff;
    ADCSRA = 0b10000101;
    ADMUX = 0b11100101;
}

uint8_t buffer[NRF_DATA_LENGTH];

main() {
    DDRD = 0xff;
    initADC();
    initPWM();
    sei();
    
    while(1) {
    }

    return 1;
}

ISR(TIMER2_OVF_vect) {
    ADCSRA |= (1<<ADSC);
    while (ADCSRA & (1<<ADSC)); 
    OCR2 = ADCH;
}
