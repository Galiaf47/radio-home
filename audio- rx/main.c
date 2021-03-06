/**
 * MCU: attiny2313a
 * Fuse bits:
 * int Low 0xe4
 * ext Low 0xff
 * High 0xdf
 * Ext 0xff
 */

#define F_CPU 16000000UL
#define BAUD 2000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/setbaud.h>
#include "../lib/nRF24L01.h"

#define CSN_PORT PORTD
#define CSN_PIN PD3
#define CE_PORT PORTD
#define CE_PIN PD4

#define SAMPLE_RATE 24000
#define TOP (10 * ((F_CPU / 10) / SAMPLE_RATE))

#define NRF_DATA_LENGTH 32
#define SET_REGISTER_DELAY 10
#define READ_DEVIDER (NRF_DATA_LENGTH / 2)

#define SETBIT(x, y) x |= (1 << y)
#define CLEARBIT(x, y) x &= (~(1 << y))

#define CSN_LOW() CLEARBIT(CSN_PORT, CSN_PIN)
#define CSN_HIGH() SETBIT(CSN_PORT, CSN_PIN)
#define CE_LOW() CLEARBIT(CE_PORT, CE_PIN)
#define CE_HIGH() SETBIT(CE_PORT, CE_PIN)

volatile uint8_t bufferCounter = 0;
volatile uint8_t currentBuffer = 0;
volatile uint8_t bufferState[2] = {0, 0};
uint8_t buffer[2][NRF_DATA_LENGTH];

/*
 * USART - SPI
 */
void initMSPI() {
    UBRRH = UBRRL = 0;
    /* Setting the XCK port pin as output, enables master mode. */
    XCK_DDR |= (1<<XCK_BIT);
    /* Set MSPI mode of operation and SPI data mode 0. */
    UCSRC = (1<<UMSEL1)|(1<<UMSEL0)|(0<<UCPHA)|(0<<UCPOL);
    /* Enable receiver and transmitter. */
    UCSRB = (1<<RXEN)|(1<<TXEN);

    /* Set baud rate. */
    /* IMPORTANT: The Baud Rate must be set after the transmitter is enabled
    */
    UBRRH = UBRRH_VALUE;
    UBRRL = UBRRL_VALUE;
    #if USE_2X
    UCSRA |= (1 << U2X);
    #else
    UCSRA &= ~(1 << U2X);
    #endif
}

uint8_t writeMSPI(uint8_t data) {
    // Wait for empty transmit buffer
    // Data Register Empty (UDRE) Flag
    while ( !( UCSRA & (1<<UDRE)) );
    // Put data into buffer, sends the data
    UDR = data;
    // Wait for data to be received
    // Receive Complete (RXC) Flag
    while (!(UCSRA & (1<<RXC))) {}
    // Get and return received data from buffer
    return UDR;
}

void initNrfRegister(uint8_t reg) {
    CSN_LOW();
    _delay_us(SET_REGISTER_DELAY);
    writeMSPI(reg);
    _delay_us(SET_REGISTER_DELAY);
}

void getNrfReceivedData(uint8_t *data) {
    initNrfRegister(R_RX_PAYLOAD);

    int i;
    for(i = 0; i < NRF_DATA_LENGTH; i++) {
        data[i] = writeMSPI(NOP);
        _delay_us(SET_REGISTER_DELAY);
    }

    CSN_HIGH();
}

void setNrfRegister(uint8_t reg, uint8_t val, uint8_t count) {
    initNrfRegister(W_REGISTER + reg);

    int i;
    for(i = 0; i < count; i++) {
        writeMSPI(val);
        _delay_us(SET_REGISTER_DELAY);
    }

    CSN_HIGH();
}

uint8_t getNfrRegister(uint8_t reg) {
    initNrfRegister(R_REGISTER + reg);
    
    uint8_t value = writeMSPI(NOP);
    _delay_us(SET_REGISTER_DELAY);

    CSN_HIGH();

    return value;
}

void setRegister(uint8_t reg, uint8_t value) {
    CSN_LOW();

    writeMSPI(W_REGISTER | (REGISTER_MASK & reg));
    writeMSPI(value);

    CSN_HIGH();
}

uint8_t getRegister(uint8_t reg) {
    CSN_LOW();

    writeMSPI(R_REGISTER | (REGISTER_MASK & reg));
    uint8_t result = writeMSPI(NOP);

    CSN_HIGH();

    return result;
}

void getData(uint8_t *data) {
    CSN_LOW();
    _delay_us(SET_REGISTER_DELAY);
    writeMSPI(R_RX_PAYLOAD);
    _delay_us(SET_REGISTER_DELAY);

    int i;
    for(i = 0; i < NRF_DATA_LENGTH; i++) {
        data[i] = writeMSPI(NOP);
        _delay_us(SET_REGISTER_DELAY);
    }

    CSN_HIGH();
}

void initNrf() {
    DDRD |= (1 << CSN_PIN) | (1 << CE_PIN);
    CSN_HIGH();//CSN to high to disable nrf
    CE_LOW();//CE to low to nothing to send/receive

    _delay_ms(100);

    // Disabled auto-acknowledgements
    setNrfRegister(EN_AA, 0x00, 1);

    // Number of retries and delay
    // "2": 750us delay; "F": 15 retries
    setNrfRegister(SETUP_RETR, 0x2F, 1);

    // data pipe 0
    setNrfRegister(EN_RXADDR, 0x01, 1);

    // 5 bytes RF_Address length
    setNrfRegister(SETUP_AW, 0b11, 1);

    // 2.401GHz
    setNrfRegister(RF_CH, 0x01, 1);

    // power and data speed
    // 00000111 bit[3]=0 1Mbps - longer rage; bit[2-1] power mode (11=-0dB; 00=-8dB)
    setNrfRegister(RF_SETUP, 0x07, 1);

    // Receiver address
    setNrfRegister(RX_ADDR_P0, 0x12, 5);

    // Transmitter address
    setNrfRegister(TX_ADDR, 0x12, 5);

    // Payload length setup
    setNrfRegister(RX_PW_P0, NRF_DATA_LENGTH, 1);

    //MASK_RX_DR = 0
    //MASK_TX_DS = 0
    //MASK_MAX_RT = 0
    //EN_CRC = 1
    //CRCO = 0 (0 = 1byte; 1 = 2byte)
    //PWR_UP = 1
    //PRIM_RX = 1
    setNrfRegister(CONFIG, 0b00001011, 1);

    _delay_ms(100);
}

void initPWM() {
    DDRB |= (1 << PB3) | (1 << PB4);
    ICR1 = TOP;
    TCCR1A = (1<<COM1A1) | (1<<COM1B0) | (1<<COM1B1);
    TCCR1A |= (1<<WGM11);
    TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS10);
    TIMSK = (1<<ICIE1) | (1<<TOIE1);
}

void print(uint8_t reg) {
    PORTB = getRegister(reg);
    _delay_ms(500);
    PORTB = 0;
    _delay_ms(500);
}

void printStatus() {
    if (getNfrRegister(STATUS) == 0b00001110) {
        SETBIT(PORTD, PD6);
        _delay_ms(100);
        CLEARBIT(PORTD, PD6);
        _delay_ms(100);
        SETBIT(PORTD, PD6);
        _delay_ms(100);
        CLEARBIT(PORTD, PD6);
        _delay_ms(100);
        SETBIT(PORTD, PD6);
        _delay_ms(100);
        CLEARBIT(PORTD, PD6);
        _delay_ms(100);
    }
}

main() {
    //Debug
    DDRD |= (1 << PD6);
    // DDRB = 0xFF;
    PORTB = 0b11100111;

    initMSPI();
    initNrf();
    initPWM();
    printStatus();
    // print(STATUS);

    CE_HIGH();
    sei();
    
    while(1) {}

    return 1;
}

uint8_t readCounter = 0;
ISR(TIMER1_CAPT_vect) {
    readCounter++;

    if (readCounter >= READ_DEVIDER) {
        readCounter = 0;

        if (bufferState[!currentBuffer] == 0) {
            if (!(getRegister(FIFO_STATUS) & (1<<RX_EMPTY))) {
                uint8_t readBuffer = !currentBuffer;
                TIMSK &= ~(1<<ICIE1);
                sei();

                // CE_LOW();
                getData(buffer[readBuffer]);
                bufferState[readBuffer] = 1;
                setRegister(STATUS, (1<<RX_DR) | (1<<MAX_RT) | (1<<TX_DS));
                // CE_HIGH();

                TIMSK |= (1<<ICIE1);
            }
        }
    }
}

uint8_t i = 0;
ISR(TIMER1_OVF_vect) {
    if (bufferState[currentBuffer] == 0) {
        currentBuffer = !currentBuffer;
        i++;
    } else {
        // PORTB = i;
        i = 0;

        OCR1A = OCR1B = buffer[currentBuffer][bufferCounter] << 2;

        bufferCounter++;
        if (bufferCounter >= NRF_DATA_LENGTH) {
            bufferCounter = 0;
            bufferState[currentBuffer] = 0;
            currentBuffer = !currentBuffer;
        }
    }
}