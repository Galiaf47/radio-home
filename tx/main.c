/**
 * Fuse bits:
 * Low 0xe4
 * High 0xdf
 * Ext 0xff
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "nRF24L01.h"

#define CSN_PORT PORTA
#define CSN_PIN PORTA1
#define CE_PORT PORTA
#define CE_PIN PORTA0

#define BITRATE 9600
#define BAUD ((F_CPU / (2 * BITRATE)) - 1)
#define NRF_DATA_LENGTH 5
#define SET_REGISTER_DELAY 100
#define NRF_LISTEN_DELAY 20

#define SETBIT(x, y) x |= (1 << y)
#define CLEARBIT(x, y) x &= (~(1 << y))

#define CSN_LOW() CLEARBIT(CSN_PORT, CSN_PIN)
#define CSN_HIGH() SETBIT(CSN_PORT, CSN_PIN)
#define CE_LOW() CLEARBIT(CE_PORT, CE_PIN)
#define CE_HIGH() SETBIT(CE_PORT, CE_PIN)

/*
 * USART - SPI
 */
void initMSPI() {
    UBRRH = 0;
    UBRRL = 0;
    /* Setting the XCK port pin as output, enables master mode. */
    XCK_DDR |= (1<<XCK_BIT);
    /* Set MSPI mode of operation and SPI data mode 0. */
    UCSRC = (1<<UMSEL1)|(1<<UMSEL0)|(0<<UCPHA)|(0<<UCPOL);
    /* Enable receiver and transmitter. */
    UCSRB = (1<<RXEN)|(1<<TXEN);
    /* Set baud rate. */
    /* IMPORTANT: The Baud Rate must be set after the transmitter is enabled
    */
    UBRRH = (BAUD >> 8) & 0xff;
    UBRRL = (BAUD & 0xff);
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

uint8_t setNrfTransmitData(uint8_t *data, uint8_t data_length) {
    initNrfRegister(W_TX_PAYLOAD);
   
    uint8_t i;
    uint8_t status;
    for (i=0; i<data_length; i++) {
        status = writeMSPI(data[i]);
        _delay_us(SET_REGISTER_DELAY);
    }

    CSN_HIGH();

   return status;
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

void initNrf() {
    //PA1 - CSN; PA0 - CE
    DDRA |= (1 << PA1) | (1 << PA0);
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
    setNrfRegister(SETUP_AW, 0x03, 1);

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

    //Primary transmitter (PRIM_RX = 0); Power up (PWR_UP = 1); CRC disabled (EN_CRC = 0)
    setNrfRegister(CONFIG, 0b00010010, 1);

    _delay_ms(100);
}


void listenNrf() {
    CE_HIGH();
    _delay_us(NRF_LISTEN_DELAY);
    CE_LOW();
    _delay_us(NRF_LISTEN_DELAY);
}

void print(uint8_t reg) {
    PORTB = getNfrRegister(reg);
    _delay_ms(500);
    PORTB = 0;
    _delay_ms(500);
}

main() {
    DDRD |= (1 << PD5);
    DDRB = 0xFF;

    initMSPI();
    initNrf();

    uint8_t data[NRF_DATA_LENGTH];// = {0x93,0x93,0x93,0x93,0x93};
    int i;
    for(i = 0; i < NRF_DATA_LENGTH; i++) {
        data[i] = 0x93;
    }

    int count = 0;
    while(1) {
        count++;
        if (count == 500) {
            count = 0;
            print(STATUS);

            uint8_t status = getNfrRegister(STATUS);
            if (status & (1<<TX_DS)) {
                setNrfRegister(STATUS, (1<<TX_DS), 1);
            }

            setNrfTransmitData(data, NRF_DATA_LENGTH);
        }

        listenNrf();
    }

    return 1;
}