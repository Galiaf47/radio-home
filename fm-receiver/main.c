/**
 * http://extremeelectronics.co.in/avr-tutorials/software-i2c-library-for-avr-mcus/
 * http://rtr.ca/fmradio/ar1000F_progguide-0.81.pdf
 */

#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include "i2csoft.h"

#define SETBIT() PORTB |= (1 << PB0)
#define CLEARBIT() PORTB &= (~(1 << PB0))
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// 7 bit address and WRITE bit
#define ADDR_W 0x20 //001 0000 0 (R/W = 0 is WRITE)
#define ADDR_R 0x21 //001 0000 1 (R/W = 1 is READ)

#define CHAN_MASK 0xFE00

#define TUNE_REG 2
#define TUNE_BIT 9

#define STC_REG 19
#define STC_BYTE 5

const uint16_t defaultRegisters[18] = {
    0xFFFB,     // R0:  1111 1111 1111 1011 
    0x5B15,     // R1:  0101 1011 0001 0101 - Mono (D3), Softmute (D2), Hardmute (D1)  !! SOFT-MUTED BY DEFAULT !!
    0xD0B9,     // R2:  1101 0000 1011 1001 - Tune/Channel
    0xA010,     // R3:  1010 0000 0001 0000 - Seekup (D15), Seek bit (D14), Space 100kHz (D13), Seek threshold: 16 (D6-D0)
    0x0780,     // R4:  0000 0111 1000 0000
    0x28AB,     // R5:  0010 1000 1010 1011
    0x6400,     // R6:  0110 0100 0000 0000
    0x1EE7,     // R7:  0001 1110 1110 0111
    0x7141,     // R8:  0111 0001 0100 0001 
    0x007D,     // R9:  0000 0000 0111 1101
    0x82C6,     // R10: 1000 0010 1100 0110 - Seek wrap (D3)
    0x4E55,     // R11: 0100 1110 0101 0101
    0x970C,     // R12: 1001 0111 0000 1100
    0xB845,     // R13: 1011 1000 0100 0101
    0xFC2D,     // R14: 1111 1100 0010 1101 - Volume control 2 (D12-D15)
    0x8097,     // R15: 1000 0000 1001 0111
    0x04A1,     // R16: 0000 0100 1010 0001
    0xDF61      // R17: 1101 1111 0110 0001
};

writeRegister(uint8_t address, uint16_t data) {
    SoftI2CStart();
    SoftI2CWriteByte(ADDR_W);
    SoftI2CWriteByte(address);
    SoftI2CWriteByte((data & 0xFF00) >> 8);
    SoftI2CWriteByte((data & 0x00FF));
    SoftI2CStop();
}

uint16_t readRegister(uint8_t address) {
    uint16_t data;

    SoftI2CStart();
    SoftI2CWriteByte(ADDR_W);
    SoftI2CWriteByte(address);

    SoftI2CStart();
    SoftI2CWriteByte(ADDR_R);
    data = (SoftI2CReadByte(1) << 8);
    data += SoftI2CReadByte(0);
    SoftI2CStop();

    return data;
}

writeRegisterBit(uint8_t address, uint8_t bit, bool value) {
    uint16_t reg = readRegister(address);

    if(value) {
        writeRegister(address, reg | (1 << bit));
    } else {
        writeRegister(address, reg & ~(1 << bit));
    }
}

init() {
    SoftI2CInit();

    uint8_t i;
    for(i = 1; i < 18; i++) {
        writeRegister(i, defaultRegisters[i]);
    }

    writeRegister(0, defaultRegisters[0]);

    while(!CHECK_BIT(readRegister(STC_REG), STC_BYTE)) {
        _delay_ms(100);
    }
}

setFrequency(uint16_t freq) {
    //tune = 0
    writeRegisterBit(TUNE_REG, TUNE_BIT, 0);

    uint16_t reg = readRegister(TUNE_REG) & CHAN_MASK;
    writeRegister(TUNE_REG, (reg | (freq - 690)));

    //tune = 1
    writeRegisterBit(TUNE_REG, TUNE_BIT, 1);
}

int main() {
    DDRB |= (1<<PB0);
    _delay_ms(1500);

    init();
    setFrequency(876);

    SETBIT();
    return 0;
}
