// SPDX-License-Identifier: GPL-3.0-only

/*
 *  Copyright (c) 2022 David Schiller <david.schiller@jku.at>
 */

#include "i2cmaster.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <util/delay.h>

// #define STATUS_LED
// #define LED PB1

#define TAG PB1
#define ARRAY_SIZE 128                        // 256 bytes
#define TAG_SIZE 256                          // 512 bytes
#define PACKET_NO TAG_SIZE / (ARRAY_SIZE * 2) // number of packets on tag
#define DELAY 100                             // milliseconds
#define ST25DV_DATA 0xA6

// globals
uint16_t adc_offset;
uint16_t adc_values[ARRAY_SIZE];

#ifdef STATUS_LED
void init_led(void) {
    PORTB &= ~_BV(LED);
    DDRB |= _BV(LED);
}

void error_led(void) { PORTB |= _BV(LED); }

void toggle_led(void) {
    PORTB |= _BV(LED);
    _delay_ms(5);
    PORTB &= ~_BV(LED);
}
#endif

uint16_t read_adc(void) {
    // settling time for internal voltage reference
    _delay_us(100);

    // 16x oversampling (2 additional bits of resolution)
    uint16_t avg = 0;
    for (int i = 0; i < 16; i++) {
        ADCSRA |= _BV(ADSC);
        loop_until_bit_is_clear(ADCSRA, ADSC);
        avg += ADCW;
    }

    // return average
    // divide unsigned value by power of 2 -> right shift
    return avg / 16 - adc_offset;

    // return 12 bit oversampled measurement
    // return (avg >> 2) - adc_offset;
}

void init_adc(void) {
    // enable internal 1.1V reference
    ADMUX = _BV(REFS1);
    // inital settling time after selecting reference
    _delay_ms(1);
    // disable digital input buffers on ADC2 and ADC3
    DIDR0 = _BV(ADC2D) | _BV(ADC3D);
    // enable ADC
    ADCSRA = _BV(ADEN);

    // offset measurement (ADC2 for both inputs, 20x gain)
    ADMUX |= _BV(MUX2) | _BV(MUX0);
    adc_offset = read_adc();

    // set ADC2 as positive input, ADC3 as negative input and gain to 20x
    ADMUX |= _BV(MUX2) | _BV(MUX1) | _BV(MUX0);
}

void write_nfc(uint16_t address, uint16_t *buffer, int size) {
    // power on tag
    PORTB |= _BV(TAG);
    // wait for boot-up
    _delay_ms(1);

    // temporarily switch clock to 2 MHz
    clock_prescale_set(clock_div_4);
    i2c_start_wait(ST25DV_DATA);
    // write address
    i2c_write(address >> 8);
    i2c_write(address);
    // write data
    for (int i = 0; i < size; i++) {
        i2c_write(buffer[i] >> 8);
        i2c_write(buffer[i]);
    }
    i2c_stop();
    clock_prescale_set(clock_div_64);
    // poll for EEPROM write completion
    i2c_start_wait(ST25DV_DATA);
    i2c_stop();

    _delay_ms(1);
    // power off tag
    PORTB &= ~_BV(TAG);
}

#if F_CPU <= 31250UL
void init_timer(void) {
    // set timer to CTC mode
    TCCR0A = _BV(WGM01);
    // set counter to overflow after one second
    OCR0A = F_CPU / 256 - 1;
    // enable timer and divide F_CPU by 256
    TCCR0B = _BV(CS02);
    // enable timer compare interrupt in mask
    TIMSK |= _BV(OCIE0A);
    sei();
}
#endif

void init_wdt(void) {
    // enable watchdog interrupts for (roughly) every one second
    WDTCR = _BV(WDIE) | _BV(WDP2) | _BV(WDP1);
    sei();
}

void sleep(void) {
    // disable ADC
    ADCSRA &= ~_BV(ADEN);
    sleep_mode();
    // enable ADC
    ADCSRA |= _BV(ADEN);
}

EMPTY_INTERRUPT(TIM0_COMPA_vect)
EMPTY_INTERRUPT(WDT_vect)

// do not push locally used registers
// we don't use the stack at all (except for return addresses)
__attribute__((OS_main)) int main(void) {
    // divide clock by 64 (125 kHz)
    clock_prescale_set(clock_div_64);
    // turn off analog comparator
    ACSR |= _BV(ACD);
    power_usi_disable();
    power_timer0_disable();
    power_timer1_disable();

    // enable pull-ups to prevent floating inputs
    // PORTB = 0xFF;

    // initialize tag VCC pin
    PORTB &= ~_BV(TAG);
    DDRB |= _BV(TAG);

    uint16_t address = 0x0000;

#ifdef STATUS_LED
    init_led();
#endif
    i2c_init();
    init_adc();
    _delay_ms(1000);

    // more precise (exactly one second)
    // set_sleep_mode(SLEEP_MODE_IDLE);
    // init_timer();

    // less precise, but allows power-down sleep
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    init_wdt();

    for (int i = 0; i < PACKET_NO; i++) {
        for (int j = 0; j < ARRAY_SIZE; j++) {
            sleep();
#ifdef STATUS_LED
            toggle_led();
#endif
            adc_values[j] = read_adc();
        }
        // ensure that no interrupts are fired when we write to the tag
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            write_nfc(address, adc_values, ARRAY_SIZE);
        }
        address += ARRAY_SIZE * 2;
    }
    cli();
    _delay_ms(1000);
#ifdef STATUS_LED
    error_led();
#endif
    // disable ADC
    ADCSRA &= ~_BV(ADEN);
    while (1) {
    };
}
