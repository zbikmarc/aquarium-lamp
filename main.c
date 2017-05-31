/*
 * main.c
 *
 * Created: 2015-01-16 18:37:51
 *  Author: Marcin ¯bik
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define BAUD 9600

#define low(x)   ((x) & 0xFF)
#define high(x)   (((x)>>8) & 0xFF)

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>

#include "spi.h"

#include "ds3231/rtc.h"

volatile char alarmFlag = 0;

// Light Levels [hour][quarter][T5=0 or LED=1]
static uint8_t lightLevels[24][4][2] = {
    {            // 0
        {31,0},  // 0 : 00
        {31,0},  // 0 : 15
        {31,0},  // 0 : 30
        {31,0}   // 0 : 45
    },
    {            // 1
        {31,0},  // 1 : 00
        {31,0},  // 1 : 15
        {31,0},  // 1 : 30
        {31,0}   // 1 : 45
    },
    {            // 2
        {31,0},  // 2 : 00
        {31,0},  // 2 : 15
        {31,0},  // 2 : 30
        {31,0}   // 2 : 45
    },
    {            // 3
        {31,0},  // 3 : 00
        {31,0},  // 3 : 15
        {31,0},  // 3 : 30
        {31,0}   // 3 : 45
    },
    {            // 4
        {31,0},  // 4 : 00
        {31,0},  // 4 : 15
        {31,0},  // 4 : 30
        {31,0}   // 4 : 45
    },
    {            // 5
        {31,0},  // 5 : 00
        {31,0},  // 5 : 15
        {31,0},  // 5 : 30
        {31,0}   // 5 : 45
    },
    {            // 6
        {39,0},  // 6 : 00
        {47,0},  // 6 : 15
        {55,0},  // 6 : 30
        {63,0}   // 6 : 45
    },
    {            // 7
        {71,0},  // 7 : 00
        {79,0},  // 7 : 15
        {95,0},  // 7 : 30
        {111,0}  // 7 : 45
    },
    {            // 8
        {127,0}, // 8 : 00
        {143,0}, // 8 : 15
        {159,0}, // 8 : 30
        {175,0}  // 8 : 45
    },
    {            // 9
        {191,0}, // 9 : 00
        {207,0}, // 9 : 15
        {223,0}, // 9 : 30
        {239,0}  // 9 : 45
    },
    {            // 10
        {250,0}, // 10 : 00
        {250,0}, // 10 : 15
        {250,0}, // 10 : 30
        {250,0}  // 10 : 45
    },
    {            // 11
        {250,0}, // 11 : 00
        {250,0}, // 11 : 15
        {250,0}, // 11 : 30
        {250,0}  // 11 : 45
    },
    {            // 12
        {250,0}, // 12 : 00
        {250,0}, // 12 : 15
        {250,0}, // 12 : 30
        {250,0}  // 12 : 45
    },
    {            // 13
        {250,0}, // 13 : 00
        {250,0}, // 13 : 15
        {250,0}, // 13 : 30
        {250,0}  // 13 : 45
    },
    {            // 14
        {250,0}, // 14 : 00
        {250,0}, // 14 : 15
        {250,0}, // 14 : 30
        {250,0}  // 14 : 45
    },
    {            // 15
        {250,0}, // 15 : 00
        {250,0}, // 15 : 15
        {250,0}, // 15 : 30
        {250,0}  // 15 : 45
    },
    {            // 16
        {250,0}, // 16 : 00
        {250,0}, // 16 : 15
        {250,0}, // 16 : 30
        {250,0}  // 16 : 45
    },
    {            // 17
        {250,0}, // 17 : 00
        {250,0}, // 17 : 15
        {239,0}, // 17 : 30
        {223,0}  // 17 : 45
    },
    {            // 18
        {207,0}, // 18 : 00
        {191,0}, // 18 : 15
        {175,0}, // 18 : 30
        {159,0}  // 18 : 45
    },
    {             // 19
        {143,71}, // 19 : 00
        {127,71}, // 19 : 15
        {95,63},  // 19 : 30
        {63,63}   // 19 : 45
    },
    {            // 20
        {55,55}, // 20 : 00
        {47,47}, // 20 : 15
        {31,47}, // 20 : 30
        {31,39}  // 20 : 45
    },
    {            // 21
        {31,39}, // 21 : 00
        {31,31}, // 21 : 15
        {31,31}, // 21 : 30
        {31,25}  // 21 : 45
    },
    {            // 22
        {31,25}, // 22 : 00
        {31,21}, // 22 : 15
        {31,21}, // 22 : 30
        {31,17}  // 22 : 45
    },
    {            // 23
        {31,15}, // 23 : 00
        {31,11}, // 23 : 15
        {31,7},  // 23 : 30
        {31,0}   // 23 : 45
    }
};

ISR(INT1_vect)
{
    alarmFlag = 1;
}

void initPWM()
{
    // Timer/PWM 0
    TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1 << COM0A1);
    //TCCR0B |= (1 << CS00)    | (1 << CS01);
    TCCR0B |= (1 << CS02);
    DDRD |= (1 << PD6);

    // Timer/PWM1
    TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);
    TCCR1B |= (1<<WGM13)  | (1<<WGM12)  | (1<<CS10);
    ICR1 = 1023;
    DDRB |= (1 << PB1);
}

void mcp4x01_set_voltage(uint8_t voltage){
    uint16_t packet = 0;

    // Set gain (  1 = 1x, 0 = 2x)
    packet |= 0 << 13;

    // Activate mode (Vout available)
    packet |= 1 << 12;

    // Set DAC output register
    packet |= voltage << 4;


    // Chip Select to low
    PORTC &= ~(1 << PC0);

    // Send high byte first
    spi_send(high(packet));
    spi_send(low(packet));

    //Chip select to hi
    PORTC |= (1 << PC0);

    // Latch down and up again
    PORTC &= ~(1 << PC1);
    _delay_ms(2);
    PORTC |= (1 << PC1);

}

int main(void)
{
    uint8_t q,statusRegister, controlRegister;
    struct tm* actualTime = NULL;


    // SET PD3 (INT1) (alarm interrupt)
    DDRD = (0 << PD3);
    PORTD = (1 << PD3);

    // SET PC0 PC1 as output and set to HI
    DDRC |= (1 << DDC0) | (1 << DDC1);
    PORTC |= (1 << PC0) | (1 << PC1);

    // Interrupt vector
    EICRA = (1 << ISC11) | (0 << ISC10);
    EIMSK = (1 << INT1);

    sei();

    spi_init(0,1,0,1,0);
    initPWM();
    twi_init_master();
    rtc_init();

//    actualTime->sec = 0;
//    actualTime->min = 11;
//    actualTime->hour = 22;
//    actualTime->mon = 5;
//    actualTime->mday = 28;
//    actualTime->wday = 7;
//    actualTime->year = 2017;
//    rtc_set_time(actualTime);

    // Set Alarm 1: when sec = 30
    rtc_write_byte(0b00110000, 0x07);
    rtc_write_byte(0b10000000, 0x08);
    rtc_write_byte(0b10000000, 0x09);
    rtc_write_byte(0b10000000, 0x0a);
    statusRegister = rtc_read_byte(0x0f);
    rtc_write_byte(statusRegister & ~0b00000001, 0x0f);

    // Set Alarm 2 every 1 minute (sec = 00)
    rtc_write_byte(0b10000000, 0x0b);
    rtc_write_byte(0b10000000, 0x0c);
    rtc_write_byte(0b10000000, 0x0d);
    statusRegister = rtc_read_byte(0x0f);
    rtc_write_byte(statusRegister & ~0b00000010, 0x0f);

    // Enable DS3231 Interrupts globally and for Alarm 1
    controlRegister = rtc_read_byte(0x0e);
    rtc_write_byte(controlRegister | 0b00000111, 0x0e);

    while(1){
        if (alarmFlag == 1)
        {

            actualTime = rtc_get_time();
            if (actualTime->min <= 15 ) {
                q = 0;
            } else if ( actualTime->min > 15 &&  actualTime->min <= 30 ) {
                q = 1;
            } else if ( actualTime->min > 30 &&  actualTime->min <= 45 ) {
                q = 2;
            } else if ( actualTime->min > 45) {
                q = 3;
            }

            mcp4x01_set_voltage(lightLevels[actualTime->hour][q][0]);
            OCR1A = lightLevels[actualTime->hour][q][1];

            alarmFlag = 0;
            // Clear both alarms flags
            statusRegister = rtc_read_byte(0x0f);
            rtc_write_byte(statusRegister & ~0b00000011, 0x0f);
        }
    }

    return 0;
}
