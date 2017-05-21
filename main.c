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
#include "uart.h"

#include "ds3231/rtc.h"

volatile char alarmFlag = 0;

// Light Levels [hour][T5=0 or LED=1]
static uint8_t lightLevels[24][2][2] = {
         {              // 24 or 00
                {63,0}, // 24 or 00 : 00
                {63,0}  // 24 or 00 : 30
         },
         {              // 1
                {63,0}, // 1 : 00
                {63,0}  // 1 : 30
         },
         {              // 2
                {63,0}, // 2 : 00
                {63,0}  // 2 : 30
         },
         {              // 3
                {63,0}, // 3 : 00
                {63,0}  // 3 : 30
         },
         {              // 5
                {63,0}, // 4 : 00
                {63,0}  // 4 : 30
         },
         {              // 5
                {63,0}, // 5 : 00
                {63,0}  // 5 : 30
         },
         {              // 6
                {79,0}, // 6 : 00
                {95,0}  // 6 : 30
         },
         {              // 7
                {111,0}, // 7 : 00
                {127,0}  // 7 : 30
         },
         {              // 8
                {143,0}, // 8 : 00
                {159,0}  // 8 : 30
         },
         {               // 9
                {175,0}, // 9 : 00
                {191,0}  // 9 : 30
         },
         {               // 10
                {207,0}, // 10 : 00
                {223,0}  // 10 : 30
         },
         {               // 11
                {239,0}, // 11 : 00
                {255,0}  // 11 : 30
         },
         {               // 12
                {255,0}, // 12 : 00
                {255,0}  // 12 : 30
         },
         {               // 13
                {255,0}, // 13 : 00
                {255,0}  // 13 : 30
         },
         {               // 14
                {255,0}, // 14 : 00
                {255,0}  // 14 : 30
         },
         {               // 15
                {255,0}, // 15 : 00
                {255,0}  // 15 : 30
         },
         {               // 16
                {255,0}, // 16 : 00
                {255,0}  // 16 : 30
         },
         {               // 17
                {255,0}, // 17 : 00
                {255,0}  // 17 : 30
         },
         {               // 18
                {223,0}, // 18 : 00
                {191,0}  // 18 : 30
         },
         {                // 19
                {159,64}, // 19 : 00
                {127,64}  // 19 : 30
         },
         {               // 20
                {95,64}, // 20 : 00
                {63,48}  // 20 : 30
         },
         {               // 21
                {63,40}, // 21 : 00
                {63,32}  // 21 : 30
         },
         {               // 22
                {63,32}, // 22 : 00
                {63,24}  // 22 : 30
         },
         {               // 23
                {63,16}, // 23 : 00
                {63,0}   // 23 : 30
         }
};



ISR(INT4_vect)
{
    alarmFlag = 1;
}

void initPWM()
{
    // Timer/PWM 0
    TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1 << COM0A1);
    //TCCR0B |= (1 << CS00)    | (1 << CS01);
    TCCR0B |= (1 << CS02);
    DDRB |= (1 << DDB7);

    // Timer/PWM1
    TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);
    TCCR1B |= (1<<WGM13)  | (1<<WGM12)  | (1<<CS10);
    ICR1 = 1023;
    DDRB |= (1 << DDB5);
}

void mcp4x01_set_voltage(uint8_t voltage){
    uint16_t packet = 0;

    // Set gain (  1 = 1x, 0 = 2x)
    packet |= 1 << 13;

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
    uint8_t statusRegister, controlRegister;
    uint8_t i = 0;
    uint8_t level[] = {0, 63, 127, 195, 255, 195 ,127, 63};
    //uint8_t level[] = {0, 0, 64, 64, 128, 128 , 250, 250};
    char buffer[256];
    struct tm* actualTime = NULL;


    // SET PE4 (INT4)
    DDRE = (0 << PE4);
    PORTE = (1 << PE4);

    // SET PC0 PC1 as output and set to HI
    DDRC |= (1 << DDC0) | (1 << DDC1);
    PORTC |= (1 << PC0) | (1 << PC1);

    // Interrupt vector
    EICRB = (1 << ISC41) | (0 << ISC40);
    EIMSK = (1 << INT4);

    sei();

    spi_init(0,1,0,1,0);
    initPWM();
    uart_init((UART_BAUD_SELECT((BAUD),F_CPU)));
    twi_init_master();
    rtc_init();

//    actualTime->sec = 0;
//    actualTime->min = 49;
//    actualTime->hour = 20;
//    actualTime->mon = 5;
//    actualTime->mday = 16;
//    actualTime->wday = 2;
//    actualTime->year = 2017;
//    rtc_set_time(actualTime);

    uart_puts("test\n\r");

    actualTime = rtc_get_time();
    sprintf(buffer, "%d-%02d-%02d %02d:%02d:%02d\n\r", actualTime->year, actualTime->mon, actualTime->mday, actualTime->hour, actualTime->min, actualTime->sec);
    uart_puts(buffer);

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

//    while(1){
//        for(i=0; i < 8; i++)
//        {
//            mcp4x01_set_voltage(level[i]);
//            _delay_ms(10000);
//        }
//    }



//    uint16_t pwm_level = 0;
//    uint16_t max_pwm_level = 1023;
//    while(1)
//    {
//        for (pwm_level = 0; pwm_level < max_pwm_level; pwm_level += 2){
//            OCR1A = pwm_level;
//            _delay_ms(100);
//        }
//        for (pwm_level = max_pwm_level; pwm_level > 0; pwm_level -= 2)
//        {
//            OCR1A = pwm_level;
//            _delay_ms(100);
//        }
//    }
//    OCR1A = 24;
    while(1){
        if (alarmFlag == 1)
        {
            uart_puts("Alarm!\r\n");
            actualTime = rtc_get_time();
            sprintf(buffer, "%d-%02d-%02d %02d:%02d:%02d\n\r", actualTime->year, actualTime->mon, actualTime->mday, actualTime->hour, actualTime->min, actualTime->sec);
            uart_puts(buffer);


            actualTime = rtc_get_time();
            if (actualTime->min < 30 ) {
                mcp4x01_set_voltage(lightLevels[actualTime->hour][0][0]);
                OCR1A = lightLevels[actualTime->hour][0][1];
                sprintf(buffer, "mcp4x01 = %d\n\rOCR1A = %d\n\r", lightLevels[actualTime->hour][0][0], lightLevels[actualTime->hour][0][1]);
                uart_puts(buffer);
            } else {
                mcp4x01_set_voltage(lightLevels[actualTime->hour][1][0]);
                OCR1A = lightLevels[actualTime->hour][1][1];
                sprintf(buffer, "mcp4x01 = %d\n\rOCR1A = %d\n\r", lightLevels[actualTime->hour][1][0], lightLevels[actualTime->hour][1][1]);
                uart_puts(buffer);
            }

            alarmFlag = 0;
            // Clear both alarms flags
            statusRegister = rtc_read_byte(0x0f);
            rtc_write_byte(statusRegister & ~0b00000011, 0x0f);
        }
    }

    return 0;
}
