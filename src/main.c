//
// Created by lacca on 15.03.2020.
//
// Arduino nano, button and VNH2SP30-E - based dc motor regulation
//
// clock prescaler must be set to 1024

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "main.h"
#include "uart.h"

#define PRES0 1024
#define PRES2 1024
#define F_CPU 16000000UL  // 16 MHz

#define BAUD 9600
#define MY_UBRR F_CPU/16/BAUD - 1

#define MS_TO_CLOCKS(ms, PRES)  ((((uint32_t)(ms) * (uint32_t)F_CPU) /  (1000 * (uint32_t)(PRES))))
#define MY_OCR0  MS_TO_CLOCKS(10, PRES0) - 1 // the real (actual) period between comparator trigger events is OCRn + 1, so subtract one
#define MY_OCR2  MS_TO_CLOCKS(10, PRES2) - 1 // the real (actual) period between comparator trigger events is OCRn + 1, so subtract one

#define FREQ_TO_CLOCKS(freq, PRES) (F_CPU / (PRES)) / (freq)

#define SET_BIT(REGISTER, BIT) do { REGISTER |= (1 << BIT); } while(0)
#define CLEAR_BIT(REGISTER, BIT) do {REGISTER &= ~(1 << BIT); } while(0)

static const uint8_t PWM_PIN = PD3;     // PWM output

//static const uint8_t EN_PIN = ADC7;     // A/B switch condition
static const uint8_t CS_PIN = PC6;     // current detection pin
static const uint8_t INA_PIN = PC4;    // A switch control pin
static const uint8_t INB_PIN = PC5;    // B switch control pin

static const uint8_t BUTTON_PIN = PD2;    // the number of the pushbutton pin

uint8_t buttonState = 0;

enum motor_state {
    STATE_STOP_1,
    STATE_MOVEMENT_FORWARD,
    STATE_STOP_2,
    STATE_MOVEMENT_BACKWARD,
};

const char* motor_state_string[] = {
        [STATE_STOP_1] = "STATE_STOP_1",
        [STATE_MOVEMENT_FORWARD] = "STATE_MOVEMENT_FORWARD",
        [STATE_STOP_2] = "STATE_STOP_2",
        [STATE_MOVEMENT_BACKWARD] = "STATE_MOVEMENT_BACKWARD",
};

#define MOTOR_GO(new_state) \
	{ \
		printf("FSM: -> " #new_state); \
		printf("\n"); \
		motor_state = new_state; \
		break; \
	}

enum motor_state motor_state;


ISR(INT0_vect) // debounce
{
        //TCNT2 = 0;
        if (PIND & (1 << BUTTON_PIN)) {
            if (buttonState) {
                TIMSK2  = (0 << OCIE0A); //Output Compare A Match Interrupt Disable
                TCNT2   = 0; //timer reset
                fprintf(stderr, "PIN : 1 BS: \n");
            }
            else {
                TIMSK2  = (1 << OCIE0A); //Output Compare A Match Interrupt Enable
                TCNT2   = 0; //timer reset
                fprintf(stderr, "PIN : 1 BS: 0\n");
            }
        }
        else {
            if (buttonState) {
                TCNT2   = 0; //timer reset
                TIMSK2  = (1 << OCIE0A); //Output Compare A Match Interrupt Enable
                fprintf(stderr, "PIN : 0 BS: 1\n");
            }
            else {
                TIMSK2  = (0 << OCIE0A); //Output Compare A Match Interrupt Disable
                TCNT2   = 0; //timer reset
                fprintf(stderr, "PIN : 0 BS: 0\n");
            }

        }
}
ISR(TIMER2_COMPA_vect) // debounce
{
        static uint16_t cycles_counter = 0;
        cycles_counter++;
        if (cycles_counter ==  MS_TO_CLOCKS(50, PRES2 * MY_OCR2)) {
            cycles_counter = 0;
            buttonState = !buttonState;
            TIMSK2  = (0 << OCIE0A); //Output Compare A Match Interrupt Disable
            TCNT2   = 0; //timer reset
            fprintf(stderr, "change button state: %d", buttonState);
            USART_Put_Char(buttonState + '0', stdout);
            USART_Put_Char('\n', stdout);
            switch (motor_state) {
                case STATE_STOP_1:
                    MOTOR_GO(STATE_MOVEMENT_FORWARD);
                case STATE_MOVEMENT_FORWARD:
                    break;
                case STATE_STOP_2:
                    break;
                case STATE_MOVEMENT_BACKWARD:
                    MOTOR_GO(STATE_MOVEMENT_FORWARD);
            }
        }
}
ISR(TIMER0_COMPA_vect) //
{
        static uint16_t cycles_counter = 0;
        if (cycles_counter == (uint16_t)MS_TO_CLOCKS(1000, (uint16_t)PRES0 * MY_OCR0)) {
            switch (motor_state) {
                case STATE_STOP_1:
                    break;
                case STATE_MOVEMENT_FORWARD:
                    MOTOR_GO(STATE_STOP_2);
                case STATE_STOP_2:
                    break;
                case STATE_MOVEMENT_BACKWARD:
                    MOTOR_GO(STATE_STOP_1);
            }
        }
        if (cycles_counter == MS_TO_CLOCKS(500, PRES0 * MY_OCR0)) {
            switch (motor_state) {
                case STATE_STOP_1:
                case STATE_MOVEMENT_FORWARD: MOTOR_GO(STATE_STOP_2);
                case STATE_STOP_2:
                case STATE_MOVEMENT_BACKWARD: MOTOR_GO(STATE_STOP_1);
            }
        }
}

ISR(ADC_vect)
{
    uint16_t data;
    data    = (ADCH << 8)
            | ADCL;
    printf("ADC: %d\n", data);
}

void setup_io()
{
    //set pins direction
    DDRD    =
              (0 << BUTTON_PIN)
            | (1 << PWM_PIN)
            ;
    DDRC    = 0
             // (0 << EN_PIN)
            | (0 << CS_PIN)
            | (1 << INA_PIN) | (1 << INB_PIN)
            ;

}

void setup_timers()
{
    //*timer0 for current sense threshold, timed stop*
    //Normal timer0 mode
    TCCR0A  = (0 << WGM00) | (0 << WGM01);
    TCCR0B  = (0 << WGM02)
            | (1 << CS02) | (1 << CS01) | (1 << CS00); // prescaler 1024
    //Output Compare A Match Interrupt Enable
    // TIMSK0   = (1 << OCIE0A);
    //Output compare
    OCR0A   = (uint8_t)MY_OCR0;

    //*timer2 for debounce*
    //Normal timer2 mode
    TCCR2A  = (0 << WGM20) | (0 << WGM21);
    TCCR2B  = (0 << WGM22)
            | (1 << CS02) | (1 << CS01) | (1 << CS00); // prescaler 1024
    //Output compare
    OCR2A   = (uint8_t)MY_OCR2;
    //Output Compare A Match Interrupt Enable
    TIMSK2  = (1 << OCIE0A);
}

void setup_ADC()
{
    ADMUX   = (1 << REFS1) | (1 << REFS0) // 1.1V reference
            | (0 << MUX3)  | (1 << MUX2) | (1 << MUX1) | (1 << MUX0); // ADC7
    ADCSRA  = (1 << ADEN)  | (1 << ADSC) // ADC Enable, start conversion
            | (1 << ADPS2) | (0 << ADPS1) | (1 << ADPS0); // division factor 32
}

int main(void)
{

    cli();
    USART_Init(MY_UBRR);
    stdin = &uart_in;
    stdout = &uart_out;
    stderr = &uart_out;
    setup_io();
    //enable external interrupt INT0 on PD2
    EIMSK |= (1 << INT0);
    //Any logical change on INT0 generates an interrupt request.
    EICRA |= (0 << ISC01) | (1 << ISC00);

    setup_timers();
    setup_ADC();
    sei();
    fprintf(stdout, "hello ");
    fprintf(stdout, "world\n");

    sleep_enable();

    while (true) {
        sleep_cpu();

    }







}
