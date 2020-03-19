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

#define BAUD 57600
#define MY_UBRR F_CPU/16/BAUD - 1

#define MS_TO_CLOCKS(ms, PRES)  ((((uint64_t)(ms) * (uint64_t)F_CPU) /  ((uint64_t)1000 * (PRES))))
#define MY_OCR0  MS_TO_CLOCKS(10, PRES0) - 1 // the real (actual) period between comparator trigger events is OCRn + 1, so subtract one
#define MY_OCR2  MS_TO_CLOCKS(10, PRES2) - 1 // the real (actual) period between comparator trigger events is OCRn + 1, so subtract one

#define FREQ_TO_CLOCKS(freq, PRES) (F_CPU / (PRES)) / (freq)

#define SET_BIT(REGISTER, BIT) do { REGISTER |= (1 << BIT); } while(0)
#define CLEAR_BIT(REGISTER, BIT) do {REGISTER &= ~(1 << BIT); } while(0)

static const uint8_t PWM_PIN = PD3;     // PWM output

//static const uint8_t EN_PIN = ADCL7;     // A/B switch condition
//static const uint8_t CS_PIN = PC6;     // current detection pin
static const uint8_t INA_PIN = PC4;    // A switch control pin
static const uint8_t INB_PIN = PC5;    // B switch control pin

static const uint8_t BUTTON_PIN = PD2;    // the number of the pushbutton pin

volatile uint8_t buttonState = 0;

uint16_t timer0_counter = 0;
uint16_t timer2_counter = 0;

uint16_t current_min = 0;
uint16_t current_max = 0;

enum motor_state {
    STATE_STOP_BACK,
    STATE_MOVEMENT_FORWARD,
    STATE_STOP_FRONT,
    STATE_MOVEMENT_BACKWARD,
};

const char* motor_state_string[] = {
        [STATE_STOP_BACK] = "STATE_STOP_BACK",
        [STATE_MOVEMENT_FORWARD] = "STATE_MOVEMENT_FORWARD",
        [STATE_STOP_FRONT] = "STATE_STOP_FRONT",
        [STATE_MOVEMENT_BACKWARD] = "STATE_MOVEMENT_BACKWARD",
};

volatile uint8_t can_read_ADC = 0;

#define MOTOR_GO(new_state) \
	{ \
		printf("FSM: %s -> %s , line: %d\n", motor_state_string[motor_state], motor_state_string[new_state], __LINE__); \
		motor_state = new_state; \
		go_##new_state(); \
		break; \
	}

volatile enum motor_state motor_state;


ISR(INT0_vect) // debounce
{
    //TCNT2 = 0;
    if (PIND & (1 << BUTTON_PIN)) {
        if (buttonState) {
            timer2_stop();
        }
        else {
            timer2_start();
        }
    }
    else {
        if (buttonState) {
            timer2_start();
        }
        else {
            timer2_stop();
        }
    }
}
ISR(TIMER2_COMPA_vect) // debounce
{
    timer2_counter++;
    printf("timer2: counter = %d\n", timer2_counter);
    if (timer2_counter == MS_TO_CLOCKS(50, PRES2 * MY_OCR2)) {
        buttonState = !buttonState;
        fprintf(stderr, "change button state: %d\n", buttonState);
        if (buttonState) {
            switch (motor_state) {
                case STATE_STOP_BACK:
                    MOTOR_GO(STATE_MOVEMENT_FORWARD);
                case STATE_MOVEMENT_FORWARD:
                    break;
                case STATE_STOP_FRONT:
                    break;
                case STATE_MOVEMENT_BACKWARD:
                    MOTOR_GO(STATE_MOVEMENT_FORWARD);
            }
        } else {
            switch (motor_state) {
                case STATE_STOP_BACK:
                    break;
                case STATE_MOVEMENT_FORWARD:
                    MOTOR_GO(STATE_MOVEMENT_BACKWARD);
                case STATE_STOP_FRONT:
                    MOTOR_GO(STATE_MOVEMENT_BACKWARD);
                case STATE_MOVEMENT_BACKWARD:
                    break;
            }
        }
        timer2_stop();
    }
}

ISR(TIMER0_COMPA_vect) //
{
    timer0_counter++;
    printf("timer0: counter = %d\n", timer0_counter);
    if (timer0_counter == MS_TO_CLOCKS(1000, PRES0 * MY_OCR0)) {
        switch (motor_state) {
            case STATE_STOP_BACK:
                break;
            case STATE_MOVEMENT_FORWARD:
                MOTOR_GO(STATE_STOP_FRONT);
            case STATE_STOP_FRONT:
                break;
            case STATE_MOVEMENT_BACKWARD:
                MOTOR_GO(STATE_STOP_BACK);
        }
        timer0_stop();
    }

    if ((timer0_counter >= MS_TO_CLOCKS(50, PRES0 * MY_OCR0))
     && (current_min > (current_max >> 2))) { // current_max * 0.5
        printf("LOCKED -- EMERGENCY STOP (min = %u, max = %u within 50 ms)", current_min, current_max);
        switch (motor_state) {
            case STATE_STOP_BACK:
                break;
            case STATE_MOVEMENT_FORWARD:
                MOTOR_GO(STATE_STOP_FRONT);
            case STATE_STOP_FRONT:
                break;
            case STATE_MOVEMENT_BACKWARD:
                MOTOR_GO(STATE_STOP_BACK);
        }
        timer0_stop();
    }
}

ISR(ADC_vect)
{
    static uint8_t current_sense_ctr = 0;

    uint16_t data;
    data  = ADCL;
    data |= ADCH << 8;

    if (data < current_min) {
        current_min = data;
    }
    if (data > current_max) {
        current_max = data;
    }

    if (data > (current_min + (current_min >> 1) + (current_min >> 2))) { // current_min * 1.75
        ++current_sense_ctr;
        printf("current sense alert (min = %u, current = %u)\n", current_min, data);
    } else {
        current_sense_ctr = 0;
    }

    if (current_sense_ctr >= 3) {
        printf("stopping by current sense (alert for last 3 measurements)\n");
        switch (motor_state) {
            case STATE_STOP_BACK:
                break;
            case STATE_MOVEMENT_FORWARD:
                MOTOR_GO(STATE_STOP_FRONT);
            case STATE_STOP_FRONT:
                break;
            case STATE_MOVEMENT_BACKWARD:
                MOTOR_GO(STATE_STOP_BACK);
        }
        current_sense_ctr = 0;
    }
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
            //| (0 << CS_PIN)
            | (1 << INA_PIN) | (1 << INB_PIN)
            ;

}

void setup_timers()
{
    //*timer0 for current sense threshold, timed stop*
    // CTC timer0 mode
    TCCR0A  = (1 << WGM01) | (0 << WGM00);
    TCCR0B  = (0 << WGM02)
            | (0 << CS02) | (0 << CS01) | (0 << CS00); // timer stop
    //Output Compare A Match Interrupt Enable
    TIMSK0   = (1 << OCIE0A);
    //Output compare
    OCR0A   = (uint8_t)MY_OCR0;

    //*timer2 for debounce*
    // CTC timer2 mode
    TCCR2A  = (1 << WGM21) | (0 << WGM20);
    TCCR2B  = (0 << WGM22)
            | (0 << CS22) | (0 << CS21) | (0 << CS20); // timer stop
    //Output compare
    OCR2A   = (uint8_t)MY_OCR2;
    //Output Compare A Match Interrupt Enable
    TIMSK2  = (1 << OCIE2A);
}

void timer0_start()
{
    printf("timer0: start\n");
    timer0_counter = 0;
    TCNT0 = 0;
    TCCR0B |= ((1 << CS02) | (0 << CS01) | (1 << CS00)); // prescaler 1024
}

void timer0_stop()
{
    printf("timer0: stop\n");
    TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00)); // stop timer
    TIFR0 |= (1 << OCF0A);
    TCNT0 = 0;
}

void timer2_start()
{
    printf("timer2: start\n");
    timer2_counter = 0;
    TCNT0 = 0;
    TCCR2B |= ((1 << CS22) | (0 << CS21) | (1 << CS20)); // prescaler 1024
}

void timer2_stop()
{
    printf("timer2: stop\n");
    TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20)); // stop timer
    TIFR2 |= (1 << OCF2A);
    TCNT2 = 0;
}


void setup_ADC()
{
    ADMUX   = (1 << REFS1) | (1 << REFS0) // 1.1V reference
            | (0 << MUX3)  | (1 << MUX2)  | (1 << MUX1) | (0 << MUX0); // ADC6
    ADCSRA  = (1 << ADEN)
            | (1 << ADSC)
            | (1 << ADIE)// ADC Enable, start conversion
            | (1 << ADATE)
            | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler 128
}

void go_STATE_STOP_FRONT()
{
    CLEAR_BIT(PORTC, INA_PIN);
    CLEAR_BIT(PORTC, INB_PIN);
    timer0_stop();
}

void go_STATE_MOVEMENT_FORWARD()
{
    SET_BIT(PORTC, INA_PIN);
    CLEAR_BIT(PORTC, INB_PIN);
    timer0_start();
    current_min = UINT16_MAX;
    current_max = 0;
}

void go_STATE_STOP_BACK()
{
    CLEAR_BIT(PORTC, INA_PIN);
    CLEAR_BIT(PORTC, INB_PIN);
    timer0_stop();
}

void go_STATE_MOVEMENT_BACKWARD()
{
    CLEAR_BIT(PORTC, INA_PIN);
    SET_BIT(PORTC, INB_PIN);
    timer0_start();
    current_min = UINT16_MAX;
    current_max = 0;
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

    SET_BIT(PORTD, PWM_PIN);

    setup_timers();
    setup_ADC();
    motor_state = STATE_STOP_BACK;

    sei();

    sleep_enable();
    while (true) {
        sleep_cpu();
    }
}
