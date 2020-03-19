//
// Created by lacca on 17.03.2020.
//

#ifndef CODE_MAIN_H
#define CODE_MAIN_H

void setup_io();
void setup_timers();
void timer0_start();
void timer0_stop();
void timer2_start();
void timer2_stop();

void go_STATE_STOP_FRONT();
void go_STATE_MOVEMENT_FORWARD();
void go_STATE_STOP_BACK();
void go_STATE_MOVEMENT_BACKWARD();

#endif //CODE_MAIN_H


