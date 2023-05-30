#pragma once
#include <stdint.h>

extern volatile int pwml;               // global variable for pwm left. -1000 to 1000
extern volatile int pwmr;               // global variable for pwm right. -1000 to 1000

extern uint8_t enable;                  // global variable for motor enable

extern int16_t batVoltage;              // global variable for battery voltage

extern volatile uint32_t buzzerTimer;
extern uint8_t buzzerCount;             // global variable for the buzzer counts. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerFreq;              // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern;           // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...
