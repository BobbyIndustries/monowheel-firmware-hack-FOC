#pragma once
#include <stdint.h>


extern uint8_t backwardDrive;
extern int16_t batVoltageCalib;
extern int16_t board_temp_deg_c;
extern int16_t left_dc_curr;
extern int16_t right_dc_curr;
extern int16_t dc_curr;
extern int16_t cmdL; 
extern int16_t cmdR; 

extern volatile uint32_t main_loop_counter;
