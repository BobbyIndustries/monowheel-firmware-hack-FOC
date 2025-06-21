/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
* Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdlib.h> // for abs()
#include "stm32f1xx_hal.h"
#include "bldc.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "BLDC_controller.h"      /* BLDC's header file */
#include "rtwtypes.h"
#include "comms.h"
#include "control.h"
#include "crc32.h"

#if defined(DEBUG_I2C_LCD) || defined(SUPPORT_LCD)
#include "hd44780.h"
#endif

void SystemClock_Config(void);

//------------------------------------------------------------------------
// Global variables set externally
//------------------------------------------------------------------------




volatile uint8_t uart_buf[200];

//---------------
#ifdef CONTROL_ADC
  #define ADC_MID 2048
  #define THROTTLE_MAX 1000
  #define ADC_MAX 4095
  #define DEAD_ZONE 64
  #define VAL_CNT 2
  #define BUFFERSIZE 256  
uint32_t index_buff_vals[VAL_CNT];
uint32_t buff_vals[VAL_CNT][BUFFERSIZE];
uint32_t cur_buff_val_sum[VAL_CNT];
uint32_t value_buffer(uint32_t in, int val)
{
  cur_buff_val_sum[val] -= buff_vals[val][index_buff_vals[val]];
  cur_buff_val_sum[val] += (buff_vals[val][index_buff_vals[val]] = in);
  index_buff_vals[val] = (index_buff_vals[val] + 1) % (BUFFERSIZE);
  return (cur_buff_val_sum[val] / (BUFFERSIZE));
}

#define sign(x) (x == 0 ? 0 : (x < 0 ? -1 : 1))

int clean_adc_full(uint32_t inval)
{
  int outval = (int)(inval) - ADC_MID;
  int abs_outval = abs(outval);
  if (abs_outval < (DEAD_ZONE / 2)) // deadzone
    return 0;
  else
    abs_outval -= (DEAD_ZONE / 2);
  if (abs_outval > (ADC_MAX - ADC_MID - DEAD_ZONE * 3 / 2))
    return THROTTLE_MAX * sign(outval);
  return abs_outval * THROTTLE_MAX / (ADC_MAX - ADC_MID - DEAD_ZONE * 3 / 2) * sign(outval);
}

int clean_adc_half(uint32_t inval)
{
  int outval = (uint32_t)inval;
  if (abs(outval) > (ADC_MAX - ((DEAD_ZONE * 3) / 2)))
    return THROTTLE_MAX;
  else if(abs(outval) < (((DEAD_ZONE * 3) / 2)))
    return 0;
  return outval * (THROTTLE_MAX / 2) / (ADC_MAX - DEAD_ZONE * 3);
}

int PowXplusX(int cleaned_adc,int maximum_value)
{
  return ((cleaned_adc * cleaned_adc / maximum_value) * 2 * SIGN(cleaned_adc) * SIGN(maximum_value)  + cleaned_adc) / 3;
}

#endif



//------------------------------------------------------------------------
// Global variables set here in main.c
//------------------------------------------------------------------------
uint8_t backwardDrive;
volatile uint32_t main_loop_counter;
int16_t batVoltageCalib;         // global variable for calibrated battery voltage
int16_t board_temp_deg_c;        // global variable for calibrated temperature in degrees Celsius
int16_t left_dc_curr;            // global variable for Left DC Link current 
int16_t right_dc_curr;           // global variable for Right DC Link current
int16_t dc_curr;                 // global variable for Total DC Link current 
int16_t cmdL;                    // global variable for Left Command 
int16_t cmdR;                    // global variable for Right Command 

//------------------------------------------------------------------------
// Local variables
//------------------------------------------------------------------------
#if defined(FEEDBACK_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART3)
typedef struct{
  uint16_t  start;
  uint16_t   steps0;
  uint16_t   steps1;
  int16_t   speedR_meas;
  int16_t   speedL_meas;
  int16_t   batVoltage;
  int16_t   boardTemp;
  uint16_t  cmdLed;
  uint16_t  checksumL;
  uint16_t  checksumH;
} SerialFeedback;
static SerialFeedback Feedback;
#endif
#if defined(FEEDBACK_SERIAL_USART2)
static uint8_t sideboard_leds_L;
#endif
#if defined(FEEDBACK_SERIAL_USART3)
static uint8_t sideboard_leds_R;
#endif

#ifdef VARIANT_TRANSPOTTER
  uint8_t  nunchuk_connected;

  static uint8_t  checkRemote = 0;
  static uint16_t distance;
  static float    steering;
  static int      distanceErr;  
  static int      lastDistance = 0;
  static uint16_t transpotter_counter = 0;
#endif

static int16_t    speed;                // local variable for speed. -1000 to 1000
#ifndef VARIANT_TRANSPOTTER
  static int16_t  steer;                // local variable for steering. -1000 to 1000
  static int16_t  steerRateFixdt;       // local fixed-point variable for steering rate limiter
  static int16_t  speedRateFixdt;       // local fixed-point variable for speed rate limiter
  static int32_t  steerFixdt;           // local fixed-point variable for steering low-pass filter
  static int32_t  speedFixdt;           // local fixed-point variable for speed low-pass filter
#endif

static uint32_t    buzzerTimer_prev = 0;
static uint32_t    inactivity_timeout_counter;
static MultipleTap MultipleTapBrake;    // define multiple tap functionality for the Brake pedal

static uint16_t rate = RATE; // Adjustable rate to support multiple drive modes on startup

#ifdef MULTI_MODE_DRIVE
  static uint8_t drive_mode;
  static uint16_t max_speed;
#endif


int main(void) {

  HAL_Init();
  __HAL_RCC_AFIO_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  SystemClock_Config();

  __HAL_RCC_DMA1_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  BLDC_Init();        // BLDC Controller Init

  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);   // Activate Latch
  Input_Lim_Init();   // Input Limitations Init
  Input_Init();       // Input Init

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  poweronMelody();
  bldc_start_calibration();
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
  
  int32_t board_temp_adcFixdt = adc_buffer.temp << 16;  // Fixed-point filter output initialized with current ADC converted to fixed-point
  int16_t board_temp_adcFilt  = adc_buffer.temp;

  #ifdef CONTROL_ADC
    for (int i = 0; i < VAL_CNT; i++)
    {
      cur_buff_val_sum[i] = index_buff_vals[i] = 0;
      for (int j = 0; j < BUFFERSIZE; j++)
        cur_buff_val_sum[i] += (buff_vals[i][j] = ADC_MID);
    }
  #endif

  // Loop until button is released
  while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) { HAL_Delay(10); }

  #ifdef MULTI_MODE_DRIVE
    // Wait until triggers are released. Exit if timeout elapses (to unblock if the inputs are not calibrated)
    int iTimeout = 0;
    while((adc_buffer.l_rx2 + adc_buffer.l_tx2) >= (input1[0].min + input2[0].min) && iTimeout++ < 300) {
      HAL_Delay(10);
    }
  #endif

  while(1) {
    if (buzzerTimer - buzzerTimer_prev > 16*DELAY_IN_MAIN_LOOP) {   // 1 ms = 16 ticks buzzerTimer

    readCommand();                        // Read Command: input1[inIdx].cmd, input2[inIdx].cmd
    calcAvgSpeed();                       // Calculate average measured speed: speedAvg, speedAvgAbs

    #ifndef VARIANT_TRANSPOTTER
      // ####### MOTOR ENABLING: Only if the initial input is very small (for SAFETY) #######
      if (enable == 0 && !rtY_Right.z_errCode && ABS(input2[inIdx].cmd) < 25){
        beepShort(6);                     // make 2 beeps indicating the motor enable
        beepShort(4); HAL_Delay(100);
        steerFixdt = speedFixdt = 0;      // reset filters
        enable = 1;                       // enable motors
        #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("-- Motors enabled --\r\n");
        #endif
      }

      // ####### VARIANT_HOVERCAR #######
      #if defined(VARIANT_HOVERCAR) || defined(VARIANT_SKATEBOARD) || defined(ELECTRIC_BRAKE_ENABLE)
        uint16_t speedBlend;                                        // Calculate speed Blend, a number between [0, 1] in fixdt(0,16,15)
        speedBlend = (uint16_t)(((CLAMP(speedAvgAbs,10,60) - 10) << 15) / 50); // speedBlend [0,1] is within [10 rpm, 60rpm]
      #endif

      #ifdef STANDSTILL_HOLD_ENABLE
        standstillHold();                                           // Apply Standstill Hold functionality. Only available and makes sense for VOLTAGE or TORQUE Mode
      #endif

      #ifdef VARIANT_HOVERCAR
      if (inIdx == CONTROL_ADC) {                                   // Only use use implementation below if pedals are in use (ADC input)
        if (speedAvgAbs < 60) {                                     // Check if Hovercar is physically close to standstill to enable Double tap detection on Brake pedal for Reverse functionality
          multipleTapDet(input1[inIdx].cmd, HAL_GetTick(), &MultipleTapBrake); // Brake pedal in this case is "input1" variable
        }

        if (input1[inIdx].cmd > 30) {                               // If Brake pedal (input1) is pressed, bring to 0 also the Throttle pedal (input2) to avoid "Double pedal" driving
          input2[inIdx].cmd = (int16_t)((input2[inIdx].cmd * speedBlend) >> 15);
          cruiseControl((uint8_t)rtP_Left.b_cruiseCtrlEna);         // Cruise control deactivated by Brake pedal if it was active
        }
      }
      #endif

      #ifdef ELECTRIC_BRAKE_ENABLE
        electricBrake(speedBlend, MultipleTapBrake.b_multipleTap);  // Apply Electric Brake. Only available and makes sense for TORQUE Mode
      #endif

      #ifdef VARIANT_HOVERCAR
      if (inIdx == CONTROL_ADC) {                                   // Only use use implementation below if pedals are in use (ADC input)
        if (speedAvg > 0) {                                         // Make sure the Brake pedal is opposite to the direction of motion AND it goes to 0 as we reach standstill (to avoid Reverse driving by Brake pedal) 
          input1[inIdx].cmd = (int16_t)((-input1[inIdx].cmd * speedBlend) >> 15);
        } else {
          input1[inIdx].cmd = (int16_t)(( input1[inIdx].cmd * speedBlend) >> 15);
        }
      }
      #endif

      #ifdef VARIANT_SKATEBOARD
        if (input2[inIdx].cmd < 0) {                                // When Throttle is negative, it acts as brake. This condition is to make sure it goes to 0 as we reach standstill (to avoid Reverse driving) 
          if (speedAvg > 0) {                                       // Make sure the braking is opposite to the direction of motion
            input2[inIdx].cmd  = (int16_t)(( input2[inIdx].cmd * speedBlend) >> 15);
          } else {
            input2[inIdx].cmd  = (int16_t)((-input2[inIdx].cmd * speedBlend) >> 15);
          }
        }
      #endif
      
      #if defined(USE_CUSTOM_FILTER)
      steer = clean_adc_full(value_buffer(input1[inIdx].raw, 0));  // convert fixed-point to integer
      speed = clean_adc_full(value_buffer(input2[inIdx].raw, 1));  // convert fixed-point to integer
      #elif defined(USE_RAW_INPUT)
      steer = input1[inIdx].raw;  // convert fixed-point to integer
      speed = input2[inIdx].raw;  // convert fixed-point to integer
      #else
      // ####### LOW-PASS FILTER #######
      rateLimiter16(input1[inIdx].cmd, rate, &steerRateFixdt);
      rateLimiter16(input2[inIdx].cmd, rate, &speedRateFixdt);
      filtLowPass32(steerRateFixdt >> 4, FILTER, &steerFixdt);
      filtLowPass32(speedRateFixdt >> 4, FILTER, &speedFixdt);
      steer = (int16_t)(steerFixdt >> 16);  // convert fixed-point to integer
      speed = (int16_t)(speedFixdt >> 16);  // convert fixed-point to integer
      #endif

      #if defined(TANK_STEERING) && !defined(VARIANT_HOVERCAR) && !defined(VARIANT_SKATEBOARD) 
        // Tank steering (no mixing)
        cmdL = steer; 
        cmdR = speed;
      #else 
        // ####### MIXER #######
        mixerFcn(speed << 4, steer << 4, &cmdR, &cmdL);   // This function implements the equations above
      #endif


      // ####### SET OUTPUTS (if the target change is less than +/- 100) #######
      #ifdef INVERT_R_DIRECTION
        pwmr = cmdR;
      #else
        pwmr = -cmdR;
      #endif
      #ifdef INVERT_L_DIRECTION
        pwml = -cmdL;
      #else
        pwml = cmdL;
      #endif
    #endif

    // ####### SIDEBOARDS HANDLING #######
    #if defined(SIDEBOARD_SERIAL_USART2)
      sideboardSensors((uint8_t)Sideboard_L.sensors);
    #endif
    #if defined(FEEDBACK_SERIAL_USART2)
      sideboardLeds(&sideboard_leds_L);
    #endif
    #if defined(SIDEBOARD_SERIAL_USART3)
      sideboardSensors((uint8_t)Sideboard_R.sensors);
    #endif
    #if defined(FEEDBACK_SERIAL_USART3)
      sideboardLeds(&sideboard_leds_R);
    #endif
    

    // ####### CALC BOARD TEMPERATURE #######
    filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &board_temp_adcFixdt);
    board_temp_adcFilt  = (int16_t)(board_temp_adcFixdt >> 16);  // convert fixed-point to integer
    board_temp_deg_c    = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) * (board_temp_adcFilt - TEMP_CAL_LOW_ADC) / (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;

    // ####### CALC CALIBRATED BATTERY VOLTAGE #######
    batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;

    // ####### CALC DC LINK CURRENT #######
    left_dc_curr  = -(rtU_Left.i_DCLink * 100) / A2BIT_CONV;   // Left DC Link Current * 100 
    right_dc_curr = -(rtU_Right.i_DCLink * 100) / A2BIT_CONV;  // Right DC Link Current * 100
    dc_curr       = left_dc_curr + right_dc_curr;            // Total DC Link Current * 100

    // ####### DEBUG SERIAL OUT #######
    #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
      if (main_loop_counter % 25 == 0) {    // Send data periodically every 125 ms      
        #if defined(DEBUG_SERIAL_PROTOCOL)
          process_debug();
        #else
          printf("in1:%i in2:%i cmdL:%i cmdR:%i BatADC:%i BatV:%i TempADC:%i Temp:%i \r\n",
            input1[inIdx].raw,        // 1: INPUT1
            input2[inIdx].raw,        // 2: INPUT2
            cmdL,                     // 3: output command: [-1000, 1000]
            cmdR,                     // 4: output command: [-1000, 1000]
            adc_buffer.batt1,         // 5: for battery voltage calibration
            batVoltageCalib,          // 6: for verifying battery voltage calibration
            board_temp_adcFilt,       // 7: for board temperature calibration
            board_temp_deg_c);        // 8: for verifying board temperature calibration
        #endif
      }
    #endif

    // ####### FEEDBACK SERIAL OUT #######
    #if defined(FEEDBACK_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART3)
      if (main_loop_counter % 4 == 0) {    // Send data periodically every 10 ms
        Feedback.start	        = (uint16_t)SERIAL_START_FRAME;
        Feedback.steps0           = steps[0];
        Feedback.steps1           = steps[1];
        #ifdef INVERT_R_DIRECTION
          Feedback.speedR_meas = (int16_t)rtY_Right.n_mot;
        #else
          Feedback.speedR_meas = -(int16_t)rtY_Right.n_mot;
        #endif
        #ifdef INVERT_L_DIRECTION
          Feedback.speedL_meas = -(int16_t)rtY_Left.n_mot;
        #else
          Feedback.speedL_meas = (int16_t)rtY_Left.n_mot;
        #endif
        Feedback.batVoltage	    = (int16_t)batVoltageCalib;
        Feedback.boardTemp	    = (int16_t)board_temp_deg_c;

        #if defined(FEEDBACK_SERIAL_USART2)
          if(__HAL_DMA_GET_COUNTER(huart2.hdmatx) == 0) {
            Feedback.cmdLed     = (uint16_t)sideboard_leds_L;
            uint32_t checksum = calc_crc32((uint8_t*)&Feedback,sizeof(SerialFeedback) - sizeof(uint16_t)*2);
            Feedback.checksumL   =  checksum & 0xFFFF;
            Feedback.checksumH   =  checksum >> 16;
            HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&Feedback, sizeof(SerialFeedback));
          }
        #endif
        #if defined(FEEDBACK_SERIAL_USART3)
          if(__HAL_DMA_GET_COUNTER(huart3.hdmatx) == 0) {
            Feedback.cmdLed     = (uint16_t)sideboard_leds_R;
            uint32_t checksum = calc_crc32((uint8_t*)&Feedback,sizeof(SerialFeedback) - sizeof(uint16_t)*2);
            Feedback.checksumL   =  checksum & 0xFFFF;
            Feedback.checksumH   =  checksum >> 16;
            HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&Feedback, sizeof(SerialFeedback));
          }
        #endif
      }
    #endif

    // ####### POWEROFF BY POWER-BUTTON #######
    poweroffPressCheck();

    // ####### BEEP AND EMERGENCY POWEROFF #######
    if (TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && speedAvgAbs < 20){  // poweroff before mainboard burns OR low bat 3
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("Powering off, temperature is too high\r\n");
      #endif
      poweroff();
    } else if ( BAT_DEAD_ENABLE && batVoltage < BAT_DEAD && speedAvgAbs < 20){
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("Powering off, battery voltage is too low\r\n");
      #endif
      poweroff();
    } else if (rtY_Left.z_errCode || rtY_Right.z_errCode) {                                           // 1 beep (low pitch): Motor error, disable motors
      enable = 0;
      beepCount(1, 24, 1);
    } else if (timeoutFlgADC) {                                                                       // 2 beeps (low pitch): ADC timeout
      beepCount(2, 24, 1);
    } else if (timeoutFlgSerial) {                                                                    // 3 beeps (low pitch): Serial timeout
      beepCount(3, 24, 1);
    } else if (timeoutFlgGen) {                                                                       // 4 beeps (low pitch): General timeout (PPM, PWM, Nunchuk)
      beepCount(4, 24, 1);
    } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) {                             // 5 beeps (low pitch): Mainboard temperature warning
      beepCount(5, 24, 1);
    } else if (BAT_LVL1_ENABLE && batVoltage < BAT_LVL1) {                                            // 1 beep fast (medium pitch): Low bat 1
      beepCount(0, 10, 6);
    } else if (BAT_LVL2_ENABLE && batVoltage < BAT_LVL2) {                                            // 1 beep slow (medium pitch): Low bat 2
      beepCount(0, 10, 30);
    } else if (BEEPS_BACKWARD && (((cmdR < -50 || cmdL < -50) && speedAvg < 0) || MultipleTapBrake.b_multipleTap)) { // 1 beep fast (high pitch): Backward spinning motors
      beepCount(0, 5, 1);
      backwardDrive = 1;
    } else {  // do not beep
      beepCount(0, 0, 0);
      backwardDrive = 0;
    }


    inactivity_timeout_counter++;

    // ####### INACTIVITY TIMEOUT #######
    if (abs(cmdL) > 50 || abs(cmdR) > 50) {
      inactivity_timeout_counter = 0;
    }

    #if defined(CRUISE_CONTROL_SUPPORT) || defined(STANDSTILL_HOLD_ENABLE)
      if ((abs(rtP_Left.n_cruiseMotTgt)  > 50 && rtP_Left.b_cruiseCtrlEna) || 
          (abs(rtP_Right.n_cruiseMotTgt) > 50 && rtP_Right.b_cruiseCtrlEna)) {
        inactivity_timeout_counter = 0;
      }
    #endif

    if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1)) {  // rest of main loop needs maybe 1ms
      #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("Powering off, wheels were inactive for too long\r\n");
      #endif
      poweroff();
    }


    // HAL_GPIO_TogglePin(LED_PORT, LED_PIN);                 // This is to measure the main() loop duration with an oscilloscope connected to LED_PIN
    // Update states
    inIdx_prev = inIdx;
    buzzerTimer_prev = buzzerTimer;
    main_loop_counter++;
    }
  }
}


// ===========================================================
/** System Clock Configuration
*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType           = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider       = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider      = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider      = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection    = RCC_PERIPHCLK_ADC;
  // PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;  // 8 MHz
  PeriphClkInit.AdcClockSelection       = RCC_ADCPCLK2_DIV4;  // 16 MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
