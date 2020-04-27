//*****************************************************************************
// RSLK-MAX test main for Lab 19
// MSP432 with RSLK-MAX
// Daniel and Jonathan Valvano
// July 11, 2019
/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/


#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/AP.h"
#include "../inc/UART0.h"
#include "../inc/Motor.h"
#include "../inc/Bump.h"
#include "../inc/ADC14.h"
#include "../inc/LPF.h"
#include "../inc/TimerA1.h"
#include "../inc/IRDistance.h"
#include "../inc/Nokia5110.h"

// ********OutValue**********
// Debugging dump of a data value to virtual serial port to PC
// data shown as 1 to 8 hexadecimal characters
// Inputs:  response (number returned by last AP call)
// Outputs: none
void OutValue(char *label,uint32_t value){
  UART0_OutString(label);
  UART0_OutUHex(value);
}

// Motor interface, see Motor.c
uint16_t LeftMotorPWMDuty;
void SetLeftDuty(void) {
    OutValue("\r\n Left Motor:", LeftMotorPWMDuty);
}

uint16_t RightMotorPWMDuty;
void SetRightDuty(void) {
    OutValue("\r\n Right Motor:", RightMotorPWMDuty);
}

// bump sensor interface, see Bump.c
uint8_t BumpSensorReadVal;
void ReadBumpSensors(void) {
    BumpSensorReadVal = Bump_Read();
    OutValue("\r\n Current Bump Sensor Value:", BumpSensorReadVal);
}

void BLE_Init(void){
    AP_Init();

    AP_GetStatus();
    AP_GetVersion();
    AP_AddService(0xFFF0);

    AP_AddCharacteristic(0xFFF1, 1, &BumpSensorReadVal, 0x01, 0x02, "Bump Sensors", ReadBumpSensors, 0);

    AP_AddCharacteristic(0xFFF2,2,&LeftMotorPWMDuty,0x02,0x08,"Left Motor",0,&SetLeftDuty);
    AP_AddCharacteristic(0xFFF3,2,&RightMotorPWMDuty,0x02,0x08,"Right Motor",0,&SetRightDuty);

    AP_RegisterService();
    AP_StartAdvertisement();
    AP_GetStatus();
}

void main(void){
  DisableInterrupts();

  Clock_Init48MHz();
  UART0_Init();
  LaunchPad_Init();
  Bump_Init();
  Motor_Init();

  EnableInterrupts();

  BLE_Init();
  UART0_OutString("\n\rBELLE Robot\n\r");

  while(1) {
      AP_BackgroundProcess();
      Motor_Forward(LeftMotorPWMDuty, RightMotorPWMDuty);
  }
}
