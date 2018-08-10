/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#ifndef __MAIN_H
#define __MAIN_H

#define UART_DEBUG_OUTPUT

// --------------------------------------------------------------------------------
#define UART_DEBUG_PORT                                  USART1
#define UART_DEBUG_BAUDRATE                              115200

#define UART_DEBUG_RX_PIN                                GPIO_PIN_10
#define UART_DEBUG_RX_GPIO_PORT                          GPIOA
#define UART_DEBUG_TX_PIN                                GPIO_PIN_9
#define UART_DEBUG_TX_GPIO_PORT                          GPIOA

// --------------------------------------------------------------------------------
#define BLE_IRQ_Pin                                      GPIO_PIN_4
#define BLE_IRQ_GPIO_Port                                GPIOA

#define BLE_CS_Pin                                       GPIO_PIN_0
#define BLE_CS_GPIO_Port                                 GPIOB

#define BLE_RSTN_Pin                                     GPIO_PIN_2
#define BLE_RSTN_GPIO_Port                               GPIOB

#define VBAT_SENSE_Pin                                   GPIO_PIN_1
#define VBAT_SENSE_GPIO_Port                             GPIOB

#define LPS25H_CS_Pin                                    GPIO_PIN_10
#define LPS25H_CS_GPIO_Port                              GPIOB

#define LIS3MDL_CS_Pin                                   GPIO_PIN_12
#define LIS3MDL_CS_GPIO_Port                             GPIOB

#define LSM6DS33_CS_Pin                                  GPIO_PIN_8
#define LSM6DS33_CS_GPIO_Port                            GPIOA

#define LED3_Pin                                         GPIO_PIN_3
#define LED3_GPIO_Port                                   GPIOB

#define LED2_Pin                                         GPIO_PIN_4
#define LED2_GPIO_Port                                   GPIOB

#define MOTOR1_Pin                                       GPIO_PIN_6
#define MOTOR1_GPIO_Port                                 GPIOB

#define MOTOR2_Pin                                       GPIO_PIN_7
#define MOTOR2_GPIO_Port                                 GPIOB

#define MOTOR3_Pin                                       GPIO_PIN_8
#define MOTOR3_GPIO_Port                                 GPIOB

#define MOTOR4_Pin                                       GPIO_PIN_9
#define MOTOR4_GPIO_Port                                 GPIOB

#define PROGRAM_NAME                                     "FCU-F401"
#define VERSION_MAIN                                     0
#define VERSION_MINOR                                    1
#define VERSION_SUB                                      0

// --------------------------------------------------------------------------------
#include "sys/_stdint.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "uart_debug.h"
#include "debug_output.h"
#include "scheduler.h"

#ifdef __cplusplus
extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif   // __MAIN_H

