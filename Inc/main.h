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
#define REMOCON_TIM2_CH1_GPIO_A_0                        GPIO_PIN_0
#define REMOCON_TIM2_CH1_PORT_A                          GPIOA

#define REMOCON_TIM2_CH2_GPIO_A_1                        GPIO_PIN_1
#define REMOCON_TIM2_CH2_PORT_A                          GPIOA

#define REMOCON_TIM2_CH3_GPIO_A_2                        GPIO_PIN_2
#define REMOCON_TIM2_CH3_PORT_A                          GPIOA

#define REMOCON_TIM2_CH4_GPIO_A_3                        GPIO_PIN_3
#define REMOCON_TIM2_CH4_PORT_A                          GPIOA

// --------------------------------------------------------------------------------
#define BLE_NG_IRQ_GPIO_A_4                              GPIO_PIN_4
#define BLE_NG_IRQ_PORT_A                                GPIOA

#define BLUE_CS_GPIO_B_0                                 GPIO_PIN_0
#define BLUE_CS_PORT_B                                   GPIOB

#define BLE_RESET_GPIO_B_2                               GPIO_PIN_2
#define BLE_RESET_PORT_B                                 GPIOB

#define BLE_SPI_1_MOSI_GPIO_A_7                          GPIO_PIN_7
#define BLE_SPI_1_MOSI_PORT_A                            GPIOA

#define BLE_SPI_1_MISO_GPIO_A_6                          GPIO_PIN_6
#define BLE_SPI_1_MISO_PORT_A                            GPIOA

#define BLE_SPI_1_CLK_GPIO_A_5                           GPIO_PIN_5
#define BLE_SPI_1_CLK_PORT_A                             GPIOA

// --------------------------------------------------------------------------------
#define USB_DP_GPIO_A_12                                 GPIO_PIN_12
#define USB_DM_GPIO_A_11                                 GPIO_PIN_11
#define USB_PORT_A                                       GPIOA

#define USB_VBUS_DETECT_GPIO_C_14                        GPIO_PIN_14
#define USB_VBUS_DETECT_PORT                             GPIOC

// --------------------------------------------------------------------------------
#define SENSOR_LSM6DS33_SPI_2_CS_GPIO_A_8                GPIO_PIN_8
#define SENSOR_LSM6DS33_SPI_2_CS_PORT                    GPIOA

#define SENSOR_LIS2MDL_SPI_2_CS_GPIO_B_12                GPIO_PIN_12
#define SENSOR_LIS2MDL_SPI_2_CS_PORT                     GPIOB

#define SENSOR_LPS22HB_SPI_2_CS_GPIO_C_13                GPIO_PIN_13
#define SENSOR_LPS22HB_SPI_2_CS_PORT                     GPIOC

#define SENSOR_SPI_2_CLK_GPIO_B_13                       GPIO_PIN_13
#define SENSOR_SPI_2_CLK_PORT                            GPIOB

#define SENSOR_SPI_2_SDA_GPIO_B_15                       GPIO_PIN_15
#define SENSOR_SPI_2_SDA_PORT                            GPIOB

// --------------------------------------------------------------------------------
#define MOTOR_1_GPIO_B_6                                 GPIO_PIN_6
#define MOTOR_1_PORT                                     GPIOB

#define MOTOR_2_GPIO_B_7                                 GPIO_PIN_7
#define MOTOR_2_PORT                                     GPIOB

#define MOTOR_3_GPIO_B_8                                 GPIO_PIN_8
#define MOTOR_3_PORT                                     GPIOB

#define MOTOR_4_GPIO_B_9                                 GPIO_PIN_9
#define MOTOR_4_PORT                                     GPIOB

// --------------------------------------------------------------------------------
#define LED_1_GPIO_B_5                                   GPIO_PIN_5
#define LED_1_PORT                                       GPIOB

#define LED_2_GPIO_B_4                                   GPIO_PIN_4
#define LED_2_PORT                                       GPIOB

// --------------------------------------------------------------------------------
#define VBAT_SENSE_GPIO_B_1                              GPIO_PIN_1
#define VBAT_SENSE_PORT                                  GPIOB

// --------------------------------------------------------------------------------
#define I2C2_SCL_GPIO_B_10                               GPIO_PIN_10
#define I2C2_SCL_PORT                                    GPIOB

#define I2C2_SDA_GPIO_B_3                                GPIO_PIN_3
#define I2C2_SDA_PORT                                    GPIOB




#if 0
// --------------------------------------------------------------------------------
#define BLE_IRQ_Pin                                      GPIO_PIN_4
#define BLE_IRQ_GPIO_Port                                GPIOA

#define BLE_CS_Pin                                       GPIO_PIN_0
#define BLE_CS_GPIO_Port                                 GPIOB

#define BLE_RSTN_Pin                                     GPIO_PIN_2
#define BLE_RSTN_GPIO_Port                               GPIOB

//#define VBAT_SENSE_Pin                                   GPIO_PIN_1
//#define VBAT_SENSE_GPIO_Port                             GPIOB

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
#endif

#define MOTOR1_Pin                                       GPIO_PIN_6
#define MOTOR1_GPIO_Port                                 GPIOB

#define MOTOR2_Pin                                       GPIO_PIN_7
#define MOTOR2_GPIO_Port                                 GPIOB

#define MOTOR3_Pin                                       GPIO_PIN_8
#define MOTOR3_GPIO_Port                                 GPIOB

#define MOTOR4_Pin                                       GPIO_PIN_9
#define MOTOR4_GPIO_Port                                 GPIOB

// --------------------------------------------------------------------------------
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

