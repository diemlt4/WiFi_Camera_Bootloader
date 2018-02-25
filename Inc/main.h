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
  * Copyright (c) 2017 STMicroelectronics International N.V. 
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Private define ------------------------------------------------------------*/
#define DCMI_POWN_Pin GPIO_PIN_2
#define DCMI_POWN_GPIO_Port GPIOE
#define DCMI_RESET_Pin GPIO_PIN_3
#define DCMI_RESET_GPIO_Port GPIOE
#define DCMI_XCLK_Pin GPIO_PIN_5
#define DCMI_XCLK_GPIO_Port GPIOE
#define RTC_SDA_Pin GPIO_PIN_0
#define RTC_SDA_GPIO_Port GPIOF
#define RTC_SCL_Pin GPIO_PIN_1
#define RTC_SCL_GPIO_Port GPIOF
#define KEY1_Pin GPIO_PIN_2
#define KEY1_GPIO_Port GPIOF
#define KEY2_Pin GPIO_PIN_3
#define KEY2_GPIO_Port GPIOF
#define KEY3_Pin GPIO_PIN_4
#define KEY3_GPIO_Port GPIOF
#define KEY4_Pin GPIO_PIN_5
#define KEY4_GPIO_Port GPIOF
#define LCD_DIM_Pin GPIO_PIN_9
#define LCD_DIM_GPIO_Port GPIOF
#define MOTOR_STEP3_Pin GPIO_PIN_0
#define MOTOR_STEP3_GPIO_Port GPIOA
#define MOTOR_DIR3_Pin GPIO_PIN_1
#define MOTOR_DIR3_GPIO_Port GPIOA
#define MCU_PC_TX_Pin GPIO_PIN_2
#define MCU_PC_TX_GPIO_Port GPIOA
#define MCU_PC_RX_Pin GPIO_PIN_3
#define MCU_PC_RX_GPIO_Port GPIOA
#define LCD_D0_Pin GPIO_PIN_0
#define LCD_D0_GPIO_Port GPIOG
#define LCD_D1_Pin GPIO_PIN_1
#define LCD_D1_GPIO_Port GPIOG
#define MOTOR_STEP1_Pin GPIO_PIN_9
#define MOTOR_STEP1_GPIO_Port GPIOE
#define MOTOR_DIR1_Pin GPIO_PIN_11
#define MOTOR_DIR1_GPIO_Port GPIOE
#define MOTOR_STEP2_Pin GPIO_PIN_13
#define MOTOR_STEP2_GPIO_Port GPIOE
#define MOTOR_DIR2_Pin GPIO_PIN_14
#define MOTOR_DIR2_GPIO_Port GPIOE
#define MOTOR_STEP4_Pin GPIO_PIN_10
#define MOTOR_STEP4_GPIO_Port GPIOB
#define MOTOR_DIR4_Pin GPIO_PIN_11
#define MOTOR_DIR4_GPIO_Port GPIOB
#define MOTOR_EN2_Pin GPIO_PIN_12
#define MOTOR_EN2_GPIO_Port GPIOB
#define MOTOR_EN3_Pin GPIO_PIN_13
#define MOTOR_EN3_GPIO_Port GPIOB
#define MOTOR_EN4_Pin GPIO_PIN_14
#define MOTOR_EN4_GPIO_Port GPIOB
#define MOTOR_EN5_Pin GPIO_PIN_15
#define MOTOR_EN5_GPIO_Port GPIOB
#define MOTOR_EN1_Pin GPIO_PIN_8
#define MOTOR_EN1_GPIO_Port GPIOD
#define MOTOR_STEP5_Pin GPIO_PIN_12
#define MOTOR_STEP5_GPIO_Port GPIOD
#define MOTOR_DIR5_Pin GPIO_PIN_13
#define MOTOR_DIR5_GPIO_Port GPIOD
#define LCD_RS_Pin GPIO_PIN_14
#define LCD_RS_GPIO_Port GPIOD
#define LCD_RW_Pin GPIO_PIN_15
#define LCD_RW_GPIO_Port GPIOD
#define LCD_D2_Pin GPIO_PIN_2
#define LCD_D2_GPIO_Port GPIOG
#define LCD_D3_Pin GPIO_PIN_3
#define LCD_D3_GPIO_Port GPIOG
#define LCD_D4_Pin GPIO_PIN_4
#define LCD_D4_GPIO_Port GPIOG
#define LCD_D5_Pin GPIO_PIN_5
#define LCD_D5_GPIO_Port GPIOG
#define LCD_D6_Pin GPIO_PIN_6
#define LCD_D6_GPIO_Port GPIOG
#define LCD_D7_Pin GPIO_PIN_7
#define LCD_D7_GPIO_Port GPIOG
#define LCD_EN_Pin GPIO_PIN_8
#define LCD_EN_GPIO_Port GPIOG
#define SD_DETECT_Pin GPIO_PIN_8
#define SD_DETECT_GPIO_Port GPIOA
#define MCU_WIFI_TX_Pin GPIO_PIN_9
#define MCU_WIFI_TX_GPIO_Port GPIOA
#define MCU_WIFI_RX_Pin GPIO_PIN_10
#define MCU_WIFI_RX_GPIO_Port GPIOA
#define MOTOR_LED_Pin GPIO_PIN_11
#define MOTOR_LED_GPIO_Port GPIOA
#define TEST_LED_Pin GPIO_PIN_12
#define TEST_LED_GPIO_Port GPIOA
#define DCMI_SCL_Pin GPIO_PIN_6
#define DCMI_SCL_GPIO_Port GPIOB
#define DCMI_SDA_Pin GPIO_PIN_7
#define DCMI_SDA_GPIO_Port GPIOB

/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_12     ((uint32_t)0x08100000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_13     ((uint32_t)0x08104000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_14     ((uint32_t)0x08108000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_15     ((uint32_t)0x0810C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_16     ((uint32_t)0x08110000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_17     ((uint32_t)0x08120000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_18     ((uint32_t)0x08140000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_19     ((uint32_t)0x08160000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_20     ((uint32_t)0x08180000) /* Base @ of Sector 8, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_21     ((uint32_t)0x081A0000) /* Base @ of Sector 9, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_22     ((uint32_t)0x081C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_23     ((uint32_t)0x081E0000) /* Base @ of Sector 11, 128 Kbytes */
      

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
