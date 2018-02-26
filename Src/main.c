/**
******************************************************************************
* File Name          : main.c
* Description        : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "stdbool.h"

/* Macro defines -------------------------------------------------------------*/
#define INFO_ADDR_START         ((uint32_t)0x08008000)
#define INFO_ADDR_END           ((uint32_t)0x0800BFFF)

#define CONFIG_ADDR_START       ((uint32_t)0x0800C000)
#define CONFIG_ADDR_END         ((uint32_t)0x0800FFFF)

#define APP_ADDR_START          ((uint32_t)0x08020000)
#define APP_ADDR_END            ((uint32_t)0x0805FFFF)

#define OTA_ADDR_START          ((uint32_t)0x08060000)
#define OTA_ADDR_END            ((uint32_t)0x0809FFFF)

/* Bootloader version: V1.02 */
#define BOOTLODER_VERSION       ((uint32_t)102)

#define READ_BUF_SIZE           1024

typedef union APP_STATUS
{
    struct
    {
        uint32_t bootloader_version;
        uint32_t app_version;
        uint32_t ota_version;
        uint32_t ota_length;
        uint16_t ota_crc;
        uint16_t reserved;
        uint32_t checksum;
    };
    uint32_t array32[6];
    uint8_t  array[24];
} App_Info_t;

typedef  void (*pFunction)(void);

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

App_Info_t app_info;
bool write_flag = false;
uint32_t jump_address = APP_ADDR_START;
uint32_t Address, SECTORError = 0;
FLASH_EraseInitTypeDef EraseInitStruct;

uint8_t read_buf[READ_BUF_SIZE];

pFunction Jump_To_Application;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static uint32_t GetSector(uint32_t Address);
static uint32_t GetChecksum32(uint32_t *pdata, uint32_t length);
static uint16_t CRC16_CCITT(const uint8_t* pdata, uint16_t length);


int main(void)
{
    int i = 0;
    int j = 0;
    uint32_t sector_error = 0;
    uint32_t first_sector = 0;
    uint32_t number_of_sector = 0;
    uint32_t write_addr = 0;
    uint32_t source_addr = 0;
    uint16_t crc = 0;
    uint32_t write_len = 0;
        
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    
    /* read app information sector data */
    for(i = 0; i < sizeof(app_info); i++)
    {
        app_info.array[i] = *((uint8_t *)(INFO_ADDR_START + i));
    }
    
    /* validate checksum */
    if(app_info.checksum == GetChecksum32(app_info.array32, (sizeof(app_info)/4) - 1))
    {       
        /* if ota version is ready, copy to app flash */
        if(app_info.ota_version > app_info.app_version )
        {
            /* erase info sector */
            HAL_FLASH_Unlock();
            
            /* Get the 1st sector to erase */
            first_sector = GetSector(APP_ADDR_START);
            
            /* Get the number of sector to erase from 1st sector*/
            number_of_sector = GetSector(APP_ADDR_END) - first_sector + 1;
            
            /* Fill EraseInit structure*/
            EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
            EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
            EraseInitStruct.Sector = first_sector;
            EraseInitStruct.NbSectors = number_of_sector;
            
            while(HAL_FLASHEx_Erase(&EraseInitStruct, &sector_error) != HAL_OK)
            {
                HAL_Delay(100);
            }
    
            /* Copy OTA Flash Data to App Flash */
            write_addr = APP_ADDR_START;
            source_addr = OTA_ADDR_START;
            while(write_len < app_info.ota_length)
            {
                write_addr = APP_ADDR_START + write_len;
                source_addr = OTA_ADDR_START + write_len;
                
                for(j = 0; j < READ_BUF_SIZE; j++)
                {
                    read_buf[j] = *((uint8_t *)(source_addr + j));
                }
                
                for(j = 0; j < READ_BUF_SIZE; )
                {
                    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, write_addr + j, read_buf[j]) == HAL_OK)
                    {
                        j++;
                    }
                }
                
                write_len += READ_BUF_SIZE;
            }
            
            /* Verify App Flash */
            crc = CRC16_CCITT((uint8_t *)APP_ADDR_START, app_info.ota_length);
            if(crc == app_info.ota_crc)
            {
                /* ota success, update app info and reboot */
                app_info.bootloader_version = BOOTLODER_VERSION;
                app_info.app_version = app_info.ota_version;
                app_info.ota_version = 0;
                app_info.ota_length = 0;
                app_info.ota_crc = 0;
                
                write_flag = true;
            }
            
            HAL_FLASH_Lock();
        }
    }
    else
    {
        /* app information is error, write default information */
        app_info.bootloader_version = BOOTLODER_VERSION;
        app_info.app_version = 0;
        app_info.ota_version = 0;
        app_info.ota_length = 0;
        app_info.ota_crc = 0;
        write_flag = true;
    }
    
    /* check bootloader version */
    if(app_info.bootloader_version != BOOTLODER_VERSION)
    {
        app_info.bootloader_version = BOOTLODER_VERSION;
        write_flag = true;        
    }
    
    /* write new app information */
    if(write_flag == true)
    {
        app_info.checksum = GetChecksum32(app_info.array32, (sizeof(app_info)/4) - 1);
        
        /* erase info sector */
        HAL_FLASH_Unlock();

        EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
        EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
        EraseInitStruct.Sector        = GetSector(INFO_ADDR_START);
        EraseInitStruct.NbSectors     = 1;
        while(HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
        {
            HAL_Delay(100);
        }
        
        /* write new info data */
        Address = INFO_ADDR_START;
        for(i = 0; i < sizeof(app_info)/4; )
        {
            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, app_info.array32[i]) == HAL_OK)
            {
                Address = Address + 4;
                i++;
            }
        }
            
        HAL_FLASH_Lock();
    }

    /* Jump to application */
    if (((*(__IO uint32_t*)APP_ADDR_START) & 0x2FFE0000 ) == 0x20000000)
    { 
        /* Jump to user application */
        jump_address = *(__IO uint32_t*) (APP_ADDR_START + 4);
        Jump_To_Application = (pFunction) jump_address;
        /* Initialize user application's Stack Pointer */
        __set_MSP(*(__IO uint32_t*) APP_ADDR_START);
        Jump_To_Application();
    }
    
    
    while (1)
    {
        /* */
    }
}

/*******************************************************************************
* @Brief   CRC16 Verify - CCITT Mode
* @Param   [in]pdata: point to data buffer
*          [in]length: data buffer length
* @Note    
* @Return  crc result, 16bit
*******************************************************************************/   
static uint16_t CRC16_CCITT(const uint8_t* pdata, uint16_t length)
{
    const uint16_t seed = 0xFFFF;
    const uint16_t poly16 = 0x1021;
    
    uint16_t i = 0;
    uint16_t wTemp = 0;      
    uint16_t wCRC = 0;      
    
    wCRC = seed;
    
    for(i = 0; i < length; i++)      
    {             
        for(int j = 0; j < 8; j++)      
        {      
            wTemp = ((pdata[i] << j) & 0x80 ) ^ ((wCRC & 0x8000) >> 8);      
    
            wCRC <<= 1;      
    
            if(wTemp != 0)       
            {
                wCRC ^= poly16;
            }     
        }      
    }      
    
    return wCRC;     
}  

static uint32_t GetChecksum32(uint32_t *pdata, uint32_t length)
{
    uint32_t checksum = 0x12345678;
    uint32_t i = 0;
    
    for(i = 0; i < length; i++)
    {
        checksum += *pdata;
        pdata++;
    }
    
    return checksum;
}

/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;
  }
  else if((Address < ADDR_FLASH_SECTOR_12) && (Address >= ADDR_FLASH_SECTOR_11))
  {
    sector = FLASH_SECTOR_11;
  }
  else if((Address < ADDR_FLASH_SECTOR_13) && (Address >= ADDR_FLASH_SECTOR_12))
  {
    sector = FLASH_SECTOR_12;
  }
  else if((Address < ADDR_FLASH_SECTOR_14) && (Address >= ADDR_FLASH_SECTOR_13))
  {
    sector = FLASH_SECTOR_13;
  }
  else if((Address < ADDR_FLASH_SECTOR_15) && (Address >= ADDR_FLASH_SECTOR_14))
  {
    sector = FLASH_SECTOR_14;
  }
  else if((Address < ADDR_FLASH_SECTOR_16) && (Address >= ADDR_FLASH_SECTOR_15))
  {
    sector = FLASH_SECTOR_15;
  }
  else if((Address < ADDR_FLASH_SECTOR_17) && (Address >= ADDR_FLASH_SECTOR_16))
  {
    sector = FLASH_SECTOR_16;
  }
  else if((Address < ADDR_FLASH_SECTOR_18) && (Address >= ADDR_FLASH_SECTOR_17))
  {
    sector = FLASH_SECTOR_17;
  }
  else if((Address < ADDR_FLASH_SECTOR_19) && (Address >= ADDR_FLASH_SECTOR_18))
  {
    sector = FLASH_SECTOR_18;
  }
  else if((Address < ADDR_FLASH_SECTOR_20) && (Address >= ADDR_FLASH_SECTOR_19))
  {
    sector = FLASH_SECTOR_19;
  }
  else if((Address < ADDR_FLASH_SECTOR_21) && (Address >= ADDR_FLASH_SECTOR_20))
  {
    sector = FLASH_SECTOR_20;
  }
  else if((Address < ADDR_FLASH_SECTOR_22) && (Address >= ADDR_FLASH_SECTOR_21))
  {
    sector = FLASH_SECTOR_21;
  }
  else if((Address < ADDR_FLASH_SECTOR_23) && (Address >= ADDR_FLASH_SECTOR_22))
  {
    sector = FLASH_SECTOR_22;
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_23) */
  {
    sector = FLASH_SECTOR_23;
  }  
  return sector;
}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{
    
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    
    /**Configure the main internal regulator output voltage 
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    
    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 8;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    
    /**Activate the Over-Drive mode 
    */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    
    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
        |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    
    /**Configure the Systick interrupt time 
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
    
    /**Configure the Systick 
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    
    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{
    
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    
}

/** Configure pins as 
* Analog 
* Input 
* Output
* EVENT_OUT
* EXTI
*/
static void MX_GPIO_Init(void)
{
    
    GPIO_InitTypeDef GPIO_InitStruct;
    
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, DCMI_POWN_Pin|MOTOR_STEP1_Pin|MOTOR_DIR1_Pin|MOTOR_STEP2_Pin 
                      |MOTOR_DIR2_Pin, GPIO_PIN_RESET);
    
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(DCMI_RESET_GPIO_Port, DCMI_RESET_Pin, GPIO_PIN_SET);
    
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, MOTOR_STEP3_Pin|MOTOR_DIR3_Pin|MOTOR_LED_Pin|TEST_LED_Pin, GPIO_PIN_RESET);
    
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOG, LCD_D0_Pin|LCD_D1_Pin|LCD_D2_Pin|LCD_D3_Pin 
                      |LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin 
                          |LCD_EN_Pin, GPIO_PIN_RESET);
    
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, MOTOR_STEP4_Pin|MOTOR_DIR4_Pin|MOTOR_EN2_Pin|MOTOR_EN3_Pin 
                      |MOTOR_EN4_Pin|MOTOR_EN5_Pin|DCMI_SCL_Pin|DCMI_SDA_Pin, GPIO_PIN_RESET);
    
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, MOTOR_EN1_Pin|MOTOR_STEP5_Pin|MOTOR_DIR5_Pin|LCD_RS_Pin 
                      |LCD_RW_Pin, GPIO_PIN_RESET);
    
    /*Configure GPIO pins : DCMI_POWN_Pin DCMI_RESET_Pin */
    GPIO_InitStruct.Pin = DCMI_POWN_Pin|DCMI_RESET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    
    /*Configure GPIO pins : KEY1_Pin KEY2_Pin KEY3_Pin KEY4_Pin */
    GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin|KEY3_Pin|KEY4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    
    /*Configure GPIO pins : MOTOR_STEP3_Pin MOTOR_DIR3_Pin MOTOR_LED_Pin TEST_LED_Pin */
    GPIO_InitStruct.Pin = MOTOR_STEP3_Pin|MOTOR_DIR3_Pin|MOTOR_LED_Pin|TEST_LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /*Configure GPIO pins : LCD_D0_Pin LCD_D1_Pin LCD_D2_Pin LCD_D3_Pin 
    LCD_D4_Pin LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin */
    GPIO_InitStruct.Pin = LCD_D0_Pin|LCD_D1_Pin|LCD_D2_Pin|LCD_D3_Pin 
        |LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
    
    /*Configure GPIO pins : MOTOR_STEP1_Pin MOTOR_DIR1_Pin MOTOR_STEP2_Pin MOTOR_DIR2_Pin */
    GPIO_InitStruct.Pin = MOTOR_STEP1_Pin|MOTOR_DIR1_Pin|MOTOR_STEP2_Pin|MOTOR_DIR2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    
    /*Configure GPIO pins : MOTOR_STEP4_Pin MOTOR_DIR4_Pin MOTOR_EN3_Pin MOTOR_EN4_Pin 
    MOTOR_EN5_Pin */
    GPIO_InitStruct.Pin = MOTOR_STEP4_Pin|MOTOR_DIR4_Pin|MOTOR_EN3_Pin|MOTOR_EN4_Pin 
        |MOTOR_EN5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /*Configure GPIO pin : MOTOR_EN2_Pin */
    GPIO_InitStruct.Pin = MOTOR_EN2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR_EN2_GPIO_Port, &GPIO_InitStruct);
    
    /*Configure GPIO pin : MOTOR_EN1_Pin */
    GPIO_InitStruct.Pin = MOTOR_EN1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR_EN1_GPIO_Port, &GPIO_InitStruct);
    
    /*Configure GPIO pins : MOTOR_STEP5_Pin MOTOR_DIR5_Pin */
    GPIO_InitStruct.Pin = MOTOR_STEP5_Pin|MOTOR_DIR5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    /*Configure GPIO pins : LCD_RS_Pin LCD_RW_Pin */
    GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_RW_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    /*Configure GPIO pin : LCD_EN_Pin */
    GPIO_InitStruct.Pin = LCD_EN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LCD_EN_GPIO_Port, &GPIO_InitStruct);
    
    /*Configure GPIO pin : SD_DETECT_Pin */
    GPIO_InitStruct.Pin = SD_DETECT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SD_DETECT_GPIO_Port, &GPIO_InitStruct);
    
    /*Configure GPIO pins : DCMI_SCL_Pin DCMI_SDA_Pin */
    GPIO_InitStruct.Pin = DCMI_SCL_Pin|DCMI_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
}

/**
* @brief  Period elapsed callback in non blocking mode
* @note   This function is called  when TIM1 interrupt took place, inside
* HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
* a global variable "uwTick" used as application time base.
* @param  htim : TIM handle
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{    
    if (htim->Instance == TIM1) 
    {
        HAL_IncTick();
    }
}

/**
* @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
void _Error_Handler(char * file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1) 
    {
    }
    /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
    
}

#endif

/**
* @}
*/ 

/**
* @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
