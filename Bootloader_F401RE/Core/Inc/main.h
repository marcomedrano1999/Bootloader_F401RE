/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */


#define FLASH_SECTOR_2_BASE_ADDRESS		0x08008000

/*  Bootloader version  */
#define BL_VERSION			0x10


/* Memory macros */
#define SRAM1_SIZE				(96*1024)	//STM32F401RE has 96 KB of SRAM
#define SRAM1_END				(SRAM1_BASE+SRAM1_SIZE)
#define SRAM2_SIZE				0	// STM32F401RE does not have second SRAM
#define SRAM2_END				0	// STM32F401RE does not have second SRAM
#define FLASH_SIZE				(512*1024)	//STM32F401RE has 512 KB of flash memory
#define BKPSRAM_SIZE			0	// STM32F401RE does not have backup SRAM
#define BKPSRAM_END				0	// STM32F401RE does not have backup SRAM

#define ADDR_VALID				0x00
#define ADDR_INVALID			0x01
#define INVALID_SECTOR			0xFF



void bootloader_jump_to_user_app(void);
void bootloader_uart_read_data(void);
void bootloader_uart_write_data(uint8_t *pBuffer, uint32_t len);
uint8_t get_bootloader_version(void);


/* Define Bootloader commands */
#define BL_GET_VER					0x51		/*	CMD to read bootloader version from the MCU	*/
#define BL_GET_HELP					0x52		/*	CMD used to know supported CMDs	*/
#define BL_GET_CID					0x53		/*	CMD used to read MCU identification number	*/
#define BL_GET_RDP_STATUS			0x54		/*	CMD used to read the FLASH read protection level	*/
#define BL_GO_TO_ADDR				0x55		/*	CMD used to jump bootloader to specified address	*/
#define BL_FLASH_ERASE				0x56		/*	CMD used to mass erase or sector erase of the user flash	*/
#define BL_MEM_WRITE				0x57		/*	CMD used to write data into different memories of the MCU	*/
#define BL_EN_R_W_PROTECT			0x58		/*	CMD used to enable read/write protection on different sectors of the user flash	*/
#define BL_MEM_READ					0x59		/*	CMD used to read data from different memories of the MCU	*/
#define BL_READ_SECTOR_STATUS		0x5A		/*	CMD used to read all sector protection status	*/
#define BL_OTP_READ					0x5B		/*	CMD used to red the OTP contents	*/
#define BL_DIS_R_W_PROTECT			0x5C		/*	CMD used to disable read/write protection on different sectors of the user flash	*/


/* Declare bootloader cmds handlers */
void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_gethelp_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_getcid_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_getrdp_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_go_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_flash_erase_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_mem_write_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_endis_rw_protect_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_mem_read_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_read_sector_status_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_otp_read_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_dis_r_w_protect_cmd(uint8_t *bl_rx_buffer);


/*  Define ACK and NACK function and macros  */
#define BL_ACK			0xA5
#define BL_NACK			0x7F
void bootloader_send_nack(void);
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len);


/* CRC macro definitions */
#define VERIFY_CRC_FAIL			1
#define VERIFY_CRC_SUCCESS		0
uint8_t bootloader_verify_crc(uint8_t* pData, uint32_t len, uint32_t crc_host);




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
