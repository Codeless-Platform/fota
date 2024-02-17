/*
 * Bootloader.h
 *
 *  Created on: Feb 16, 2024
 *      Author: Alaa
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_

#include "usart.h"
#include "crc.h"

#define HOSTM_MAX_SIZE    200

#define CBL_GET_VER_CMD              0x10
#define CBL_GET_HELP_CMD             0x11
#define CBL_GET_CID_CMD              0x12
#define CBL_GO_TO_ADDR_CMD           0x14
#define CBL_FLASH_ERASE_CMD          0x15
#define CBL_MEM_WRITE_CMD            0x16

#define CRC_VERIFING_FAILED  0X00
#define CRC_VERIFING_PASS  0X01

#define SEND_NACK        0xAB
#define SEND_ACK         0xCD

#define CBL_VENDOR_ID                100
#define CBL_SW_MAJOR_VERSION         1
#define CBL_SW_MINOR_VERSION         1
#define CBL_SW_PATCH_VERSION         0

#define INVALID_PAGE_NUMBER          0x00
#define VALID_PAGE_NUMBER            0x01
#define UNSUCCESSFUL_ERASE           0x02
#define SUCCESSFUL_ERASE             0x03

#define CBL_FLASH_MAX_PAGE_NUMBER    16
#define CBL_FLASH_MASS_ERASE         0xFF

#define HAL_SUCCESSFUL_ERASE         0xFFFFFFFFU

#define ADDRESS_IS_INVALID           0x00
#define ADDRESS_IS_VALID             0x01

#define FLASH_PAYLOAD_WRITE_FAILED  0x00
#define FLASH_PAYLOAD_WRITE_PASSED  0x01

#define STM32F103_SRAM_SIZE         (20 * 1024)
#define STM32F103_FLASH_SIZE         (64 * 1024)
#define STM32F103_SRAM_END          (SRAM_BASE + STM32F103_SRAM_SIZE)
#define STM32F103_FLASH_END          (FLASH_BASE + STM32F103_FLASH_SIZE)


typedef enum{
	BL_NACK=0,
	BL_ACK
}BL_status;

void BL_SendMessage(char *format,...);
BL_status BL_FetchHostCommand();
static uint32_t BL_CRC_verfiy(uint8_t * pdata,uint32_t DataLen,uint32_t HostCRC);
static void BL_Send_ACK(uint8_t dataLen);
static void BL_Send_NACK();
static void BL_Get_Version(uint8_t *Host_buffer);
static void BL_Get_Help(uint8_t *Host_buffer);
static void BL_Get_Chip_ID_Number(uint8_t *Host_buffer);
static void BL_Flash_Erase(uint8_t *Host_buffer);
static uint8_t Perform_Flash_Erase(uint32_t PageAddress, uint8_t page_Number);
static void BL_Write_Data(uint8_t *Host_buffer);
static uint8_t BL_Address_Verification(uint32_t Addresss);
static uint8_t FlashMemory_Paylaod_Write(uint16_t * pdata,uint32_t StartAddress,uint8_t Payloadlen);

#endif /* INC_BOOTLOADER_H_ */
