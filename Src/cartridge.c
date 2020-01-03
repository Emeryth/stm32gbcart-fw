/*
 * cartridge.c
 *
 *  Created on: Jan 2, 2020
 *      Author: andrju
 */

#include "cartridge.h"
#include <string.h>
#include "main.h"

//#include "gejmboj.h"
#include "lsdj.h"
//#include "save.h"
//#include "roccow.h"

uint8_t sram[RAM1_SIZE];
uint8_t sram2[RAM2_SIZE]__attribute__((section(".ccmram")));

cartridge_t cartridge = { .rom = lsdj_gb, .ram = sram, .rom_bank = 1,
		.ram_bank = 0, };

void load_ram(void) {

//	memcpy(sram, roccow_sav, RAM1_SIZE);
//	memcpy(sram2, roccow_sav + RAM1_SIZE, RAM2_SIZE);

	uint32_t Address = SRAM_SAVE_ADDR;

	 for(int i=0;i < RAM1_SIZE/4;i++){
		 ((uint32_t*)sram)[i]=*((uint32_t*)Address);
		 Address+=4;
	 }
	 for(int i=0;i < RAM2_SIZE/4;i++){
		 ((uint32_t*)sram2)[i]=*((uint32_t*)Address);
		 Address+=4;
	 }

}

static FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t SectorError = 0;

void erase_ram(void){

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = FLASH_SECTOR_11;
	EraseInitStruct.NbSectors = 1;
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {

		Error_Handler();
	}
}

void save_ram(void) {
	LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin);
	HAL_FLASH_Unlock();

	erase_ram();

	uint32_t Address = SRAM_SAVE_ADDR;
	int i=0;

	  for(i=0;i < RAM1_SIZE/4;i++)
	  {
	    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, ((uint32_t*)sram)[i]) == HAL_OK)
	    {
	      Address = Address + 4;
	    }
	    else
	    {
	      Error_Handler();
	    }
	  }
	  i=0;
	  for(i=0;i < RAM2_SIZE/4;i++)
	  {
	    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, ((uint32_t*)sram2)[i]) == HAL_OK)
	    {
	      Address = Address + 4;
	    }
	    else
	    {
	      Error_Handler();
	    }
	  }

	HAL_FLASH_Lock();

	LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);

	HAL_PWR_EnterSTANDBYMode();
}
