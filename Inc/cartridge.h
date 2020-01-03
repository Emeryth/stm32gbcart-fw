/*
 * cartridge.h
 *
 *  Created on: Jan 2, 2020
 *      Author: andrju
 */

#ifndef INC_CARTRIDGE_H_
#define INC_CARTRIDGE_H_

#include <stdint.h>

#define RAM_SIZE (128*1024)
#define RAM1_SIZE (96*1024)
#define RAM2_SIZE (32*1024)

#define ROM_SAVE_ADDR 0x08020000
#define SRAM_SAVE_ADDR 0x080E0000

#define ROM_MAX_SIZE (SRAM_SAVE_ADDR-ROM_SAVE_ADDR)
#define ROM_MAX_BANK (ROM_MAX_SIZE/(16*1024)-1)

typedef struct{

	const uint8_t *rom;
	uint8_t *ram;
	int rom_bank;
	int ram_bank;

}cartridge_t;

extern cartridge_t cartridge;
extern uint8_t sram[RAM1_SIZE];
extern uint8_t sram2[RAM2_SIZE];

void load_ram(void);
void save_ram(void);
void erase_ram(void);
void erase_rom(void);

#endif /* INC_CARTRIDGE_H_ */
