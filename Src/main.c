/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cartridge.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
//  MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */
	LL_GPIO_SetPinPull(VGB_GPIO_Port, VGB_Pin, LL_GPIO_PULL_DOWN);
	for (int i = 0; i < 1000; i++) {
		NOP10
	}
	if (!LL_GPIO_IsInputPinSet(VGB_GPIO_Port, VGB_Pin)){
		// Cartridge is not in a Game Boy, enable USB
		HAL_InitTick(TICK_INT_PRIORITY);
		MX_USB_DEVICE_Init();
		while (1);
	}

	LL_GPIO_ResetOutputPin(RST_GPIO_Port, RST_Pin);
	load_ram();
	for (int i = 0; i < 1000; i++) {
		NOP10
	}
	LL_GPIO_SetOutputPin(RST_GPIO_Port, RST_Pin);
	NVIC_EnableIRQ(EXTI2_IRQn);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		while (!LL_GPIO_IsInputPinSet(CLK_GPIO_Port, CLK_Pin)) {

		}
		D0_GPIO_Port->MODER = 0x0000;
		NOP10
		NOP10
		NOP10
		NOP10
		LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);

		uint32_t address = LL_GPIO_ReadInputPort(A0_GPIO_Port);
		uint32_t control = LL_GPIO_ReadInputPort(RD_GPIO_Port);

		if (!(address & A15_Pin)) {
			// ROM operation

			if (!(control & RD_Pin)) {
				// read from ROM

				D0_GPIO_Port->MODER = 0x5555;

				if (address & A14_Pin) { // read from switchable bank

					LL_GPIO_WriteOutputPort(D0_GPIO_Port,
							cartridge.rom[(address & 0x3FFF)
									| (cartridge.rom_bank << 14)]);
				} else {
					// read from bank 0
					LL_GPIO_WriteOutputPort(D0_GPIO_Port,
							cartridge.rom[address & 0x3FFF]);
				}

			} else if ((address & A13_Pin)) {
				//change ROM bank

				while (LL_GPIO_IsInputPinSet(CLK_GPIO_Port, CLK_Pin)) {

				}
				NOP10

				cartridge.rom_bank = LL_GPIO_ReadInputPort(D0_GPIO_Port) & 0x3F;
				if (cartridge.rom_bank == 0) {
					cartridge.rom_bank++;
				}
				if (cartridge.rom_bank >= ROM_MAX_BANK)
					cartridge.rom_bank = ROM_MAX_BANK;

			} else if ((address & A14_Pin)) {
				// change RAM bank
				while (LL_GPIO_IsInputPinSet(CLK_GPIO_Port, CLK_Pin)) {

				}
				NOP10

				uint8_t bank = LL_GPIO_ReadInputPort(D0_GPIO_Port) & 0xF;
				if (bank >= 12) {
					cartridge.ram = sram2;
					cartridge.ram_bank = bank - 12;
				} else {
					cartridge.ram = sram;
					cartridge.ram_bank = bank;
				}
			}

		} else if (!(control & (CS_Pin)) && !(address & A14_Pin)) {
			// RAM operation

			if (!(control & RD_Pin)) {
				// read from RAM
				D0_GPIO_Port->MODER = 0x5555;
				LL_GPIO_WriteOutputPort(D0_GPIO_Port,
						cartridge.ram[(address & 0x1FFF)
								| (cartridge.ram_bank << 13)]);
			} else {
				// write to RAM

				while (LL_GPIO_IsInputPinSet(CLK_GPIO_Port, CLK_Pin)) {

				}
				NOP10

				uint8_t data = LL_GPIO_ReadInputPort(D0_GPIO_Port) & 0xFF;
				cartridge.ram[(address & 0x1FFF) | (cartridge.ram_bank << 13)] =
						data;

			}

		}

		LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
		while (LL_GPIO_IsInputPinSet(CLK_GPIO_Port, CLK_Pin)) {

		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);

	if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5) {
		Error_Handler();
	}
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
	LL_RCC_HSE_Enable();

	/* Wait till HSE is ready */
	while (LL_RCC_HSE_IsReady() != 1) {

	}
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 168,
	LL_RCC_PLLP_DIV_2);
	LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 168,
	LL_RCC_PLLQ_DIV_7);
	LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
	while (LL_RCC_PLL_IsReady() != 1) {

	}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {

	}
	LL_SetSystemCoreClock(168000000);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	LL_EXTI_InitTypeDef EXTI_InitStruct = { 0 };
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

	/**/
	LL_GPIO_ResetOutputPin(GPIOA, LED3_Pin | LED2_Pin | LED1_Pin);

	/**/
	LL_GPIO_ResetOutputPin(RST_GPIO_Port, RST_Pin);

	/**/
	GPIO_InitStruct.Pin = VBUS_Pin | VGB_Pin | D0_Pin | D1_Pin | D2_Pin | D3_Pin
			| D4_Pin | D5_Pin | D6_Pin | D7_Pin | WR_Pin | RD_Pin | CS_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = A0_Pin | A1_Pin | A2_Pin | A10_Pin | A11_Pin | A12_Pin
			| A13_Pin | A14_Pin | A15_Pin | A3_Pin | A4_Pin | A5_Pin | A6_Pin
			| A7_Pin | A8_Pin | A9_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LED3_Pin | LED2_Pin | LED1_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = RST_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

	/**/
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE8);

	/**/
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTD, LL_SYSCFG_EXTI_LINE2);

	/**/
	EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_8;
	EXTI_InitStruct.LineCommand = ENABLE;
	EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
	LL_EXTI_Init(&EXTI_InitStruct);

	/**/
	EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_2;
	EXTI_InitStruct.LineCommand = ENABLE;
	EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
	LL_EXTI_Init(&EXTI_InitStruct);

	/**/
	LL_GPIO_SetPinPull(CLK_GPIO_Port, CLK_Pin, LL_GPIO_PULL_NO);

	/**/
	LL_GPIO_SetPinPull(BUTTON_GPIO_Port, BUTTON_Pin, LL_GPIO_PULL_NO);

	/**/
	LL_GPIO_SetPinMode(CLK_GPIO_Port, CLK_Pin, LL_GPIO_MODE_INPUT);

	/**/
	LL_GPIO_SetPinMode(BUTTON_GPIO_Port, BUTTON_Pin, LL_GPIO_MODE_INPUT);

	/* EXTI interrupt init*/
	NVIC_SetPriority(EXTI2_IRQn,
			NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
//	NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
