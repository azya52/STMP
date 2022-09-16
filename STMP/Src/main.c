/*
 * ucDisassembler.cpp
 * STM32 based cart for Elektronika MK-90, see https://github.com/azya52/STMP
 * Copyright (c) 2019, Alexander Zakharyan
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "main.h"
#include "ff.h"

FATFS FATFS_Obj;
FIL	appFile;
char fileList[256][12+1];
uint8_t readBuffer;

static void MX_GPIO_PullUp(void);
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void PVD_Config();

static FRESULT openFile(char *filename) {
    DIR dj;
    FILINFO fno;
    FRESULT fr;

    fr = f_opendir(&dj, "");
    if(fr == FR_OK){
		uint32_t i = 0;
		do {
			fr = f_readdir(&dj, &fno);
			if (fr == FR_OK && (strcmp(fno.fname, "AUTORUN.BIN") != 0) && !(fno.fattrib & AM_DIR)) {
				strcpy(fileList[i++], fno.fname);
			}
		} while (fr == FR_OK && i<255);
		f_closedir(&dj);
    }

    if(filename != NULL) {
    	fr = f_open(&appFile, filename, FA_READ);
    }

    if (fr != FR_OK && fileList[0]) {
    	fr = f_open(&appFile, fileList[0], FA_READ);
    }

    if (fr == FR_OK) {
    	uint32_t lktbl[256];
		lktbl[0] = 256;
		appFile.cltbl = lktbl;
		fr = f_lseek(&appFile, CREATE_LINKMAP);
	}

    return fr;
}

static void fastGpioInitOutputPP(GPIO_TypeDef *GPIOx, uint32_t pinNum) {
	#define  GPIO_CR_CNF_GP_OUTPUT_PP   0x00000000U
	uint32_t position = ((pinNum - 8U) << 2);
    MODIFY_REG(GPIOx->CRH, ((GPIO_CRL_MODE0 | GPIO_CRL_CNF0) << position),
    		((GPIO_SPEED_FREQ_HIGH + GPIO_CR_CNF_GP_OUTPUT_PP) << position));
}

static void fastGpioInitOutputPP_DATA(void) {
	#define  GPIO_CR_CNF_GP_OUTPUT_OD   0x00000004U
    MODIFY_REG(PORT_DATA->CRH, ((GPIO_CRL_MODE0 | GPIO_CRL_CNF0) << ((PIN_DATA_NUM - 8) << 2)),
    		((GPIO_SPEED_FREQ_HIGH + GPIO_CR_CNF_GP_OUTPUT_PP) << ((PIN_DATA_NUM - 8) << 2)));
}

static void fastGpioInitInputPU(GPIO_TypeDef *GPIOx, uint32_t pinNum) {
	#define  GPIO_CR_MODE_INPUT         0x00000000U //!< 00: Input mode (reset state)
	#define  GPIO_CR_CNF_INPUT_PU_PD    0x00000008U //!< 10: Input with pull-up / pull-down
	uint32_t position = (pinNum < 8) ? (pinNum << 2U) : ((pinNum - 8U) << 2U);
	__IO uint32_t *configregister = (pinNum < 8) ? &GPIOx->CRL : &GPIOx->CRH;
	GPIOx->BSRR = 0x01U << pinNum;	//pull-up
    MODIFY_REG(*configregister, ((GPIO_CRL_MODE0 | GPIO_CRL_CNF0) << position),
    		((GPIO_CR_MODE_INPUT + GPIO_CR_CNF_INPUT_PU_PD) << position));
}

static void fastGpioInitEvtFal(GPIO_TypeDef *GPIOx, uint32_t pinNum) {
    __HAL_RCC_AFIO_CLK_ENABLE();
	MODIFY_REG(AFIO->EXTICR[pinNum >> 2], (0x0F << (4 * (pinNum & 0x03))),
				(GPIO_GET_INDEX(GPIOx)) << (4 * (pinNum & 0x03)));

	CLEAR_BIT(EXTI->IMR, 0x01U << pinNum); //GPIO_MODE_IT
	SET_BIT(EXTI->EMR, 0x01U << pinNum); //GPIO_MODE_EVT
	CLEAR_BIT(EXTI->RTSR, 0x01U << pinNum); //RISING_EDGE
	SET_BIT(EXTI->FTSR, 0x01U << pinNum); //FALLING_EDGE
}

static uint32_t receiveData(uint32_t bitCount) {
	uint32_t data = 0;
	for(uint32_t i = 0; i < bitCount; i++) {
		while ((PORT_CLK->IDR & PIN_CLK) != GPIO_PIN_RESET &&
				(PORT_SELECT->IDR & PIN_SELECT) == GPIO_PIN_RESET);
		data <<= 1;
		if ((PORT_DATA->IDR & PIN_DATA) != GPIO_PIN_RESET) {
			data++;
		}
		while((PORT_CLK->IDR & PIN_CLK) == GPIO_PIN_RESET);
	}
	return data;
}

static void sendByte(uint32_t data) {
	for(uint32_t i = 0; i < 8; i++) {
		while((PORT_CLK->IDR & PIN_CLK) != GPIO_PIN_RESET &&
				 (PORT_SELECT->IDR & PIN_SELECT) == GPIO_PIN_RESET);
		if(data & 0x80){
			PORT_DATA->BSRR = PIN_DATA;
		} else {
			PORT_DATA->BSRR = PIN_DATA << 16;
		}
		data <<= 1;
		while((PORT_CLK->IDR & PIN_CLK) == GPIO_PIN_RESET);
	}
}

static void parseCommand(uint32_t command) {
	switch(command) {
		// query status
		case 0x00: {
			fastGpioInitOutputPP_DATA();
			sendByte(0x00);
			fastGpioInitInputPU(PORT_DATA, PIN_DATA_NUM);
			break;
		}

		// set 24b address
		case 0xA8: {
			f_lseek(&appFile, receiveData(24));
			f_read_byte(&appFile, &readBuffer);
			break;
		}

		// set 16b address
		case 0xA0: {
			f_lseek(&appFile, receiveData(16));
			f_read_byte(&appFile, &readBuffer);
			break;
		}

		// read postincrement
		case 0xD8:
		case 0xD0: {
			fastGpioInitOutputPP_DATA();
			PORT_LED->BSRR = PIN_LED << 16;

			while ((PORT_SELECT->IDR & PIN_SELECT) == GPIO_PIN_RESET) {
				sendByte(readBuffer);
				f_read_byte(&appFile, &readBuffer);
			}

			f_lseek(&appFile, appFile.fptr - 1);
			f_read_byte(&appFile, &readBuffer);

			PORT_LED->BSRR = PIN_LED;
			fastGpioInitInputPU(PORT_DATA, PIN_DATA_NUM);
			break;
		}

		case 0xF0: {
			fastGpioInitOutputPP_DATA();
			uint32_t i = 0;
            uint32_t j = 0;

            do {
            	j = 0;
                do {
                	sendByte(fileList[i][j]);
                } while (fileList[i][j++]);
            } while (fileList[i++][0]);

			sendByte(0xFF);
			fastGpioInitInputPU(PORT_DATA, PIN_DATA_NUM);
			break;
		}

		case 0xF1: {
			char filename[12+1] = {0,};
			uint32_t i = 0;

			do {
				filename[i] = receiveData(8);
			} while (filename[i] && ++i < 13);

			if (openFile(filename) != FR_OK) {
				Error_Handler();
			}
			break;
		}

		 // unknown command
		default: {
			Error_Handler();
			break;
		}
	}

	while((PORT_SELECT->IDR & PIN_SELECT) == GPIO_PIN_RESET);
}

int main(void) {
	HAL_Init();
	PVD_Config();
	MX_GPIO_PullUp();
	MX_GPIO_Init();
	SystemClock_Config();
	MX_USB_DEVICE_Init();
	HAL_SuspendTick();	//disable exit from sleep by pvd?

	while (1) {
		if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO)) {
			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFE);
			SystemClock_Config();
			if(f_mount(&FATFS_Obj, "0", 1) == FR_OK) {
				openFile("autorun.bin");
			}
		} else {
			HAL_PWR_EnterSLEEPMode(0, PWR_SLEEPENTRY_WFE);
		}
		if ((PORT_SELECT->IDR & PIN_SELECT) == GPIO_PIN_RESET &&
			  (PORT_CLK->IDR & PIN_CLK) == GPIO_PIN_RESET) {
			parseCommand(receiveData(8));
		}
	}
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
static void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_GPIO_PullUp(void) {
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/* all pin input with pull-down */
	WRITE_REG(GPIOA->CRL, 0x88888888U);
	WRITE_REG(GPIOA->CRH, 0x88888888U);
	GPIOA->BSRR = 0x0000;
	WRITE_REG(GPIOB->CRL, 0x88888888U);
	WRITE_REG(GPIOB->CRH, 0x88888888U);
	GPIOB->BSRR = 0xFFFF;
	WRITE_REG(GPIOC->CRH, 0x88888888U);
	GPIOC->BSRR = 0x0000 | PIN_LED;
	WRITE_REG(GPIOD->CRH, 0x88888888U);
	GPIOD->BSRR = 0x0000;
}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
  fastGpioInitInputPU(PORT_DATA, PIN_DATA_NUM);
  fastGpioInitInputPU(PORT_SELECT, PIN_SELECT_NUM);
  fastGpioInitEvtFal(PORT_SELECT, PIN_SELECT_NUM);
  fastGpioInitInputPU(PORT_CLK, PIN_CLK_NUM);
  fastGpioInitEvtFal(PORT_CLK, PIN_CLK_NUM);
  fastGpioInitOutputPP(PORT_LED, PIN_LED_NUM);
}


static void PVD_Config() {
	__HAL_RCC_PWR_CLK_ENABLE();	//Enable Power Clock

	HAL_NVIC_SetPriority(PVD_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(PVD_IRQn);

    PWR_PVDTypeDef sConfigPVD;
    sConfigPVD.PVDLevel = PWR_CR_PLS_2V9;
    sConfigPVD.Mode =  PWR_PVD_MODE_IT_RISING;
   	HAL_PWR_ConfigPVD(&sConfigPVD); //event on supply voltage < 2.8V
    HAL_PWR_EnablePVD();
}

void PVD_IRQHandler(void) {
    HAL_PWR_PVD_IRQHandler();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
	HAL_ResumeTick();
	while(1){
		HAL_Delay(100);
		PORT_LED->ODR ^= PIN_LED;
	}
}
