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
#include "usb_device.h"
#include "ff.h"

FIL	appFile;
char fileList[256][12+1];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

FRESULT openFile(char *filename) {
    DIR dj;
    FILINFO fno;
    FRESULT fr;

    fr = f_opendir(&dj, "");
    if(fr == FR_OK){
		fr = f_readdir(&dj, &fno);
		uint8_t i = 0;
		while (fr == FR_OK && fno.fname[0] && i<255) {
			if(strcmp(fno.fname, "AUTORUN.BIN")!=0){
				strcpy(fileList[i], fno.fname);
				i++;
			}
			fr = f_readdir(&dj, &fno);
		}
		f_closedir(&dj);
    }

/*
    fr = f_findfirst(&dj, &fno, "", "*");
    uint8_t i = 0;
    while (fr == FR_OK && fno.fname[0] && i<255) {
    	if(strcmp(fno.fname, "AUTORUN.BIN")!=0){
    		strcpy(fileList[i], fno.fname);
        	i++;
    	}
    	fr = f_findnext(&dj, &fno);
    }
    f_closedir(&dj);
*/
    if(filename != NULL) {
    	fr = f_open(&appFile, filename, FA_READ);
    }

    if(fr != FR_OK){
    	if (fileList[0]) {
    		fr = f_open(&appFile, fileList[0], FA_READ);
    	}
    }

    if (fr == FR_OK) {
    	uint32_t lktbl[256];
		lktbl[0] = 256;
		appFile.cltbl = lktbl;
		f_lseek(&appFile, CREATE_LINKMAP);
	}

    return fr;
}

void fastGpioInitOutputPP(GPIO_TypeDef *GPIOx, uint8_t pinNum) {
	#define  GPIO_CR_CNF_GP_OUTPUT_PP   0x00000000U
	uint32_t position = (pinNum < 8) ? (pinNum << 2U) : ((pinNum - 8U) << 2U);
	__IO uint32_t *configregister = (pinNum < 8) ? &GPIOx->CRL : &GPIOx->CRH;
    MODIFY_REG((*configregister), ((GPIO_CRL_MODE0 | GPIO_CRL_CNF0) << position),
    		((GPIO_SPEED_FREQ_LOW + GPIO_CR_CNF_GP_OUTPUT_PP) << position));
}


void fastGpioInitInputPU(GPIO_TypeDef *GPIOx, uint8_t pinNum) {
	#define  GPIO_CR_MODE_INPUT         0x00000000U //!< 00: Input mode (reset state)
	#define  GPIO_CR_CNF_INPUT_PU_PD    0x00000008U //!< 10: Input with pull-up / pull-down
	uint32_t position = (pinNum < 8) ? (pinNum << 2U) : ((pinNum - 8U) << 2U);
	__IO uint32_t *configregister = (pinNum < 8) ? &GPIOx->CRL : &GPIOx->CRH;
	GPIOx->BSRR = 0x01U << pinNum;
    MODIFY_REG((*configregister), ((GPIO_CRL_MODE0 | GPIO_CRL_CNF0) << position),
    		((GPIO_CR_MODE_INPUT + GPIO_CR_CNF_INPUT_PU_PD) << position));
}

void fastGpioInitEvtFal(GPIO_TypeDef *GPIOx, uint32_t pinNum) {
    __HAL_RCC_AFIO_CLK_ENABLE();
	uint32_t temp = AFIO->EXTICR[pinNum >> 2U];
	CLEAR_BIT(temp, (0x0FU) << (4U * (pinNum & 0x03U)));
	SET_BIT(temp, (GPIO_GET_INDEX(GPIOx)) << (4U * (pinNum & 0x03U)));
	AFIO->EXTICR[pinNum >> 2U] = temp;

	CLEAR_BIT(EXTI->IMR, 0x01U << pinNum); //GPIO_MODE_IT
	SET_BIT(EXTI->EMR, 0x01U << pinNum); //GPIO_MODE_EVT
	CLEAR_BIT(EXTI->RTSR, 0x01U << pinNum); //RISING_EDGE
	SET_BIT(EXTI->FTSR, 0x01U << pinNum); //FALLING_EDGE
}

uint32_t receiveData(uint8_t bitCount) {
	uint32_t data = 0;
	for(uint8_t i=0; i<bitCount; i++) {
		while((PORT_CLK->IDR & PIN_CLK) != GPIO_PIN_RESET &&
				(PORT_SELECT->IDR & PIN_SELECT) == GPIO_PIN_RESET);
		data <<= 1;
		if((PORT_DATA->IDR & PIN_DATA) != GPIO_PIN_RESET){
			data += 1;
		}
		while((PORT_CLK->IDR & PIN_CLK) == GPIO_PIN_RESET &&
				(PORT_SELECT->IDR & PIN_SELECT) == GPIO_PIN_RESET);
	}
	return data;
}

void sendByte(uint32_t data) {
	for(uint8_t i=0; i<8; i++) {
		while((PORT_CLK->IDR & PIN_CLK) != GPIO_PIN_RESET &&
				 (PORT_SELECT->IDR & PIN_SELECT) == GPIO_PIN_RESET);
		if(data & 0x80){
			PORT_DATA->BSRR = PIN_DATA;
		} else {
			PORT_DATA->BSRR = PIN_DATA << 16;
		}
		data <<= 1;
		while((PORT_CLK->IDR & PIN_CLK) == GPIO_PIN_RESET &&
				(PORT_SELECT->IDR & PIN_SELECT) == GPIO_PIN_RESET);
	}
}

void parseCommand(uint8_t command) {
	switch(command) {
		case 0x00: // query status
			fastGpioInitOutputPP(PORT_DATA, PIN_DATA_NUM);
			sendByte(0x00);
			fastGpioInitInputPU(PORT_DATA, PIN_DATA_NUM);
		break;

		case 0xA8: { // set 24b address
			uint32_t currentAddress = receiveData(24);
			f_lseek(&appFile, currentAddress);
		}
		break;

		case 0xA0: { // set 16b address
			uint32_t currentAddress = receiveData(16);
			f_lseek(&appFile, currentAddress);
		}
		break;

		case 0xD8:
		case 0xD0: // read postincrement
			fastGpioInitOutputPP(PORT_DATA, PIN_DATA_NUM);
			PORT_LED->BSRR = PIN_LED << 16;
			uint8_t	readBuffer;
			while((PORT_SELECT->IDR & PIN_SELECT) == GPIO_PIN_RESET) {
				if(f_read_byte(&appFile, &readBuffer)!=FR_OK){
					readBuffer = 0;
				}
				sendByte(readBuffer);
			}
			PORT_LED->BSRR = PIN_LED;
			fastGpioInitInputPU(PORT_DATA, PIN_DATA_NUM);
		break;

		case 0xF0:	{
			fastGpioInitOutputPP(PORT_DATA, PIN_DATA_NUM);
			uint8_t i=0;
			while(fileList[i][0]){
				for(uint8_t j = 0; fileList[i][j]; j++){
					sendByte(fileList[i][j]);
				}
				sendByte(0x00);
				i++;
			}
			sendByte(0xFF);
			fastGpioInitInputPU(PORT_DATA, PIN_DATA_NUM);
		}
		break;

		case 0xF1:	{
				char filename[12+1] = {0,};
				uint8_t i = 0;
				filename[i] = receiveData(8);
				while(filename[i] != 0x00 && i<12){
					filename[++i] = receiveData(8);
				}
				openFile(filename);
			}
			break;

		default: // unknown command
			Error_Handler();
			break;
	}
	while((PORT_SELECT->IDR & PIN_SELECT) == GPIO_PIN_RESET);
}

int main(void) {
	HAL_Init();
	//HAL_DBGMCU_EnableDBGSleepMode();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_USB_DEVICE_Init();

	FATFS FATFS_Obj;
	if(f_mount(&FATFS_Obj, "0", 1) == FR_OK) {
		openFile("autorun.bin");
	}

	while (1) {
		HAL_PWR_EnterSLEEPMode(0, PWR_SLEEPENTRY_WFE);
		if((PORT_SELECT->IDR & PIN_SELECT) == GPIO_PIN_RESET &&
			  (PORT_CLK->IDR & PIN_CLK) == GPIO_PIN_RESET){
			parseCommand(receiveData(8));
		}
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6; //RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4; //2
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4; //1
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL; //RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  PORT_LED->BSRR = PIN_LED;

  fastGpioInitInputPU(PORT_DATA, PIN_DATA_NUM);
  fastGpioInitInputPU(PORT_SELECT, PIN_SELECT_NUM);
  fastGpioInitOutputPP(PORT_LED, PIN_LED_NUM);
  fastGpioInitInputPU(PORT_CLK, PIN_CLK_NUM);
  fastGpioInitEvtFal(PORT_CLK, PIN_CLK_NUM);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
	while(1){
		HAL_Delay(100);
		PORT_LED->ODR ^= PIN_LED;
	}
}
