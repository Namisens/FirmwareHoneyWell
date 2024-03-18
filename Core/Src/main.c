/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "mx25r1635f.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TEMPSENSOR_CAL1_ADDR   ((uint16_t*) (0x1FFF75A8UL))
#define TEMPSENSOR_CAL2_ADDR   ((uint16_t*) (0x1FFF75CAUL))

#define MX25R1635F_PAGE_PROGRAM_CMD        0x02
#define MX25R1635F_READ_DATA_CMD           0x03
#define MX25R1635F_WRITE_DISABLE_CMD   		 0x04
#define MX25R1635F_READ_STATUS_REG_CMD     0x05
#define MX25R1635F_WRITE_ENABLE_CMD 	     0x06

#define MX25R1635F_READ_ID_CMD 						 0x9F
#define MX25R1635F_SECTOR_ERASE_CMD 			 0x20
#define MX25R1635F_CHIP_ERASE_CMD 				 0xC7
#define MX25R1635F_BLOCK_ERASE_CMD 				 0xD8

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t rawtxData[3];
volatile int convCompleted = 0;
volatile uint8_t rx_buffer[4]={0};
volatile uint8_t msg_received_bool = 0;
char msg[100];
int msg_length;
char spi_buffer[20];
uint8_t manufacturer_id;
uint16_t device_id;
uint8_t msg_to_read[100];
uint8_t addr;
uint8_t wip;
char msg1[] = "Hello, this is a test message to be written to flash\r\n";




uint8_t Timeout = 100;
uint8_t success[4] = {0x01, 0x01, 0x01, 0x01};


uint16_t ts_cal1;
uint16_t ts_cal2;
uint16_t ts_cal1_temp = TEMPSENSOR_CAL1_TEMP;
uint16_t ts_cal2_temp = TEMPSENSOR_CAL2_TEMP;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void getTemperature(uint16_t rawValue, float *temp);
void WriteToFlash(void);
void refresh_rx_buffer(void);
void MX_FLASH_WriteData(uint32_t address, uint8_t *data, uint16_t size);
void MX_FLASH_EraseSector(uint32_t sector_address);
void MX_FLASH_ReadData(uint32_t address, uint8_t *data, uint16_t size);
void MX_FLASH_EraseChip(uint32_t sector_address);
void MX_FLASH_ReadID(uint8_t *manufacturer_id, uint16_t *device_id);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	// Get the calibration values
	ts_cal1 = *TEMPSENSOR_CAL1_ADDR;
	ts_cal2 = *TEMPSENSOR_CAL2_ADDR;



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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

//Start the timer
  HAL_TIM_Base_Start(&htim14);
//HAL SPI
//HAL_SPI_TransmitReceive(&hspi1, *tx_buffer, *rx_buffer, 10, 100); // @suppress("Avoid magic numbers")
  HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_buffer, sizeof(rx_buffer));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	 //HAL_UART_Receive(&huart1, rx_buffer, sizeof(rx_buffer), 100);

	 //if(rx_buffer[0] == 0x01 || rx_buffer[0] == 0x02 || rx_buffer[0] == 0x03){
	 if (msg_received_bool == 1) {
		 switch (rx_buffer[0]) {
			case 0x01:// Connect to the device
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
				if (memcmp(rx_buffer, success, sizeof(success)) == 0) {
					HAL_UART_Transmit(&huart1, (uint8_t*) rx_buffer,sizeof(rx_buffer), Timeout);
				}
				msg_received_bool = 0;

				break;
			case 0x02:// Command to start ADC conversion and send temperature data
				if (rx_buffer[1] == 0x01) {
			    // Start ADC conversion
				HAL_ADC_Start_DMA(&hadc1, (uint32_t*) rawtxData, 3);

				// Transmit raw data
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);

				uint16_t rawValue = rawtxData[2];// Get Temperature Value of the MCU
				float temp;
				getTemperature(rawValue, &temp);

				HAL_UART_Transmit(&huart1, (uint8_t*)rawtxData, 4, Timeout);
				HAL_UART_Transmit(&huart1, (uint8_t*) &temp, sizeof(temp), Timeout);
//				sprintf(msg,"T1 : %d, T2 : %d, T3 : %d\n", rawtxData[0], rawtxData[1], rawtxData[2]);
//				HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), Timeout);

				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
				msg_received_bool = 0;

				}
				break;
			case 0x03:// Command to set up different Multiplexer Options // @suppress("Avoid magic numbers")
				switch (rx_buffer[1]){
					case 0x01:
						// Set up MUX options
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // MUX 1
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // MUX 2
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);// MUX 3
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // MUX 4
						// Prepare response
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1, (uint8_t*) success, 1, Timeout);
						msg_received_bool = 0;

						break;

					case 0x02:
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // MUX 1
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // MUX 2
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);// MUX 3
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // MUX 4
						// Prepare response
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1, (uint8_t*) success, 1, Timeout);
						msg_received_bool = 0;

						break;

					case 0x03: // @suppress("Avoid magic numbers")
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // MUX 1
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // MUX 2
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // MUX 3
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // MUX 4
						// Prepare response
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1, (uint8_t*) success, 1, Timeout);
						msg_received_bool = 0;

						break;

					case 0x04: // @suppress("Avoid magic numbers")
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // MUX 1
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // MUX 2
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // MUX 3
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // MUX 4
						// Prepare response
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1, (uint8_t*) success, 1, Timeout);
						msg_received_bool = 0;

						break;

					case 0x05: // @suppress("Avoid magic numbers")
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); // MUX 1
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // MUX 2
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // MUX 3
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // MUX 4
						// Prepare response
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1, (uint8_t*) success, 1, Timeout);
						msg_received_bool = 0;

						break;

					case 0x06: // @suppress("Avoid magic numbers")
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // MUX 1
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // MUX 2
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // MUX 3
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // MUX 4
						// Prepare response
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1, (uint8_t*) success, 1, Timeout);
						msg_received_bool = 0;

						break;

					case 0x07: // @suppress("Avoid magic numbers")
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); // MUX 1
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // MUX 2
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // MUX 3
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // MUX 4
						// Prepare response
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1, (uint8_t*) success, 1, Timeout);
						msg_received_bool = 0;

						break;

					default:
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // MUX 1
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // MUX 2
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // MUX 3
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // MUX 4
						// Prepare response
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
						HAL_UART_Transmit(&huart1, (uint8_t*) success, 1, Timeout);
						msg_received_bool = 0;
						break;

				}
				break;

			case 0x04: 
				switch (rx_buffer[1]){
				case 0x01:
					// Read Flash Information

					// Example data to be written
					
					msg_length = sizeof(msg1);
					uint32_t address_to_write = 0x018000; 

					// Write data to flash
					sprintf(msg, "Data written to flash: %s", msg1);
					mx25r1635f_write_data_and_wait(address_to_write, (uint8_t*)msg1, msg_length);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
					HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), Timeout);
					break;

				case 0x02:
					//Read Flash Information
					uint32_t address_to_read = 0x018000; // Start address to read data
					uint16_t size;
					size = sizeof(msg1);
					mx25r1635f_read_data(address_to_read, msg_to_read, size);
					sprintf(msg, "Data read from flash: %s\r\n", msg_to_read);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
					HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), Timeout);
					break;

				case 0x03:
					//Erase Flash Information
					uint32_t address_to_erase = 0x000000; // Start address to erase data
					MX_FLASH_EraseChip(address_to_erase);
					sprintf(msg, "Flash Erased\r\n");
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
					HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), Timeout);
					break;

				case 0x04: // @suppress("Avoid magic numbers")
					//Load Flash Information
					MX_FLASH_ReadID(&manufacturer_id, &device_id);

					// Print the IDs
					sprintf(msg,"Manufacturer ID: 0x%02X\r\n", manufacturer_id);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
					HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), Timeout);
					sprintf(msg,"Device ID: 0x%04X\r\n", device_id);
					HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), Timeout);
					break;

				}

			default:
				//refresh_rx_buffer();
				msg_received_bool = 0;
				refresh_rx_buffer();

				break;
		 }
	 }




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Receive data over UART



  }

  /* USER CODE END 3 */
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	convCompleted = 1;
	HAL_ADC_Stop_DMA(&hadc1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	// Transmission is complete
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	// Received data is in rx_buffer
	// Process received data
	msg_received_bool = 1;
	HAL_UART_Receive_IT(&huart1, (uint8_t*) rx_buffer, sizeof(rx_buffer));

}

void refresh_rx_buffer() {
	memset(rx_buffer, 0, sizeof(rx_buffer));
}

void getTemperature(uint16_t rawValue, float *temp) {
	// Calculate temperature
	float difference =((float) rawValue - (float) ts_cal1);
	float slope = (((float) ts_cal2_temp -(float) ts_cal1_temp)/((float) ts_cal2 - (float) ts_cal1));
	float offset = (float) ts_cal1_temp;
	*temp = (difference * slope) + offset;
}

void MX_FLASH_WriteData(uint32_t address, uint8_t *data, uint16_t size) {
    // Enable write operation
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_RESET); // CS low)

    // Send write enable command
    uint8_t cmd_enable_write = MX25R1635F_WRITE_ENABLE_CMD;
    HAL_SPI_Transmit(&hspi1, &cmd_enable_write, 1, HAL_MAX_DELAY);

    // Write data
    uint8_t cmd_write_data[4];
    cmd_write_data[0] = MX25R1635F_PAGE_PROGRAM_CMD;
    cmd_write_data[1] = (address >> 16) & 0xFF;
    cmd_write_data[2] = (address >> 8) & 0xFF;
    cmd_write_data[3] = address & 0xFF;
    HAL_SPI_Transmit(&hspi1, cmd_write_data, 4, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, data, size, HAL_MAX_DELAY);

    // Disable write operation
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_SET); // CS high
}

//Erase flash sector
void MX_FLASH_EraseSector(uint32_t sector_address) {
    // Enable write operation
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_RESET); // CS low)

    // Send write enable command
    uint8_t cmd_enable_write = MX25R1635F_WRITE_ENABLE_CMD;
    HAL_SPI_Transmit(&hspi1, &cmd_enable_write, 1, HAL_MAX_DELAY);

    // Send sector erase command
    uint8_t cmd_erase_sector[4];
    cmd_erase_sector[0] = MX25R1635F_SECTOR_ERASE_CMD;
    cmd_erase_sector[1] = (sector_address >> 16) & 0xFF;
    cmd_erase_sector[2] = (sector_address >> 8) & 0xFF;
    cmd_erase_sector[3] = sector_address & 0xFF;
    HAL_SPI_Transmit(&hspi1, cmd_erase_sector, 4, HAL_MAX_DELAY);

    // Disable write operation
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_SET); // CS low

    uint8_t wip=1;

    uint8_t read_status_register_command = MX25R1635F_READ_STATUS_REG_CMD;

    while(wip==1){
      HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin,
                        GPIO_PIN_RESET);
      HAL_SPI_Transmit(&hspi1, &read_status_register_command, 1, Timeout);
      HAL_SPI_Receive(&hspi1, (uint8_t*)msg, 1, Timeout);
      HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin,
                        GPIO_PIN_SET);

      //Check if Write in Progress
      wip = msg[0] & 0x01;
    }


}

//Erase Chip
void MX_FLASH_EraseChip(uint32_t sector_address) {
    // Enable write operation
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_RESET); // CS low)

    // Send write enable command
    uint8_t cmd_enable_write = MX25R1635F_WRITE_ENABLE_CMD;
    HAL_SPI_Transmit(&hspi1, &cmd_enable_write, 1, HAL_MAX_DELAY);

    // Send sector erase command
    uint8_t cmd_erase_sector[4];
    cmd_erase_sector[0] = MX25R1635F_CHIP_ERASE_CMD;
    cmd_erase_sector[1] = (sector_address >> 16) & 0xFF;
    cmd_erase_sector[2] = (sector_address >> 8) & 0xFF;
    cmd_erase_sector[3] = sector_address & 0xFF;
    HAL_SPI_Transmit(&hspi1, cmd_erase_sector, 4, HAL_MAX_DELAY);

    // Disable write operation
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_SET); // CS low
}

// Read data from flash
void MX_FLASH_ReadData(uint32_t address, uint8_t *data, uint16_t size) {
    // Enable read operation
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_RESET); // CS low

    // Send read command
    uint8_t cmd_read_data[4];
    cmd_read_data[0] = MX25R1635F_READ_DATA_CMD;
    cmd_read_data[1] = (address >> 16) & 0xFF;
    cmd_read_data[2] = (address >> 8) & 0xFF;
    cmd_read_data[3] = address & 0xFF;
    HAL_SPI_Transmit(&hspi1, cmd_read_data, 4, HAL_MAX_DELAY);

    // Receive data
    HAL_SPI_Receive(&hspi1, data, size, HAL_MAX_DELAY);

    // Disable read operation
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_SET); // CS high
}

void MX_FLASH_ReadID(uint8_t *manufacturer_id, uint16_t *device_id) {
    // Enable read operation
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_RESET); // CS low

    // Send read ID command
    uint8_t cmd_read_id = MX25R1635F_READ_ID_CMD;
    HAL_SPI_Transmit(&hspi1, &cmd_read_id, 1, HAL_MAX_DELAY);

    // Receive manufacturer and device IDs
    HAL_SPI_Receive(&hspi1, manufacturer_id, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, (uint8_t*)device_id, 2, HAL_MAX_DELAY);

    // Disable read operation
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_SET); // CS high
}

void WriteToFlash() {
    // Write to flash

    uint8_t write_enable_cmd = MX25R1635F_WRITE_ENABLE_CMD;
    uint8_t read_status_register_command = MX25R1635F_READ_STATUS_REG_CMD;
    uint8_t  page_program_command = MX25R1635F_PAGE_PROGRAM_CMD;
    uint8_t  read_data_command = MX25R1635F_READ_DATA_CMD;

    // Say something
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    msg_length = sprintf(msg, "Writing to Flash\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*) msg, msg_length, Timeout);

    //Enable Write Operations

    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &write_enable_cmd, 1, Timeout);
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_SET);

    //Read Status Register
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &read_status_register_command, 1, Timeout);
    HAL_SPI_Receive(&hspi1, (uint8_t*)spi_buffer, 1, Timeout);
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_SET);


    //Print out status register
    msg_length = sprintf(msg, " Status: 0x%02X\r\n",(uint8_t) spi_buffer[0]);
    HAL_UART_Transmit(&huart1, (uint8_t*) msg, msg_length, Timeout);

    //Test Write
    spi_buffer[0] = 0xAB;
    spi_buffer[1] = 0xCD;
    spi_buffer[2] = 0xEF;

    //Starting Address
    addr = 0x05;

    //Write 3 bytes starting at the given address
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &page_program_command, 1, Timeout);
    HAL_SPI_Transmit(&hspi1, &addr, 1, Timeout);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)spi_buffer, 3, Timeout);
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_SET);

    wip = 1;
    while (wip == 1) {
        //Read Status Register
        HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin,
                          GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, &read_status_register_command, 1, Timeout);
        HAL_SPI_Receive(&hspi1, (uint8_t*)spi_buffer, 1, Timeout);
        HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin,
                          GPIO_PIN_SET);

        //Check if Write in Progress
        wip = spi_buffer[0] & 0x01;
    }

    //Read the 3 bytes back
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &read_data_command, 1, Timeout);
    HAL_SPI_Transmit(&hspi1, &addr, 1, Timeout);
    HAL_SPI_Receive(&hspi1, (uint8_t*)spi_buffer, 3, Timeout);
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_SET);

    //Print out the 3 bytes
    msg_length = sprintf(msg, "Read: 0x%02X 0x%02X 0x%02X\r\n", spi_buffer[0], spi_buffer[1], spi_buffer[2]);
    HAL_UART_Transmit(&huart1, (uint8_t*) msg, msg_length, Timeout);

    //Read Status Register
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &read_status_register_command, 1, Timeout);
    HAL_SPI_Receive(&hspi1, (uint8_t*)spi_buffer, 1, Timeout);
    HAL_GPIO_WritePin(CHIP_SELECT_FLASH_GPIO_Port, CHIP_SELECT_FLASH_Pin, GPIO_PIN_SET);

    //Print out status register
    msg_length = sprintf(msg, " Status: 0x%02X",(uint8_t) spi_buffer[0]);
    HAL_UART_Transmit(&huart1, (uint8_t*) msg, msg_length, Timeout);

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
