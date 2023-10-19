/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <ublox_neo8.h>
#include <STM32_SPI.h>
#include <SX127X.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	// Position
	float latitude, longitude, altitude;
	// GPS Time
	uint8_t seconds, minutes, hours;
	// Packet Identifier
	uint16_t packet_id;
} __attribute__((packed)) telemetry_packet_values_t;

// Union of Packet data as struct and byte array
typedef union {
	telemetry_packet_values_t values;
	uint8_t raw[sizeof(telemetry_packet_values_t)];
} telemetry_packet_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMEZONE_OFFSET -3
#define TRANSMISSION_INTERVAL 300000
#define TRANSMISSION_TIMEOUT 2000
#define FIX_SEARCH_TIMEOUT 120000

#define UART4_RX_BUFFER_SIZE 255
#define UART4_RX_MAIN_BUFFER_SIZE 255

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t rx_buffer[UART4_RX_BUFFER_SIZE];
uint8_t main_buffer[UART4_RX_MAIN_BUFFER_SIZE];
uint16_t global_size = 0;

HAL_StatusTypeDef state_machine_status = HAL_OK;
HAL_StatusTypeDef transmitter_status = HAL_OK;

bool DIO0;
bool DIO1;
bool can_i_transmit = false;
bool fixed = false;

telemetry_packet_values_t telemetry_packet_values;
telemetry_packet_t telemetry_packet;

uint16_t packet_counter = 1;

unsigned long fix_search_time_stamp = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


HAL_StatusTypeDef LoRa_Config(SX127X_t* SX127X) {
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t InitErrorCounter = 0;

	/*
	 * Fdev and BitRate must respect the following formula: 0.5 <= (2*Fdev)/(BitRate) <= 10
	 * Also, Fdev + (BitRate/2) <= 250 kHz   and   600 Hz < Fdev < 200kHz
	 * BitRate must also respect: BitRate < 2*Bandwidth
	 */

	// Basic stuff first:
	SX127X->spi_bus = hspi2;
	SX127X->ss_gpio_port = SS_COMM_TX_GPIO_Port;
	SX127X->ss_pin = SS_COMM_TX_Pin;
	SX127X->reset_gpio_port =RESET_COMM_TX_GPIO_Port;
	SX127X->reset_pin = RESET_COMM_TX_Pin;

	status = SX127X_Reset(SX127X);
	if (status != HAL_OK) InitErrorCounter++;

	status = SX127X_set_op_mode(SX127X, SLEEP);
	if (status != HAL_OK) InitErrorCounter++;

	status = SX127X_set_modulation(SX127X, LORA_Modulation);
	if (status != HAL_OK) InitErrorCounter++;

	status = SX127X_set_frequency(SX127X, 916E6); // remember to put the LoWFrequencyModeOn bit to its correct position
	if (status != HAL_OK) InitErrorCounter++;

	status = SX127X_set_lna_gain(SX127X, LnaGainG1);
	if (status != HAL_OK) InitErrorCounter++;

	status = SX127X_set_lna_boost(SX127X, false);
	if (status != HAL_OK) InitErrorCounter++;

	// TX Config:

	status = SX127X_set_pa_output(SX127X, PA_BOOST_Pin);
	if (status != HAL_OK) InitErrorCounter++;

	status = SX127X_set_tx_power(SX127X, 20);
	if (status != HAL_OK) InitErrorCounter++;

	// Specific Config:

	status = LoRa_set_FIFO_base_address(SX127X, 0x00, 0x00);
	if (status != HAL_OK) InitErrorCounter++;

	status = LoRa_set_signal_bandwidth(SX127X, LoRa_BW_125);
	if (status != HAL_OK) InitErrorCounter++;

	status = LoRa_set_spreading_factor(SX127X, 11);
	if (status != HAL_OK) InitErrorCounter++;

	status = LoRa_set_coding_rate(SX127X, LoRa_CR_4_5);
	if (status != HAL_OK) InitErrorCounter++;

	status = LoRa_set_preamble_lenght(SX127X, 8);
	if (status != HAL_OK) InitErrorCounter++;

	status = LoRa_set_sync_word(SX127X, 'A');
	if (status != HAL_OK) InitErrorCounter++;

	status = LoRa_enable_crc(SX127X);
	if (status != HAL_OK) InitErrorCounter++;

	status = SX127X_set_DIO_mapping(SX127X, 0b01000000, 0b00000000);
	if (status != HAL_OK) InitErrorCounter++;

	status = SX127X_set_op_mode(SX127X, STANDBY);
	if (status != HAL_OK) InitErrorCounter++;

	return status;
}



HAL_StatusTypeDef FSK_Config(SX127X_t* SX127X) {
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t InitErrorCounter = 0;

	/*
	 * Fdev and BitRate must respect the following formula: 0.5 <= (2*Fdev)/(BitRate) <= 10
	 * Also, Fdev + (BitRate/2) <= 250 kHz   and   600 Hz < Fdev < 200kHz
	 * BitRate must also respect: BitRate < 2*Bandwidth
	 */

	// Basic stuff first:
	SX127X->spi_bus = hspi2;
	SX127X->ss_gpio_port = SS_COMM_TX_GPIO_Port;
	SX127X->ss_pin = SS_COMM_TX_Pin;
	SX127X->reset_gpio_port =RESET_COMM_TX_GPIO_Port;
	SX127X->reset_pin = RESET_COMM_TX_Pin;

	status = SX127X_Reset(SX127X);
	if (status != HAL_OK) InitErrorCounter++;

	status = SX127X_set_op_mode(SX127X, SLEEP);
	if (status != HAL_OK) InitErrorCounter++;

	status = SX127X_set_modulation(SX127X, FSK_Modulation);
	if (status != HAL_OK) InitErrorCounter++;

	status = SX127X_set_frequency(SX127X, 915E6); // remember to put the LoWFrequencyModeOn bit to its correct position
	if (status != HAL_OK) InitErrorCounter++;

	// FSK Specific Settings:
	status = FSK_set_freq_deviation(SX127X, 10000, FSK_BITRATE_1_2_KBPS); // Bitrate.BitrateValue
	if (status != HAL_OK) InitErrorCounter++;

	status = FSK_set_bitrate(SX127X, FSK_BITRATE_1_2_KBPS); // Bitrate.RegisterValue
	if (status != HAL_OK) InitErrorCounter++;

	status = FSK_set_preamb_len(SX127X, 20, Polarity_AA);
	if (status != HAL_OK) InitErrorCounter++;

	status = FSK_set_sync_word(SX127X, 4, 0xF0F01234);
	if (status != HAL_OK) InitErrorCounter++;

	status = FSK_set_packet_format(SX127X, VariableLength);
	if (status != HAL_OK) InitErrorCounter++;

	status = FSK_set_data_processing_mode(SX127X, PacketMode);
	if (status != HAL_OK) InitErrorCounter++;

	status = FSK_set_payload_length(SX127X, 64); // This represents different things in different modes. In this case: Max RX length. More on the .h file.
	if (status != HAL_OK) InitErrorCounter++;

	status = FSK_enable_crc(SX127X);
	if (status != HAL_OK) InitErrorCounter++;

	status = FSK_set_crc_autoclear(SX127X, false);
	if (status != HAL_OK) InitErrorCounter++;

	status = FSK_set_encoding(SX127X, None);
	if (status != HAL_OK) InitErrorCounter++;

	status = FSK_set_data_shaping(SX127X, NoFiltering);
	if (status != HAL_OK) InitErrorCounter++;

	status = FSK_set_pa_ramp_time(SX127X, PaRampTime_40_us);
	if (status != HAL_OK) InitErrorCounter++;

	status = FSK_set_auto_restart_RX(SX127X, AutorestartRX_Off);
	if (status != HAL_OK) InitErrorCounter++;

	// RX config:

	status = FSK_set_rx_bandwidth(SX127X, FSK_BW_31_3_KHZ);
	if (status != HAL_OK) InitErrorCounter++;

	status = FSK_set_afc_bandwidth(SX127X, FSK_BW_31_3_KHZ);
	if (status != HAL_OK) InitErrorCounter++;

	status = FSK_enable_preamble_detector(SX127X, 2, 7);
	if (status != HAL_OK) InitErrorCounter++;

	status = SX127X_set_lna_gain(SX127X, LnaGainG1);
	if (status != HAL_OK) InitErrorCounter++;

	status = SX127X_set_lna_boost(SX127X, false);
	if (status != HAL_OK) InitErrorCounter++;

	status = FSK_set_rssi_smoothing(SX127X, RssiSmoothing_8_Samples);
	if (status != HAL_OK) InitErrorCounter++;

	// TX Config:

	status = SX127X_set_pa_output(SX127X, PA_BOOST_Pin);
	if (status != HAL_OK) InitErrorCounter++;

	status = SX127X_set_tx_power(SX127X, 20);
	if (status != HAL_OK) InitErrorCounter++;

	status = SX127X_set_DIO_mapping(SX127X, 0b00001000, 0b11110001);
	if (status != HAL_OK) InitErrorCounter++;

	status = SX127X_set_op_mode(SX127X, STANDBY);
	if (status != HAL_OK) InitErrorCounter++;

	return status;
}

HAL_StatusTypeDef get_gps_data(ublox_gps_t gps, ublox_pvt_t *pvt, telemetry_packet_t *packet){
	telemetry_packet_values_t packet_values;
	uint8_t attempts = 0;
	printf("\nCollecting GPS data.");

	while(ublox_get(gps, pvt) && attempts < 5){
		printf("\nFailed to collect GPS packet :(. %d out of 5 attempts.", attempts++);
		HAL_Delay(1000);
	}

	if (attempts < 5) { // Check for errors
		packet_values.latitude = (float) pvt->lat / 1e7;
		packet_values.longitude = (float) pvt->lng / 1e7;
		packet_values.altitude = (float) pvt->hMSL / 1e3;
		packet_values.hours = pvt->hour;
		packet_values.minutes = pvt->minute;
		packet_values.seconds = pvt->second;
		packet->values = packet_values;

		printf("\nCollected data:");
		printf("\nCoordiates (Lat,Long): %f,%f",packet_values.latitude, packet_values.longitude);
		printf("\nAltitude: %f", packet_values.altitude);
		printf("\nCurrent Time: ");
		if(((packet_values.hours + TIMEZONE_OFFSET + 24) % 24 ) < 10) printf("0");
		printf("%d:",(packet_values.hours + TIMEZONE_OFFSET + 24) % 24 );
		if(packet_values.minutes < 10) printf("0");
		printf("%d:",packet_values.minutes);
		if(packet_values.seconds < 10) printf("0");
		printf("%d\n",packet_values.seconds);
		return HAL_OK;

	}else{
		printf("\nFailed to collect GPS packet for real :(");
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef telemetry_transmit(SX127X_t *SX127X, telemetry_packet_t packet){
	HAL_StatusTypeDef status = HAL_OK;
	unsigned long time_stamp = HAL_GetTick();

	status = LoRa_Transmit(SX127X, packet.raw, sizeof(packet.raw));
	if(status != HAL_OK) return HAL_ERROR;

	status = HAL_BUSY;

	while(!DIO0){ // wait for the radio to finish transmitting. Yes I'm aware this is polling and shit :)
		if(HAL_GetTick() - time_stamp > TRANSMISSION_TIMEOUT){
			status = HAL_TIMEOUT;
			break;
		}

		HAL_Delay(10);
	}

	if(DIO0 && status == HAL_BUSY) {
		printf("\nPacket has been sent successfully! :)");
		return HAL_OK;
	}else{
		printf("\nRadio took way too long, something went wrong :(");
		printf("\nReseting and reconfiguring the radio for the next time around.");
		SX127X_Reset(SX127X);
		if(LoRa_Config(SX127X) == HAL_OK) printf( "\nAll configurations succeeded :)\n");
		else printf( "\nErrors were found during configuration. Fuck.\n");
		return HAL_ERROR;
	}
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

	//  HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size)



	printf("\n\n ------------------------------------------------------\r\n");
	printf(" ----- Caurin Tracker - Zenith GPS Tracker v.1.0  -----\r\n");
	printf(" ------------------------------------------------------\r\n\n\n");



	printf("Configuring GPS Module.\r\n");
	uart_connection_t conn = { .uart = &huart4 };
	ublox_gps_t gps = { .conn = conn };
	ublox_pvt_t pvt;
	ublox_init(gps);
	HAL_Delay(3000);

	MX_UART4_Init();

	ublox_nav_status_t gps_status;

	ublox_power_mode_setup(gps, BALANCED, 0, 0);

	printf("Configuring Radio (LoRa).\r\n");
	SX127X_t SX127X;
	if(LoRa_Config(&SX127X) == HAL_OK) printf( "\nAll configurations succeeded :)\r\n");
	else printf( "\nErrors were found during configurations. Fuck.\r\n");

	printf("\nWaiting for GPS fix.\r\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		while(1){

			//MX_UART4_Init();
			fixed = ublox_check_fix(gps, &gps_status);

			HAL_Delay(1000);
			printf(".\r\n");
			printf("Time: %d\r\n", gps_status.time);
			printf("Fix type: %d\r\n", gps_status.fixType);
			printf("Flags: %d\r\n", gps_status.flags);
			printf("FixStat: %d\r\n", gps_status.fixStat);
			printf("Flags2: %d\r\n", gps_status.flags2);
			printf("ttff: %d\r\n", gps_status.ttff);
			printf("msss: %d\r\n\n", gps_status.msss);

			if(HAL_GetTick() - fix_search_time_stamp < FIX_SEARCH_TIMEOUT && fixed == true){
				state_machine_status = HAL_OK;
				break;
			} else if(HAL_GetTick() - fix_search_time_stamp > FIX_SEARCH_TIMEOUT){
				printf("Fix search timeout. Sleeping for one minute.\r\n");
				state_machine_status = HAL_TIMEOUT;
				ublox_power_management_request(gps, 60000); // Sleep GPS for 60 seconds
				break;
			}else{
				state_machine_status = HAL_OK;
			}
		}


		if(state_machine_status != HAL_TIMEOUT){ // GPS data was acquired successfully, lets transmit
			printf("\nGPS fixed! Putting the GPS to sleep and transmitting packet six times.\r\n");
			get_gps_data(gps, &pvt, &telemetry_packet);
			ublox_power_management_request(gps, 60000); // Sleep GPS for 60 seconds
			telemetry_packet.values.packet_id = packet_counter ++;
			for(uint8_t i = 0; i < 6; i++){
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(EXTERNAL_GPIO_GPIO_Port, EXTERNAL_GPIO_Pin, GPIO_PIN_SET);
				transmitter_status = telemetry_transmit(&SX127X, telemetry_packet);
				if(transmitter_status == HAL_OK) printf(" Packet number %d sent!\r\n", packet_counter);
				else printf("Error transmitting message.\r\n");
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(EXTERNAL_GPIO_GPIO_Port, EXTERNAL_GPIO_Pin, GPIO_PIN_RESET);
				HAL_Delay(14000);
			}

			HAL_Delay(1000);
			printf("\nBeginning new fix search.\r\n");
			ublox_init(gps);
			HAL_Delay(4000);
			MX_UART4_Init();

		}else{
			HAL_Delay(61000); // Wait for the next search session
			printf("\nBeginning new fix search.\r\n");
			ublox_init(gps);
			HAL_Delay(4000);
			MX_UART4_Init();
		}

		fix_search_time_stamp = HAL_GetTick();

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
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

