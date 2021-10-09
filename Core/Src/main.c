/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "i2c.h"
#include "lwip.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "lwip/udp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BUFFER_SIZE 50
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#else

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART2 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void udp_rx_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, ip_addr_t *addr, u16_t port);

void setup_protocols(int protocol, uint8_t* uart_buff, uint8_t* i2c_buff, uint8_t* spi_buff, int size);

void send_data(int protocol, uint8_t* buffer);

void transmit_payload(int protocol, uint8_t* payload, size_t len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t timer_elapsed = 0;
uint8_t i2c_full = 0;
uint8_t spi_full = 0;
uint8_t uart_full = 0;

uint8_t buffers[3];

const int server_port = 10;
extern struct netif gnetif; // my addr
struct udp_pcb* server_sock = NULL;

ip_addr_t sender_addr;
u16_t sender_port = 0;
int socket_collected = 0;
char socket_data[255] = {0};
uint8_t last_buffer[BUFFER_SIZE] = {0};
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
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI4_Init();
  MX_SPI1_Init();
  MX_LWIP_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);

	uint8_t uart_buff[BUFFER_SIZE] = {0};
	uint8_t i2c_buff[BUFFER_SIZE] = {0};
	uint8_t spi_buff[BUFFER_SIZE] = {0};

	server_sock = udp_new();
	udp_bind(server_sock, &gnetif.ip_addr, server_port);
	udp_recv(server_sock, (udp_recv_fn) udp_rx_callback, NULL);

	uint8_t* payload = NULL;
	int protocol = -1;
	size_t len = 0;

	//  test code
	//  size_t len = 8;
	//	uint8_t* payload = malloc(sizeof(uint8_t) * (len + 1));
	//	memset(payload, '\0', (len + 1));
	//	memcpy(payload, "Payload1", len);
	//	int protocol = 1;
	//	setup_protocols(protocol, uart_buff, i2c_buff, spi_buff, len);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	const int timer_half_period = htim4.Init.Period / 2;
  while (1) {
	  // set ARP table for MAC addresses
	  ethernetif_input(&gnetif);
	  // printf("ip: %s\n\r", ip4addr_ntoa(&gnetif.ip_addr));

	  if(socket_collected) {
		  // fill buffers to allow drop
		  transmit_payload(protocol, payload, len);

		  printf("sender ip: %s:%d, data: %s\n\r", ip4addr_ntoa(&sender_addr), sender_port, socket_data);

		  char* separator_idx = strstr(socket_data, ",");
		  socket_data[separator_idx - socket_data] = '\0';
		  protocol = atoi(socket_data);

		  const char* data = &socket_data[(separator_idx - socket_data) + 1];
		  const int prev_len = len;
		  len = strlen(data);
		  const int payload_size = sizeof(uint8_t) * (len + 1);
		  if (payload == NULL) {
			  payload = malloc(payload_size);
		  } else if (len != prev_len) {
			  payload = realloc(payload, payload_size);
		  }
		  memset(payload, '\0', len + 1);
		  memcpy(payload, data, len);
		  setup_protocols(protocol, uart_buff, i2c_buff, spi_buff, len);
		  socket_collected = 0;
	  }

	  if (htim4.Instance -> CNT % timer_half_period == 0 && payload != NULL) {
		  transmit_payload(protocol, payload, len);
	  }

	  if (uart_full || i2c_full || spi_full) {
		  setup_protocols(protocol, uart_buff, i2c_buff, spi_buff, len);
		  uart_full = 0;
		  i2c_full = 0;
		  spi_full = 0;
	  }
	  if (timer_elapsed) {
		  // printf("timer %d\n\r", counter);
		  timer_elapsed = 0;
		  send_data(protocol, last_buffer);
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_UART7|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void transmit_payload(int protocol, uint8_t* payload, size_t len){
	if (protocol > 0) {
	  if (protocol == 1) {
		  HAL_UART_Transmit(&huart7, payload, len, 100);
	  } else if (protocol == 2) {
		  HAL_I2C_Master_Transmit(&hi2c1, 4, payload, len, 100);
	  } else if (protocol == 3) {
		  HAL_SPI_Transmit(&hspi4, payload, len, 100);
	  }
	}
}


void resotre_buffer(uint8_t* buffer, int size) {
	memset(last_buffer, '\0', BUFFER_SIZE);
	memcpy(last_buffer, buffer, size);
	memset(buffer, 0, BUFFER_SIZE);
}

void setup_protocols(
	int protocol, uint8_t* uart_buff, uint8_t* i2c_buff, uint8_t* spi_buff, int size
) {
	if (protocol == 1) {
		if (strlen((char*) uart_buff)) {
			printf("Uart: [%s]\n\r", uart_buff);
		}
		resotre_buffer(uart_buff, size);
		HAL_UART_Receive_IT(&huart5, uart_buff, size);
		// UART_Start_Receive_IT(&huart2, buff, 10);
	} else if (protocol == 2) {
		if (strlen((char*) i2c_buff)) {
			printf("I2C: [%s]\n\r", i2c_buff);
		}
		resotre_buffer(i2c_buff, size);
		HAL_I2C_Slave_Receive_IT(&hi2c2, i2c_buff, size);
	} else if (protocol == 3) {
		if (strlen((char*) spi_buff)) {
			printf("SPI: [%s]\n\r", spi_buff);
		}
		resotre_buffer(spi_buff, size);
		HAL_SPI_Receive_IT(&hspi1, spi_buff, size);
	}
}


void send_data(int protocol, uint8_t* buffer) {
	if (protocol != 1 && protocol != 2 && protocol != 3) {
		// printf("No protocol specified\n\r");
		return;
	}
	size_t size = strlen((char*) buffer);
	if (!size) {
		// printf("Nothing to send\n\r");
		return;
	}
	err_t is_connected = udp_connect(server_sock, &sender_addr, sender_port);
	if (is_connected == ERR_OK) {
	  struct pbuf* udp_buffer = pbuf_alloc(PBUF_TRANSPORT, size + 2, PBUF_RAM);
	  if (udp_buffer != NULL) {
		  memset(udp_buffer -> payload, '\0', size);
		  memset(udp_buffer -> payload, protocol + '0', 1);
		  memcpy(udp_buffer -> payload + 1, ",", 1);
		  memcpy(udp_buffer -> payload + 2, buffer, size);
		  if (udp_send(server_sock, udp_buffer)) {
			  printf("Send failed\n\r");
		  }
		  pbuf_free(udp_buffer);
	  }
	  udp_disconnect(server_sock);
	}
}

/**
  * @brief This function is called when an UDP datagram has been received on the port UDP_PORT.
  * @param arg user supplied argument (udp_pcb.recv_arg)
  * @param pcb the udp_pcb which received data
  * @param p the packet buffer that was received
  * @param addr the remote IP address from which the packet was received
  * @param port the remote port from which the packet was received
  * @retval None
  */
void udp_rx_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, ip_addr_t *addr, u16_t port) {
	sender_addr = *addr;
	sender_port = port;
	memset(socket_data, 0, sizeof(socket_data));
	memcpy(socket_data, p -> payload, p -> len);
	socket_collected = 1;
	pbuf_free(p);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim -> Instance == TIM4) {
		timer_elapsed = 1;
	} else {
		UNUSED(htim);
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi -> Instance == SPI1) {
		spi_full = 1;
	} else {
		UNUSED(hspi);
	}
}

/**
 * This function allows to reset following callbacks:
       (+) MasterTxCpltCallback : callback for Master transmission end of transfer.
       (+) MasterRxCpltCallback : callback for Master reception end of transfer.
       (+) SlaveTxCpltCallback  : callback for Slave transmission end of transfer.
       (+) SlaveRxCpltCallback  : callback for Slave reception end of transfer.
       (+) ListenCpltCallback   : callback for end of listen mode.
       (+) MemTxCpltCallback    : callback for Memory transmission end of transfer.
       (+) MemRxCpltCallback    : callback for Memory reception end of transfer.
       (+) ErrorCallback        : callback for error detection.
       (+) AbortCpltCallback    : callback for abort completion process.
       (+) MspInitCallback      : callback for Msp Init.
       (+) MspDeInitCallback    : callback for Msp DeInit.
 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c -> Instance == I2C2) {
		i2c_full = 1;
	} else {
		UNUSED(hi2c);
	}
}


/*
(+) TxHalfCpltCallback        : Tx Half Complete Callback.
(+) TxCpltCallback            : Tx Complete Callback.
(+) RxHalfCpltCallback        : Rx Half Complete Callback.
(+) RxCpltCallback            : Rx Complete Callback.
(+) ErrorCallback             : Error Callback.
(+) AbortCpltCallback         : Abort Complete Callback.
(+) AbortTransmitCpltCallback : Abort Transmit Complete Callback.
(+) AbortReceiveCpltCallback  : Abort Receive Complete Callback.
(+) WakeupCallback            : Wakeup Callback.
(+) RxFifoFullCallback        : Rx Fifo Full Callback.
(+) TxFifoEmptyCallback       : Tx Fifo Empty Callback.
(+) MspInitCallback           : UART MspInit.
(+) MspDeInitCallback         : UART MspDeInit.
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart -> Instance == UART5) {
		uart_full = 1;
	} else {
		UNUSED(huart);
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
