/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N_IMU_CHARS 78
#define N_CAMERAS_CHARS 18
#define N_MOCAP_CHARS 18
#define N_CHARS (N_IMU_CHARS + N_CAMERAS_CHARS + N_MOCAP_CHARS)

#define N_BYTES 16

//#define ALIGN_SUBS_INTERVAL 2560000 // = 1/6 * htim2.Init.Period

#define INPUT_PC_DATA_LENGTH 5
#define ALIGN_FRAMES_CMD 34
#define START_TRIGGER_CMD 56
#define STOP_TRIGGER_CMD 57

#define THRES 126
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c2_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart4_rx;

/* USER CODE BEGIN PV */
uint8_t str[N_CHARS];

uint16_t count = 0;
uint8_t flag_read_imu_values = 0;
uint8_t flag_transmit_to_lidar = 0;

uint8_t flag_cameras_ts_ready = 0;
uint8_t flag_mocap_ts_ready = 0;
uint8_t buf_flag_cameras_ts_ready = 0;
uint8_t buf_flag_mocap_ts_ready = 0;

uint8_t flag_alignment_received = 0;

uint8_t dat[N_BYTES];
uint8_t dat_[N_BYTES];
uint8_t dat_buf[N_BYTES];


uint8_t soft_rtc_h = 0;
uint8_t soft_rtc_m = 0;
uint8_t soft_rtc_s = 0;
uint32_t soft_rtc_subs = 0;

uint8_t soft_rtc_imu_h = 0;
uint8_t soft_rtc_imu_m = 0;
uint8_t soft_rtc_imu_s = 0;
uint32_t soft_rtc_imu_subs = 0;

uint8_t soft_rtc_cameras_h = 0;
uint8_t soft_rtc_cameras_m = 0;
uint8_t soft_rtc_cameras_s = 0;
uint32_t soft_rtc_cameras_subs = 0;

uint8_t soft_rtc_mocap_h = 0;
uint8_t soft_rtc_mocap_m = 0;
uint8_t soft_rtc_mocap_s = 0;
uint32_t soft_rtc_mocap_subs = 0;

int32_t alignment_subs_signed = 0;
uint32_t alignment_subs_low = 0;
uint32_t alignment_subs_high = 0;
uint32_t alignment_subs_received = 0;
uint32_t tim2_counter_per = 0;

uint8_t input_buf[INPUT_PC_DATA_LENGTH];
uint8_t flag_data_received_from_pc;

typedef struct {
	uint8_t cmd;
	uint32_t data;
} received_tuple;

uint8_t flag_pause_triggering = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*int __io_putchar(int ch) {
  uint8_t c[1];
  c[0] = ch & 0x00FF;
  HAL_UART_Transmit(&huart4, &*c, 1, 10);
  return ch;
}
int _write(int file,char *ptr, int len) {
  int DataIdx;
  for(DataIdx= 0; DataIdx< len; DataIdx++) {
    __io_putchar(*ptr++);
  }
  return len;
}*/

//uint32_t read_TIM5() {
//  return TIM5->CNT;
//}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	/*if(GPIO_Pin == GPIO_PIN_9)
  {
  	flag_read_imu_values = 1;
  	soft_rtc_imu_subs = read_TIM5();
		soft_rtc_imu_s = soft_rtc_s;
		soft_rtc_imu_m = soft_rtc_m;
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
  }*/
//}

void setup_mpu(void) {
	uint8_t dat[] = {5, 16, 1, 1, 8, 8};
	uint8_t adds[] = {107, 55, 26, 56, 27, 28};
	uint8_t n_of_bytes = sizeof(dat) / sizeof(dat[0]);
	for (uint8_t idx=0; idx<n_of_bytes; idx++) {
		HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, adds[idx], 1, &dat[idx], 1, 1000);
	}
}

void make_message(void) {
	sprintf(str,
		"i0"																				//2
		"%02x %02x %04x %04x "											//16
		"%04x %04x %04x %04x %04x %04x %04x "			 	//35
		"%04x %04x %04x %04x "											//20
		"%04x"																			//4
		"\n", 																			//1
																								//=78
		(uint8_t)soft_rtc_imu_m,
		(uint8_t)soft_rtc_imu_s,
		(uint16_t)(soft_rtc_imu_subs>>16),
		(uint16_t)(soft_rtc_imu_subs),

		(uint16_t)(dat_buf[0]<<8 | dat_buf[1]),
		(uint16_t)(dat_buf[2]<<8 | dat_buf[3]),
		(uint16_t)(dat_buf[4]<<8 | dat_buf[5]),
		(uint16_t)(dat_buf[6]<<8 | dat_buf[7]),
		(uint16_t)(dat_buf[8]<<8 | dat_buf[9]),
		(uint16_t)(dat_buf[10]<<8 | dat_buf[11]),
		(uint16_t)(dat_buf[12]<<8 | dat_buf[13]),

		(uint16_t)(dat_[1]<<8 | dat_[0]),
		(uint16_t)(dat_[3]<<8 | dat_[2]),
		(uint16_t)(dat_[5]<<8 | dat_[4]),
		(uint16_t)(dat_[7]<<8 | dat_[6]),

		count
	);
	if (buf_flag_cameras_ts_ready == 1) {
		sprintf(str + N_IMU_CHARS,
			"c0"																				//2
			"%02x %02x %04x %04x"	 										  //15
			"\n", 																			//1
																									//=18
			(uint8_t)soft_rtc_cameras_m,
			(uint8_t)soft_rtc_cameras_s,
			(uint16_t)(soft_rtc_cameras_subs>>16),
			(uint16_t)(soft_rtc_cameras_subs)
		);
	}
  if (buf_flag_mocap_ts_ready == 1) {
		sprintf(str + N_IMU_CHARS + buf_flag_cameras_ts_ready * N_CAMERAS_CHARS,
			"m0"																				//2
			"%02x %02x %04x %04x"											  //15
			"\n", 																			//1
																									//=18
			(uint8_t)soft_rtc_mocap_m,
			(uint8_t)soft_rtc_mocap_s,
			(uint16_t)(soft_rtc_mocap_subs>>16),
			(uint16_t)(soft_rtc_mocap_subs)
		);
	}
}

void delay(uint16_t n) {
	for (uint16_t i=0; i<n; i++) {
		;
	}
}

void cp() {
	for (uint8_t i=0; i<N_BYTES; i++) {
		dat_buf[i] = dat[i];
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == htim2.Instance) {
		soft_rtc_s ++;
		if (soft_rtc_s == 60) {
			soft_rtc_s = 0;
			soft_rtc_m ++;
			if (soft_rtc_m == 60) {
				soft_rtc_m = 0;
				soft_rtc_h ++;
				if (soft_rtc_h == 24) {
					soft_rtc_h = 0;
				}
			}
		}
	}
}
	/*else if (htim->Instance == htim1.Instance)
	{
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)==GPIO_PIN_SET) { // if timer output is HIGH

			soft_rtc_lidar_subs = read_TIM5();
			soft_rtc_lidar_s = soft_rtc_s;
			soft_rtc_lidar_m = soft_rtc_m;
			//HAL_RTC_GetTime(&hrtc, &sTime_lidar, RTC_FORMAT_BIN);
			//HAL_RTC_GetDate(&hrtc, &sDate_lidar, RTC_FORMAT_BIN);
			flag_lidar_ts_ready = 1;
			flag_transmit_to_lidar = 1;
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		}
	}
	else if (htim->Instance == htim2.Instance)
	{
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)==GPIO_PIN_RESET) { // if timer output is HIGH // is not needed

			soft_rtc_cameras_subs = read_TIM5();
			soft_rtc_cameras_s = soft_rtc_s;
			soft_rtc_cameras_m = soft_rtc_m;

			//HAL_RTC_GetTime(&hrtc, &sTime_cam, RTC_FORMAT_BIN);
			//HAL_RTC_GetDate(&hrtc, &sDate_cam, RTC_FORMAT_BIN);
			flag_cameras_ts_ready = 1;
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		}
	}*/
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART4) {
    	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
    	//flag_alignment_received = 1;
    	flag_data_received_from_pc = 1;
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == htim2.Instance) {
  	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
  		soft_rtc_imu_s = soft_rtc_s;
			soft_rtc_imu_m = soft_rtc_m;
			soft_rtc_imu_subs = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
  		flag_read_imu_values = 1;
			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
  	}
  	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			soft_rtc_cameras_s = soft_rtc_s;
			soft_rtc_cameras_m = soft_rtc_m;
			soft_rtc_cameras_subs = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			flag_cameras_ts_ready = 1;
			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
  	}
  	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			soft_rtc_mocap_s = soft_rtc_s;
			soft_rtc_mocap_m = soft_rtc_m;
			soft_rtc_mocap_subs = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			flag_mocap_ts_ready = 1;
			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
  	}
  }
}

void receive_from_pc(void) {
	HAL_UART_Receive_DMA(&huart4, input_buf, INPUT_PC_DATA_LENGTH);
}

received_tuple parse_data(void) {
	received_tuple received;
	received.cmd = input_buf[0];
	received.data = (uint32_t)(input_buf[1] | input_buf[2] << 8 | input_buf[3] << 16 | input_buf[4] << 24);
	return received;
}

/*void align_timer(void) {
	alignment_subs_signed -= (int32_t)alignment_subs_received; // must not overfill uint32_t
	while (alignment_subs_signed < (int32_t)alignment_subs_low) {
		alignment_subs_signed += ALIGN_SUBS_INTERVAL;
	}
	TIM2->CCR1 = (uint32_t)alignment_subs_signed;
}*/
/*void align_timer(void) {
	alignment_subs_signed -= (int32_t)alignment_subs_received; // must not overfill uint32_t
	while (alignment_subs_signed < 0) {
		alignment_subs_signed += (htim2.Init.Period + 1);
	}
	TIM2->CCR1 = (uint32_t)alignment_subs_signed;
}*/
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
  HAL_Delay(1000);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, RESET);
  //HAL_Delay(1000);
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, SET);
  //HAL_Delay(1000);
  setup_mpu();
  //HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	//HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_1);
	//delay(10);
	//HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
	//delay(10);
	//delay(10);
	//HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
  //HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);
  //HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);


  //Update_event = TIM_CLK/((PSC + 1)*(ARR + 1)*(RCR + 1))
  //TIM_CLK = timer clock input
  //PSC = 16-bit prescaler register
  //ARR = 16/32-bit Autoreload register
  //RCR = 16-bit repetition counter

  //76.8 / ((0 + 1) * (1 + 1)) / 2 = 19.2 MHz TIM3 IMU
  //76.8 * 1000000 / ((7680 - 1 + 1) * (1000 - 1 + 1)) / 2 = 5 Hz TIM2 CAMERAS
	//76.8 * 1000000 / ((7680 - 1 + 1) * (5000 - 1 + 1)) / 2 = 1 Hz TIM1 LIDAR
	//76.8 * 1000000 / ((3 - 1 + 1) * (25600000 - 1 + 1)) = 1 Hz TIM5

/*----------|  |------------
|                          |
|                          |
|                          |
|                          |
|::                      ::|
|::                      ::|
|::                      ::|
|::                      ::|
|::                      ::|
|::                      ::|
|::                      ::|
|::      [D13 ORNG]      ::|
|:: [D12 GRN]  [D14 RED] ::|
|::      [D15 BLUE]      ::|
|::                      ::|
|::                      ::|
|::             [D5 USB] ::|
--------------------------*/
	//receive_alignment();
	receive_from_pc();
	dat_[0] = 0b10000010;
	HAL_I2C_Mem_Write(&hi2c2, 0b0011100<<1, 0x20, 1, dat_, 1, 1000);
	dat_[0] = 0b00001100;
	HAL_I2C_Mem_Write(&hi2c2, 0b0011100<<1, 0x23, 1, dat_, 1, 1000);
	dat_[0] = 0;
	HAL_I2C_Mem_Write(&hi2c2, 0b0011100<<1, 0x22, 1, dat_, 1, 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)); // forward timer output signal to led pin
  	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)); // forward timer output signal to led pin
  	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)); // forward timer output signal to led pin
  	if (flag_read_imu_values == 1) {
			//HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
			flag_read_imu_values = 0;
      buf_flag_cameras_ts_ready = flag_cameras_ts_ready;
      buf_flag_mocap_ts_ready = flag_mocap_ts_ready;
      if (buf_flag_cameras_ts_ready == 1) {
              flag_cameras_ts_ready = 0;
      }
      if (buf_flag_mocap_ts_ready == 1) {
              flag_mocap_ts_ready = 0;
      }
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
			count++;
			cp();
			HAL_I2C_Mem_Read_DMA(&hi2c1, 0x68<<1, 59, 1, dat, 14);
			HAL_I2C_Mem_Read_DMA(&hi2c2, 0b0011100<<1, 0x28, 1, dat_, 8);
			if (abs((int8_t)dat[0])>THRES || abs((int8_t)dat[2])>THRES || abs((int8_t)dat[4])>THRES ||
					abs((int8_t)dat[8])>THRES || abs((int8_t)dat[10])>THRES || abs((int8_t)dat[12])>THRES) {
				//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
			}
			else {
				//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
			}

			uint8_t mes_length = N_IMU_CHARS + buf_flag_cameras_ts_ready * N_CAMERAS_CHARS + buf_flag_mocap_ts_ready * N_MOCAP_CHARS;
			make_message();
			HAL_UART_Transmit_DMA(&huart4, str, mes_length);//, 1000);	//HAL_UART_Transmit_DMA(&huart4, str, N_CHARS);

			//if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9) != RESET) {__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);}
			//HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

		/*if (flag_data_received_from_pc == 1) {
				received_tuple received = parse_data();
				if (received.cmd == START_TRIGGER_CMD) {
					HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
				}
				else if (received.cmd == ALIGN_FRAMES_CMD) {
					alignment_subs_received = received.data;
					align_timer();
					HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
				}
				else if (received.cmd == STOP_TRIGGER_CMD) {
					HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
				}

				flag_data_received_from_pc = 0;
				receive_from_pc();
			}
		  }*/
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
			if(count & 1024) { HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);}
			if (flag_pause_triggering == 1) {
        if (count == 10000) {
          HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
          HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
        }
        if (count == 11000) {
          HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
          HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
          flag_pause_triggering = 0;
        }
			}


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
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
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_4);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 25600000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ETRF;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 5-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ETRF;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 2000000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD12 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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
