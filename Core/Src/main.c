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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "math.h"

#include "fatfs.h"
#include "tinywav.h"

#include "global.h"
#include "imu.h"
#include "lcd.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile float current_heading = 0;
volatile float prev_heading = 0;

volatile float selected_heading = 0;
volatile int heading_set = 0;

char audio_file[] = "gunshot_downscaled.wav";             // main audio file to use
char filter_path_prefix[] = "filters/";      // post-fix format of filter data files
char filter_path_postfix[] = "degrees.bin";  // post-fix format of filter data files
char output_folder[] = "output/";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// To handle Blue Button's press events
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Blue Button pressed
    if (GPIO_Pin == GPIO_PIN_13)
    {
        selected_heading = current_heading;
        heading_set = 1;
    }
}

// To handle timer events
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Interrupt from TIMER3
    if (htim == &htim3)
    {
        // Below code is executed every 10ms
        current_heading += imu_get_gyro_Z(&hi2c1) * DELTA_T;

        // Make sure the heading is between 0 and 360
        current_heading = fmod(current_heading, 360.0f);
        if (current_heading < 0) {
            current_heading += 360;
        }

        // Only send to the app if the heading is changed
        if(abs((int)current_heading - (int)prev_heading) >= 1)
        {
          write_serial(&huart2, "H%03d\n", (int)current_heading);
        }
        prev_heading = current_heading;
    }
}


/**
 * Copy array b starting at index "start_index_b" into target array a starting
 * at index "start_index_a", with length n
 * @ Author: Gia Minh Nguyen
 */
void copy_array_f(float* dest, float* src, int dest_offset, int src_offset, int length) {
	for(int i = 0; i < length; i++) {
		dest[dest_offset + i] = src[src_offset + i];
	}
}

void stereo_to_mono(float* l_channel, float* r_channel, float* out, int length) {
	for(int i = 0; i < length; i++) {
		out[i] = (float) (l_channel[i] + r_channel[i]) / 2;
	}
}

/**
 * Convert the audio file specified at the top of the program to binaural
 * sound of the given direction. Generated binaural sound will be
 * stored in a new file titled "{degrees}_{original file name}.wav"
 * @ Param:
 * 		degrees(int): direction of the sound to emulate
 * @ Author: Gia Minh Nguyen
 */
void binaural_compute(int degrees) {
	// Turn on GREEN LED to indicate SD card is busy => do not unmount
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	// Mount SD card
	static FATFS FatFs;
	FRESULT f_res;
	f_res = f_mount(&FatFs, "", 1); //1=mount now
	if (f_res != FR_OK) {
		write_serial(&huart2, "f_mount error (%i)\r\n", f_res);
	  return;
	}

	// get snapped angle that is multiple of 30 degrees
	int snap_seg = (int) round(((float) degrees) / 30);
	int snap_deg = snap_seg * 30;
  snap_deg = snap_deg % 360;

	// convert degrees to char
	char char_snap_deg[3] = "";
	char char_degrees[3] = "";
	sprintf(char_snap_deg, "%d", snap_deg);
	sprintf(char_degrees, "%d", degrees);

	// build filter file path
	char filter_path[32] = "";
	strcat(filter_path, filter_path_prefix);
	strcat(filter_path, char_snap_deg);
	strcat(filter_path, "_");
	strcat(filter_path, filter_path_postfix);

	// build output file path
	char output_path[64] = "";
	strcat(output_path, output_folder);
	strcat(output_path, char_degrees);
	strcat(output_path, "_");
	strcat(output_path, "degrees_");
	strcat(output_path, audio_file);

	FIL f_file;
	UINT byteCheck;

	write_serial(&huart2, "filter path: %s (%i)\r\n", filter_path);
	write_serial(&huart2, "output path: %s (%i)\r\n", output_path);
	// load filter's LR channels
	f_res = f_open(&f_file, filter_path, FA_READ);
	if(f_res != FR_OK) {
		write_serial(&huart2, "f_open error %s (%i)\r\n", filter_path);
	}

	static float filter_l[256];  // given filters have 256 samples each channel
	static float filter_r[256];
	f_read(&f_file, filter_l, sizeof(float)*256, &byteCheck);
	f_read(&f_file, filter_r, sizeof(float)*256, &byteCheck);
	f_close(&f_file);

	// setup for audio file comprehension and format
	static TinyWav tw; // address to store read audio file
	FIL tw_fil;
	tw.fp = &tw_fil;
	uint32_t sample_rate;

	// load audio file
	f_res = tinywav_open_read(&tw, audio_file, TW_SPLIT);

	// get # of elements of data block per channel
	uint32_t data_size = tw.h.Subchunk2Size / (sizeof(float) * 2);  // 2 means trim audio by half
	sample_rate = tw.h.SampleRate;          // get audio's sample rate
	uint32_t data_left = data_size;
	uint32_t iteration = (uint32_t)ceil((float)data_size/CONVOLVE_BLOCK_SIZE);

	// prepare output file
	static TinyWav tw_out;
	FIL tw_out_fil;
	tw_out.fp = &tw_out_fil;
	tinywav_open_write(&tw_out,
	    NUM_CHANNELS,
	    sample_rate,
	    TW_FLOAT32, // the output samples will be 32-bit floats. TW_INT16 is also supported
	    TW_SPLIT,   // the samples to be written will be provided by an array of pointer
								  // that points to different sub-arrays: [[L,L,L,L], [R,R,R,R]]
	    output_path // the output path
	);

	// Prepare array to cache the tail of current samples to prepend to next convolving block
	static float cache_last_samples[NUM_CHANNELS * (FILTER_SIZE - 1)] = {0};
	float* cache_ptrs[NUM_CHANNELS];

	// For audio read
	// samples are cached in TW_SPLIT format: [[L,L,L,L], [R,R,R,R]]
	static float samples[(NUM_CHANNELS * CONVOLVE_BLOCK_SIZE) + ((FILTER_SIZE - 1) * NUM_CHANNELS)] = {0};
	// create pointers for left and right channel in samples array
	float* sample_ptrs[NUM_CHANNELS];

	// For audio write
	// array to store converted binaural sample
	static float sample_out[(NUM_CHANNELS * CONVOLVE_BLOCK_SIZE) + ((FILTER_SIZE - 1) * NUM_CHANNELS)] = {0};
	float* sample_out_ptrs[NUM_CHANNELS];

	// sample pointers to offset for prepended cache when i > 0
	static float* sample_ptrs_offset[NUM_CHANNELS];
	float* sample_out_ptrs_offset[NUM_CHANNELS];

	// generate pointers to different channel section for both read and write
	for (int j = 0; j < NUM_CHANNELS; ++j) {
		cache_ptrs[j] = cache_last_samples + j * (FILTER_SIZE - 1);
		sample_ptrs[j] = samples + j * (CONVOLVE_BLOCK_SIZE + (FILTER_SIZE - 1));
		sample_out_ptrs[j] = sample_out + j * (CONVOLVE_BLOCK_SIZE + (FILTER_SIZE - 1));
		sample_ptrs_offset[j] = sample_ptrs[j] + (FILTER_SIZE - 1);
		sample_out_ptrs_offset[j] = sample_out_ptrs[j] + (FILTER_SIZE - 1);
	}

	uint32_t sample_length;
	uint32_t input_seq_length;

	for (int i = 0; i < iteration; ++i) {
		input_seq_length = data_left < CONVOLVE_BLOCK_SIZE ? data_left : CONVOLVE_BLOCK_SIZE;
		if(i == 0) {
			sample_length = input_seq_length;
		} else {
			sample_length = input_seq_length + FILTER_SIZE - 1;
		}

	  if(i == 0) {  // 1st iteration (edge case) as there are no data to prepend
			tinywav_read_f(&tw, sample_ptrs, input_seq_length);

			// Cache left channel
			copy_array_f(cache_ptrs[0], sample_ptrs[0], 0, input_seq_length - FILTER_SIZE + 1, FILTER_SIZE - 1);

			// Convolution: A:= filter, B:= input seq
			// Convolution for Left channel
			arm_conv_partial_f32(filter_l, FILTER_SIZE, sample_ptrs[0], input_seq_length,
													 sample_out_ptrs[0], 0, input_seq_length);

			// Transfer right samples to left buffer
			copy_array_f(sample_ptrs[0], sample_ptrs[1], 0, 0, input_seq_length);
			// cache right channel
			copy_array_f(cache_ptrs[1], sample_ptrs[0], 0, input_seq_length - FILTER_SIZE + 1, FILTER_SIZE - 1);

			// Convolution for Right channel
			arm_conv_partial_f32(filter_r, FILTER_SIZE, sample_ptrs[0], input_seq_length,
													 sample_out_ptrs[1], 0, input_seq_length);

			tinywav_write_f(&tw_out, sample_out_ptrs, input_seq_length);

		} else {  // i > 0 => prepend data first and then convolve so convolution is continuous
			memset(samples, 0, sizeof(samples));
			memset(sample_out, 0, sizeof(sample_out));

			tinywav_read_f(&tw, sample_ptrs_offset, input_seq_length);

			// load left cache
			copy_array_f(sample_ptrs[0], cache_ptrs[0], 0, 0, FILTER_SIZE - 1);
			// Cache left channel
			copy_array_f(cache_ptrs[0], sample_ptrs_offset[0], 0, input_seq_length - FILTER_SIZE + 1, FILTER_SIZE - 1);
			// Convolution: A:= filter, B:= input seq
			// Convolution for Left channel
			arm_conv_partial_f32(filter_l, FILTER_SIZE, sample_ptrs[0], sample_length,
													 sample_out_ptrs[0], FILTER_SIZE - 1, input_seq_length);

			// Load right cache
			copy_array_f(sample_ptrs[0], cache_ptrs[1], 0, 0, FILTER_SIZE - 1);
			// Transfer right samples to left buffer
			copy_array_f(sample_ptrs_offset[0], sample_ptrs_offset[1], 0, 0, input_seq_length);
			// Cache right channel
			copy_array_f(cache_ptrs[1], sample_ptrs_offset[0], 0, input_seq_length - FILTER_SIZE + 1, FILTER_SIZE - 1);
			// Convolution for Right channel
			arm_conv_partial_f32(filter_r, FILTER_SIZE, sample_ptrs[0], sample_length,
													 sample_out_ptrs[1], FILTER_SIZE - 1, input_seq_length);

			tinywav_write_f(&tw_out, sample_out_ptrs_offset, input_seq_length);
		}

		data_left -= input_seq_length;

		// print to console every 10 rounds or end of loop
		if(i % 10 == 0 || i == iteration - 1) {
			write_serial(&huart2, "done convolution block: %d / %lu \r\n", i, iteration - 1);
		}
	}

	tinywav_close_write(&tw_out);
	tinywav_close_read(&tw);

	// Unmount SD
	f_mount(NULL, "", 0);

	lcd_set_cursor(0, 9);
	lcd_send_string("XXX");

    // Turn off GREEN LED to indicate SD card is NOT busy
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
    lcd_init();
    imu_init(&hi2c1);

    // Check connection with the MPU
    uint8_t who_am_i;
    HAL_I2C_Mem_Read(&hi2c1, IMU_ADDRESS, 0x75, 1, &who_am_i, 1, 1000);
    if (who_am_i == 0x68)
    {
        write_serial(&huart2, "Connected to MPU6050!\n");
    }
    else
    {
        write_serial(&huart2, "No connection!\n");
    }

    // Check connection with the LCD
    HAL_StatusTypeDef res = HAL_I2C_IsDeviceReady(&hi2c1, LCD_ADDRESS, 1, 100);
    if (res == HAL_OK)
    {
        write_serial(&huart2, "Connected to LCD!\n");
    }
    else
    {
        write_serial(&huart2, "No connection!\n");
    }

    lcd_send_string("Calibrating...");
    imu_calibarate(&hi2c1);
    lcd_clear();

    write_serial(&huart2, "H000\n");
    // No heading selected
    lcd_send_string("Heading: XXX");

    HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        if(heading_set) {
            heading_set = 0;
            
            lcd_set_cursor(0, 9);
            lcd_send_string("%03d", (int)selected_heading);

            binaural_compute((int)selected_heading);
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64000 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 13;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Green_LED_Pin */
  GPIO_InitStruct.Pin = Green_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Green_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
