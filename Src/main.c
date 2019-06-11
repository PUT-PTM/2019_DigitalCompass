/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>

#include "font.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
    typedef struct vectorint
    {
      int16_t x, y, z;
    } vectori;
    typedef struct vectorfloat
    {
      float x, y, z;
    } vectorf;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LSM303D_ADDR				0x3a
#define LSM303D_TEMP_OUT			0x05
#define LSM303D_STATUS_M			0x07
#define LSM303D_OUT_X_M				0x08
#define LSM303D_OUT_Y_M				0x0a
#define LSM303D_OUT_Z_M				0x0c
#define LSM303D_WHO_AM_I			0x0f
#define LSM303D_CTRL0				0x1f
#define LSM303D_CTRL1				0x20
#define LSM303D_CTRL2				0x21
#define LSM303D_CTRL3				0x22
#define LSM303D_CTRL4				0x23
#define LSM303D_CTRL5				0x24
#define LSM303D_CTRL6				0x25
#define LSM303D_CTRL7				0x26
#define LSM303D_STATUS				0x27
#define LSM303D_OUT_X_A				0x28
#define LSM303D_OUT_Y_A				0x2a
#define LSM303D_OUT_Z_A				0x2c

#define LCD_DC			GPIO_PIN_1
#define LCD_CE			GPIO_PIN_2
#define LCD_RST			GPIO_PIN_3

#define PCD8544_FUNCTION_SET		0x20
#define PCD8544_DISP_CONTROL		0x08
#define PCD8544_DISP_NORMAL			0x0c
#define PCD8544_SET_Y				0x40
#define PCD8544_SET_X				0x80
#define PCD8544_H_TC				0x04
#define PCD8544_H_BIAS				0x10
#define PCD8544_H_VOP				0x80

#define LCD_BUFFER_SIZE			(84 * 48 / 8)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
int16_t a_x;
int16_t a_y;
int16_t a_z;
int16_t m_x;
int16_t m_y;
int16_t m_z;
float heading;
float heading2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
uint8_t lsm_read_reg(uint8_t reg)
{
	uint8_t value = 0;

	HAL_I2C_Mem_Read(&hi2c1, LSM303D_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);
	return value;
}

void lsm_write_reg(uint8_t reg, uint8_t value)
{
	HAL_I2C_Mem_Write(&hi2c1, LSM303D_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);
}

int16_t lsm_read_value(uint8_t reg)
{
	int16_t value = 0;

	HAL_I2C_Mem_Read(&hi2c1, LSM303D_ADDR, reg | 0x80, 1, (uint8_t*)&value, sizeof(value), HAL_MAX_DELAY);

	return value;
}

// LCD
static uint8_t lcd_buffer[LCD_BUFFER_SIZE];

void lcd_cmd(uint8_t cmd)
{
	HAL_GPIO_WritePin(GPIOC, LCD_CE|LCD_DC, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &cmd, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, LCD_CE|LCD_DC, GPIO_PIN_SET);
}

void lcd_setup(void)
{
	HAL_GPIO_WritePin(GPIOC, LCD_RST, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, LCD_RST, GPIO_PIN_SET);

	lcd_cmd(PCD8544_FUNCTION_SET | 1);
	lcd_cmd(PCD8544_H_BIAS | 4);
	lcd_cmd(PCD8544_H_VOP | 0x2f);
	lcd_cmd(PCD8544_FUNCTION_SET);
	lcd_cmd(PCD8544_DISP_NORMAL);
}

void lcd_clear(void)
{
	memset(lcd_buffer, 0, LCD_BUFFER_SIZE);
}

void lcd_draw_text(int row, int col, const char* text)
{
	int i;
	uint8_t* pbuf = &lcd_buffer[row * 84 + col];
	while ((*text) && (pbuf < &lcd_buffer[LCD_BUFFER_SIZE - 6])) {
		int ch = *text++;
		const uint8_t* font = &font_ASCII[ch - ' '][0];
		for (i = 0; i < 5; i++) {
			*pbuf++ = *font++;
		}
		*pbuf++ = 0;
	}
}

void lcd_copy(void)
{
	HAL_GPIO_WritePin(GPIOC, LCD_DC, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, LCD_CE, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, lcd_buffer, LCD_BUFFER_SIZE, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, LCD_CE, GPIO_PIN_SET);
}

// Data gathering - LSM303D
int16_t convert_data(int16_t value) {
	if(((value & ( 1 << 16 )) >> 16) == 1) {
		return ~value + 1;
	} else {
		return value;
	}
}

void vector_crossi(const vectori *a, const vectori *b, vectorf *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

void vector_crossf(const vectori *a, const vectorf *b, vectorf *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

float vector_doti(const vectorf *a, const vectori *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

float vector_dotf(const vectorf *a, const vectorf *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

void vector_normalize(vectorf *a)
{
  float mag = sqrt(vector_dotf(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

float head(int16_t a1, int16_t a2, int16_t a3, int16_t m1, int16_t m2, int16_t m3)
{
	vectori from = {1, 0, 0};
	vectori temp_m = {m1, m2, m3};
	vectori a = {a1, a2, a3};

    // compute E and N
    vectorf E;
    vectorf N;
    vector_crossi(&temp_m, &a, &E); //i, i, f
    vector_normalize(&E); //f
    vector_crossf(&a, &E, &N); //i, f, f
    vector_normalize(&N); //f

    // compute heading
    float head = atan2(vector_doti(&E, &from), vector_doti(&N, &from)) * 180 / 3.14159;
    if (head < 0) head += 360;
    return head;
}
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
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  TIM3->CCR1 = 1000;

	uint8_t who_am_i = lsm_read_reg(0x0F);

	 if (who_am_i == 0x49) {
		 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	 } else {
		 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	 }

	 lsm_write_reg(LSM303D_CTRL1, 0x57);
	 lsm_write_reg(LSM303D_CTRL5, 0x64);
	 lsm_write_reg(LSM303D_CTRL7, 0x00);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  for(int i=0;i<1000000;i++) {}

	  a_x = lsm_read_value(LSM303D_OUT_X_A);
	  a_y = lsm_read_value(LSM303D_OUT_Y_A);
	  a_z = lsm_read_value(LSM303D_OUT_Z_A);
	  m_x = lsm_read_value(LSM303D_OUT_X_M);
	  m_y = lsm_read_value(LSM303D_OUT_Y_M);
	  m_z = lsm_read_value(LSM303D_OUT_Z_M);

	  int16_t xGaussData = m_x * 0.160;
	  int16_t yGaussData = m_y * 0.160;

		  heading = atan2(yGaussData, xGaussData) * (180 / 3.14159);

		  for(int i=0;i<10000000;i++) ;

		  heading2 = head(a_x, a_y, a_z, m_x, m_y, m_z);

		  char text[10];
		  int more = (int)heading2;
		  int less = (int)(heading2 * 100) % 100;
		  sprintf(text, "Kat: %03d\.%02d", more, less);

		  char text2[12];
		  char *text3;

		  if (heading2 >= 337.5 || heading2 < 22.5) {
			  text3 = " N";
		  } else if (heading2 >= 292.5 && heading2 < 337.5) {
			  text3 = "NW";
		  } else if (heading2 >= 247.5 && heading2 < 292.5) {
			  text3 = " W";
		  } else if (heading2 >= 202.5 && heading2 < 247.5) {
			  text3 = "SW";
		  } else if (heading2 >= 157.5 && heading2 < 202.5) {
			  text3 = " S";
		  } else if (heading2 >= 112.5 && heading2 < 157.5) {
			  text3 = "SE";
		  } else if (heading2 >= 67.5 && heading2 < 112.5) {
			  text3 = " E";
		  } else if (heading2 >= 22.5 && heading2 < 67.5) {
			  text3 = "NE";
		  }

		  sprintf(text2, "Kierunek: %s", text3);

		  lcd_setup();
		  lcd_clear();

		  lcd_draw_text(0, 0 * 10, text);
		  lcd_draw_text(1, 0 * 6, text2);
		  lcd_copy();
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 299;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
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
  sConfigOC.Pulse = 50;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
