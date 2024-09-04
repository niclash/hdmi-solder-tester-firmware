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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>

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
static GPIO_TypeDef *pinToPort[] = {
    NULL, PIN1_GPIO_Port, PIN2_GPIO_Port, PIN3_GPIO_Port, PIN4_GPIO_Port, PIN5_GPIO_Port, PIN6_GPIO_Port,
    PIN7_GPIO_Port, PIN8_GPIO_Port, PIN9_GPIO_Port, PIN10_GPIO_Port, PIN11_GPIO_Port, PIN12_GPIO_Port, PIN13_GPIO_Port,
    PIN14_GPIO_Port, PIN15_GPIO_Port, PIN16_GPIO_Port, PIN17_GPIO_Port, PIN18_GPIO_Port, PIN19_GPIO_Port};

static int pinToPin[] = {
    -1, PIN1_Pin, PIN2_Pin, PIN3_Pin, PIN4_Pin, PIN5_Pin, PIN6_Pin, PIN7_Pin, PIN8_Pin, PIN9_Pin,
    PIN10_Pin, PIN11_Pin, PIN12_Pin, PIN13_Pin, PIN14_Pin, PIN15_Pin, PIN16_Pin, PIN17_Pin,
    PIN18_Pin, PIN19_Pin};

static GPIO_TypeDef *segmentToGpioPort[] = {SEG_A_GPIO_Port, SEG_B_GPIO_Port, SEG_C_GPIO_Port, SEG_D_GPIO_Port,
                                            SEG_E_GPIO_Port, SEG_F_GPIO_Port, SEG_G_GPIO_Port, SEG_DP_GPIO_Port};
static int segmentToGpio[] = {SEG_A_Pin, SEG_B_Pin, SEG_C_Pin, SEG_D_Pin, SEG_E_Pin, SEG_F_Pin, SEG_G_Pin, SEG_DP_Pin};

static int shortCircuits[HDMI_PINS + 1] = {0};

static bool start = false;
static bool singlestep = false;

static bool no_errors;
static int error_number = 0;
static int disp_pos = 0;
static GPIO_PinState toggle;
static uint32_t tick;
static uint32_t next_blink;
static uint32_t disp_time;
static int blink_count;


bool segments[4][8] = {false};

void setInputDirection(int pin) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = pinToPin[pin];
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(pinToPort[pin], &GPIO_InitStruct);
}

void setOutputDirection(int pin) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = pinToPin[pin];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(pinToPort[pin], &GPIO_InitStruct);
}

void run_dots() {
  if (tick > next_blink) {
    next_blink = tick + 100;
    segments[0][7] = false;
    segments[1][7] = false;
    segments[2][7] = false;
    segments[3][7] = false;
    segments[blink_count][7] = true;
    blink_count = (blink_count + 1) % 4;
  }
}

void find_next_error() {
  for (; shortCircuits[error_number] == 0 && error_number <= HDMI_PINS; ++error_number) {
    // do nothing
  }
  if (error_number > HDMI_PINS)
    error_number = 0;
}

void print_digit(int pos, int value) {
  bool *segm = segments[pos];
  switch (value) {
    case 0:
      segm[0] = true;
      segm[1] = true;
      segm[2] = true;
      segm[3] = true;
      segm[4] = true;
      segm[5] = true;
      segm[6] = false;
      segm[7] = false;
      break;
    case 1:
      segm[0] = false;
      segm[1] = true;
      segm[2] = true;
      segm[3] = false;
      segm[4] = false;
      segm[5] = false;
      segm[6] = false;
      segm[7] = false;
      break;
    case 2:
      segm[0] = true;
      segm[1] = true;
      segm[2] = false;
      segm[3] = true;
      segm[4] = true;
      segm[5] = false;
      segm[6] = true;
      segm[7] = false;
      break;
    case 3:
      segm[0] = true;
      segm[1] = true;
      segm[2] = true;
      segm[3] = true;
      segm[4] = false;
      segm[5] = false;
      segm[6] = true;
      segm[7] = false;
      break;
    case 4:
      segm[0] = false;
      segm[1] = true;
      segm[2] = true;
      segm[3] = false;
      segm[4] = false;
      segm[5] = true;
      segm[6] = true;
      segm[7] = false;
      break;
    case 5:
      segm[0] = true;
      segm[1] = false;
      segm[2] = true;
      segm[3] = true;
      segm[4] = false;
      segm[5] = true;
      segm[6] = true;
      segm[7] = false;
      break;
    case 6:
      segm[0] = true;
      segm[1] = false;
      segm[2] = true;
      segm[3] = true;
      segm[4] = true;
      segm[5] = true;
      segm[6] = true;
      segm[7] = false;
      break;
    case 7:
      segm[0] = true;
      segm[1] = true;
      segm[2] = true;
      segm[3] = false;
      segm[4] = false;
      segm[5] = false;
      segm[6] = false;
      segm[7] = false;
      break;
    case 8:
      segm[0] = true;
      segm[1] = true;
      segm[2] = true;
      segm[3] = true;
      segm[4] = true;
      segm[5] = true;
      segm[6] = true;
      segm[7] = false;
      break;
    case 9:
      segm[0] = true;
      segm[1] = true;
      segm[2] = true;
      segm[3] = true;
      segm[4] = false;
      segm[5] = true;
      segm[6] = true;
      segm[7] = false;
      break;
    default:
      segm[0] = false;
      segm[1] = false;
      segm[2] = false;
      segm[3] = false;
      segm[4] = false;
      segm[5] = false;
      segm[6] = false;
      segm[7] = false;
      break;
  }
}

void display_loop() {
  HAL_GPIO_WritePin(CATH1_GPIO_Port, CATH1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CATH2_GPIO_Port, CATH2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CATH3_GPIO_Port, CATH3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CATH4_GPIO_Port, CATH4_Pin, GPIO_PIN_SET);
  switch (disp_pos) {
    case 0:
      HAL_GPIO_WritePin(CATH1_GPIO_Port, CATH1_Pin, GPIO_PIN_RESET);
      break;
    case 1:
      HAL_GPIO_WritePin(CATH2_GPIO_Port, CATH2_Pin, GPIO_PIN_RESET);
      break;
    case 2:
      HAL_GPIO_WritePin(CATH3_GPIO_Port, CATH3_Pin, GPIO_PIN_RESET);
      break;
    case 3:
      HAL_GPIO_WritePin(CATH4_GPIO_Port, CATH4_Pin, GPIO_PIN_RESET);
      break;
  }
  for (int i = 0; i < 8; i++) {
    bool segm = segments[disp_pos][i];
    HAL_GPIO_WritePin(segmentToGpioPort[i], segmentToGpio[i], segm ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  disp_pos = (disp_pos + 1) % 4;
}

void clear_disp()
{
  print_digit(0,-1);
  print_digit(1,-1);
  print_digit(2,-1);
  print_digit(3,-1);
}

void show_number(int left, int right) {
  clear_disp();
  int first = left / 10;
  int second = left % 10;
  int third = right / 10;
  int forth = right % 10;
  if (first == 0)
    first = -1;
  print_digit(0, first);
  print_digit(1, second);
  if (third == 0)
    third = -1;
  print_digit(2, third);
  print_digit(3, forth);
}

void show_ok() {
  clear_disp();
  print_digit(1,0);
  print_digit(3,0);
}
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {

    tick = HAL_GetTick();
    if (start) {
      no_errors = true;
      for (int pin = 1; pin <= HDMI_PINS; pin++) {
        setInputDirection(pin);
      }
      for (int pinOut = 1; pinOut <= HDMI_PINS; pinOut++) {
        setOutputDirection(pinOut);
        HAL_GPIO_WritePin(pinToPort[pinOut], pinToPin[pinOut], GPIO_PIN_SET);
        shortCircuits[pinOut] = 0;
        for (int pin = 1; pin <= HDMI_PINS; pin++) {
          if (pin != pinOut) {
            GPIO_PinState state = HAL_GPIO_ReadPin(pinToPort[pin], pinToPin[pin]);
            if (state == GPIO_PIN_SET) {
              shortCircuits[pinOut] = pin;
              no_errors = false;
              singlestep = true;
            }
          }
          HAL_Delay(2);
        }
        HAL_GPIO_WritePin(pinToPort[pinOut], pinToPin[pinOut], GPIO_PIN_RESET);
        setInputDirection(pinOut);
      }
    }
    start = false;
    if (no_errors) {
      show_ok();
    } else if (singlestep) {
      find_next_error();
      show_number(error_number, shortCircuits[error_number]);
      singlestep = false;
    }
    if (tick > disp_time) {
      display_loop();
      disp_time = tick + 5;
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SEG_DP_GPIO_Port, SEG_DP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PIN1_GPIO_Port, PIN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CATH1_Pin | CATH2_Pin | CATH3_Pin | SEG_B_Pin
                           | SEG_A_Pin | CATH4_Pin | SEG_G_Pin | SEG_F_Pin
                           | SEG_E_Pin | SEG_D_Pin | SEG_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW5_Pin SW4_Pin */
  GPIO_InitStruct.Pin = SW5_Pin | SW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SEG_DP_Pin */
  GPIO_InitStruct.Pin = SEG_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SEG_DP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PIN18_Pin */
  GPIO_InitStruct.Pin = PIN18_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PIN18_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW3_Pin */
  GPIO_InitStruct.Pin = SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SW3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PIN1_Pin */
  GPIO_InitStruct.Pin = PIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PIN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN3_Pin PIN4_Pin PIN6_Pin PIN7_Pin
                           PIN9_Pin PIN10_Pin PIN12_Pin PIN13_Pin
                           PIN14_Pin PIN19_Pin PIN15_Pin PIN16_Pin
                           PIN2_Pin */
  GPIO_InitStruct.Pin = PIN3_Pin | PIN4_Pin | PIN6_Pin | PIN7_Pin
                        | PIN9_Pin | PIN10_Pin | PIN12_Pin | PIN13_Pin
                        | PIN14_Pin | PIN19_Pin | PIN15_Pin | PIN16_Pin
                        | PIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CATH1_Pin CATH2_Pin CATH3_Pin SEG_B_Pin
                           SEG_A_Pin CATH4_Pin SEG_G_Pin SEG_F_Pin
                           SEG_E_Pin SEG_D_Pin SEG_C_Pin */
  GPIO_InitStruct.Pin = CATH1_Pin | CATH2_Pin | CATH3_Pin | SEG_B_Pin
                        | SEG_A_Pin | CATH4_Pin | SEG_G_Pin | SEG_F_Pin
                        | SEG_E_Pin | SEG_D_Pin | SEG_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN17_Pin PIN11_Pin PIN8_Pin PIN5_Pin */
  GPIO_InitStruct.Pin = PIN17_Pin | PIN11_Pin | PIN8_Pin | PIN5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin & SW4_Pin)
    start = true;
  if (GPIO_Pin & SW5_Pin)
    singlestep = true;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
