/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * The entire control program below was created in March 2026
  * By Vu Nam Hung a.k.a. Karukosa using STM32CubdIDE 1.14.1
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "button_input.h"
#include "tm1637.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRESET_COUNT                  6U
#define USER_TEMPERATURE_MAX          134
#define USER_TEMPERATURE_MIN          0
#define USER_TIME_MINUTES_MAX         9999
#define USER_TIME_MINUTES_MIN         0
#define BUTTON_DEBOUNCE_MS            30U
#define BUTTON_LONG_PRESS_MS          2000U
#define BUTTON_REPEAT_MS              200U
#define DISPLAY_BLINK_MS              500U
#define PROCESS_TICK_MS               60000U
#define BUZZER_BEEP_MS                150

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
typedef enum {
  MODE_PRESET_1 = 0,
  MODE_PRESET_2,
  MODE_PRESET_3,
  MODE_PRESET_4,
  MODE_PRESET_5,
  MODE_PRESET_6,
  MODE_USER
} ProgramMode;

typedef enum {
  USER_FIELD_TEMPERATURE = 0,
  USER_FIELD_TIME
} UserField;

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  GPIO_TypeDef *ledPort;
  uint16_t ledPin;
  uint16_t temperature;
  uint16_t sterilizeMinutes;
  uint16_t dryMinutes;
} PresetConfig;

static TM1637Handle temperatureDisplay;
static TM1637Handle timeDisplay;

static const PresetConfig presetConfigs[PRESET_COUNT] = {
    {B_P1_GPIO_Port, B_P1_Pin, LED_P1_GPIO_Port, LED_P1_Pin, 121U, 20U, 12U},
    {B_P2_GPIO_Port, B_P2_Pin, LED_P2_GPIO_Port, LED_P2_Pin, 121U, 15U, 0U},
    {B_P3_GPIO_Port, B_P3_Pin, LED_P3_GPIO_Port, LED_P3_Pin, 132U, 7U, 10U},
    {B_P4_GPIO_Port, B_P4_Pin, LED_P4_GPIO_Port, LED_P4_Pin, 134U, 7U, 10U},
    {B_P5_GPIO_Port, B_P5_Pin, LED_P5_GPIO_Port, LED_P5_Pin, 134U, 10U, 20U},
    {B_P6_GPIO_Port, B_P6_Pin, LED_P6_GPIO_Port, LED_P6_Pin, 134U, 5U, 5U},
};

static ButtonInput presetButtons[PRESET_COUNT];
static ButtonInput startButton;
static ButtonInput setButton;
static ButtonInput upButton;
static ButtonInput downButton;
static ButtonInput userButton;

static ProgramMode activeMode = MODE_PRESET_1;
static UserField userField = USER_FIELD_TEMPERATURE;
static uint16_t userTemperature = 105U;
static uint16_t userTimeMinutes = 15U;
static uint16_t targetTemperature = 105U;
static uint16_t targetTimeMinutes = 15U;
static uint16_t remainingMinutes = 15U;
static uint8_t processRunning = 0U;
static uint32_t processTick = 0U;
static uint32_t displayBlinkTick = 0U;
static uint32_t buzzerOffTick = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
static void InitializeApplication(void);
static void InitializeButtons(void);
static void UpdateButtons(void);
static void HandleUserInterface(void);
static uint16_t GetPresetTotalMinutes(uint8_t presetIndex);
static void HandlePresetSelection(uint8_t presetIndex);
static void HandleUserModeSelection(void);
static void ToggleUserField(void);
static void ApplyAdjustment(int16_t delta);
static void HandleStartButton(void);
static void UpdateProcessState(void);
static void RefreshOutputs(void);
static void UpdatePresetLeds(void);
static void UpdateDisplay(void);
static void SetBuzzer(uint8_t enabled);

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  InitializeApplication();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	UpdateButtons();
	HandleUserInterface();
	UpdateProcessState();
	RefreshOutputs();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_C3_Pin|LED_C4_Pin|LED_C5_Pin|LED_C6_Pin
                          |LED_C7_Pin|LED_ALARM_Pin|LED_LW_Pin|LED_HW_Pin
                          |LED_C1_Pin|LED_C2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|CLK1_Pin|DIO1_Pin|CLK2_Pin
                          |DIO2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |LED_P1_Pin|LED_P2_Pin|LED_P3_Pin|LED_P4_Pin
                          |LED_P5_Pin|LED_P6_Pin|LED_START_Pin|LED_USER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_C3_Pin LED_C4_Pin LED_C5_Pin LED_C6_Pin
                           LED_C7_Pin LED_ALARM_Pin LED_LW_Pin LED_HW_Pin
                           LED_C1_Pin LED_C2_Pin */
  GPIO_InitStruct.Pin = LED_C3_Pin|LED_C4_Pin|LED_C5_Pin|LED_C6_Pin
                          |LED_C7_Pin|LED_ALARM_Pin|LED_LW_Pin|LED_HW_Pin
                          |LED_C1_Pin|LED_C2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : B_P1_Pin B_P2_Pin B_P3_Pin B_P4_Pin
                           B_P5_Pin B_P6_Pin B_START_Pin B_SET_Pin
                           B_UP_Pin B_DOWN_Pin B_USER_Pin */
  GPIO_InitStruct.Pin = B_P1_Pin|B_P2_Pin|B_P3_Pin|B_P4_Pin
                          |B_P5_Pin|B_P6_Pin|B_START_Pin|B_SET_Pin
                          |B_UP_Pin|B_DOWN_Pin|B_USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin CLK1_Pin DIO1_Pin CLK2_Pin
                           DIO2_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|CLK1_Pin|DIO1_Pin|CLK2_Pin
                          |DIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           LED_P1_Pin LED_P2_Pin LED_P3_Pin LED_P4_Pin
                           LED_P5_Pin LED_P6_Pin LED_START_Pin LED_USER_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |LED_P1_Pin|LED_P2_Pin|LED_P3_Pin|LED_P4_Pin
                          |LED_P5_Pin|LED_P6_Pin|LED_START_Pin|LED_USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void InitializeApplication(void)
{
  tm1637Init(&temperatureDisplay, TM1637_DISPLAY_1);
  tm1637Init(&timeDisplay, TM1637_DISPLAY_2);

  InitializeButtons();
  targetTemperature = presetConfigs[0].temperature;
  targetTimeMinutes = GetPresetTotalMinutes(0U);
  remainingMinutes = targetTimeMinutes;
  RefreshOutputs();
}

static void InitializeButtons(void)
{
  for (uint8_t i = 0; i < PRESET_COUNT; ++i) {
    ButtonInput_Init(&presetButtons[i], presetConfigs[i].port, presetConfigs[i].pin, GPIO_PIN_SET);
  }

  ButtonInput_Init(&startButton, B_START_GPIO_Port, B_START_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&setButton, B_SET_GPIO_Port, B_SET_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&upButton, B_UP_GPIO_Port, B_UP_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&downButton, B_DOWN_GPIO_Port, B_DOWN_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&userButton, B_USER_GPIO_Port, B_USER_Pin, GPIO_PIN_SET);
}

static void UpdateButtons(void)
{
  uint32_t now = HAL_GetTick();

  for (uint8_t i = 0; i < PRESET_COUNT; ++i) {
    ButtonInput_Update(&presetButtons[i], now, BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS, BUTTON_REPEAT_MS);
  }

  ButtonInput_Update(&startButton, now, BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS, BUTTON_REPEAT_MS);
  ButtonInput_Update(&setButton, now, BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS, BUTTON_REPEAT_MS);
  ButtonInput_Update(&upButton, now, BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS, BUTTON_REPEAT_MS);
  ButtonInput_Update(&downButton, now, BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS, BUTTON_REPEAT_MS);
  ButtonInput_Update(&userButton, now, BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS, BUTTON_REPEAT_MS);
}

static void HandleUserInterface(void)
{
  for (uint8_t i = 0; i < PRESET_COUNT; ++i) {
    if (ButtonInput_ConsumePressed(&presetButtons[i]) != 0U && processRunning == 0U) {
      HandlePresetSelection(i);
    }
  }

  if (ButtonInput_ConsumePressed(&userButton) != 0U && processRunning == 0U) {
    HandleUserModeSelection();
  }

  if (ButtonInput_ConsumePressed(&setButton) != 0U && activeMode == MODE_USER && processRunning == 0U) {
    ToggleUserField();
  }

  if (processRunning == 0U) {
    if (ButtonInput_ConsumePressed(&upButton) != 0U) {
      ApplyAdjustment(1);
    }
    if (ButtonInput_ConsumePressed(&downButton) != 0U) {
      ApplyAdjustment(-1);
    }
    if (ButtonInput_ConsumeRepeat(&upButton) != 0U) {
      ApplyAdjustment(10);
    }
    if (ButtonInput_ConsumeRepeat(&downButton) != 0U) {
      ApplyAdjustment(-10);
    }
  }

  HandleStartButton();

  (void)ButtonInput_ConsumeReleased(&upButton);
  (void)ButtonInput_ConsumeReleased(&downButton);
  (void)ButtonInput_ConsumeReleased(&setButton);
  (void)ButtonInput_ConsumeReleased(&startButton);
  (void)ButtonInput_ConsumeReleased(&userButton);
  for (uint8_t i = 0; i < PRESET_COUNT; ++i) {
    (void)ButtonInput_ConsumeReleased(&presetButtons[i]);
  }
}

static uint16_t GetPresetTotalMinutes(uint8_t presetIndex)
{
  return (uint16_t)(presetConfigs[presetIndex].sterilizeMinutes + presetConfigs[presetIndex].dryMinutes);
}

static void HandlePresetSelection(uint8_t presetIndex)
{
  activeMode = (ProgramMode)presetIndex;
  userField = USER_FIELD_TEMPERATURE;
  targetTemperature = presetConfigs[presetIndex].temperature;
  targetTimeMinutes = GetPresetTotalMinutes(presetIndex);
  remainingMinutes = targetTimeMinutes;
}

static void HandleUserModeSelection(void)
{
  activeMode = MODE_USER;
  targetTemperature = userTemperature;
  targetTimeMinutes = userTimeMinutes;
  remainingMinutes = targetTimeMinutes;
}

static void ToggleUserField(void)
{
  if (userField == USER_FIELD_TEMPERATURE) {
    userField = USER_FIELD_TIME;
  }
  else {
    userField = USER_FIELD_TEMPERATURE;
  }
}

static void ApplyAdjustment(int16_t delta)
{
  int32_t newValue;

  if (activeMode != MODE_USER) {
    return;
  }

  if (userField == USER_FIELD_TEMPERATURE) {
    newValue = (int32_t)userTemperature + delta;
    if (newValue > USER_TEMPERATURE_MAX) {
      newValue = USER_TEMPERATURE_MAX;
    }
    if (newValue < USER_TEMPERATURE_MIN) {
      newValue = USER_TEMPERATURE_MIN;
    }
    userTemperature = (uint16_t)newValue;
    targetTemperature = userTemperature;
  }
  else {
    newValue = (int32_t)userTimeMinutes + delta;
    if (newValue > USER_TIME_MINUTES_MAX) {
      newValue = USER_TIME_MINUTES_MAX;
    }
    if (newValue < USER_TIME_MINUTES_MIN) {
      newValue = USER_TIME_MINUTES_MIN;
    }
    userTimeMinutes = (uint16_t)newValue;
    targetTimeMinutes = userTimeMinutes;
    remainingMinutes = targetTimeMinutes;
  }
}

static void HandleStartButton(void)
{
  if (ButtonInput_ConsumePressed(&startButton) == 0U) {
    return;
  }

  if (processRunning == 0U) {
    processRunning = 1U;
    remainingMinutes = targetTimeMinutes;
    processTick = HAL_GetTick();
    SetBuzzer(1U);
  }
  else {
    processRunning = 0U;
    remainingMinutes = targetTimeMinutes;
    SetBuzzer(0U);
  }
}

static void UpdateProcessState(void)
{
  uint32_t now = HAL_GetTick();

  if (buzzerOffTick != 0U && now >= buzzerOffTick) {
    SetBuzzer(0U);
  }

  if (processRunning == 0U) {
    return;
  }

  if ((now - processTick) >= PROCESS_TICK_MS) {
    processTick += PROCESS_TICK_MS;

    if (remainingMinutes > 0U) {
      --remainingMinutes;
    }

    if (remainingMinutes == 0U) {
      processRunning = 0U;
      SetBuzzer(1U);
    }
  }
}

static void RefreshOutputs(void)
{
  UpdatePresetLeds();
  UpdateDisplay();
}

static void UpdatePresetLeds(void)
{
  for (uint8_t i = 0; i < PRESET_COUNT; ++i) {
    GPIO_PinState state = (activeMode == (ProgramMode)i) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(presetConfigs[i].ledPort, presetConfigs[i].ledPin, state);
  }

  HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin,
                    (activeMode == MODE_USER) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_START_GPIO_Port, LED_START_Pin,
                    (processRunning != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void UpdateDisplay(void)
{
  uint32_t now = HAL_GetTick();
  uint16_t displayTemperature = targetTemperature;
  uint16_t displayTime = (processRunning != 0U) ? remainingMinutes : targetTimeMinutes;
  uint8_t showTemperature = 1U;
  uint8_t showTime = 1U;

  if ((now - displayBlinkTick) >= DISPLAY_BLINK_MS) {
    displayBlinkTick = now;
  }

  if (processRunning == 0U && activeMode == MODE_USER) {
    if (userField == USER_FIELD_TEMPERATURE && (now - displayBlinkTick) >= (DISPLAY_BLINK_MS / 2U)) {
      showTemperature = 0U;
    }
    if (userField == USER_FIELD_TIME && (now - displayBlinkTick) >= (DISPLAY_BLINK_MS / 2U)) {
      showTime = 0U;
    }
  }

  if (showTemperature == 0U) {
    tm1637Clear(&temperatureDisplay);
  }
  else {
    tm1637DisplayDecimal(&temperatureDisplay, displayTemperature, 0);
  }

  if (showTime == 0U) {
    tm1637Clear(&timeDisplay);
  }
  else {
    tm1637DisplayDecimal(&timeDisplay, displayTime, 0);
  }
}

static void SetBuzzer(uint8_t enabled)
{
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, (enabled != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  if (enabled != 0U) {
    buzzerOffTick = HAL_GetTick() + BUZZER_BEEP_MS;
  }
  else {
    buzzerOffTick = 0U;
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
