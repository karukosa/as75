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
  *Made by Vũ Nam Hưng aka Karukosa
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "button_input.h"
#include "max31865.h"
#include "pid.h"
#include "tm1637.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint16_t steamTempTenths;
  uint8_t sterilizeMinutes;
  uint8_t dryMinutes;
} ProgramConfig;

typedef enum {
  APP_MODE_IDLE = 0,
  APP_MODE_READY,
  APP_MODE_RUN_PROGRAM,
  APP_MODE_USER_EDIT
} AppMode;

typedef enum {
  USER_FIELD_TEMP = 0,
  USER_FIELD_STERILIZE,
  USER_FIELD_DRY
} UserField;

typedef enum {
  RUN_STAGE_IDLE = 0,
  RUN_STAGE_VACUUM,
  RUN_STAGE_HEAT,
  RUN_STAGE_HOLD,
  RUN_STAGE_VENT,
  RUN_STAGE_DRY
} RunStage;

typedef enum {
  APP_ERROR_NONE = 0,
  APP_ERROR_PT100 = 1,
  APP_ERROR_WATER = 2,
  APP_ERROR_DOOR = 3,
  APP_ERROR_HEAT_TIMEOUT = 4,
  APP_ERROR_OVER_TEMPERATURE = 5
} AppErrorCode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUTTON_DEBOUNCE_MS 40U
#define BUTTON_LONG_PRESS_MS 650U
#define BUTTON_REPEAT_MS 120U
#define BLINK_PERIOD_MS 350U
#define DISPLAY_SWAP_MS 1200U
#define BUZZER_SHORT_MS 300U
#define PT100_SAMPLE_MS 500U
#define WATER_REFILL_TIMEOUT_MS 120000U
#define RUN_STAGE_VACUUM_MS 780000U
#define RUN_STAGE_VENT_DRAIN_MS 120000U
#define RUN_STAGE_VENT_RELEASE_MS 120000U
#define RUN_STAGE_VENT_VACUUM_MS 180000U
#define RUN_STAGE_VENT_MS (RUN_STAGE_VENT_DRAIN_MS + RUN_STAGE_VENT_RELEASE_MS + RUN_STAGE_VENT_VACUUM_MS)
#define RUN_COMPLETE_BLINK_MS 3000U
#define RUN_STAGE_VACUUM_SUB_STEPS 5U
#define HEATER_PID_WINDOW_MS 2000U
#define HEATER_PID_SAMPLE_MS 500U
#define HEATER_PID_OUTPUT_MAX_PERCENT 100.0
#define HEATER_PID_KP 18.0
#define HEATER_PID_KI 0.35
#define HEATER_PID_KD 15.0
#define EMERGENCY_STOP_TEMP_TENTHS 1380
#define HEAT_TIMEOUT_MS 2100000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
static TM1637Handle display1;
static TM1637Handle display2;
static Max31865Handle pt100Sensor;
static ButtonInput programButtons[6];
static ButtonInput buttonUser;
static ButtonInput buttonStart;
static ButtonInput buttonSet;
static ButtonInput buttonUp;
static ButtonInput buttonDown;

static const ProgramConfig programPresets[6] = {
    {1210U, 15U, 0U}, {121U, 20U, 15U}, {1320U, 7U, 10U},
    {1340U, 7U, 10U}, {1340U, 10U, 20U}, {1340U, 5U, 5U}};

static ProgramConfig userConfig = {1210U, 25U, 15U};
static ProgramConfig activeConfig = {1210U, 25U, 15U};
static AppMode appMode = APP_MODE_IDLE;
static UserField selectedUserField = USER_FIELD_TEMP;
static uint8_t activeProgramIndex = 0xFFU;
static uint32_t lastDisplaySwapTick = 0U;
static uint32_t programStartTick = 0U;
static RunStage runStage = RUN_STAGE_IDLE;
static uint32_t runStageStartTick = 0U;
static uint32_t runStageDurationMs = 0U;
static uint8_t runCompleteLatched = 0U;
static uint32_t runCompleteTick = 0U;

static int16_t pt100TempTenths = 0;
static uint8_t pt100TemperatureValid = 0U;
static uint8_t pt100FaultCode = 0U;
static uint32_t lastPt100SampleTick = 0U;

static uint8_t buzzerActive = 0U;
static uint8_t buzzerPhaseIsOn = 0U;
static uint8_t buzzerPhasesRemaining = 0U;
static uint32_t buzzerPhaseDurationMs = 0U;
static uint32_t buzzerPhaseTick = 0U;
static uint8_t startupWaterReady = 0U;
static PID_TypeDef heaterPid;
static double heaterPidInput = 0.0;
static double heaterPidOutput = 0.0;
static double heaterPidSetpoint = 0.0;
static uint32_t heaterPidWindowStartTick = 0U;
static uint8_t heaterPidReady = 0U;
static uint32_t heaterPidOnTimeMs = 0U;
static AppErrorCode appErrorCode = APP_ERROR_NONE;
static uint8_t runUsesUserConfig = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
static void App_InitUi(void);
static void App_UpdateButtons(void);
static void App_HandleInput(uint32_t now);
static void App_UpdateDisplay(uint32_t now);
static void App_UpdateLeds(uint32_t now);
static void App_StartProgram(uint8_t index, const ProgramConfig *cfg);
static void App_BeginRun(void);
static void App_AdjustUserField(int16_t delta);
static uint8_t App_BlinkState(uint32_t now);
static void App_DisplayStValue(uint8_t minutes);
static void App_DisplayDrValue(uint8_t minutes);
static uint8_t App_EncodeSegmentChar(char c);
static void App_RequestShortBeep(void);
static void App_RequestPatternBeep(uint8_t blinks, uint32_t phaseMs);
static void App_UpdateBuzzer(uint32_t now);
static void App_UpdateRunState(uint32_t now);
static void App_MoveToNextRunStage(uint32_t now);
static void App_ActivateRunStage(RunStage stage, uint32_t now);
static uint8_t App_IsRunStageTimedOut(uint32_t now);
static void App_ApplyRunOutputs(uint32_t now);
static void App_InitPt100(void);
static void App_UpdatePt100(uint32_t now);
static void App_DisplayError(TM1637Handle *display, AppErrorCode code);
static uint8_t App_PreStartChecks(void);
static uint8_t App_CheckWaterReady(void);
static uint8_t App_CheckDoorClosed(void);
static uint8_t App_IsWaterSufficient(void);
static void App_HandleStartupChecks(void);
static void App_InitHeaterPid(void);
static void App_PrepareHoldPid(uint32_t now);
static GPIO_PinState App_ComputeHoldHeaterState(uint32_t now);
static void App_EmergencyStop(uint8_t isOverTemperature);
static void App_ResetToInitialIdle(void);
static void App_RaiseError(AppErrorCode code);

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
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
  App_InitUi();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	uint32_t now = HAL_GetTick();

	MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    App_HandleStartupChecks();
    App_UpdateButtons();
    App_HandleInput(HAL_GetTick());
    App_UpdatePt100(HAL_GetTick());
    App_UpdateRunState(HAL_GetTick());
    App_UpdateDisplay(HAL_GetTick());
    App_UpdateLeds(HAL_GetTick());
    App_UpdateBuzzer(HAL_GetTick());
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
  HAL_GPIO_WritePin(GPIOE, LD_C3_Pin|LD_C4_Pin|LD_C5_Pin|LD_C6_Pin
                          |LD_C7_Pin|LD_Alarm_Pin|LD_LW_Pin|LD_HW_Pin
                          |SSR_Heater_Pin|SSR_HResistor_Pin|Relay_Valve_1_Pin|Relay_Valve_2_Pin
                          |Relay_Valve_3_Pin|Relay_Valve_4_Pin|LD_C1_Pin|LD_C2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DRDY_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Buzzer_Pin|Relay_Valve_5_Pin|CLK1_Pin|DIO1_Pin
                          |CLK2_Pin|DIO2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Relay_Pump_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD6_Pin|LD_P1_Pin|LD_P2_Pin|LD_P3_Pin
                          |LD_P4_Pin|LD_P5_Pin|LD_P6_Pin|LD_Start_Pin
                          |LD_User_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD_C3_Pin LD_C4_Pin LD_C5_Pin LD_C6_Pin
                           LD_C7_Pin LD_Alarm_Pin LD_LW_Pin LD_HW_Pin
                           SSR_Heater_Pin SSR_HResistor_Pin Relay_Valve_1_Pin Relay_Valve_2_Pin
                           Relay_Valve_3_Pin Relay_Valve_4_Pin LD_C1_Pin LD_C2_Pin */
  GPIO_InitStruct.Pin = LD_C3_Pin|LD_C4_Pin|LD_C5_Pin|LD_C6_Pin
                          |LD_C7_Pin|LD_Alarm_Pin|LD_LW_Pin|LD_HW_Pin
                          |SSR_Heater_Pin|SSR_HResistor_Pin|Relay_Valve_1_Pin|Relay_Valve_2_Pin
                          |Relay_Valve_3_Pin|Relay_Valve_4_Pin|LD_C1_Pin|LD_C2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : B_P1_Pin B_P2_Pin B_P3_Pin B_P4_Pin
                           B_P5_Pin B_P6_Pin B_Start_Pin B_Set_Pin
                           B_Up_Pin B_Down_Pin B_User_Pin Water_Sennor_Pin */
  GPIO_InitStruct.Pin = B_P1_Pin|B_P2_Pin|B_P3_Pin|B_P4_Pin
                          |B_P5_Pin|B_P6_Pin|B_Start_Pin|B_Set_Pin
                          |B_Up_Pin|B_Down_Pin|B_User_Pin|Water_Sennor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DRDY_Pin CS_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_Pin Relay_Valve_5_Pin CLK1_Pin DIO1_Pin
                           CLK2_Pin DIO2_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|Relay_Valve_5_Pin|CLK1_Pin|DIO1_Pin
                          |CLK2_Pin|DIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : L_Switch_Pin */
  GPIO_InitStruct.Pin = L_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(L_Switch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay_Pump_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD6_Pin LD_P1_Pin LD_P2_Pin LD_P3_Pin
                           LD_P4_Pin LD_P5_Pin LD_P6_Pin LD_Start_Pin
                           LD_User_Pin */
  GPIO_InitStruct.Pin = Relay_Pump_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD6_Pin|LD_P1_Pin|LD_P2_Pin|LD_P3_Pin
                          |LD_P4_Pin|LD_P5_Pin|LD_P6_Pin|LD_Start_Pin
                          |LD_User_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_SD_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void App_InitUi(void)
{
  tm1637Init(&display1, TM1637_DISPLAY_1);
  tm1637Init(&display2, TM1637_DISPLAY_2);
  tm1637SetBrightness(&display1, 8);
  tm1637SetBrightness(&display2, 8);

  /* Only LD_LW/LD_HW are active-low, keep all status LEDs OFF at idle */
  HAL_GPIO_WritePin(LD_Alarm_GPIO_Port, LD_Alarm_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD_LW_GPIO_Port, LD_LW_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LD_HW_GPIO_Port, LD_HW_Pin, GPIO_PIN_SET);

  ButtonInput_Init(&programButtons[0], B_P1_GPIO_Port, B_P1_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&programButtons[1], B_P2_GPIO_Port, B_P2_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&programButtons[2], B_P3_GPIO_Port, B_P3_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&programButtons[3], B_P4_GPIO_Port, B_P4_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&programButtons[4], B_P5_GPIO_Port, B_P5_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&programButtons[5], B_P6_GPIO_Port, B_P6_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&buttonUser, B_User_GPIO_Port, B_User_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&buttonStart, B_Start_GPIO_Port, B_Start_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&buttonSet, B_Set_GPIO_Port, B_Set_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&buttonUp, B_Up_GPIO_Port, B_Up_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&buttonDown, B_Down_GPIO_Port, B_Down_Pin, GPIO_PIN_SET);

  activeConfig = programPresets[0];
  startupWaterReady = 0U;
  App_InitPt100();
  App_UpdatePt100(HAL_GetTick());
  App_UpdateDisplay(HAL_GetTick());
}

static void App_UpdateButtons(void)
{
  uint32_t now = HAL_GetTick();

  for (uint8_t i = 0U; i < 6U; ++i) {
    ButtonInput_Update(&programButtons[i], now, BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS, BUTTON_REPEAT_MS);
  }

  ButtonInput_Update(&buttonUser, now, BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS, BUTTON_REPEAT_MS);
  ButtonInput_Update(&buttonStart, now, BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS, BUTTON_REPEAT_MS);
  ButtonInput_Update(&buttonSet, now, BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS, BUTTON_REPEAT_MS);
  ButtonInput_Update(&buttonUp, now, BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS, BUTTON_REPEAT_MS);
  ButtonInput_Update(&buttonDown, now, BUTTON_DEBOUNCE_MS, BUTTON_LONG_PRESS_MS, BUTTON_REPEAT_MS);
}

static void App_HandleInput(uint32_t now)
{
  if (startupWaterReady == 0U) {
    return;
  }

  for (uint8_t i = 0U; i < 6U; ++i) {
    if (ButtonInput_ConsumePressed(&programButtons[i]) != 0U) {
      if (appMode == APP_MODE_RUN_PROGRAM) {
        App_RequestShortBeep();
        continue;
      }
      /* Nút chương trình hoạt động kiểu "radio":
         chọn P mới thì P cũ tự bỏ chọn, luôn giữ đúng 1 chương trình đang chọn. */
      App_StartProgram(i, &programPresets[i]);
      App_RequestShortBeep();
    }
  }

  if (ButtonInput_ConsumePressed(&buttonUser) != 0U) {
    if (appMode == APP_MODE_RUN_PROGRAM) {
      App_RequestShortBeep();
    }
    else {
      if (appMode == APP_MODE_USER_EDIT) {
        appMode = APP_MODE_IDLE;
        activeProgramIndex = 0xFFU;
        runCompleteLatched = 0U;
      }
      else {
        appMode = APP_MODE_USER_EDIT;
        selectedUserField = USER_FIELD_TEMP;
        activeProgramIndex = 0xFFU;
        activeConfig = userConfig;
        runCompleteLatched = 0U;
        lastDisplaySwapTick = now;
      }
      App_RequestShortBeep();
    }
  }

  if (ButtonInput_ConsumePressed(&buttonStart) != 0U) {
    if (appMode == APP_MODE_RUN_PROGRAM) {
      App_ResetToInitialIdle();
      App_RequestShortBeep();
    }
    else if (appMode == APP_MODE_READY || appMode == APP_MODE_USER_EDIT) {
      HAL_GPIO_WritePin(LD_Alarm_GPIO_Port, LD_Alarm_Pin, GPIO_PIN_RESET);
      if (appMode == APP_MODE_USER_EDIT) {
        userConfig = activeConfig;
      }
      if (App_PreStartChecks() != 0U) {
        App_BeginRun();
        App_RequestPatternBeep(2U, 500U);
      }
    }
    else {
      App_RequestShortBeep();
      }
    }

  if (appMode == APP_MODE_USER_EDIT) {
    if (ButtonInput_ConsumePressed(&buttonSet) != 0U) {
      selectedUserField = (UserField)(((uint8_t)selectedUserField + 1U) % 3U);
      App_RequestShortBeep();
    }

    if (ButtonInput_ConsumePressed(&buttonUp) != 0U) {
      App_AdjustUserField(1);
      App_RequestShortBeep();
    }
    if (ButtonInput_ConsumeRepeat(&buttonUp) != 0U) {
      App_AdjustUserField(10);
    }

    if (ButtonInput_ConsumePressed(&buttonDown) != 0U) {
      App_AdjustUserField(-1);
      App_RequestShortBeep();
    }
    if (ButtonInput_ConsumeRepeat(&buttonDown) != 0U) {
      App_AdjustUserField(-10);
    }

    userConfig = activeConfig;
  }
}

static void App_UpdateDisplay(uint32_t now)
{
  uint8_t blinkState = App_BlinkState(now);
  uint32_t elapsedMs;
  uint32_t remainingMs;
  uint8_t remainingMinutes;

  if (appErrorCode != APP_ERROR_NONE) {
    tm1637Clear(&display1);
    App_DisplayError(&display2, appErrorCode);
    return;
  }
  else if (appMode == APP_MODE_RUN_PROGRAM) {
    if (pt100TemperatureValid != 0U) {
      tm1637DisplayDecimalTenths(&display2, pt100TempTenths);
    }
    else {
      tm1637Clear(&display1);
      App_DisplayError(&display2, APP_ERROR_PT100);
      return;
    }
  }
  else {
    tm1637DisplayDecimalTenths(&display2, activeConfig.steamTempTenths);
  }

  if (appMode == APP_MODE_USER_EDIT) {
    if (selectedUserField == USER_FIELD_STERILIZE) {
      if (blinkState != 0U) {
        App_DisplayStValue(activeConfig.sterilizeMinutes);
      }
      else {
        tm1637Clear(&display1);
      }
    }
    else if (selectedUserField == USER_FIELD_DRY) {
      if (blinkState != 0U) {
        App_DisplayDrValue(activeConfig.dryMinutes);
      }
      else {
        tm1637Clear(&display1);
      }
    }
    else {
      App_DisplayStValue(activeConfig.sterilizeMinutes);
    }

    return;
    }

  if (appMode == APP_MODE_RUN_PROGRAM) {
      if (runStage == RUN_STAGE_HEAT) {
        elapsedMs = now - runStageStartTick;
        remainingMs = (elapsedMs >= HEAT_TIMEOUT_MS) ? 0U : (HEAT_TIMEOUT_MS - elapsedMs);
        remainingMinutes = (uint8_t)((remainingMs + 59999U) / 60000U);
        App_DisplayStValue(remainingMinutes);
        return;
      }

      if (runStage == RUN_STAGE_DRY) {
        elapsedMs = now - runStageStartTick;
        remainingMs = (elapsedMs >= runStageDurationMs) ? 0U : (runStageDurationMs - elapsedMs);
        remainingMinutes = (uint8_t)((remainingMs + 59999U) / 60000U);
        App_DisplayDrValue(remainingMinutes);
        return;
      }
    }

    if ((((now - lastDisplaySwapTick) / DISPLAY_SWAP_MS) % 2U) == 0U) {
      App_DisplayStValue(activeConfig.sterilizeMinutes);
    }
    else {
    App_DisplayDrValue(activeConfig.dryMinutes);
  }
}

static void App_UpdateLeds(uint32_t now)
{
  GPIO_TypeDef *programPorts[6] = {LD_P1_GPIO_Port, LD_P2_GPIO_Port, LD_P3_GPIO_Port,
                                   LD_P4_GPIO_Port, LD_P5_GPIO_Port, LD_P6_GPIO_Port};
  uint16_t programPins[6] = {LD_P1_Pin, LD_P2_Pin, LD_P3_Pin, LD_P4_Pin, LD_P5_Pin, LD_P6_Pin};
  GPIO_PinState blink = App_BlinkState(now) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState isUserEdit = (appMode == APP_MODE_USER_EDIT) ? GPIO_PIN_SET : GPIO_PIN_RESET;

  for (uint8_t i = 0U; i < 6U; ++i) {
    GPIO_PinState state = GPIO_PIN_RESET;
    if (activeProgramIndex == i) {
      state = (appMode == APP_MODE_USER_EDIT) ? blink : GPIO_PIN_SET;
    }
    HAL_GPIO_WritePin(programPorts[i], programPins[i], state);
  }

  if (appMode == APP_MODE_USER_EDIT) {
      HAL_GPIO_WritePin(LD_User_GPIO_Port, LD_User_Pin, blink);
  }
  else if (appMode == APP_MODE_RUN_PROGRAM && runUsesUserConfig != 0U) {
    HAL_GPIO_WritePin(LD_User_GPIO_Port, LD_User_Pin, GPIO_PIN_SET);
  }
  else {
    HAL_GPIO_WritePin(LD_User_GPIO_Port, LD_User_Pin, GPIO_PIN_RESET);
  }

  if (runCompleteLatched != 0U) {
    GPIO_PinState completeState = GPIO_PIN_SET;
    if ((now - runCompleteTick) < RUN_COMPLETE_BLINK_MS) {
      completeState = blink;
    }

    HAL_GPIO_WritePin(LD_C1_GPIO_Port, LD_C1_Pin, completeState);
    HAL_GPIO_WritePin(LD_C2_GPIO_Port, LD_C2_Pin, completeState);
    HAL_GPIO_WritePin(LD_C3_GPIO_Port, LD_C3_Pin, completeState);
    HAL_GPIO_WritePin(LD_C4_GPIO_Port, LD_C4_Pin, completeState);
    HAL_GPIO_WritePin(LD_C5_GPIO_Port, LD_C5_Pin, completeState);
    HAL_GPIO_WritePin(LD_C6_GPIO_Port, LD_C6_Pin, completeState);
    HAL_GPIO_WritePin(LD_C7_GPIO_Port, LD_C7_Pin, completeState);
  }
  else if (appMode == APP_MODE_RUN_PROGRAM) {
    GPIO_PinState c1State = GPIO_PIN_RESET;
    GPIO_PinState c2State = GPIO_PIN_RESET;
    GPIO_PinState c3State = GPIO_PIN_RESET;
    GPIO_PinState c4State = GPIO_PIN_RESET;
    GPIO_PinState c5State = GPIO_PIN_RESET;
    GPIO_PinState c6State = GPIO_PIN_RESET;
    GPIO_PinState c7State = GPIO_PIN_RESET;

    if (runStage == RUN_STAGE_VACUUM) {
      c1State = blink;
    }
    else if (runStage == RUN_STAGE_HEAT) {
      c1State = GPIO_PIN_SET;
      c2State = blink;
    }
    else if (runStage == RUN_STAGE_HOLD) {
      uint8_t holdSegment = 0U;
      c1State = GPIO_PIN_SET;
      c2State = GPIO_PIN_SET;
      if (runStageDurationMs > 0U) {
        uint32_t elapsed = now - runStageStartTick;
        holdSegment = (uint8_t)((elapsed * 3U) / runStageDurationMs);
        if (holdSegment > 2U) {
          holdSegment = 2U;
         }
      }

      c3State = (holdSegment > 0U) ? GPIO_PIN_SET : blink;
      c4State = (holdSegment > 1U) ? GPIO_PIN_SET : ((holdSegment == 1U) ? blink : GPIO_PIN_RESET);
      c5State = (holdSegment == 2U) ? blink : GPIO_PIN_RESET;
    }
      else if (runStage == RUN_STAGE_VENT) {
        c1State = GPIO_PIN_SET;
        c2State = GPIO_PIN_SET;
        c3State = GPIO_PIN_SET;
        c4State = GPIO_PIN_SET;
        c5State = GPIO_PIN_SET;
        c6State = blink;
      }
      else if (runStage == RUN_STAGE_DRY) {
        c1State = GPIO_PIN_SET;
        c2State = GPIO_PIN_SET;
        c3State = GPIO_PIN_SET;
        c4State = GPIO_PIN_SET;
        c5State = GPIO_PIN_SET;
        c6State = GPIO_PIN_SET;
        c7State = blink;
      }

    HAL_GPIO_WritePin(LD_C1_GPIO_Port, LD_C1_Pin, c1State);
    HAL_GPIO_WritePin(LD_C2_GPIO_Port, LD_C2_Pin, c2State);
    HAL_GPIO_WritePin(LD_C3_GPIO_Port, LD_C3_Pin, c3State);
    HAL_GPIO_WritePin(LD_C4_GPIO_Port, LD_C4_Pin, c4State);
    HAL_GPIO_WritePin(LD_C5_GPIO_Port, LD_C5_Pin, c5State);
    HAL_GPIO_WritePin(LD_C6_GPIO_Port, LD_C6_Pin, c6State);
    HAL_GPIO_WritePin(LD_C7_GPIO_Port, LD_C7_Pin, c7State);
  }
  else {
    HAL_GPIO_WritePin(LD_C1_GPIO_Port, LD_C1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD_C2_GPIO_Port, LD_C2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD_C3_GPIO_Port, LD_C3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD_C4_GPIO_Port, LD_C4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD_C5_GPIO_Port, LD_C5_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD_C6_GPIO_Port, LD_C6_Pin, GPIO_PIN_RESET);
  }

  HAL_GPIO_WritePin(LD_Start_GPIO_Port, LD_Start_Pin, (appMode == APP_MODE_RUN_PROGRAM) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  if (isUserEdit == GPIO_PIN_SET) {
    HAL_GPIO_WritePin(LD_Start_GPIO_Port, LD_Start_Pin, blink);
  }
}

static void App_StartProgram(uint8_t index, const ProgramConfig *cfg)
{
  if (cfg == NULL || index >= 6U) {
    return;
  }

  activeProgramIndex = index;
  activeConfig = *cfg;
  appMode = APP_MODE_READY;
  lastDisplaySwapTick = HAL_GetTick();
  runCompleteLatched = 0U;
  runUsesUserConfig = 0U;
}

 static void App_BeginRun(void)
 {
  runUsesUserConfig = (appMode == APP_MODE_USER_EDIT) ? 1U : 0U;
  programStartTick = HAL_GetTick();
  appErrorCode = APP_ERROR_NONE;
  appMode = APP_MODE_RUN_PROGRAM;
  lastDisplaySwapTick = programStartTick;
  runCompleteLatched = 0U;
  App_ActivateRunStage(RUN_STAGE_VACUUM, programStartTick);
}

static void App_AdjustUserField(int16_t delta)
{
  int32_t nextValue;

  if (selectedUserField == USER_FIELD_TEMP) {
    nextValue = (int32_t)activeConfig.steamTempTenths + delta;
    if (nextValue < 1050) {
      nextValue = 1050;
    }
    if (nextValue > 1340) {
      nextValue = 1340;
    }
    activeConfig.steamTempTenths = (uint16_t)nextValue;
  }
  else if (selectedUserField == USER_FIELD_STERILIZE) {
    nextValue = (int32_t)activeConfig.sterilizeMinutes + delta;
    if (nextValue < 0) {
      nextValue = 0;
    }
    if (nextValue > 99) {
      nextValue = 99;
    }
    activeConfig.sterilizeMinutes = (uint8_t)nextValue;
  }
  else {
    nextValue = (int32_t)activeConfig.dryMinutes + delta;
    if (nextValue < 0) {
      nextValue = 0;
    }
    if (nextValue > 99) {
      nextValue = 99;
    }
    activeConfig.dryMinutes = (uint8_t)nextValue;
  }
}

static uint8_t App_BlinkState(uint32_t now)
{
  return (((now / BLINK_PERIOD_MS) % 2U) == 0U) ? 1U : 0U;
}

static void App_DisplayStValue(uint8_t minutes)
{
  uint8_t segments[4] = {0};
  segments[0] = App_EncodeSegmentChar('S');
  segments[1] = App_EncodeSegmentChar('t');
  segments[2] = App_EncodeSegmentChar((char)('0' + ((minutes / 10U) % 10U)));
  segments[3] = App_EncodeSegmentChar((char)('0' + (minutes % 10U)));
  tm1637DisplaySegments(&display1, segments);
}

static void App_DisplayDrValue(uint8_t minutes)
{
  uint8_t segments[4] = {0};
  segments[0] = App_EncodeSegmentChar('D');
  segments[1] = App_EncodeSegmentChar('r');
  segments[2] = App_EncodeSegmentChar((char)('0' + ((minutes / 10U) % 10U)));
  segments[3] = App_EncodeSegmentChar((char)('0' + (minutes % 10U)));
  tm1637DisplaySegments(&display1, segments);
}

static uint8_t App_EncodeSegmentChar(char c)
{
  switch (c) {
    case '0': return 0x3f;
    case '1': return 0x06;
    case '2': return 0x5b;
    case '3': return 0x4f;
    case '4': return 0x66;
    case '5': return 0x6d;
    case '6': return 0x7d;
    case '7': return 0x07;
    case '8': return 0x7f;
    case '9': return 0x6f;
    case 'S': return 0x6d;
    case 't': return 0x78;
    case 'D': return 0x5e;
    case 'E': return 0x79;
    case 'r': return 0x50;
    default: return 0x00;
  }
}

static void App_InitPt100(void)
{
  Max31865_Init(&pt100Sensor, &hspi1, CS_GPIO_Port, CS_Pin, 430.0f, 100.0f);
  if (Max31865_Begin(&pt100Sensor, MAX31865_3WIRE, 1U) == 0U) {
    pt100TemperatureValid = 0U;
    pt100FaultCode = 0xFFU;
    App_RaiseError(APP_ERROR_PT100);
    return;
  }

  lastPt100SampleTick = HAL_GetTick() - PT100_SAMPLE_MS;
  pt100TemperatureValid = 0U;
  pt100FaultCode = 0U;
}

static void App_UpdatePt100(uint32_t now)
{
  int16_t measuredTempTenths;

  if ((now - lastPt100SampleTick) < PT100_SAMPLE_MS) {
    return;
  }

  lastPt100SampleTick = now;
   if (Max31865_ReadTemperatureTenthsC(&pt100Sensor, &measuredTempTenths) == 0U) {
     pt100TemperatureValid = 0U;
     pt100FaultCode = 0xFFU;
     App_RaiseError(APP_ERROR_PT100);
     return;
   }

   pt100FaultCode = Max31865_ReadFault(&pt100Sensor, MAX31865_FAULT_NONE);
   if (pt100FaultCode != 0U) {
     pt100TemperatureValid = 0U;
     App_RaiseError(APP_ERROR_PT100);
     return;
   }

  if (measuredTempTenths < 0) {
    measuredTempTenths = 0;
  }

  pt100TempTenths = measuredTempTenths;
  pt100TemperatureValid = 1U;
  if (appErrorCode == APP_ERROR_PT100) {
    appErrorCode = APP_ERROR_NONE;
    HAL_GPIO_WritePin(LD_Alarm_GPIO_Port, LD_Alarm_Pin, GPIO_PIN_RESET);
    }
  }

static void App_DisplayError(TM1637Handle *display, AppErrorCode code)
{
  uint8_t segments[4] = {0};
  uint8_t errorNumber = (uint8_t)code;

  if (display == NULL) {
    return;
  }

  if (errorNumber > 99U) {
    errorNumber = 99U;
  }

  segments[0] = App_EncodeSegmentChar('E');
  segments[1] = App_EncodeSegmentChar('r');
  segments[2] = App_EncodeSegmentChar((char)('0' + ((errorNumber / 10U) % 10U)));
  segments[3] = App_EncodeSegmentChar((char)('0' + (errorNumber % 10U)));
  tm1637DisplaySegments(display, segments);
}

static uint8_t App_PreStartChecks(void)
{
  if (App_CheckWaterReady() == 0U) {
    return 0U;
  }

  if (App_CheckDoorClosed() == 0U) {
    return 0U;
  }

  return 1U;
}

static uint8_t App_CheckWaterReady(void)
{
  uint32_t startTick = HAL_GetTick();

  /* PC11 (Water_Sennor): HIGH = thiếu nước, LOW = đủ nước */
  if (HAL_GPIO_ReadPin(Water_Sennor_GPIO_Port, Water_Sennor_Pin) == GPIO_PIN_SET) {
    HAL_GPIO_WritePin(Relay_Valve_1_GPIO_Port, Relay_Valve_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LD_LW_GPIO_Port, LD_LW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD_HW_GPIO_Port, LD_HW_Pin, GPIO_PIN_SET);

    while ((HAL_GetTick() - startTick) < WATER_REFILL_TIMEOUT_MS) {
      uint32_t now = HAL_GetTick();

      if (HAL_GPIO_ReadPin(Water_Sennor_GPIO_Port, Water_Sennor_Pin) == GPIO_PIN_RESET) {
        break;
      }

      /* Tránh vòng lặp busy-wait làm "đơ" hiển thị/còi trong lúc chờ cấp nước. */
      App_UpdateDisplay(now);
      App_UpdateLeds(now);
      App_UpdateBuzzer(now);
      HAL_Delay(10U);
    }

    HAL_GPIO_WritePin(Relay_Valve_1_GPIO_Port, Relay_Valve_1_Pin, GPIO_PIN_RESET);
    if (HAL_GPIO_ReadPin(Water_Sennor_GPIO_Port, Water_Sennor_Pin) == GPIO_PIN_SET) {
      App_RaiseError(APP_ERROR_WATER);
      return 0U;
    }
  }

    if (appErrorCode == APP_ERROR_WATER) {
      appErrorCode = APP_ERROR_NONE;
    }
    HAL_GPIO_WritePin(LD_LW_GPIO_Port, LD_LW_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LD_HW_GPIO_Port, LD_HW_Pin, GPIO_PIN_RESET);
    if (appErrorCode == APP_ERROR_NONE) {
      HAL_GPIO_WritePin(LD_Alarm_GPIO_Port, LD_Alarm_Pin, GPIO_PIN_RESET);
  }
    return 1U;
}

static uint8_t App_IsWaterSufficient(void)
{
  /* PC11 (Water_Sennor): HIGH = thiếu nước, LOW = đủ nước */
  return (HAL_GPIO_ReadPin(Water_Sennor_GPIO_Port, Water_Sennor_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static uint8_t App_CheckDoorClosed(void)
{
  /* PB13 (L_Switch): HIGH = cửa đã đóng */
  if (HAL_GPIO_ReadPin(L_Switch_GPIO_Port, L_Switch_Pin) == GPIO_PIN_SET) {
    if (appErrorCode == APP_ERROR_DOOR) {
      appErrorCode = APP_ERROR_NONE;
      HAL_GPIO_WritePin(LD_Alarm_GPIO_Port, LD_Alarm_Pin, GPIO_PIN_RESET);
    }
    return 1U;
  }

  App_RaiseError(APP_ERROR_DOOR);
  return 0U;
}

static void App_HandleStartupChecks(void)
{
  if (startupWaterReady != 0U) {
    return;
  }

  if (App_CheckWaterReady() != 0U) {
    startupWaterReady = 1U;
    HAL_GPIO_WritePin(LD_Alarm_GPIO_Port, LD_Alarm_Pin, GPIO_PIN_RESET);
  }
}


static void App_RequestShortBeep(void)
{
  App_RequestPatternBeep(1U, BUZZER_SHORT_MS);
}

static void App_RequestPatternBeep(uint8_t blinks, uint32_t phaseMs)
{
  if (blinks == 0U || phaseMs == 0U) {
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
    buzzerActive = 0U;
    return;
  }

  buzzerActive = 1U;
  buzzerPhaseIsOn = 1U;
  buzzerPhasesRemaining = (uint8_t)(blinks * 2U);
  buzzerPhaseDurationMs = phaseMs;
  buzzerPhaseTick = HAL_GetTick();
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
}

static void App_InitHeaterPid(void)
{
  PID2(&heaterPid, &heaterPidInput, &heaterPidOutput, &heaterPidSetpoint,
       HEATER_PID_KP, HEATER_PID_KI, HEATER_PID_KD, _PID_CD_DIRECT);
  /* Output PID được chuẩn hóa 0..100 (%) để quy đổi ra thời gian ON trong mỗi cửa sổ. */
  PID_SetOutputLimits(&heaterPid, 0.0, HEATER_PID_OUTPUT_MAX_PERCENT);
  PID_SetSampleTime(&heaterPid, (int32_t)HEATER_PID_SAMPLE_MS);
  PID_SetMode(&heaterPid, _PID_MODE_AUTOMATIC);
  heaterPidWindowStartTick = HAL_GetTick();
  heaterPidOnTimeMs = 0U;
  heaterPidReady = 1U;
}

static void App_PrepareHoldPid(uint32_t now)
{
  if (heaterPidReady == 0U) {
    App_InitHeaterPid();
  }

  heaterPidSetpoint = (double)activeConfig.steamTempTenths / 10.0;
  heaterPidInput = (double)pt100TempTenths / 10.0;
  heaterPidOutput = 0.0;
  heaterPidOnTimeMs = 0U;
  PID_SetMode(&heaterPid, _PID_MODE_MANUAL);
  PID_SetMode(&heaterPid, _PID_MODE_AUTOMATIC);
  heaterPidWindowStartTick = now;
}

static GPIO_PinState App_ComputeHoldHeaterState(uint32_t now)
{
  uint32_t windowElapsed;

  if (heaterPidReady == 0U) {
    App_InitHeaterPid();
  }

  if (pt100TemperatureValid == 0U) {
    return GPIO_PIN_RESET;
  }

  heaterPidSetpoint = (double)activeConfig.steamTempTenths / 10.0;
  heaterPidInput = (double)pt100TempTenths / 10.0;
  (void)PID_Compute(&heaterPid);
  heaterPidOnTimeMs = (uint32_t)((heaterPidOutput * (double)HEATER_PID_WINDOW_MS) / HEATER_PID_OUTPUT_MAX_PERCENT);
  if (heaterPidOnTimeMs > HEATER_PID_WINDOW_MS) {
    heaterPidOnTimeMs = HEATER_PID_WINDOW_MS;
  }

  while ((now - heaterPidWindowStartTick) >= HEATER_PID_WINDOW_MS) {
    heaterPidWindowStartTick += HEATER_PID_WINDOW_MS;
  }

  windowElapsed = now - heaterPidWindowStartTick;
  /* SSR nhận tín hiệu ON/OFF: bật trong khoảng heaterPidOnTimeMs, tắt phần còn lại của cửa sổ. */
  if (windowElapsed < heaterPidOnTimeMs) {
    return GPIO_PIN_SET;
  }

  return GPIO_PIN_RESET;
}

static void App_UpdateBuzzer(uint32_t now)
{
  if (buzzerActive == 0U) {
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
    return;
  }

  if ((now - buzzerPhaseTick) < buzzerPhaseDurationMs) {
    return;
  }

  buzzerPhaseTick = now;
  if (buzzerPhasesRemaining > 0U) {
    --buzzerPhasesRemaining;
  }

  if (buzzerPhasesRemaining == 0U) {
    buzzerActive = 0U;
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
    return;
  }

  buzzerPhaseIsOn = (uint8_t)(1U - buzzerPhaseIsOn);
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, (buzzerPhaseIsOn != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void App_UpdateRunState(uint32_t now)
{
  App_ApplyRunOutputs(now);

  if (appMode != APP_MODE_RUN_PROGRAM) {
    return;
  }

  if (pt100TemperatureValid == 0U) {
    App_RaiseError(APP_ERROR_PT100);
    App_EmergencyStop(0U);
    return;
  }

  if (App_CheckDoorClosed() == 0U) {
    App_EmergencyStop(0U);
    return;
  }

  if ((runStage == RUN_STAGE_VACUUM || runStage == RUN_STAGE_HEAT || runStage == RUN_STAGE_HOLD) &&
         (App_IsWaterSufficient() == 0U)) {
    App_RaiseError(APP_ERROR_WATER);
    App_EmergencyStop(0U);
    return;
  }

  if ((pt100TemperatureValid != 0U) && (pt100TempTenths >= EMERGENCY_STOP_TEMP_TENTHS)) {
	App_RaiseError(APP_ERROR_OVER_TEMPERATURE);
    App_EmergencyStop(1U);
    return;
  }

  if ((runStage == RUN_STAGE_HEAT) && ((now - runStageStartTick) >= HEAT_TIMEOUT_MS)) {
    App_RaiseError(APP_ERROR_HEAT_TIMEOUT);
    App_EmergencyStop(0U);
    return;
  }

  if (App_IsRunStageTimedOut(now) != 0U) {
    App_MoveToNextRunStage(now);
  }

  if (runStage == RUN_STAGE_IDLE) {
    appMode = APP_MODE_READY;
    runCompleteLatched = 1U;
    runCompleteTick = now;
    App_RequestPatternBeep(3U, 1000U);
  }
}

static void App_ApplyRunOutputs(uint32_t now)
{
  GPIO_PinState pumpState = GPIO_PIN_RESET;
  GPIO_PinState valve2State = GPIO_PIN_RESET;
  GPIO_PinState valve3State = GPIO_PIN_RESET;
  GPIO_PinState valve4State = GPIO_PIN_RESET;
  GPIO_PinState steamHeaterState = GPIO_PIN_RESET;
  GPIO_PinState dryHeaterState = GPIO_PIN_RESET;

  if (appMode == APP_MODE_RUN_PROGRAM) {
    switch (runStage) {
      case RUN_STAGE_VACUUM: {
        uint32_t elapsed = now - runStageStartTick;
        uint32_t stepMs = RUN_STAGE_VACUUM_MS / RUN_STAGE_VACUUM_SUB_STEPS;
        uint8_t stepIndex = 0U;
        if (stepMs > 0U) {
          stepIndex = (uint8_t)(elapsed / stepMs);
        }
        if (stepIndex >= RUN_STAGE_VACUUM_SUB_STEPS) {
          stepIndex = RUN_STAGE_VACUUM_SUB_STEPS - 1U;
        }

        if ((stepIndex % 2U) == 0U) {
          /* 3 lần hút chân không: bật pump + valve 2 */
          pumpState = GPIO_PIN_SET;
          valve2State = GPIO_PIN_SET;
        }
        else {
          /* Xen kẽ 2 lần gia nhiệt bằng Heater PE10 */
          steamHeaterState = GPIO_PIN_SET;
        }
        break;
      }

      case RUN_STAGE_HEAT:
        /* Gia nhiệt đến đúng nhiệt độ mục tiêu của chương trình P1-P6 hoặc User */
        if ((pt100TemperatureValid != 0U) && (pt100TempTenths < (int16_t)activeConfig.steamTempTenths)) {
          steamHeaterState = GPIO_PIN_SET;
        }
        break;

      case RUN_STAGE_HOLD:
        /* Giữ nhiệt: đóng/cắt Heater theo PID */
        steamHeaterState = App_ComputeHoldHeaterState(now);
        break;

      case RUN_STAGE_VENT: {
        uint32_t elapsed = now - runStageStartTick;

        if (elapsed < RUN_STAGE_VENT_DRAIN_MS) {
          /* 2 phút đầu: mở valve 3 xả nước trong buồng. */
          valve3State = GPIO_PIN_SET;
        }
        else if (elapsed < (RUN_STAGE_VENT_DRAIN_MS + RUN_STAGE_VENT_RELEASE_MS)) {
          /* 2 phút tiếp theo: mở valve 4 xả khí. */
          valve4State = GPIO_PIN_SET;
        }
        else {
          /* 3 phút cuối: bật pump + valve 2 để hút chân không. */
          pumpState = GPIO_PIN_SET;
          valve2State = GPIO_PIN_SET;
        }
        break;
      }

      case RUN_STAGE_DRY:
        /* Giai đoạn sấy: bật pump liên tục và điều khiển PE11 giữ quanh 80°C */
        pumpState = GPIO_PIN_SET;
        if ((pt100TemperatureValid != 0U) && (pt100TempTenths < 780)) {
          dryHeaterState = GPIO_PIN_SET;
        }
        else if (pt100TempTenths > 820) {
          dryHeaterState = GPIO_PIN_RESET;
        }
        else {
          dryHeaterState = HAL_GPIO_ReadPin(SSR_HResistor_GPIO_Port, SSR_HResistor_Pin);
        }
        break;

      case RUN_STAGE_IDLE:
      default:
        break;
    }
  }
  else if (runCompleteLatched != 0U) {
    /* Sau khi hoàn tất, giữ mở valve 4 để cân bằng áp buồng. */
    valve4State = GPIO_PIN_SET;
  }

  HAL_GPIO_WritePin(Relay_Pump_GPIO_Port, Relay_Pump_Pin, pumpState);
  HAL_GPIO_WritePin(Relay_Valve_2_GPIO_Port, Relay_Valve_2_Pin, valve2State);
  HAL_GPIO_WritePin(Relay_Valve_3_GPIO_Port, Relay_Valve_3_Pin, valve3State);
  HAL_GPIO_WritePin(Relay_Valve_4_GPIO_Port, Relay_Valve_4_Pin, valve4State);
  HAL_GPIO_WritePin(SSR_Heater_GPIO_Port, SSR_Heater_Pin, steamHeaterState);
  HAL_GPIO_WritePin(SSR_HResistor_GPIO_Port, SSR_HResistor_Pin, dryHeaterState);
}

static uint8_t App_IsRunStageTimedOut(uint32_t now)
{
  if (runStage == RUN_STAGE_HEAT) {
	/* HEAT dùng điều kiện nhiệt độ đạt setpoint thay cho timer stage. */
    if (pt100TemperatureValid == 0U) {
      return 0U;
    }
    return (pt100TempTenths >= (int16_t)activeConfig.steamTempTenths) ? 1U : 0U;
  }

  if (runStageDurationMs == 0U) {
    return 1U;
  }

  return ((now - runStageStartTick) >= runStageDurationMs) ? 1U : 0U;
}

static void App_MoveToNextRunStage(uint32_t now)
{
  switch (runStage) {
    case RUN_STAGE_VACUUM:
      App_ActivateRunStage(RUN_STAGE_HEAT, now);
      break;
    case RUN_STAGE_HEAT:
      App_ActivateRunStage(RUN_STAGE_HOLD, now);
      break;
    case RUN_STAGE_HOLD:
      App_ActivateRunStage(RUN_STAGE_VENT, now);
      break;
    case RUN_STAGE_VENT:
      App_ActivateRunStage(RUN_STAGE_DRY, now);
      break;
    case RUN_STAGE_DRY:
    default:
      App_ActivateRunStage(RUN_STAGE_IDLE, now);
      break;
  }
}

static void App_ActivateRunStage(RunStage stage, uint32_t now)
{
  runStage = stage;
  runStageStartTick = now;

  switch (stage) {
    case RUN_STAGE_VACUUM:
      runStageDurationMs = RUN_STAGE_VACUUM_MS;
      break;
    case RUN_STAGE_HEAT:
      /* Pha HEAT không chạy theo timer cố định.
    	       Kết thúc khi App_IsRunStageTimedOut() xác nhận PT100 đạt setpoint. */
      runStageDurationMs = 0U;
      break;
    case RUN_STAGE_HOLD:
      /* Giữ nhiệt theo thời gian tiệt trùng (St). */
      runStageDurationMs = (uint32_t)activeConfig.sterilizeMinutes * 60000U;
      App_PrepareHoldPid(now);
      break;
    case RUN_STAGE_VENT:
      runStageDurationMs = RUN_STAGE_VENT_MS;
      break;
    case RUN_STAGE_DRY:
      /* Sấy theo thời gian sấy (Dr). */
      runStageDurationMs = (uint32_t)activeConfig.dryMinutes * 60000U;
      break;
    case RUN_STAGE_IDLE:
    default:
      runStageDurationMs = 0U;
      break;
  }
}
static void App_EmergencyStop(uint8_t isOverTemperature)
{
  appMode = APP_MODE_IDLE;
  runStage = RUN_STAGE_IDLE;
  runStageDurationMs = 0U;
  activeProgramIndex = 0xFFU;
  runCompleteLatched = 0U;
  runUsesUserConfig = 0U;

  HAL_GPIO_WritePin(SSR_Heater_GPIO_Port, SSR_Heater_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SSR_HResistor_GPIO_Port, SSR_HResistor_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Pump_GPIO_Port, Relay_Pump_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Valve_2_GPIO_Port, Relay_Valve_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Valve_3_GPIO_Port, Relay_Valve_3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Valve_4_GPIO_Port, Relay_Valve_4_Pin, GPIO_PIN_RESET);

  if (isOverTemperature != 0U) {
    HAL_GPIO_WritePin(LD_Alarm_GPIO_Port, LD_Alarm_Pin, GPIO_PIN_SET);
    App_RequestPatternBeep(3U, 500U);
  }
  else if (appErrorCode == APP_ERROR_NONE) {
    HAL_GPIO_WritePin(LD_Alarm_GPIO_Port, LD_Alarm_Pin, GPIO_PIN_RESET);
  }
}

static void App_ResetToInitialIdle(void)
{
  appMode = APP_MODE_IDLE;
  runStage = RUN_STAGE_IDLE;
  runStageDurationMs = 0U;
  activeProgramIndex = 0xFFU;
  runCompleteLatched = 0U;
  runUsesUserConfig = 0U;
  appErrorCode = APP_ERROR_NONE;
  HAL_GPIO_WritePin(LD_Alarm_GPIO_Port, LD_Alarm_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(SSR_Heater_GPIO_Port, SSR_Heater_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SSR_HResistor_GPIO_Port, SSR_HResistor_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Pump_GPIO_Port, Relay_Pump_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Valve_2_GPIO_Port, Relay_Valve_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Valve_3_GPIO_Port, Relay_Valve_3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Valve_4_GPIO_Port, Relay_Valve_4_Pin, GPIO_PIN_RESET);
}

static void App_RaiseError(AppErrorCode code)
{
  if (code == APP_ERROR_NONE) {
    return;
  }

  if (appErrorCode != code) {
    App_RequestPatternBeep(3U, 500U);
  }

  appErrorCode = code;
  HAL_GPIO_WritePin(LD_Alarm_GPIO_Port, LD_Alarm_Pin, GPIO_PIN_SET);
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
