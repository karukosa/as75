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
#include <stdio.h>

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
@@ -194,58 +195,66 @@ static uint8_t App_BlinkState(uint32_t now);
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
static GPIO_PinState App_ReadWaterLevelStableState(void);
static void App_HandleStartupChecks(void);
static void App_InitHeaterPid(void);
static void App_PrepareHoldPid(uint32_t now);
static GPIO_PinState App_ComputeHoldHeaterState(uint32_t now);
static void App_EmergencyStop(uint8_t isOverTemperature);
static void App_ResetToInitialIdle(void);
static void App_RaiseError(AppErrorCode code);
static void App_LogPt100Itm(uint32_t now);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0U) {
    ITM_SendChar((uint32_t)ch);
  }
  return ch;
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

@@ -936,107 +945,122 @@ static void App_InitPt100(void)
  Max31865_Init(&pt100Sensor, &hspi3, CS_GPIO_Port, CS_Pin,  PT100_RREF_OHMS, PT100_RNOMINAL_OHMS);
  if (Max31865_Begin(&pt100Sensor, PT100_WIRE_MODE, 1U) == 0U) {
	  pt100TemperatureValid = 0U;
	  pt100FaultCode = 0xFFU;
	  if (appMode == APP_MODE_RUN_PROGRAM) {
	    App_RaiseError(APP_ERROR_PT100);
	  }
  return;
  }
  Max31865_EnableBias(&pt100Sensor, 1U);
  Max31865_AutoConvert(&pt100Sensor, 1U);
  HAL_Delay(70U);
  Max31865_ClearFault(&pt100Sensor);
  fault = Max31865_ReadFault(&pt100Sensor, MAX31865_FAULT_AUTO);
  if (fault != 0U) {
    pt100TemperatureValid = 0U;
    pt100FaultCode = fault;
    return;
  }

  lastPt100SampleTick = HAL_GetTick() - PT100_SAMPLE_MS;
  pt100TemperatureValid = 0U;
  pt100FaultCode = 0U;
}

static void App_UpdatePt100(uint32_t now)␊
{␊
  int16_t measuredTempTenths;␊

  if ((now - lastPt100SampleTick) < PT100_SAMPLE_MS) {
    return;
  }

  lastPt100SampleTick = now;
  /* Xóa cờ lỗi tồn trước khi đo để tránh treo Er01 giả khi cảm biến/PT100 vẫn tốt. */
  Max31865_ClearFault(&pt100Sensor);
  if (Max31865_ReadTemperatureTenthsC(&pt100Sensor, &measuredTempTenths) == 0U) {
       pt100TemperatureValid = 0U;
       pt100FaultCode = 0xFFU;
       if (appMode == APP_MODE_RUN_PROGRAM) {
         App_RaiseError(APP_ERROR_PT100);
       }
       return;
   }

  pt100FaultCode = Max31865_ReadFault(&pt100Sensor, MAX31865_FAULT_NONE);
    if (pt100FaultCode != 0U) {
      uint8_t retry;

      /* Retry 2 lần để lọc nhiễu tức thời trên bus SPI/PT100. */
      for (retry = 0U; retry < 2U; retry++) {
        Max31865_ClearFault(&pt100Sensor);
        if (Max31865_ReadTemperatureTenthsC(&pt100Sensor, &measuredTempTenths) == 0U) {
          pt100TemperatureValid = 0U;
          if (appMode == APP_MODE_RUN_PROGRAM) {
            App_RaiseError(APP_ERROR_PT100);
          }
          return;
        }

        pt100FaultCode = Max31865_ReadFault(&pt100Sensor, MAX31865_FAULT_NONE);
        if (pt100FaultCode == 0U) {
          break;
        }
      }

      if (pt100FaultCode != 0U) {
        pt100TemperatureValid = 0U;
        App_LogPt100Itm(now);
        if (appMode == APP_MODE_RUN_PROGRAM) {
          App_RaiseError(APP_ERROR_PT100);
        }
        return;
      }
    }

  pt100TempTenths = measuredTempTenths;
  pt100TemperatureValid = 1U;
  App_LogPt100Itm(now);
  if (appErrorCode == APP_ERROR_PT100) {
    appErrorCode = APP_ERROR_NONE;
    HAL_GPIO_WritePin(LD_Alarm_GPIO_Port, LD_Alarm_Pin, GPIO_PIN_RESET);
    }
  }

static void App_LogPt100Itm(uint32_t now)
{
  if (pt100TemperatureValid == 0U) {
    printf("[PT100][%lu ms] Fault=0x%02X\r\n", (unsigned long)now, pt100FaultCode);
    return;
  }

  printf("[PT100][%lu ms] Temp=%d.%d C\r\n",
         (unsigned long)now,
         (int)(pt100TempTenths / 10),
         (int)(pt100TempTenths < 0 ? -(pt100TempTenths % 10) : (pt100TempTenths % 10)));
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
