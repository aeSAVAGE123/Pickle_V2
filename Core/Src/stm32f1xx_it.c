/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
#include "bsp_basic_tim.h"
#include "bsp_led.h"
#include "bsp_debug_usart.h"
//#include "protocol.h"
#include "bsp_adc.h"
#include "bsp_pid.h"
#include "bsp_usart.h"
#include "bsp_usart_blt.h"
#include "bsp_SysTick.h"

extern unsigned char UART_RxPtr;
extern unsigned int Task_Delay[];
extern void TimingDelay_Decrement(void);
extern void TimeStamp_Increment(void);

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */


/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();                       //这行代码调用了 HAL_IncTick 函数，通常用于增加系统滴答计数器。在HAL库中，这个函数用于跟踪系统时间，通常每1毫秒递增一次。
  /* USER CODE BEGIN SysTick_IRQn 1 */
    unsigned char i;
    TimingDelay_Decrement();
    TimeStamp_Increment();

    for(i=0;i<NumOfTask;i++)
    {
        if(Task_Delay[i])
        {
            Task_Delay[i]--;
        }
    }
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/
static uint16_t adc_buff_down[10000];
static uint16_t adc_buff_up[10000];
uint8_t i = 0;
/**
  * @brief  基本定时器中断服务函数----PID计算使用
  * @param  None
  * @retval None
  */
void BASIC_TIM_IRQHandler (void)
{
    HAL_TIM_IRQHandler(&TIM_TimeBaseStructure1);
}

void BASIC_TIM5_IRQHandler (void)
{
    HAL_TIM_IRQHandler(&TIM_TimeBaseStructure5);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == (&TIM_TimeBaseStructure1))      //100Hz
    {
        LED1_TOGGLE
//        set_computer_Speed_Location_value(Send_Speed_CMD, positiondown_adc_mean);
//        set_computer_Speed_Location_value(Send_Speed_CMD,positionup_adc_mean);
//        set_computer_Speed_Location_value(Send_Speed_CMD, Rotation1_adc_mean);
//        set_computer_Speed_Location_value(Send_Speed_CMD, Rotation2_adc_mean);
//        set_computer_Speed_Location_value(Send_Speed_CMD, Rotation3_adc_mean);
        set_computer_Speed_Location_value(Send_Speed_CMD, Dropping_adc_mean);
        if (pid3_enable)
        {
            if (fabs(pid3.err) > 200)
            {
                Pflag3 = 1;
                Iflag3 = 0;
                Iflagz3 = 0;
            } else if (fabs(pid3.err) > 50 && fabs(pid3.err) <= 200)
            {
                Pflag3 = 1;
                Iflag3 = 1;
                Iflagz3 = 0;
            } else if (fabs(pid3.err) <= 50)
            {
                Pflag3 = 1;
                Iflag3 = 0;
                Iflagz3 = 1;
            }
            motor3_pid_control();
        }
        if(pid4_enable)
        {
                if (fabs(pid4.err) > 200 )
                {
                    Pflag4 = 1;
                    Iflag4 = 0;
                    Iflagz4 = 0;
                }
                else if (fabs(pid4.err) > 50 && fabs(pid4.err) <= 200)
                {
                    Pflag4 = 1;
                    Iflag4 = 1;
                    Iflagz4 = 0;
                }
                else if (fabs(pid4.err) <= 50)
                {
                    Pflag4 = 1;
                    Iflag4 = 0;
                    Iflagz4 = 1;
                }
                motor4_pid_control();
        }
    }

    if (htim == (&TIM_TimeBaseStructure5))
    {
        if (firststart == 0)
        {
            firststart = 1;
        }
        else
        {
//            LED2_TOGGLE
            pid3_enable = 0;
            MOTOR3_FWD_DISABLE();
            MOTOR3_REV_DISABLE();
            pid4_enable = 0;
            MOTOR4_FWD_DISABLE();
            MOTOR4_REV_DISABLE();
            HAL_TIM_Base_Stop_IT(&TIM_TimeBaseStructure5);
        }
    }
}

void ADC_POSITION_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
}

/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  */
void ADC_DMA_IRQ_Handler(void)
{
    HAL_DMA_IRQHandler(&DMA_Init_Handle);
}

void USART_IRQHandler(void)
{
    uint8_t data[1];

    data[0] = __HAL_UART_FLUSH_DRREGISTER(&UartHandle);      // 从数据寄存器读取接收到的数据并清除 RXNE 标志
// data[0] = UartHandle.Instance->DR;
// 检查 RXNE 中断是否发生
    if(__HAL_UART_GET_IT_SOURCE(&UartHandle, UART_IT_RXNE) != RESET)
    {
        data[0] = UartHandle.Instance->DR;                         // 读取接收寄存器中的数据
        PushArr(data_buff,data[0]);                      // 将数据推送到缓冲区
        __HAL_UART_CLEAR_FLAG(&UartHandle, UART_IT_RXNE);          // 清除 RXNE 中断标志
    }
    HAL_UART_IRQHandler(&UartHandle);                       // 调用 HAL 库的 UART 中断处理函数
    __HAL_UART_ENABLE_IT(&UartHandle,UART_IT_RXNE);               // 重新使能 RXNE 中断
}

void BLE_UARTx_IRQHandler(void)
{
    bsp_USART_Process();
}