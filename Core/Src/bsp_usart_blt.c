//
// Created by Giselle on 2024/3/17.
//
/**
  ******************************************************************************
  * @file    bsp_usart1.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   HC05串口驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 霸道 STM32 开发板
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */

#include "bsp_usart_blt.h"
#include <stdarg.h>
#include <string.h>
#include "bsp_hc05.h"
#include "bsp_led.h"

UART_HandleTypeDef UART_InitStructure;
extern ReceiveData BLE_USART_ReceiveData;
#define UART_BUFF_SIZE2      128
volatile    uint16_t uart_p2 = 0;
uint8_t     uart_buff2[UART_BUFF_SIZE2];
#define TERMINATOR '\n' // 设定终止符，比如回车符或其他标识符
volatile uint8_t data_received = 0; // 标志位，指示是否接收到完整数据


/*
 * 函数名：USARTx_Config
 * 描述  ：USART GPIO 配置,工作模式配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void BLE_USART_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
//    USART_InitTypeDef USART_InitStructure;

    /* config UART4 clock */
    BLE_UART_CLK();
    UART_RX_BLE_CLK_ENABLE();
    UART_TX_BLE_CLK_ENABLE();

    /* UART4 GPIO config */
    /* Configure UART4 Tx (PC10) as alternate function push-pull */
    GPIO_InitStructure.Pin = BLE_UART_TX_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BLE_UART_TX_PORT, &GPIO_InitStructure);

    /* Configure UART4 Rx (PC11) as input floating */
    GPIO_InitStructure.Pin = BLE_UART_RX_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BLE_UART_RX_PORT, &GPIO_InitStructure);

    /* UART4 mode config */
    UART_InitStructure.Instance = BLE_UARTx;
    UART_InitStructure.Init.BaudRate = BLE_UART_BAUD_RATE;
    UART_InitStructure.Init.WordLength = USART_WORDLENGTH_8B;
    UART_InitStructure.Init.StopBits = USART_STOPBITS_1;
    UART_InitStructure.Init.Parity = USART_PARITY_NONE ;
    UART_InitStructure.Init.Mode = UART_MODE_TX_RX;
    UART_InitStructure.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    HAL_UART_Init(&UART_InitStructure);
    /* 使能串口2接收中断 */

    /*配置串口接收中断 */
    __HAL_UART_ENABLE_IT(&UART_InitStructure,UART_IT_RXNE);
}

/*****************  发送字符串 **********************/
void BLE_Usart_SendString(uint8_t *str)
{
    unsigned int k=0;
    do
    {
        HAL_UART_Transmit( &UART_InitStructure,(uint8_t *)(str + k) ,1,1000);         //逐个发送一个以null结尾的字符串
        k++;
    } while(*(str + k)!='\0');

}

void bsp_USART_Process(void)
{
    if (__HAL_UART_GET_IT_SOURCE(&UART_InitStructure, UART_IT_RXNE) != RESET )
    {
        HAL_UART_Receive(&UART_InitStructure, (uint8_t *)&uart_buff2[uart_p2], 1, 1000);    // 接收一个字节数据
//        HAL_UART_Receive_IT(&UART_InitStructure, (uint8_t *) &uart_buff2[uart_p2], sizeof(uart_buff2));
        uart_p2++;
    }
    HAL_UART_IRQHandler(&UART_InitStructure);
//    __HAL_UART_ENABLE_IT(&UART_InitStructure,UART_IT_RXNE);               // 重新使能 RXNE 中断
}


//获取接收到的数据和长度
char *get_rebuff(uint16_t *len)
{
    *len = uart_p2;   // 获取当前数据长度
    return (char *)uart_buff2;  // 返回缓冲区指针
}


//清空缓冲区
void clean_rebuff(void)
{
    memset(uart_buff2, 0, sizeof(uart_buff2));      // 清空缓冲区，将 uart_buff2 缓冲区的所有字节设置为 0
    uart_p2 = 0;                                    // 重置缓冲区指针，重置这个指针意味着下一次接收的数据将从缓冲区的起始位置存储。
}