/******************************************************************************
 * @file    vcom.c
 * @author  MCD Application Team
 * @version V1.1.4
 * @date    08-January-2018
 * @brief   manages virtual com port
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include "hw.h"
#include "vcom.h"
#include <stdarg.h>
#include "app_fifo.h"
#include "tiny_vsnprintf.h"
#include "low_power_manager.h"
#include "timeServer.h"
#include "command.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BUFSIZE 256
#define TX_BUFFER 512
//#define USARTX_IRQn USART2_IRQn
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t uart_rx_fifo_buff[BUFSIZE];
app_fifo_t uart_rxFifo;
uint8_t uart_tx_fifo_buff[TX_BUFFER];
app_fifo_t uart_txFifo;
static uint8_t isTxBusy = false;
//static uint8_t isWake = false;
uint8_t TxBuffer[BUFSIZE];
uint8_t RxBuffer[BUFSIZE];
/* Uart Handle */
static UART_HandleTypeDef UartHandle;
static UART_HandleTypeDef Uart2Handle;
extern TimerEvent_t RxWaitTimer;
uint32_t uart_rx_timer = 0;
/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/

void vcom_Init(void)
{
    /*## Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* UART1 configured as follow:
        - Word Length = 8 Bits
        - Stop Bit = One Stop bit
        - Parity = ODD parity
        - BaudRate = 921600 baud
        - Hardware flow control disabled (RTS and CTS signals) */
    UartHandle.Instance        = USARTX;

    UartHandle.Init.BaudRate   = 9600;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits   = UART_STOPBITS_1;
    UartHandle.Init.Parity     = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode       = UART_MODE_TX_RX;

// if(HAL_MultiProcessor_Init(&UartHandle, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
    if(HAL_UART_Init(&UartHandle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    HAL_NVIC_SetPriority(USARTX_IRQn, 0x1, 0);
    HAL_NVIC_EnableIRQ(USARTX_IRQn);
		
		 Uart2Handle.Instance        = USARTX2;

    Uart2Handle.Init.BaudRate   = 9600;
    Uart2Handle.Init.WordLength = UART_WORDLENGTH_8B;
    Uart2Handle.Init.StopBits   = UART_STOPBITS_1;
    Uart2Handle.Init.Parity     = UART_PARITY_NONE;
    Uart2Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    Uart2Handle.Init.Mode       = UART_MODE_TX_RX;

    Uart2Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    Uart2Handle.Init.OverSampling = UART_OVERSAMPLING_16;
    Uart2Handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    Uart2Handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

// if(HAL_MultiProcessor_Init(&UartHandle, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
    if(HAL_UART_Init(&Uart2Handle) != HAL_OK)
    {  
        /* Initialization Error */
        Error_Handler(); 
    }

    HAL_NVIC_SetPriority(USARTX2_IRQn, 0x1, 0);
    HAL_NVIC_EnableIRQ(USARTX2_IRQn);

}

void MOLTRES_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        HAL_UART_RxCpltCallback(huart);

    }
}

void vcom_IRQHandler(void)
{
    HAL_UART_IRQHandler(&UartHandle);
    MOLTRES_UART_IRQHandler(&UartHandle);
}

void vcom2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&Uart2Handle);
    MOLTRES_UART_IRQHandler(&Uart2Handle);
}

void vcom_DeInit(void)
{
    HAL_UART_DeInit(&UartHandle);
	HAL_UART_DeInit(&Uart2Handle);
}

void OnUartTxStartEvent( void *p_event_data, uint16_t event_size )
{
    if(isTxBusy == 0)
    {
        uint32_t tx_size;
        tx_size = BUFSIZE;
        app_fifo_read(&uart_txFifo,TxBuffer,&tx_size);
        if(tx_size)
        {
            LPM_SetStopMode(LPM_UART_TX_Id, LPM_Disable );
            isTxBusy = 1;
            HAL_UART_Transmit_IT(&UartHandle, (uint8_t *)TxBuffer, tx_size);
            HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_SET);
            HAL_UART_Transmit_IT(&Uart2Handle, (uint8_t *)TxBuffer, tx_size);
        }
        else
            HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_RESET);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    isTxBusy = 0;
    LPM_SetStopMode(LPM_UART_TX_Id, LPM_Enable );
    app_sched_event_put(NULL, NULL, OnUartTxStartEvent);
}

void vcom_Send( char *format, ... )
{
    va_list args;
    va_start(args, format);
    uint32_t len= 0;

    char tempBuff[BUFSIZE];

    BACKUP_PRIMASK();
    DISABLE_IRQ();

    /*convert into string at buff[0] of length iw*/
    len = tiny_vsnprintf_like(&tempBuff[0], sizeof(tempBuff), format, args);

    app_fifo_write(&uart_txFifo,(uint8_t*)tempBuff,&len);
    RESTORE_PRIMASK();

    va_end(args);
    app_sched_event_put(NULL, NULL, OnUartTxStartEvent);
}



/**
  * @brief UART MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  *           - NVIC configuration for UART interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{

    /*##-1- Enable peripherals and GPIO Clocks #################################*/

    /* Enable USART1 clock */
    USARTX_CLK_ENABLE();
    USARTX2_CLK_ENABLE();
    /*##-2- Configure peripheral GPIO ##########################################*/
    vcom_IoInit( );
}

void vcom_IoInit(void)
{
//	uint32_t i=1000;
    GPIO_InitTypeDef  GPIO_InitStruct= {0};

    /* Enable GPIO TX/RX clock */

    USARTX_TX_GPIO_CLK_ENABLE();
    USARTX_RX_GPIO_CLK_ENABLE();
    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = USARTX_TX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USARTX_TX_AF;

    HAL_GPIO_Init(USARTX_TX_GPIO_PORT, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = USARTX_RX_PIN;
    GPIO_InitStruct.Alternate = USARTX_RX_AF;

    HAL_GPIO_Init(USARTX_RX_GPIO_PORT, &GPIO_InitStruct);

    HAL_UART_RxCpltCallback(&UartHandle);
		
		
		GPIO_InitTypeDef  GPIO_InitStruct1= {0};

    /* Enable GPIO TX/RX clock */

    USARTX2_TX_GPIO_CLK_ENABLE();
    USARTX2_RX_GPIO_CLK_ENABLE();
    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct1.Pin       = USARTX2_TX_PIN;
    GPIO_InitStruct1.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct1.Pull      = GPIO_PULLUP;
    GPIO_InitStruct1.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct1.Alternate = USARTX2_TX_AF;

    HAL_GPIO_Init(USARTX2_TX_GPIO_PORT, &GPIO_InitStruct1);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct1.Pin = USARTX2_RX_PIN;
    GPIO_InitStruct1.Alternate = USARTX2_RX_AF;

    HAL_GPIO_Init(USARTX2_RX_GPIO_PORT, &GPIO_InitStruct1);

    HAL_UART_RxCpltCallback(&Uart2Handle);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA,USART2_DE_Pin,GPIO_PIN_RESET);

    /*Configure GPIO pins : PAPin PAPin PAPin PAPin
                             PAPin */
    GPIO_InitStruct1.Pin = USART2_DE_Pin;
    GPIO_InitStruct1.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct1.Pull = GPIO_NOPULL;
    GPIO_InitStruct1.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct1);
}

void vcom_RxInt( void )
{

    HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
    HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
    LPM_SetStopMode(LPM_UART_RX_Id, LPM_Disable );
//    TimerStop(&RxWaitTimer);
//    TimerStart(&RxWaitTimer);
}

void vcom_IoDeInit(void)
{
   GPIO_InitTypeDef GPIO_InitStructure= {0};

    USARTX_TX_GPIO_CLK_ENABLE();
    USARTX_RX_GPIO_CLK_ENABLE();

    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;

    GPIO_InitStructure.Pin =  USARTX_TX_PIN|USARTX2_TX_PIN ;
    HAL_GPIO_Init(  USARTX_TX_GPIO_PORT, &GPIO_InitStructure );
    HAL_GPIO_Init(  USARTX2_TX_GPIO_PORT, &GPIO_InitStructure );

    GPIO_InitTypeDef GPIO_InitStructure2= {0};
    GPIO_InitStructure2.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure2.Pull = GPIO_PULLUP;
    GPIO_InitStructure2.Pin =  USARTX_RX_PIN|USARTX2_RX_PIN ;
    HAL_GPIO_Init(  USARTX_RX_GPIO_PORT, &GPIO_InitStructure2 );
    HW_GPIO_SetIrq( USARTX_RX_GPIO_PORT, USARTX_RX_PIN, 0, &vcom_RxInt );
    HAL_GPIO_Init(  USARTX2_RX_GPIO_PORT, &GPIO_InitStructure2 );
    HW_GPIO_SetIrq( USARTX2_RX_GPIO_PORT, USARTX2_RX_PIN, 0, &vcom_RxInt );
//			TimerStop(&RxWaitTimer);
}

/**
  * @brief UART MSP DeInit
  * @param huart: uart handle
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    vcom_IoDeInit( );
}

uint8_t IsNewCharReceived(void)
{
    if(fifo_length(&uart_rxFifo))
        return SET;
    else
        return RESET;
}

char GetNewChar(void)
{
    uint8_t tempdata;
    if(app_fifo_get(&uart_rxFifo, &tempdata) == NRF_SUCCESS)
    {
        return (char)tempdata;
    }
    else
        return 0;
}

void uart2_event_handle_schedule(void *p_event_data, uint16_t event_size)
{
    uint8_t temp_buff[8];
    uint32_t temp_buff_rec_size;
    temp_buff_rec_size = 2;
    temp_buff[0]=0x0d;
    temp_buff[1]=0x0a;
    app_fifo_write(&uart_rxFifo,temp_buff,&temp_buff_rec_size);
    CMD_Process();
    /*
    uint32_t psu_event_rx_rec_size;
    uint8_t psu_event_rx_buff[BUFSIZE];

    psu_event_rx_rec_size = BUFSIZE;
    app_fifo_read(&uart_rxFifo,psu_event_rx_buff,&psu_event_rx_rec_size);

    if(psu_event_rx_rec_size )
    {
    	uint32_t i;
    	psu_event_rx_buff[psu_event_rx_rec_size] = 0;
    	for(i=0;i<psu_event_rx_rec_size;i++)
    	{
    		PRINTF("%02X ",psu_event_rx_buff[i]);
    //			PRINTF("%02X %c",psu_event_rx_buff[i],psu_event_rx_buff[i]);
    	}
    	PRINTF("\r\n");
    }
    */
}

void vcom_rxcheck(void)
{
    app_sched_event_put(NULL, NULL, uart2_event_handle_schedule);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &UartHandle || huart == &Uart2Handle)
    {
        uint32_t uart2_rx_rec_size;

        uart2_rx_rec_size = huart->RxXferSize - huart->RxXferCount;
        if(uart2_rx_rec_size &&(RxBuffer[0] != 0xff))
        {
//            TimerStop(&RxWaitTimer);
//            TimerStart(&RxWaitTimer);
            app_fifo_write(&uart_rxFifo,RxBuffer,&uart2_rx_rec_size);
            app_sched_event_put(NULL, NULL, uart2_event_handle_schedule);
        }

        HAL_UART_AbortReceive(huart);
        HAL_UART_Receive_IT(huart,RxBuffer,BUFSIZE);
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == &UartHandle)
    {
//		HAL_UART_RxCpltCallback(&UartHandle);
        //	PRINTF("Uart Error\r\n");

    }
}

void start_uart2_uart(void)
{
    app_fifo_init(&uart_rxFifo,uart_rx_fifo_buff,BUFSIZE);
    app_fifo_init(&uart_txFifo,uart_tx_fifo_buff,TX_BUFFER);
    HAL_UART_RxCpltCallback(&UartHandle);
	  HAL_UART_RxCpltCallback(&Uart2Handle);
}

uint32_t get_rx_number(void)
{
	return (UartHandle.RxXferSize - UartHandle.RxXferCount);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
