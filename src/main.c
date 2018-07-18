/*
/ _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
\____ \| ___ |    (_   _) ___ |/ ___)  _ \
_____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
   (C)2013 Semtech

Description: Generic lora driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
/******************************************************************************
 * @file    main.c
 * @author  MCD Application Team
 * @version V1.1.4
 * @date    08-January-2018
 * @brief   this is the main!
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

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "low_power_manager.h"
#include "lora.h"
#include "timeServer.h"
#include "version.h"
#include "command.h"
#include "at.h"
#include "lora.h"
#include "app_scheduler.h"

#include "test_rf.h"

#include "i2c.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define SCHED_MAX_EVENT_DATA_SIZE      20 //sizeof(TimerEvent_t) /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                10

#define LORAWAN_ADR_ON                              1

/**
 * When fast wake up is enabled, the mcu wakes up in ~20us and
 * does not wait for the VREFINT to be settled. THis is ok for
 * most of the case except when adc must be used in this case before
 * starting the adc, you must make sure VREFINT is settled
 */
#define ENABLE_FAST_WAKEUP
/*!
 * Number of trials for the join request.
 */
#define JOINREQ_NBTRIALS                            3

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
//extern uint8_t isTxBusy;
extern uint8_t SX1276Read( uint8_t addr );

/* call back when LoRa has received a frame*/
static void LoraRxData(lora_AppData_t *AppData);
/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );
/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

/* Private variables ---------------------------------------------------------*/
/* load call backs*/
static LoRaMainCallback_t LoRaMainCallbacks = { HW_GetBatteryLevel,
                                                HW_GetTemperatureLevel,
                                                HW_GetUniqueId,
                                                HW_GetRandomSeed,
                                                LoraRxData,
                                                LORA_HasJoined,
                                                LORA_ConfirmClass
                                              };

/**
 * Initialises the Lora Parameters
 */
static LoRaParam_t LoRaParamInit = {LORAWAN_ADR_ON,
                                    DR_0,
                                    LORAWAN_PUBLIC_NETWORK,
                                    JOINREQ_NBTRIALS
                                   };



/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
void gsensor_init(void)
{
	uint8_t data = 0x3F;
	HAL_I2C_Mem_Write(&hi2c1, 0x30, 0x20, 1, &data, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, 0x30, 0x0f, 1, &data, 1, 100);
	PRINTF("g: %02X\r\n", data);
}

TimerEvent_t TestTimer;
TimerEvent_t RxWaitTimer;
TimerEvent_t SensorTimer;
uint32_t tx_counter;
uint32_t rx_counter;
extern uint8_t joinning;

char SendData[64]={0};
void OnSensorEvent( void )
{
	int8_t data[6];
	HAL_I2C_Mem_Read(&hi2c1, 0x30, 0x28, 1, data+0, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, 0x30, 0x29, 1, data+1, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, 0x30, 0x2A, 1, data+2, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, 0x30, 0x2B, 1, data+3, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, 0x30, 0x2C, 1, data+4, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, 0x30, 0x2D, 1, data+5, 1, 100);
	sprintf(SendData, "AT+SEND=2:G=%d,%d,%d\r\n",data[1],data[3],data[5]);
	//PRINTF(SendData);
	parse_cmd(SendData);
}

void OnSensorTimerEvent( void )
{
	if(joinning == 2)
	{
		TimerSetValue( &SensorTimer,  60000);
		app_sched_event_put(NULL, NULL, OnSensorEvent);
	}
	TimerStart(&SensorTimer);
}

void OnTestTimerEvent( void )
{
    TST_stop( );
    if(tx_counter)
    {
        PRINTF("Send lora frame: %d\n\r",tx_counter);
        tx_counter++;
    }
    TST_TX_LoraStart( "TEST", strlen("TEST") );

}

void OnLoRaRxEvent( void *p_event_data, uint16_t event_size )
{
    TST_stop( );
    if(rx_counter>1)
    {
        PRINTF("Recv lora frame: %d\n\r",rx_counter - 1);
    }
    TST_RX_LoraStart( );
}


void OnRxWaitTimerEvent( void )
{
    vcom_rxcheck();
    LPM_SetStopMode(LPM_UART_RX_Id, LPM_Enable );
}


int main(void)
{
    /* STM32 HAL library initialization*/
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    TimerInit( &TestTimer, OnTestTimerEvent );
    TimerSetValue( &TestTimer,  1300);
    TimerInit( &RxWaitTimer, OnRxWaitTimerEvent );
    TimerSetValue( &RxWaitTimer,  250);
    TimerInit( &SensorTimer, OnSensorTimerEvent );
    TimerSetValue( &SensorTimer,  10000);	
	
    HAL_Init( );
    /* Configure the system clock*/
    SystemClock_Config();

    /* Configure the hardware*/
    HW_Init();

#ifndef USE_BOOTLOADER
    HAL_Delay(3000);
#endif

    /* Configure Debug mode */
    DBG_Init();

    /* USER CODE BEGIN 1 */
    CMD_Init();

	gsensor_init();
    /*Disable standby mode*/
    LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);

    PRINTF("ATtention command interface\n\r");
    volatile uint8_t readdata;
    PRINTF("Version: ");
    PRINTF(AT_VERSION_STRING);
    PRINTF("\n\r");
    readdata = 0;
    readdata = SX1276Read( 0x42 );
    PRINTF("lora test1: %X\n\r",readdata);
    readdata = 0;
    readdata = SX1276Read( 0x4d );
    PRINTF("lora test2: %X\n\r",readdata);
    //PRINTF("ATtention command interface\n\r");
    /* USER CODE END 1 */

    /* Configure the Lora Stack*/
    LORA_Init(&LoRaMainCallbacks, &LoRaParamInit);

	LORA_Join();

    TimerStart(&SensorTimer);
    /* main loop*/
    while (1)
    {
        app_sched_execute();
        /* Handle UART commands */
        // CMD_Process();
        /*
         * low power section
         */
        DISABLE_IRQ();
        /*
         * if an interrupt has occurred after DISABLE_IRQ, it is kept pending
         * and cortex will not enter low power anyway
         * don't go in low power mode if we just received a char
         */
        //if(!isTxBusy)
        //if ( (IsNewCharReceived() == RESET))
        {
#ifndef LOW_POWER_DISABLE
            LPM_EnterLowPower();
#endif
        }

        ENABLE_IRQ();

        /* USER CODE BEGIN 2 */
        /* USER CODE END 2 */
    }
}


static void LoraRxData(lora_AppData_t *AppData)
{
    set_at_receive(AppData->Port, AppData->Buff, AppData->BuffSize);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    Error_Handler();
}
#endif

static void LORA_HasJoined( void )
{
//#if( OVER_THE_AIR_ACTIVATION != 0 )
    PRINTF("JOINED\n\r");
//#endif
}

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
    DBG_PRINTF("switch to class %c done\n\r","ABC"[Class] );
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
