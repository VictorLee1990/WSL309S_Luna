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
#include "i2c.h"

#include "test_rf.h"

//#include "i2c.h"
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
/*TI_HDC2010 Define*/
#define TI_HDC2010YPAR_ADDR            	(0x82)
#define TI_HDC2010YPAR_WHO_AM_I1        (0xFF)
#define TI_HDC2010YPAR_WHO_AM_I2        (0xFE)
#define H0                              (0x02)
#define H1                              (0x03)
#define T0                              (0x00)
#define T1                              (0x01)

/*HTS221_Define*/
#define STM_HTS221_ADDR             		(0xBE)
#define STM_HTS221_WHO_AM_I             (0x0F)
#define CTRL_REG1                   		(0x20)
#define ACTIVE_MODE                 		(0x80)
#define BDU_NO_UPDATE               		(0x04)
#define ODR_1Hz                     		(0x01)
#define ODR_12Hz                     		(0x11)
#define AUTO_INCREMENT              		(0x80)
#define HUMIDITY_OUT_L              		(0x28)
#define TEMP_OUT_L                  		(0x2A)
#define H0_rH_x2                    		(0x30)
#define H1_rH_x2                    		(0x31)
#define T0_degC_x8                  		(0x32)
#define T1_degC_x8                  		(0x33)
#define T1_T0_MSB                   		(0x35)
#define H0_T0_OUT                   		(0x36)
#define H1_T0_OUT                   		(0x3A)
#define T0_OUT                      		(0x3C)
#define T1_OUT                      		(0x3E)
/* Private function prototypes -----------------------------------------------*/
//extern uint8_t isTxBusy;
extern uint8_t SX1276Read( uint8_t addr );

/* call back when LoRa has received a frame*/
static void LoraRxData(lora_AppData_t *AppData);
/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );
/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

void getSignalState(void);
void getTemp_Humi(void);
extern void parse_cmd(const char *cmd);
uint8_t I2C_Temp_Humi_Config(void);
void I2C_Temp_Humi_GetData(float *Temperature,float *Humidity);
uint8_t I2C_Temp_Humi_Config2(void);
void I2C_Temp_Humi_GetData2(float *Temperature,float *Humidity);
/* Private variables ---------------------------------------------------------*/
float send_sensorData[11]= { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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


bool LoRaJoined = false;
uint8_t lastSignal;
uint8_t IO_signalState;
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
void gsensor_init(void)
{
/*	uint8_t data = 0x3F;
	HAL_I2C_Mem_Write(&hi2c1, 0x30, 0x20, 1, &data, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, 0x30, 0x0f, 1, &data, 1, 100);
	PRINTF("g: %02X\r\n", data);*/
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
	/*int8_t data[6];
	HAL_I2C_Mem_Read(&hi2c1, 0x30, 0x28, 1, data+0, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, 0x30, 0x29, 1, data+1, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, 0x30, 0x2A, 1, data+2, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, 0x30, 0x2B, 1, data+3, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, 0x30, 0x2C, 1, data+4, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, 0x30, 0x2D, 1, data+5, 1, 100);
	sprintf(SendData, "AT+SEND=2:G=%d,%d,%d\r\n",data[1],data[3],data[5]);
	//PRINTF(SendData);
	parse_cmd(SendData);*/
	
	 getTemp_Humi();
}

void OnSensorTimerEvent( void )
{
/*	if(joinning == 2)
	{
		TimerSetValue( &SensorTimer,  60000);
		app_sched_event_put(NULL, NULL, OnSensorEvent);
	}
	TimerStart(&SensorTimer);*/
	
	  
		app_sched_event_put(NULL, NULL, OnSensorEvent);
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
    TimerSetValue( &SensorTimer,  600000);	
	
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

	//gsensor_init();
    /*Disable standby mode*/
    LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);

    uint8_t status = I2C_Temp_Humi_Config2();
//    I2C_Temp_Humi_Config();
		
    PRINTF("ATtention command interface\n\r");
    volatile uint8_t readdata;
    PRINTF("Version: ");
    PRINTF(AT_VERSION_STRING);
    PRINTF("\n\r");
		PRINTF("Zone: ");
    PRINTF(AT_ZONE_STRING);
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

  //  TimerStart(&SensorTimer);
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
	PRINTF("RxData\n\r");
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
	   LoRaJoined = true;
    PRINTF("JOINED\n\r");
  	OnSensorTimerEvent();
//#endif
}

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
    DBG_PRINTF("switch to class %c done\n\r","ABC"[Class] );
}
//------------------------------------------------------------------------------------------//

void getTemp_Humi(void)
{
    uint8_t Temp_Array[10]= {0};
    uint8_t Humi_Array[10]= {0};
    float Temp_float=0.0;
    float Humi_float=0.0;
    I2C_Temp_Humi_GetData2(&Temp_float,&Humi_float);
    sprintf(Temp_Array,"%.1f",Temp_float);
    PRINTF("STM_HTS221_Temp : ");
    PRINTF(Temp_Array);
    PRINTF("\r\n");

    sprintf(Humi_Array,"%.1f",Humi_float);
    PRINTF("STM_HTS221_Humi : ");
    PRINTF(Humi_Array);
    PRINTF("\r\n");
//    I2C_Temp_Humi_GetData(&Temp_float,&Humi_float);
////    sensor_data.temperature = (int16_t)Temp_float;
////    sensor_data.humidity = (int16_t)Humi_float;
//    sprintf(Temp_Array,"%d",(int16_t)Temp_float);
//    PRINTF("TI_HDC2010_Temp : ");
//    PRINTF(Temp_Array);
//    PRINTF("\r\n");
//    sprintf(Humi_Array,"%.0f",Humi_float);
//    PRINTF("TI_HDC2010_Humi : ");
//    PRINTF(Humi_Array);
//    PRINTF("\r\n");

 
    if(LoRaJoined)
    {
        char SendData[64]= {0};
        sprintf(SendData,"AT+SEND=2:0,0,0,0,0,0,%.1f,0,0,%.1f,0\r\n",Humi_float,Temp_float);
        PRINTF(SendData);
        parse_cmd(SendData);
    }
}

//void I2C_Temp_Humi_GetData(float *Temperature,float *Humidity)
//{
////    uint8_t dataH0[1];
////    uint8_t dataH1[1];
////    uint8_t dataT0[1];
////    uint8_t dataT1[1];
////    uint8_t Temp_Humi_WriteBuff[2]= {0x00,0x01};
////    uint8_t Temp_Humi_ReadBuff[2]= {0};
////    Temp_Humi_WriteBuff[0] = 0x02;
////    HAL_I2C_Mem_Write(&hi2c1, TI_HDC2010YPAR_ADDR, 0x0F, 1, &Temp_Humi_WriteBuff[0], 1, 100);
////    HAL_I2C_Mem_Write(&hi2c1, TI_HDC2010YPAR_ADDR, 0x0F, 1, &Temp_Humi_WriteBuff[1], 1, 100);
////    HAL_Delay(100);
////    HAL_I2C_Mem_Read(&hi2c1, TI_HDC2010YPAR_ADDR, H0, 1, dataH0, 1, 100);
////    HAL_I2C_Mem_Read(&hi2c1, TI_HDC2010YPAR_ADDR, H1, 1, dataH1, 1, 100);
////    *Humidity = (((dataH1[0] * 256 + dataH0[0])) / 65536.0)*100;

////    HAL_I2C_Mem_Read(&hi2c1, TI_HDC2010YPAR_ADDR, T0, 1, dataT0, 1, 100);
////    HAL_I2C_Mem_Read(&hi2c1, TI_HDC2010YPAR_ADDR, T1, 1, dataT1, 1, 100);
////    *Temperature = (((dataT1[0] * 256 + dataT0[0])) / 65536.0)*165 - 40;

//}

//uint8_t I2C_Temp_Humi_Config(void)
//{
////    uint8_t Temp_Humi_WriteBuff[2]= {0};
////    uint8_t Temp_Humi_ReadBuff[2]= {0};
////    uint16_t temp_write = TI_HDC2010YPAR_WHO_AM_I1;
////    uint8_t Status = 0;
//////    temp_write = (SI_SI7013_WHO_AM_I1<< 8 )+ (SI_SI7013_WHO_AM_I2 );

////    HAL_I2C_Mem_Read(&hi2c1, TI_HDC2010YPAR_ADDR, temp_write, 1, &Temp_Humi_ReadBuff[0], 1, 100);
////    temp_write = TI_HDC2010YPAR_WHO_AM_I2;
////    HAL_I2C_Mem_Read(&hi2c1, TI_HDC2010YPAR_ADDR, temp_write, 1, &Temp_Humi_ReadBuff[1], 1, 100);
////    temp_write= (Temp_Humi_ReadBuff[0]<< 8 )+ (Temp_Humi_ReadBuff[1] );

////    if(temp_write == 0x07D0)//Temp_Humi_Who am I check//
////    {
////        Status = 1;
////    }
////    else
////    {
////        Status = 0;
////    }

////    return Status;
//}

void I2C_Temp_Humi_GetData2(float *Temperature,float *Humidity)
{
    uint8_t dataT0c[1];
    uint8_t dataT1c[1];
    uint8_t dataT0T1c[1];
    uint8_t dataH0r[1];
    uint8_t dataH1r[1];
    uint8_t dataT0[2];
    uint8_t dataT1[2];
    uint8_t dataH0[2];
    uint8_t dataH1[2];
    uint8_t dataT[2];
    uint8_t dataH[2];
    short T_out;
    short T0c_out;
    short T1c_out;
    short T0_out;
    short T1_out;
    short H_out;
    short H0r_out;
    short H1r_out;
    short H0_out;
    short H1_out;

//	if(I2C_Temp_Humi_Config() == 1)
    {
        HAL_I2C_Mem_Read(&hi2c1, STM_HTS221_ADDR, T0_degC_x8, 1, dataT0c, 1, 100);
        HAL_I2C_Mem_Read(&hi2c1, STM_HTS221_ADDR, T1_degC_x8, 1, dataT1c, 1, 100);
        HAL_I2C_Mem_Read(&hi2c1, STM_HTS221_ADDR, T1_T0_MSB, 1, dataT0T1c,1, 100);
        // Calculate T0_degC_OUT and T1_degC_OUT
        T0c_out = ((int)dataT0T1c[0] & 0x03)*256  + (int)dataT0c[0];
        T1c_out = ((int)dataT0T1c[0] & 0x0c)*64  + (int)dataT1c[0];

        HAL_I2C_Mem_Read(&hi2c1, STM_HTS221_ADDR, (T0_OUT | AUTO_INCREMENT), 1, dataT0, 2, 100);
        HAL_I2C_Mem_Read(&hi2c1, STM_HTS221_ADDR, (T1_OUT | AUTO_INCREMENT), 1, dataT1, 2, 100);

        HAL_I2C_Mem_Read(&hi2c1, STM_HTS221_ADDR, H0_rH_x2, 1, dataH0r, 1, 100);
        HAL_I2C_Mem_Read(&hi2c1, STM_HTS221_ADDR, H1_rH_x2, 1, dataH1r, 1, 100);

        HAL_I2C_Mem_Read(&hi2c1, STM_HTS221_ADDR, (H0_T0_OUT | AUTO_INCREMENT), 1, dataH0, 2, 100);
        HAL_I2C_Mem_Read(&hi2c1, STM_HTS221_ADDR, (H1_T0_OUT | AUTO_INCREMENT), 1, dataH1, 2, 100);

        // Read temperature register
        HAL_I2C_Mem_Read(&hi2c1, STM_HTS221_ADDR, (TEMP_OUT_L | AUTO_INCREMENT), 1, dataT, 2, 100);
        // Read Humidity register
        HAL_I2C_Mem_Read(&hi2c1, STM_HTS221_ADDR, (HUMIDITY_OUT_L | AUTO_INCREMENT), 1, dataH, 2, 100);

        T_out = (int)dataT[0] + (int)dataT[1]*256;
        T0c_out = T0c_out /8;
        T1c_out = T1c_out /8;

        T0_out = (int)dataT0[0] + (int)dataT0[1] * 256;
        T1_out = (int)dataT1[0] + (int)dataT1[1] * 256;

        *Temperature = (float)(T_out - T0_out) * (float)(T1c_out - T0c_out) / (float)(T1_out - T0_out)  +  T0c_out;

        H_out = (int)dataH[0] + (int)dataH[1] * 256;
        H0r_out = (int)dataH0r[0];
        H0r_out = H0r_out /2;
        H1r_out = (int)dataH1r[0];
        H1r_out = H1r_out /2;

        H0_out = (int)dataH0[0] + (int)dataH0[1] * 256;
        H1_out = (int)dataH1[0] + (int)dataH1[1] * 256;
        *Humidity = (float)(H_out - H0_out) * (float)(H1r_out - H0r_out) / (float)(H1_out - H0_out)  +  H0r_out;

        //if(*Temperature >= 100) *Temperature=99.9999;
        //if(*Humidity >= 100) *Humidity=99.9999;
    }

}

uint8_t I2C_Temp_Humi_Config2(void)
{
    uint8_t Temp_Humi_WriteBuff[2]= {0};
    uint8_t Temp_Humi_ReadBuff[2]= {0};
    uint8_t Status = 0;

    HAL_I2C_Mem_Read(&hi2c1, STM_HTS221_ADDR, STM_HTS221_WHO_AM_I, 1, Temp_Humi_ReadBuff, 1, 100);

    if(Temp_Humi_ReadBuff[0] == 0xBC)//Temp_Humi_Who am I check//
    {
        Temp_Humi_WriteBuff[0] = ACTIVE_MODE | BDU_NO_UPDATE | ODR_12Hz;//setting register//
        HAL_I2C_Mem_Write(&hi2c1, STM_HTS221_ADDR, CTRL_REG1, 1, Temp_Humi_WriteBuff, 1, 100);
        HAL_I2C_Mem_Read(&hi2c1, STM_HTS221_ADDR, CTRL_REG1, 1, Temp_Humi_ReadBuff, 1, 100);
        if(Temp_Humi_ReadBuff[0] == (ACTIVE_MODE | BDU_NO_UPDATE | ODR_12Hz))
        {
            Status = 1;
        }
        else
        {
            Status = 0;
        }

    }
    else
    {
        Status = 0;
    }

    return Status;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
