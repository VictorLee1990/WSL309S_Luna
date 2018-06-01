/******************************************************************************
  * @file    lora.c
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
  * @brief   lora API to drive the lora state Machine
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
#include "timeServer.h"
#include "LoRaMac.h"
#include "lora.h"
#include "lora-test.h"
#include "tiny_sscanf.h"


/**
  * Lora Configuration
  */
typedef struct
{
    LoraState_t otaa;        /*< ENABLE if over the air activation, DISABLE otherwise */
    LoraState_t duty_cycle;  /*< ENABLE if dutycyle is on, DISABLE otherwise */
    uint8_t DevEui[8];           /*< Device EUI */
    uint8_t AppEui[8];           /*< Application EUI */
    uint8_t AppKey[16];          /*< Application Key */
    uint8_t NwkSKey[16];         /*< Network Session Key */
    uint8_t AppSKey[16];         /*< Application Session Key */
    int16_t Rssi;                /*< Rssi of the received packet */
    uint8_t Snr;                 /*< Snr of the received packet */
    uint8_t application_port;    /*< Application port we will receive to */
    LoraConfirm_t ReqAck;      /*< ENABLE if acknowledge is requested */
    McpsConfirm_t *McpsConfirm;  /*< pointer to the confirm structure */
    int8_t TxDatarate;
    uint8_t TRx;
    uint32_t RXW1;
    uint32_t RXW2;
    uint32_t TXF1;
    uint32_t TXF2;
    uint32_t TXF3;
    uint32_t TXF4;
    uint32_t TXF5;
    uint32_t TXF6;
    uint32_t TXF7;
    uint32_t TXF8;
		uint32_t DevAddr;
    bool AdrEnable;
    bool EnablePublicNetwork;		
		uint8_t Class;
		uint8_t TxPower;
		uint32_t JoinDleay1;
		uint32_t JoinDleay2;		
		uint32_t Rx1Dleay;
		uint32_t Rx2Dleay;
		int8_t RxDatarate;
    uint32_t Flag;
} lora_configuration_t;

#define FLAG_VALUE 0x3132fffe
static lora_configuration_t lora_config =
{
    .otaa = ((OVER_THE_AIR_ACTIVATION == 0) ? LORA_DISABLE : LORA_ENABLE),
#if defined( REGION_EU868 )
    .duty_cycle = LORA_ENABLE,
#else
    .duty_cycle = LORA_DISABLE,
#endif
    .DevEui = LORAWAN_DEVICE_EUI,
    .AppEui = LORAWAN_APPLICATION_EUI,
    .AppKey = LORAWAN_APPLICATION_KEY,
    .NwkSKey = LORAWAN_NWKSKEY,
    .AppSKey = LORAWAN_APPSKEY,
    .Rssi = 0,
    .Snr = 0,
    .ReqAck = LORAWAN_UNCONFIRMED_MSG,
    .McpsConfirm = NULL,
    .TxDatarate = 0,
    .TRx = 0,
    .RXW1 = 0,
    .RXW2 = 505300000	,
    .TXF1 = 470300000 ,
    .TXF2 = 470500000 ,
    .TXF3 = 470700000 ,
    .TXF4 = 470900000 ,
    .TXF5 = 471100000 ,
    .TXF6 = 471300000 ,
    .TXF7 = 471500000 ,
    .TXF8 = 471700000 ,
		.DevAddr = 0,
		.AdrEnable = LORAWAN_ADR_ON,
		.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK,
		.Class = CLASS_A,
		.TxPower = 0,
		.JoinDleay1 = 5000,
		.JoinDleay2 = 6000,
		.Rx1Dleay = 1000,
		.Rx2Dleay = 2000,
		.RxDatarate = 0,
    .Flag = FLAG_VALUE
};
/*
static lora_configuration_flash_t lora_config_flash =
{
  .otaa = ((OVER_THE_AIR_ACTIVATION == 0) ? LORA_DISABLE : LORA_ENABLE),
#if defined( REGION_EU868 )
  .duty_cycle = LORA_ENABLE,
#else
  .duty_cycle = LORA_DISABLE,
#endif
  .DevEui = LORAWAN_DEVICE_EUI,
  .AppEui = LORAWAN_APPLICATION_EUI,
  .AppKey = LORAWAN_APPLICATION_KEY,
  .NwkSKey = LORAWAN_NWKSKEY,
  .AppSKey = LORAWAN_APPSKEY,
  .ReqAck = LORAWAN_UNCONFIRMED_MSG,
  .TxDatarate = 0,
	.TRx = 0,
	.RXW1 = 0,
	.RXW2 = 505300000	,
	.TXF1 = 470300000 ,
	.TXF2 = 470500000 ,
	.TXF3 = 470700000 ,
	.TXF4 = 470900000 ,
	.TXF5 = 471100000 ,
	.TXF6 = 471300000 ,
	.TXF7 = 471500000 ,
	.TXF8 = 471700000 ,
	.Flag = FLAG_VALUE
};
*/
/*!
 * Join requests trials duty cycle.
 */
#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE           10000  // 10 [s] value in ms

#if defined( REGION_EU868 )

#include "LoRaMacTest.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

#define USE_SEMTECH_DEFAULT_CHANNEL_LINEUP          0

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 )

#define LC4                { 867100000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC5                { 867300000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC6                { 867500000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC7                { 867700000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC8                { 867900000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC9                { 868800000, 0, { ( ( DR_7 << 4 ) | DR_7 ) }, 2 }
#define LC10               { 868300000, 0, { ( ( DR_6 << 4 ) | DR_6 ) }, 1 }

#endif

#endif

static MlmeReqJoin_t JoinParameters;


static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

/*
 * Defines the LoRa parameters at Init
 */
static LoRaParam_t* LoRaParamInit;
static LoRaMacPrimitives_t LoRaMacPrimitives;
static LoRaMacCallback_t LoRaMacCallbacks;
static MibRequestConfirm_t mibReq;

static LoRaMainCallback_t *LoRaMainCallbacks;
/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] McpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
        case MCPS_UNCONFIRMED:
        {
            // Check Datarate
            // Check TxPower
            break;
        }
        case MCPS_CONFIRMED:
        {
            // Check Datarate
            // Check TxPower
            // Check AckReceived
            // Check NbTrials
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        default:
            break;
        }
    }
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    lora_AppData_t AppData;

    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
    case MCPS_UNCONFIRMED:
    {
        break;
    }
    case MCPS_CONFIRMED:
    {
        break;
    }
    case MCPS_PROPRIETARY:
    {
        break;
    }
    case MCPS_MULTICAST:
    {
        break;
    }
    default:
        break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot
    if (certif_running() == true )
    {
        certif_DownLinkIncrement( );
    }

    if( mcpsIndication->RxData == true )
    {
        switch( mcpsIndication->Port )
        {
        case CERTIF_PORT:
            certif_rx( mcpsIndication, &JoinParameters );
            break;
        default:

            AppData.Port = mcpsIndication->Port;
            AppData.BuffSize = mcpsIndication->BufferSize;
            AppData.Buff = mcpsIndication->Buffer;
            lora_config.Rssi = mcpsIndication->Rssi;
            lora_config.Snr  = mcpsIndication->Snr;
            LoRaMainCallbacks->LORA_RxData( &AppData );
            break;
        }
    }
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] MlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    switch( mlmeConfirm->MlmeRequest )
    {
    case MLME_JOIN:
    {
        if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
        {
            // Status is OK, node has joined the network
            LoRaMainCallbacks->LORA_HasJoined();
        }
        else
        {
            // Join was not successful. Try to join again
            LORA_Join();
        }
        break;
    }
    case MLME_LINK_CHECK:
    {
        if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
        {
            // Check DemodMargin
            // Check NbGateways
            if (certif_running() == true )
            {
                certif_linkCheck( mlmeConfirm);
            }
        }
        break;
    }
    default:
        break;
    }
}
/**
 *  lora Init
 */
void LORA_Init (LoRaMainCallback_t *callbacks, LoRaParam_t* LoRaParam )
{
    /* init the Tx Duty Cycle*/
    LoRaParamInit = LoRaParam;

    /* init the main call backs*/
    LoRaMainCallbacks = callbacks;

		LoRaMainCallbacks->BoardGetUniqueId( lora_config.DevEui );
		lora_config_load();
    srand1( LoRaMainCallbacks->BoardGetRandomSeed( ) );
    // Choose a random device address
    DevAddr = randr( 0, 0x01FFFFFF );	
	
			if(lora_config.DevAddr ==0)
			{
			lora_config.DevAddr = DevAddr;
			}

			MibRequestConfirm_t mib;
			mib.Param.DevAddr = lora_config.DevAddr;
			mib.Type = MIB_DEV_ADDR;
			LoRaMacMibSetRequestConfirm(&mib);		



    PRINTF("If OTAA enabled\n\r");
    PRINTF("DevEui= %02X", lora_config.DevEui[0]) ;
    for(int i=1; i<8 ; i++) {
        PRINTF("-%02X", lora_config.DevEui[i]);
    };
    PRINTF("\n\r");
    PRINTF("AppEui= %02X", lora_config.AppEui[0]) ;
    for(int i=1; i<8 ; i++) {
        PRINTF("-%02X", lora_config.AppEui[i]);
    };
    PRINTF("\n\r");
    PRINTF("AppKey= %02X", lora_config.AppKey[0]) ;
    for(int i=1; i<16; i++) {
        PRINTF(" %02X", lora_config.AppKey[i]);
    };
    PRINTF("\n\n\r");


    PRINTF("If ABP enabled\n\r");
    PRINTF("DevEui= %02X", lora_config.DevEui[0]) ;
    for(int i=1; i<8 ; i++) {
        PRINTF("-%02X", lora_config.DevEui[i]);
    };
    PRINTF("\n\r");
    PRINTF("DevAdd=  %08X\n\r", lora_config.DevAddr) ;
    PRINTF("NwkSKey= %02X", lora_config.NwkSKey[0]) ;
    for(int i=1; i<16 ; i++) {
        PRINTF(" %02X", lora_config.NwkSKey[i]);
    };
    PRINTF("\n\r");
    PRINTF("AppSKey= %02X", lora_config.AppSKey[0]) ;
    for(int i=1; i<16 ; i++) {
        PRINTF(" %02X", lora_config.AppSKey[i]);
    };
    PRINTF("\n\r");
    LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
    LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
    LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
    LoRaMacCallbacks.GetBatteryLevel = LoRaMainCallbacks->BoardGetBatteryLevel;
#if defined( REGION_AS923 )
    LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_AS923 );
#elif defined( REGION_AU915 )
    LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_AU915 );
#elif defined( REGION_CN470 )
    LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN470 );
#elif defined( REGION_CN779 )
    LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN779 );
#elif defined( REGION_EU433 )
    LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU433 );
#elif defined( REGION_IN865 )
    LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_IN865 );
#elif defined( REGION_EU868 )
    LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU868 );
#elif defined( REGION_KR920 )
    LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_KR920 );
#elif defined( REGION_US915 )
    LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_US915 );
#elif defined( REGION_US915_HYBRID )
    LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_US915_HYBRID );
#else
#error "Please define a region in the compiler options."
#endif

    mibReq.Type = MIB_ADR;
    mibReq.Param.AdrEnable = lora_config.AdrEnable;
    LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_PUBLIC_NETWORK;
    mibReq.Param.EnablePublicNetwork = lora_config.EnablePublicNetwork;
    LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_DEVICE_CLASS;
    mibReq.Param.Class= lora_config.Class;
    LoRaMacMibSetRequestConfirm( &mibReq );

		LoRaMacTestSetDutyCycleOn((lora_config.duty_cycle == LORA_ENABLE) ? 1 : 0);

    mibReq.Type = MIB_CHANNELS_TX_POWER;
    mibReq.Param.ChannelsTxPower= lora_config.TxPower;
    LoRaMacMibSetRequestConfirm( &mibReq );
	
    mibReq.Type = MIB_JOIN_ACCEPT_DELAY_1;
    mibReq.Param.JoinAcceptDelay1= lora_config.JoinDleay1;
    LoRaMacMibSetRequestConfirm( &mibReq );	

    mibReq.Type = MIB_JOIN_ACCEPT_DELAY_2;
    mibReq.Param.JoinAcceptDelay2= lora_config.JoinDleay2;
    LoRaMacMibSetRequestConfirm( &mibReq );	
		
    mibReq.Type = MIB_RX2_CHANNEL;
    mibReq.Param.Rx2Channel.Datarate= lora_config.RxDatarate;
    LoRaMacMibSetRequestConfirm( &mibReq );		
		
    mibReq.Type = MIB_RECEIVE_DELAY_1;
    mibReq.Param.ReceiveDelay1= lora_config.Rx1Dleay;
    LoRaMacMibSetRequestConfirm( &mibReq );	

    mibReq.Type = MIB_RECEIVE_DELAY_2;
    mibReq.Param.ReceiveDelay2= lora_config.Rx2Dleay;
    LoRaMacMibSetRequestConfirm( &mibReq );	

		
}


void LORA_Join( void)
{
    if (lora_config.otaa == LORA_ENABLE)
    {
        MlmeReq_t mlmeReq;

        mlmeReq.Type = MLME_JOIN;
        mlmeReq.Req.Join.DevEui = lora_config.DevEui;
        mlmeReq.Req.Join.AppEui = lora_config.AppEui;
        mlmeReq.Req.Join.AppKey = lora_config.AppKey;
        mlmeReq.Req.Join.NbTrials = LoRaParamInit->NbTrials;

        JoinParameters = mlmeReq.Req.Join;

        LoRaMacMlmeRequest( &mlmeReq );
    }
    else
    {
        MlmeReq_t mlmeReq;

        mlmeReq.Type = MLME_JOIN;
        mlmeReq.Req.Join.DevEui = lora_config.DevEui;
        mlmeReq.Req.Join.AppEui = lora_config.AppEui;
        mlmeReq.Req.Join.AppKey = lora_config.AppKey;
        mlmeReq.Req.Join.NbTrials = LoRaParamInit->NbTrials;

        JoinParameters = mlmeReq.Req.Join;

        mibReq.Type = MIB_NET_ID;
        mibReq.Param.NetID = LORAWAN_NETWORK_ID;
        LoRaMacMibSetRequestConfirm( &mibReq );

        mibReq.Type = MIB_DEV_ADDR;
        mibReq.Param.DevAddr = DevAddr;
        LoRaMacMibSetRequestConfirm( &mibReq );

        mibReq.Type = MIB_NWK_SKEY;
        mibReq.Param.NwkSKey = lora_config.NwkSKey;
        LoRaMacMibSetRequestConfirm( &mibReq );

        mibReq.Type = MIB_APP_SKEY;
        mibReq.Param.AppSKey = lora_config.AppSKey;
        LoRaMacMibSetRequestConfirm( &mibReq );

        mibReq.Type = MIB_NETWORK_JOINED;
        mibReq.Param.IsNetworkJoined = true;
        LoRaMacMibSetRequestConfirm( &mibReq );

        LoRaMainCallbacks->LORA_HasJoined();
    }
}

LoraFlagStatus LORA_JoinStatus( void)
{
    MibRequestConfirm_t mibReq;

    mibReq.Type = MIB_NETWORK_JOINED;

    LoRaMacMibGetRequestConfirm( &mibReq );

    if( mibReq.Param.IsNetworkJoined == true )
    {
        return LORA_SET;
    }
    else
    {
        return LORA_RESET;
    }
}

LoraErrorStatus LORA_send(lora_AppData_t* AppData, LoraConfirm_t IsTxConfirmed)
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    /*if certification test are on going, application data is not sent*/
    if (certif_running() == true)
    {
        return LORA_ERROR;
    }

    if( LoRaMacQueryTxPossible( AppData->BuffSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = lora_config_tx_datarate_get() ;
    }
    else
    {
        if( IsTxConfirmed == LORAWAN_UNCONFIRMED_MSG )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppData->Port;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppData->BuffSize;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData->Buff;
            mcpsReq.Req.Unconfirmed.Datarate = lora_config_tx_datarate_get() ;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppData->Port;
            mcpsReq.Req.Confirmed.fBufferSize = AppData->BuffSize;
            mcpsReq.Req.Confirmed.fBuffer = AppData->Buff;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = lora_config_tx_datarate_get() ;
        }
    }
    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return LORA_SUCCESS;
    }
    return LORA_ERROR;
}

LoraErrorStatus LORA_RequestClass( DeviceClass_t newClass )
{
    LoraErrorStatus Errorstatus = LORA_SUCCESS;
    MibRequestConfirm_t mibReq;
    DeviceClass_t currentClass;

    mibReq.Type = MIB_DEVICE_CLASS;
    LoRaMacMibGetRequestConfirm( &mibReq );

    currentClass = mibReq.Param.Class;
    /*attempt to swicth only if class update*/
    if (currentClass != newClass)
    {
        switch (newClass)
        {
        case CLASS_A:
        {
            if (currentClass == CLASS_A)
            {
                mibReq.Param.Class = CLASS_A;
                if( LoRaMacMibSetRequestConfirm( &mibReq ) == LORAMAC_STATUS_OK )
                {
                    /*switch is instantanuous*/
                    LoRaMainCallbacks->LORA_ConfirmClass(CLASS_A);
                }
                else
                {
                    Errorstatus = LORA_ERROR;
                }
            }
            break;
        }
        case CLASS_C:
        {
            if (currentClass != CLASS_A)
            {
                Errorstatus = LORA_ERROR;
            }
            /*switch is instantanuous*/
            mibReq.Param.Class = CLASS_C;
            if( LoRaMacMibSetRequestConfirm( &mibReq ) == LORAMAC_STATUS_OK )
            {
                LoRaMainCallbacks->LORA_ConfirmClass(CLASS_C);
            }
            else
            {
                Errorstatus = LORA_ERROR;
            }
            break;
        }
        default:
            break;
        }
    }
    return Errorstatus;
}

void LORA_GetCurrentClass( DeviceClass_t *currentClass )
{
    MibRequestConfirm_t mibReq;

    mibReq.Type = MIB_DEVICE_CLASS;
    LoRaMacMibGetRequestConfirm( &mibReq );

    *currentClass = mibReq.Param.Class;
}


void lora_config_otaa_set(LoraState_t otaa)
{
    lora_config.otaa = otaa;
}


LoraState_t lora_config_otaa_get(void)
{
    return lora_config.otaa;
}

void lora_config_duty_cycle_set(LoraState_t duty_cycle)
{
    lora_config.duty_cycle = duty_cycle;
    LoRaMacTestSetDutyCycleOn((duty_cycle == LORA_ENABLE) ? 1 : 0);
}

LoraState_t lora_config_duty_cycle_get(void)
{
    return lora_config.duty_cycle;
}

uint8_t *lora_config_deveui_get(void)
{
    return lora_config.DevEui;
}

void lora_config_deveui_set(uint8_t deveui[8])
{
    memcpy1(lora_config.DevEui, deveui, sizeof(lora_config.DevEui));
}

uint8_t *lora_config_appeui_get(void)
{
    return lora_config.AppEui;
}

void lora_config_appeui_set(uint8_t appeui[8])
{
    memcpy1(lora_config.AppEui, appeui, sizeof(lora_config.AppEui));
}


uint8_t *lora_config_appkey_get(void)
{
    return lora_config.AppKey;
}

void lora_config_appkey_set(uint8_t appkey[16])
{
    memcpy1(lora_config.AppKey, appkey, sizeof(lora_config.AppKey));
}

uint8_t *lora_config_appskey_get(void)
{
    return lora_config.AppSKey;
}

void lora_config_appskey_set(uint8_t appskey[16])
{
    memcpy1(lora_config.AppSKey, appskey, sizeof(lora_config.AppSKey));
}

uint8_t *lora_config_nwkskey_get(void)
{
    return lora_config.NwkSKey;
}

void lora_config_nwkskey_set(uint8_t nwkskey[16])
{
    memcpy1(lora_config.NwkSKey, nwkskey, sizeof(lora_config.NwkSKey));
}

void lora_config_reqack_set(LoraConfirm_t reqack)
{
    lora_config.ReqAck = reqack;
}

LoraConfirm_t lora_config_reqack_get(void)
{
    return lora_config.ReqAck;
}

int8_t lora_config_snr_get(void)
{
    return lora_config.Snr;
}

int16_t lora_config_rssi_get(void)
{
    return lora_config.Rssi;
}

void lora_config_tx_datarate_set(int8_t TxDataRate)
{
    lora_config.TxDatarate =TxDataRate;
}

int8_t lora_config_tx_datarate_get(void )
{
    return lora_config.TxDatarate;
}

LoraState_t lora_config_isack_get(void)
{
    if (lora_config.McpsConfirm == NULL)
    {
        return LORA_DISABLE;
    }
    else
    {
        return (lora_config.McpsConfirm->AckReceived ? LORA_ENABLE : LORA_DISABLE);
    }
}

void lora_config_trx_set(uint8_t enable)
{
    lora_config.TRx = enable;
}

uint8_t lora_config_trx_get(void)
{
    return lora_config.TRx;
}

void lora_config_rx1_set(uint32_t rxfreq)
{
    lora_config.RXW1 = rxfreq;
}

uint32_t lora_config_rx1_get(void)
{
    return lora_config.RXW1;
}

void lora_config_rx2_set(uint32_t rxfreq)
{
    lora_config.RXW2 = rxfreq;
}

uint32_t lora_config_rx2_get(void)
{
    return lora_config.RXW2;
}

void lora_config_tx1_set(uint32_t txfreq)
{
    lora_config.TXF1 = txfreq;
    RegionCN470SetFrq(0, txfreq);
}

uint32_t lora_config_tx1_get(void)
{
    return lora_config.TXF1;
}

void lora_config_tx2_set(uint32_t txfreq)
{
    lora_config.TXF2 = txfreq;
    RegionCN470SetFrq(1, txfreq);
}

uint32_t lora_config_tx2_get(void)
{
    return lora_config.TXF2;
}
void lora_config_tx3_set(uint32_t txfreq)
{
    lora_config.TXF3 = txfreq;
    RegionCN470SetFrq(2, txfreq);
}

uint32_t lora_config_tx3_get(void)
{
    return lora_config.TXF3;
}
void lora_config_tx4_set(uint32_t txfreq)
{
    lora_config.TXF4 = txfreq;
    RegionCN470SetFrq(3, txfreq);
}

uint32_t lora_config_tx4_get(void)
{
    return lora_config.TXF4;
}
void lora_config_tx5_set(uint32_t txfreq)
{
    lora_config.TXF5 = txfreq;
    RegionCN470SetFrq(4, txfreq);
}

uint32_t lora_config_tx5_get(void)
{
    return lora_config.TXF5;
}
void lora_config_tx6_set(uint32_t txfreq)
{
    lora_config.TXF6 = txfreq;
    RegionCN470SetFrq(5, txfreq);
}

uint32_t lora_config_tx6_get(void)
{
    return lora_config.TXF6;
}
void lora_config_tx7_set(uint32_t txfreq)
{
    lora_config.TXF7 = txfreq;
    RegionCN470SetFrq(6, txfreq);
}

uint32_t lora_config_tx7_get(void)
{
    return lora_config.TXF7;
}
void lora_config_tx8_set(uint32_t txfreq)
{
    lora_config.TXF8 = txfreq;
    RegionCN470SetFrq(7, txfreq);
}

uint32_t lora_config_tx8_get(void)
{
    return lora_config.TXF8;
}

void lora_config_adr_set(uint8_t value)
{
    lora_config.AdrEnable = value;
}

void lora_config_txp_set(uint8_t value)
{
    lora_config.TxPower = value;
}

void lora_config_rx2_dr(uint8_t value)
{
    lora_config.RxDatarate = value;
}

void lora_config_join_delay1_set(uint32_t value)
{
    lora_config.JoinDleay1 = value;
}

void lora_config_join_delay2_set(uint32_t value)
{
    lora_config.JoinDleay2 = value;
}

void lora_config_rx1_delay_set(uint32_t value)
{
    lora_config.Rx1Dleay = value;
}

void lora_config_rx2_delay_set(uint32_t value)
{
    lora_config.Rx2Dleay = value;
}

void lora_config_class_set(uint8_t value)
{
    lora_config.Class = value;
}

void lora_config_public_network_set(uint8_t value)
{
    lora_config.EnablePublicNetwork = value;
}

uint8_t lora_config_adr_get(void)
{
    return lora_config.AdrEnable;
}

void lora_config_save(void)
{
	  MibRequestConfirm_t mib;

		mib.Type = MIB_DEV_ADDR;
		LoRaMacMibGetRequestConfirm(&mib);

		lora_config.DevAddr = mib.Param.DevAddr;
	
    setFlashData((uint8_t*)&lora_config, sizeof(lora_config));
}

void lora_config_load(void)
{
    lora_configuration_t lora_config_temp;
    memset(&lora_config_temp, 0, sizeof(lora_config_temp));
    getFlashData((uint8_t*)&lora_config_temp, sizeof(lora_config_temp));
    if(lora_config_temp.Flag == FLAG_VALUE)
    {
        memcpy((uint8_t*)&lora_config, (uint8_t*)&lora_config_temp, sizeof(lora_config_temp));
        PRINTF("lora_config_load\r\n");
    }
}

/* Dummy data sent periodically to let the tester respond with start test command*/
static TimerEvent_t TxcertifTimer;

void OnCertifTimer( void)
{
    uint8_t Dummy[1]= {1};
    lora_AppData_t AppData;
    AppData.Buff=Dummy;
    AppData.BuffSize=sizeof(Dummy);
    AppData.Port = 224;

    LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

void lora_wan_certif( void )
{
    LORA_Join( );
    TimerInit( &TxcertifTimer,  OnCertifTimer); /* 5s */
    TimerSetValue( &TxcertifTimer,  5000); /* 5s */
    TimerStart( &TxcertifTimer );

}


//#define DATA_ADDRESS 0x0800F800
#define FLASH_USER_START_ADDR   0x0800F800   /* flash???? */
#define FLASH_USER_END_ADDR     0x0800FC00   /* flash???? */

#define BLOCK_SIZE (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR)
uint8_t getFlashData(uint8_t *data, uint32_t data_size)
{
    uint32_t real_size;
    uint8_t u8Return = 0;
    uint8_t *u8Data;
    uint8_t isValid;
    uint32_t i,j;
    real_size = ((data_size + 3) / 4) * 4;
    for(i = 0  ; i < BLOCK_SIZE ; i+= real_size)
    {
        isValid = 0;
        for(j =0; j<real_size; j++)
        {
            u8Data = (uint8_t*)(FLASH_USER_START_ADDR + i + j);
            if(*u8Data != 0x00)
            {
                u8Data = (uint8_t*)(FLASH_USER_START_ADDR + i );
                memcpy(data, u8Data, data_size);
                isValid = 1;
                u8Return = 1;
                break;
            }
        }
        if(!isValid)
        {
            //u8Data = (uint8_t*)(DATA_ADDRESS + i );
            break;
        }
    }
    return u8Return;
}


void setFlashData(uint8_t *data, uint32_t data_size)
{
    uint32_t real_size;
    uint8_t *u8Data;
    uint8_t isValid;
    uint32_t i,j;
    real_size = ((data_size + 3) / 4) * 4;

    for(i = 0  ; i < BLOCK_SIZE ; i+= real_size)
    {
        isValid = 1;
        for(j =0; j<real_size; j++)
        {
            u8Data = (uint8_t*)(FLASH_USER_START_ADDR + i + j);
            if(*u8Data != 0x00)
            {
                isValid = 0;
                break;
            }
        }
        if(isValid)
        {
            for(j=0; j<real_size; j+=4)
            {
                uint32_t *writedata;
                writedata = (uint32_t*)(data+j);
                HAL_FLASH_Unlock();
                HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_USER_START_ADDR + i + j,*writedata);
                HAL_FLASH_Lock();
            }
            break;
        }
    }
    if(!isValid)
    {
        FLASH_EraseInitTypeDef f;
        uint32_t PageError = 0;
        f.TypeErase = FLASH_TYPEERASE_PAGES;
        f.PageAddress = FLASH_USER_START_ADDR;
        f.NbPages = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR + 1) >> 7;
        HAL_FLASH_Unlock();
        HAL_FLASHEx_Erase(&f, &PageError);
        HAL_FLASH_Lock();
        for(j=0; j<real_size; j+=4)
        {
            uint32_t *writedata;
            writedata = (uint32_t*)(data+j);
            HAL_FLASH_Unlock();
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_USER_START_ADDR + j,*writedata);
            HAL_FLASH_Lock();
        }
    }

}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

