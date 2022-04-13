/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       radio.h
 * \brief      Generic radio driver ( radio abstraction )
 *
 * \version    2.0.B2 
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Gregory Cristian on Apr 25 2013
 */
#ifndef __RADIO_H__
#define __RADIO_H__
#include"PUBLIC\Std_Types.h"
#include "protocol\Proto_NetComm.h"
/*!
 * SX1272 and SX1276 General parameters definition
 */
#define LORA                                        1         // [0: OFF, 1: ON]
#define LORA_FREQ_CHANNEL_MAX        14//                    25

#define LORA_FREQ_TXCHANNEL         0//                    25
#define LORA_FREQ_RXCHANNEL         1//                    25

/*!
 * RF process function return codes
 */
typedef enum
{
    RF_IDLE,
    RF_BUSY,
    RF_RX_DONE,
    RF_RX_TIMEOUT,
    RF_TX_DONE,
    RF_TX_TIMEOUT,
    RF_LEN_ERROR,
    RF_CHANNEL_EMPTY,
    RF_CHANNEL_ACTIVITY_DETECTED,
}tRFProcessReturnCodes;



typedef enum
{
    RAIDO_STATE_IDLE,
    RAIDO_STATE_RXING,
    RAIDO_STATE_TX_RUNNING,
    RAIDO_STATE_CAD_RUNNING,
    RAIDO_STATE_SLEEP_INIT,
}Radio_States;

/*!
 * Radio driver structure defining the different function pointers
 */
typedef struct sRadioDriver
{
    void ( *Init )( void );
    void ( *Reset )( void );
    void ( *StartRx )( void );
    void ( *GetRxPacket )( void *buffer, uint16_t *size );
    void ( *SetTxPacket )( const void *buffer, uint16_t size );
    uint32_t ( *Process )( void );
}tRadioDriver;



typedef struct {
    uint8 eercode;
    uint8 bw;
    uint8 rsf;
    uint8 ssf;
}Radio_RfSpeed;

typedef struct
{
    uint8 rfreq;// 中心频点
    uint8 Power;// 功率
    uint8 Speed;// 功率
    void (*ConnectCallBack)(void);
    void (*ProcCallBack)(Proto_NetcommType nettype,uint8* pindat,uint16 lenth);
}Radio_ParaType;

typedef struct
{
    uint16    Lenth;
    uint8       *pData;
} Radio_ComType;


typedef enum
{
    RADIO_SET_SLEEP= 0,
    RADIO_SET_CAD_DET,
    RADIO_SET_RX,
    RADIO_SET_TX,
    RADIO_SCAN_PROC,
    RADIO_HEAD_LENTH,
    RADIO_CMD_MAX
} Radio_CmdType;


#define RADIO_CHECK            STD_FALSE

extern const uint32 Radio_LoraFreqTable[LORA_FREQ_CHANNEL_MAX][2];
extern const Radio_RfSpeed Radio_RfSpeedTable[];
extern const Dr_OpertonType RadioDrive;
extern void Radio_Check(void);
extern void Radio_TickInit(void);


#endif // __RADIO_H__
