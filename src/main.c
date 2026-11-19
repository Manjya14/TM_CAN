/*==================================================================================================
* Project : RTD AUTOSAR 4.7
* Platform : CORTEXM
* Peripheral : S32K3XX
* Dependencies : none
*
* Autosar Version : 4.7.0
* Autosar Revision : ASR_REL_4_7_REV_0000
* Autosar Conf.Variant :
* SW Version : 6.0.0
* Build Version : S32K3_RTD_6_0_0_D2506_ASR_REL_4_7_REV_0000_20250610
*
* Copyright 2020 - 2025 NXP
*
* NXP Confidential and Proprietary. This software is owned or controlled by NXP and may only be 
*   used strictly in accordance with the applicable license terms.  By expressly 
*   accepting such terms or by downloading, installing, activating and/or otherwise 
*   using the software, you are agreeing that you have read, and that you agree to 
*   comply with and are bound by, such license terms.  If you do not agree to be 
*   bound by the applicable license terms, then you may not retain, install,
*   activate or otherwise use the software.
==================================================================================================*/

/**
*   @file main.c
*
*   @addtogroup main_module main module documentation
*   @{
*/

/* Including necessary configuration files. */
#include "FlexCAN_Ip.h"
#include "FlexCAN_Ip_Cfg.h"
#include "Stm_Ip.h"
#include "Stm_Ip_SA_PBcfg.h"
#include "Clock_Ip.h"
#include "Clock_Ip_Cfg.h"
#include "Siul2_Port_Ip.h"
#include "Siul2_Port_Ip_Cfg.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Use your exact generated array and count declarations */
extern const Siul2_Port_Ip_PinSettingsConfig g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals[];
#ifndef NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals
  #define NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals (3U)
#endif

/* ============================================================
 * Hardware Peripheral Definitions
 * ============================================================ */
#define CAN_INSTANCE        (0U)
#define CAN_TX_MB           (0U)
#define CAN_RX_MB           (1U)

#define CAN_TX_ID           (0x700U)
#define CAN_RX_ID           (0x708U)

#define STM_INSTANCE        (0U)
#define STM_CHANNEL         (0U)

/* STM ticks for 1 ms period (assumes 48 MHz Platform Clock: 48,000,000 / 1000) */
#define STM_TICKS_1MS       (48000U)
#define TIMER_100MS_PERIOD  (100U)

/* ============================================================
 * CAN-TP Types & State Machine
 * ============================================================ */
typedef enum
{
    TP_TX_IDLE = 0,
    TP_TX_SEND_SF,
    TP_TX_SEND_FF,
    TP_TX_WAIT_FC,
    TP_TX_SEND_CF,
    TP_TX_COMPLETE,
    TP_TX_ERROR
} TP_TxState;

typedef struct
{
    TP_TxState state;
    const uint8_t *data;
    uint16_t length;
    uint16_t index;
    uint8_t sequenceNumber;
    uint8_t blockSize;
    uint8_t blockCounter;
    uint8_t stMin;
    uint32_t lastCfTime;
} CAN_TP_TxType;

/* ============================================================
 * Global Variables
 * ============================================================ */
static CAN_TP_TxType tpTx;
static volatile uint32_t systemTimeMs = 0U;
static volatile bool timer100msFlag = false;

/* RX buffer for FlexCAN polling */
static Flexcan_Ip_MsgBuffType rxMsgBuffer;

/* TX Info Structure */
static Flexcan_Ip_DataInfoType txInfo = {
    .msg_id_type = FLEXCAN_MSG_ID_STD,
    .data_length = 8U,
    .fd_enable   = (boolean)FALSE,
    .enable_brs  = (boolean)FALSE,
    .is_remote   = (boolean)FALSE
};

/* Test Multi-Frame Payload (20 bytes) */
static const uint8_t testData[20] =
{
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
    0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x10,
    0x20, 0x30, 0x40, 0x50
};

/* ============================================================
 * Timer Functions (Direct Hardware Counter Polling)
 * ============================================================ */
static inline uint32_t Timer_GetTimeMs(void)
{
    return systemTimeMs;
}

static void Timer_Poll(void)
{
    static uint32_t lastTickCount = 0U;

    /* Direct read of 32-bit hardware free-running counter */
    uint32_t currentCount = IP_STM_0->CNT;

    if ((currentCount - lastTickCount) >= STM_TICKS_1MS)
    {
        lastTickCount += STM_TICKS_1MS;
        systemTimeMs++;

        if ((systemTimeMs % TIMER_100MS_PERIOD) == 0U)
        {
            timer100msFlag = true;
        }
    }
}

/* ============================================================
 * Low-Level CAN Transmit & Receive (Polling)
 * ============================================================ */
static void CAN_SendFrame(uint32_t id, const uint8_t *data)
{
    FlexCAN_Ip_Send(CAN_INSTANCE, CAN_TX_MB, &txInfo, id, (uint8_t *)data);

    while (FlexCAN_Ip_GetTransferStatus(CAN_INSTANCE, CAN_TX_MB) == FLEXCAN_STATUS_BUSY)
    {
        /* Polling wait */
    }
}

/* ============================================================
 * CAN-TP Implementation
 * ============================================================ */
static void CAN_TP_Init(void)
{
    tpTx.state = TP_TX_IDLE;
    tpTx.data = NULL;
    tpTx.length = 0U;
    tpTx.index = 0U;
    tpTx.sequenceNumber = 1U;
    tpTx.blockSize = 0U;
    tpTx.blockCounter = 0U;
    tpTx.stMin = 0U;
    tpTx.lastCfTime = 0U;
}

static bool CAN_TP_StartTx(const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U) || (tpTx.state != TP_TX_IDLE))
    {
        return false;
    }

    tpTx.data = data;
    tpTx.length = length;
    tpTx.index = 0U;
    tpTx.sequenceNumber = 1U;
    tpTx.blockSize = 0U;
    tpTx.blockCounter = 0U;
    tpTx.stMin = 0U;
    tpTx.lastCfTime = Timer_GetTimeMs();

    if (length <= 7U)
    {
        tpTx.state = TP_TX_SEND_SF;
    }
    else
    {
        tpTx.state = TP_TX_SEND_FF;
    }

    return true;
}

static void CAN_TP_SendSF(void)
{
    uint8_t frame[8];
    memset(frame, 0xAA, sizeof(frame));

    frame[0] = (uint8_t)(tpTx.length & 0x0FU);

    for (uint8_t i = 0U; i < tpTx.length; i++)
    {
        frame[i + 1U] = tpTx.data[i];
    }

    CAN_SendFrame(CAN_TX_ID, frame);
    tpTx.state = TP_TX_COMPLETE;
}

static void CAN_TP_SendFF(void)
{
    uint8_t frame[8];
    memset(frame, 0xAA, sizeof(frame));

    frame[0] = (uint8_t)(0x10U | ((tpTx.length >> 8U) & 0x0FU));
    frame[1] = (uint8_t)(tpTx.length & 0xFFU);

    for (uint8_t i = 0U; i < 6U; i++)
    {
        frame[i + 2U] = tpTx.data[i];
    }

    tpTx.index = 6U;
    CAN_SendFrame(CAN_TX_ID, frame);
    tpTx.state = TP_TX_WAIT_FC;
}

static void CAN_TP_ProcessFC(const uint8_t *data)
{
    uint8_t flowStatus = data[0] & 0x0FU;

    if (flowStatus == 0U) /* Continue to Send */
    {
        tpTx.blockSize = data[1];
        tpTx.stMin = data[2];
        tpTx.blockCounter = 0U;
        tpTx.lastCfTime = Timer_GetTimeMs();
        tpTx.state = TP_TX_SEND_CF;
    }
    else if (flowStatus == 1U) /* Wait Frame */
    {
        tpTx.state = TP_TX_WAIT_FC;
    }
    else /* Error / Abort */
    {
        tpTx.state = TP_TX_ERROR;
    }
}

static void CAN_TP_SendCF(void)
{
    uint8_t frame[8];
    memset(frame, 0xAA, sizeof(frame));

    frame[0] = (uint8_t)(0x20U | (tpTx.sequenceNumber & 0x0FU));

    uint16_t remaining = tpTx.length - tpTx.index;
    uint8_t bytesToSend = (remaining > 7U) ? 7U : (uint8_t)remaining;

    for (uint8_t i = 0U; i < bytesToSend; i++)
    {
        frame[i + 1U] = tpTx.data[tpTx.index + i];
    }

    CAN_SendFrame(CAN_TX_ID, frame);

    tpTx.index += bytesToSend;
    tpTx.sequenceNumber = (tpTx.sequenceNumber + 1U) & 0x0FU;
    tpTx.blockCounter++;
    tpTx.lastCfTime = Timer_GetTimeMs();

    if (tpTx.index >= tpTx.length)
    {
        tpTx.state = TP_TX_COMPLETE;
    }
    else
    {
        if ((tpTx.blockSize != 0U) && (tpTx.blockCounter >= tpTx.blockSize))
        {
            tpTx.state = TP_TX_WAIT_FC;
            tpTx.blockCounter = 0U;
        }
    }
}

static void CAN_TP_Poll(void)
{
    if (FlexCAN_Ip_GetTransferStatus(CAN_INSTANCE, CAN_RX_MB) == FLEXCAN_STATUS_SUCCESS)
    {
        if (rxMsgBuffer.msgId == CAN_RX_ID)
        {
            uint8_t frameType = (rxMsgBuffer.data[0] >> 4U) & 0x0FU;

            if (frameType == 0x03U) /* Flow Control */
            {
                CAN_TP_ProcessFC(rxMsgBuffer.data);
            }
        }

        FlexCAN_Ip_Receive(CAN_INSTANCE, CAN_RX_MB, &rxMsgBuffer, (boolean)TRUE);
    }
}

static void CAN_TP_Process(void)
{
    switch (tpTx.state)
    {
        case TP_TX_IDLE:
            break;

        case TP_TX_SEND_SF:
            CAN_TP_SendSF();
            break;

        case TP_TX_SEND_FF:
            CAN_TP_SendFF();
            break;

        case TP_TX_WAIT_FC:
            break;

        case TP_TX_SEND_CF:
        {
            uint32_t currentTime = Timer_GetTimeMs();
            if ((currentTime - tpTx.lastCfTime) >= tpTx.stMin)
            {
                CAN_TP_SendCF();
            }
            break;
        }

        case TP_TX_COMPLETE:
        case TP_TX_ERROR:
        default:
            tpTx.state = TP_TX_IDLE;
            break;
    }
}

static void Application_100ms_Task(void)
{
    if (tpTx.state == TP_TX_IDLE)
    {
        (void)CAN_TP_StartTx(testData, (uint16_t)sizeof(testData));
    }
}

/* ============================================================
 * Main Function
 * ============================================================ */
int main(void)
{
    /* 1. Clocks Initialization */
    Clock_Ip_Init(&Clock_Ip_aClockConfig[0]);

    /* 2. Initialize Pins using YOUR exact generated array */
    Siul2_Port_Ip_Init(NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals,
                       g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals);

    /* 3. FlexCAN Controller Initialization */
    FlexCAN_Ip_Init(CAN_INSTANCE, &FlexCAN_State0, &FlexCAN_Config0);
    FlexCAN_Ip_ExitFreezeMode(CAN_INSTANCE);

    /* 4. Configure & Arm CAN RX Message Buffer 1 */
    Flexcan_Ip_DataInfoType rxInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = 8U,
        .fd_enable   = (boolean)FALSE,
        .enable_brs  = (boolean)FALSE,
        .is_remote   = (boolean)FALSE
    };
    FlexCAN_Ip_ConfigRxMb(CAN_INSTANCE, CAN_RX_MB, &rxInfo, CAN_RX_ID);
    FlexCAN_Ip_Receive(CAN_INSTANCE, CAN_RX_MB, &rxMsgBuffer, (boolean)TRUE);

    /* 5. STM Timer Hardware Initialization */
    Stm_Ip_Init(STM_INSTANCE, &STM_0_InitConfig_PB);

    /* 6. CAN-TP Layer Init */
    CAN_TP_Init();

    /* 7. Super-Loop */
    while (1)
    {
        /* Poll System Timer */
        Timer_Poll();

        /* Poll CAN RX for Flow Control responses */
        CAN_TP_Poll();

        /* Process Multi-Frame CAN-TP State Machine */
        CAN_TP_Process();

        /* 100 ms Periodic Task Dispatch */
        if (timer100msFlag)
        {
            timer100msFlag = false;
            Application_100ms_Task();
        }
    }

    return 0;
}

/** @} */
