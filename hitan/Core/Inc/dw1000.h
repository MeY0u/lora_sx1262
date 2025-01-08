/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   Single-sided two-way ranging (SS TWR) initiator example code
 *
 *           This is a simple code example which acts as the initiator in a SS TWR distance measurement exchange. This application sends a "poll"
 *           frame (recording the TX time-stamp of the poll), after which it waits for a "response" message from the "DS TWR responder" example
 *           code (companion to this application) to complete the exchange. The response message contains the remote responder's time-stamps of poll
 *           RX, and response TX. With this data and the local time-stamps, (of poll TX and response RX), this example application works out a value
 *           for the time-of-flight over-the-air and, thus, the estimated distance between the two devices, which it writes to the LCD.
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
//#ifdef EX_06A_DEF
#ifndef DW_H
#define DW_H

#include <stdio.h>
#include <string.h>

#include "stdio.h"
#include "string.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "./decadriver/deca_device_api.h"
#include "./decadriver/deca_regs.h"
//#include "ucpd.h"
#include "./platform/deca_spi.h"
#include "./platform/port.h"

/* Example application name and version to display on LCD screen. */
#define APP_NAME "SS TWR INIT v1.3"

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 0

extern COM_InitTypeDef BspCOMInit;
/* Default communication configuration. We use here EVK1000's mode 4. See NOTE 1 below. */
static dwt_config_t config = {
    2,                //Channel number.
    DWT_PRF_64M,      //Pulse repetition frequency.
	DWT_PLEN_256,     //Preamble length. Used in TX only.
    DWT_PAC8,         //Preamble acquisition chunk size. Used in RX only.
    9,                //TX preamble code. Used in TX only.
    9,                //RX preamble code. Used in RX only.
    0,                //0 to use standard SFD, 1 to use non-standard SFD.
    DWT_BR_6M8,       //Data rate.
    DWT_PHRMODE_STD,  //PHY header mode.
    (256 + 1 + 8 - 8)     //SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
};

/*static dwt_config_t config = {
    2,                Channel number.
    DWT_PRF_16M,      Pulse repetition frequency.
	DWT_PLEN_1024,     Preamble length. Used in TX only.
	DWT_PAC64,         Preamble acquisition chunk size. Used in RX only.
    3,                TX preamble code. Used in TX only.
    3,                RX preamble code. Used in RX only.
    0,                0 to use standard SFD, 1 to use non-standard SFD.
	DWT_BR_850K,       Data rate.
    DWT_PHRMODE_STD,  PHY header mode.
    (1025 + 16 - 64)     SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
};*/


#define TX_ANT_DLY 16550
#define RX_ANT_DLY 16500


#define ANCHOR_NUM 2

/* Frames used in the ranging process. See NOTE 3 below. */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, ANCHOR_NUM, ANCHOR_NUM, 'X', 'X', 0xE0, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'X', 'X', ANCHOR_NUM, ANCHOR_NUM, 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define TAG_DEST_POLL_IDX7 7
#define TAG_DEST_POLL_IDX8 8
#define TAG_DEST_RESP_IDX5 5
#define TAG_DEST_RESP_IDX6 6
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4

#define MAX_POWER 0x1F1F1F1FUL
#define LEDS_ON	1
#define LEDS_OFF 0
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2  s and 1  s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 0
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 1400//3500

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* String used to display measured distance on LCD screen (16 characters maximum). */
//char dist_str[16] = {0};\

/* Declaration of static functions. */

int	dwmInit();
int dwRange(void);
void dwConfig(dwt_config_t *config);
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);

#ifdef __cplusplus
}
#endif

#endif

