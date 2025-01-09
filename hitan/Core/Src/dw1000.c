#include "dw1000.h"
#include "main.h"
extern LoraRxInfo_t rxInfo;


int	dwmInit(){
	/* Reset and initialise DW1000.
	     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
	     * performance. */
	    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
	    port_set_dw1000_slowrate();
	    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
	    {
	        //lcd_display_str("INIT FAILED");
	        return 0;
	    }

	    //dwt_setlnapamode(0x03); //While using PA LNA Enable
	    return 1;
}

void dwConfig(dwt_config_t *config){
	dwt_setlnapamode(0x03); //While using PA LNA Enable


	    port_set_dw1000_fastrate();

	    /* Configure DW1000. See NOTE 6 below. */
	    dwt_configure(config);
//	    dwt_configure(&config);

	    /* Apply default antenna delay value. See NOTE 2 below. */
	    dwt_setrxantennadelay(RX_ANT_DLY);
	    dwt_settxantennadelay(TX_ANT_DLY);


	    //printf("0x04 = %x\n\r",dwt_read32bitreg(SYS_CFG_ID));
	    dwt_setsmarttxpower(DISABLE);
	    //printf("0x04 = %x\n\r",dwt_read32bitreg(SYS_CFG_ID));
	    dwt_write32bitreg(TX_POWER_ID,MAX_POWER); //max power refer user manual pg.106
	    //printf("0x1E = %x\n\r",dwt_read32bitreg(TX_POWER_ID));
	    dwt_setleds(LEDS_ON);

	    if(config->prf==DWT_PRF_16M){ //set lower SNR treshould for NLOS operation
		   dwt_write8bitoffsetreg(LDE_IF_ID, LDE_CFG1_OFFSET, 0x07); //refer user manual pg.175
		   dwt_write16bitoffsetreg(LDE_IF_ID, LDE_CFG2_OFFSET, 0x0003); //refer user manual pg.177
		}
	    /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
	     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
	    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
	    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

}
int dwRange(void)
{
	uint8_t tag_dest;
	HAL_UART_Receive(hcom_uart,(uint8_t*)&tag_dest,sizeof(tag_dest),HAL_MAX_DELAY);
	//WDT_Refresh(); //Refresh the Watchdog counter

	tx_poll_msg[TAG_DEST_POLL_IDX7] = tag_dest;
	tx_poll_msg[TAG_DEST_POLL_IDX8] = tag_dest;
	rx_resp_msg[TAG_DEST_RESP_IDX6] = tag_dest;
	rx_resp_msg[TAG_DEST_RESP_IDX5] = tag_dest;
	//HAL_GPIO_TogglePin(TIMING_GPIO_GPIO_Port, TIMING_GPIO_Pin);
	/* Write frame data to DW1000 and prepare transmission. See NOTE 7 below. */
	tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

	/* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
	 * set by dwt_setrxaftertxdelay() has elapsed. */
	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

	/* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
		//WDT_Refresh();
	};

	/* Increment frame sequence number after transmission of the poll message (modulo 256). */
	frame_seq_nb++;

	if (status_reg & SYS_STATUS_RXFCG) {
		uint32 frame_len;
		/* Clear good RX frame event in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

		/* A frame has been received, read it into the local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
		if (frame_len <= RX_BUF_LEN) {
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}

		/* Check that the frame is the expected response from the companion "SS TWR responder" example.
		 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
		rx_buffer[ALL_MSG_SN_IDX] = 0;
		if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {
			uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
			int32 rtd_init, rtd_resp;
			float clockOffsetRatio ;

			/* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
			poll_tx_ts = dwt_readtxtimestamplo32();
			resp_rx_ts = dwt_readrxtimestamplo32();

			/* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
			clockOffsetRatio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_2 / 1.0e6) ;

			/* Get timestamps embedded in response message. */
			resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
			resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

			/* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
			rtd_init = resp_rx_ts - poll_tx_ts;
			rtd_resp = resp_tx_ts - poll_rx_ts;

			tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
			distance = tof * SPEED_OF_LIGHT;
			if (rxInfo.new_data){
				__disable_irq();
				printf("*0x%x:0x%x:%3.2f|0x%x:%d:%d:%d#\n\r",ANCHOR_NUM, tag_dest, distance,ANCHOR_NUM,(int)rxInfo.rssi,(int)rxInfo.snr, rxInfo.buffer[0]);
				rxInfo.new_data = false;
				__enable_irq();
			}
			else {
				__disable_irq();
				printf("*0x%x:0x%x:%3.2f#\n\r",ANCHOR_NUM, tag_dest, distance);
				__enable_irq();
			}
		}
		else{
			if (rxInfo.new_data){
				__disable_irq();
				printf("*0x%x:0x%x:0000|0x%x:%d:%d:%d#\n\r",ANCHOR_NUM, tag_dest,ANCHOR_NUM,(int)rxInfo.rssi, (int)rxInfo.snr, rxInfo.buffer[0]);
				rxInfo.new_data = false;
				__enable_irq();
			}
			else {
				__disable_irq();
				printf("*0x%x:0x%x:0000#\n\r",ANCHOR_NUM, tag_dest);
				__enable_irq();
			}
		}
	}
	else {

		/* Clear RX error/timeout events in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
		if (rxInfo.new_data){
				__disable_irq();
				printf("*0x%x:0x%x:0000|0x%x:%d:%d:%d#\n\r",ANCHOR_NUM, tag_dest,ANCHOR_NUM,(int)rxInfo.rssi, (int)rxInfo.snr, rxInfo.buffer[0]);
				rxInfo.new_data = false;

				__enable_irq();
			}
			else {
				__disable_irq();
				printf("*0x%x:0x%x:0000#\n\r",ANCHOR_NUM, tag_dest);
				__enable_irq();
			}
		/* Reset RX to properly reinitialize LDE operation. */
		dwt_rxreset();
	}

	/* Execute a delay between ranging exchanges. */
	//Sleep(RNG_DELAY_MS);
	//HAL_GPIO_TogglePin(TIMING_GPIO_GPIO_Port, TIMING_GPIO_Pin);

}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn resp_msg_get_ts()
 *
 * @brief Read a given timestamp value from the response message. In the timestamp fields of the response message, the
 *        least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to get
 *         ts  timestamp value
 *
 * @return none
 */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < RESP_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}
//#endif
/***************************************************
 * NOTES:
 *
 * 1. The single-sided two-way ranging scheme implemented here has to be considered carefully as the accuracy of the distance measured is highly
 *    sensitive to the clock offset error between the devices and the length of the response delay between frames. To achieve the best possible
 *    accuracy, this response delay must be kept as low as possible. In order to do so, 6.8 Mbps data rate is used in this example and the response
 *    delay between frames is defined as low as possible. The user is referred to User Manual for more details about the single-sided two-way ranging
 *    process.  NB:SEE ALSO NOTE 11.
 * 2. The sum of the values is the TX to RX antenna delay, this should be experimentally determined by a calibration process. Here we use a hard coded
 *    value (expected to be a little low so a positive error will be seen on the resultant distance estimate. For a real production application, each
 *    device should have its own antenna delay properly calibrated to get good precision when performing range measurements.
 * 3. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
 *    following:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 4 below.
 *     - byte 7/8: source address, see NOTE 4 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10 -> 13: poll message reception timestamp.
 *     - byte 14 -> 17: response message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 4. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete response frame sent by the responder at the
 *    6.8M data rate used (around 200  s).
 * 6. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 7. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 9. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *    subtraction.
 * 10. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW1000 API Guide for more details on the DW1000 driver functions.
 * 11. The use of the carrier integrator value to correct the TOF calculation, was added Feb 2017 for v1.3 of this example.  This significantly
 *     improves the result of the SS-TWR where the remote responder unit's clock is a number of PPM offset from the local inmitiator unit's clock.
 *     As stated in NOTE 2 a fixed offset in range will be seen unless the antenna delsy is calibratred and set correctly.
 *
 **************************************************/
