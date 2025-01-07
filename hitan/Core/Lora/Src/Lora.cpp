#include "../Inc/Lora.h"
#include <stm32u5xx_hal_def.h>
/*!
 * Radio events function pointer
 */
static RadioCallbacks_t RadioEvents = {
    .txDone = &OnTxDone,
    .rxDone = &OnRxDone,
    .rxPreambleDetect = NULL,
    .rxSyncWordDone = NULL,
    .rxHeaderDone = NULL,
    .txTimeout = &OnTxTimeout,
    .rxTimeout = &OnRxTimeout,
    .rxError = &OnRxError,
    .cadDone = NULL,
};

volatile AppStates_t TxState = SEND_PACKET;
volatile AppStates_t RxState = RECEIVE_PACKET;

RadioConfigurations_t radioConfiguration;
RadioFlags_t radioFlags = {
    .txDone = false,
    .txTimeout = false,
};

//const Messages_t *messageToSend = &PingMsg;

SX126xHal Radio(&hspi1, SX_SPI1_CS_GPIO_Port, SX_SPI1_CS_Pin,
				SX_BUSY_GPIO_Port, SX_BUSY_Pin, SX_DIO1_GPIO_Port, SX_DIO1_Pin,
				NULL, 0, NULL, 0, // Represent DIO2 and DIO3
				SX_RESET_GPIO_Port, SX_RESET_Pin, SX_Mode_FRx_Pin, SX_Mode_SX126X_Pin,
				SX_ANT_SW_GPIO_Port, SX_ANT_SW_Pin, &RadioEvents);

uint8_t RxBufferSize = RX_BUFFER_SIZE;
uint8_t RxBuffer[RX_BUFFER_SIZE];
int8_t RssiValue = 0;
int8_t SnrValue = 0;
int8_t TxCounter = 0;
bool new_data = false;
void Lora_init()
{
	Radio.Init();
	SetConfiguration(&radioConfiguration);
	ConfigureGeneralRadio(&Radio, &radioConfiguration);
}

//void Lora_Operation_RX()
//{
//	uint8_t uart_buf;
//	Lora_init();
//	RxState = RECEIVE_PACKET;
//	ConfigureRadioRx(&Radio, &radioConfiguration);
//	Radio.SetRx(radioConfiguration.rxTimeout);
//	while(true){
//		HAL_UART_Receive(hcom_uart,&uart_buf,1,HAL_MAX_DELAY);
//		if(uart_buf != 0 && new_data){
////		if(uart_buf != 0){
//			__disable_irq();
//			new_data = false;
//	    	printf("*0x%x:0x%x:0000|0x%x:%d:%d:%d#\n\r",ANCHOR_NUM, tag_dest,ANCHOR_NUM,RssiValue,SnrValue, RxBuffer[0]);
//	    	__enable_irq();
//		}
//		else{
//	    	printf("*0x%x:0x%x:0000#\n\r",ANCHOR_NUM, tag_dest);
//		}
//	}
//}
//
//
//void RunRXStateMachine(){
//	switch(RxState){
//		case RECEIVE_PACKET:{ // holds time i give IRQ time for a break to use UART
//			RxState = WAIT_RECEIVE_DONE;
//			break;
//		}
//		case WAIT_RECEIVE_DONE:{
//			if(radioFlags.rxDone == true){
//				radioFlags.rxDone = false;
//				Radio.GetPayload( RxBuffer, &RxBufferSize, RX_BUFFER_SIZE );
//				//printf("RX = %s\n\r",RxBuffer);
//				RssiValue = Radio.GetRssiInst();
//				GetRssiSnr(&RssiValue, &SnrValue);
//				new_data = true;
//					    	//printf("%d , %d \n", RssiValue,SnrValue);
//				RxState = RECEIVE_PACKET;
//	        }
//			if(radioFlags.rxTimeout == true){
//				radioFlags.rxTimeout = false;
//				RxState = RECEIVE_PACKET;
//	        }
//			break;
//		}

//	    case PACKET_RECEIVED:{
//	    	Radio.GetPayload( RxBuffer, &RxBufferSize, RX_BUFFER_SIZE );
//	    	//printf("RX = %s\n\r",RxBuffer);
//	    	RssiValue = Radio.GetRssiInst();
//	    	GetRssiSnr(&RssiValue, &SnrValue);
//	    	new_data = true;
//	    	//printf("%d , %d \n", RssiValue,SnrValue);
//	    	RxState = RECEIVE_PACKET;
//	    	break;
//	    }
//	}
//}
//void RunRXStateMachine(){
//	switch(RxState){
//		case RECEIVE_PACKET:{
//			RxState = WAIT_RECEIVE_DONE;
//			break;
//		}
//		case WAIT_RECEIVE_DONE:{
//			if(radioFlags.rxDone == true){
//				radioFlags.rxDone = false;
//				RxState = PACKET_RECEIVED;
//	        }
//			if(radioFlags.rxTimeout == true){
//				radioFlags.rxTimeout = false;
//				RxState = RECEIVE_PACKET;
//	        }
//			break;
//		}
//
//	    case PACKET_RECEIVED:{
//	    	Radio.GetPayload( RxBuffer, &RxBufferSize, RX_BUFFER_SIZE );
//	    	//printf("RX = %s\n\r",RxBuffer);
//	    	RssiValue = Radio.GetRssiInst();
//	    	GetRssiSnr(&RssiValue, &SnrValue);
//	    	new_data = true;
//	    	//printf("%d , %d \n", RssiValue,SnrValue);
//	    	RxState = RECEIVE_PACKET;
//	    	break;
//	    }
//	}
//}

		//// Quick fix for daniel - Liron
void Lora_Operation_RX()
{
	Lora_init();
	RxState = RECEIVE_PACKET;
	while(true){
		RunRXStateMachine();
	}
}

void RunRXStateMachine(){
	static uint8_t uart_buf = 0;
	static uint8_t led = 0;
	switch(RxState){
		case RECEIVE_PACKET:{
			ConfigureRadioRx(&Radio, &radioConfiguration);
			Radio.SetRx(radioConfiguration.rxTimeout);
			RxState = WAIT_RECEIVE_DONE;
			break;
		}
		case WAIT_RECEIVE_DONE:{
			if(radioFlags.rxDone == true){
				if(++led % 2)
					HAL_GPIO_WritePin(SX_LED_TX_GPIO_Port, SX_LED_TX_Pin, GPIO_PIN_SET); // Turn LED ON
				else
				{
					led = 0;
					HAL_GPIO_WritePin(SX_LED_TX_GPIO_Port, SX_LED_TX_Pin, GPIO_PIN_RESET); // Turn LED ON
				}
				radioFlags.rxDone = false;
				RxState = PACKET_RECEIVED;

	        }
			if(radioFlags.rxTimeout == true){
				radioFlags.rxTimeout = false;
				RxState = RECEIVE_PACKET;
	        }
			break;
		}

	    case PACKET_RECEIVED:{
	    	Radio.GetPayload( RxBuffer, &RxBufferSize, RX_BUFFER_SIZE );
	    	RssiValue = Radio.GetRssiInst();
	    	GetRssiSnr(&RssiValue, &SnrValue);
	    	RxState = RECEIVE_PACKET;
	    	HAL_UART_Receive(hcom_uart,&uart_buf,1,HAL_MAX_DELAY);
	    	if(uart_buf != 0x0){
				HAL_GPIO_WritePin(SX_LED_TX_GPIO_Port, SX_LED_TX_Pin, GPIO_PIN_RESET); // Turn LED OFF
	    		uart_buf = 0;
	    		__disable_irq();

	    		printf("*0x%x:0x%x:0000|0x%x:%d:%d:%d#\n\r",ANCHOR_NUM, tag_dest,ANCHOR_NUM,RssiValue,SnrValue,RxBuffer[0]);
	    		__enable_irq();
	    	}

	    	break;
	    }
	}
}
void Lora_Operation_TX()
{
	Lora_init();

	// initialize transmitter
	TxState = SEND_PACKET;

	while(true){
//		HAL_Delay(50);
		RunTXStateMachine();
	}
}

void RunTXStateMachine(){
	//HAL_Delay(50);
	static uint8_t index = 0;
	if (index == MAX_INDEX)
		index = 0; //0 = Daniel, //1 = Chen
	Messages_t PingMsg = {index};


	Messages_t *messageToSend = &PingMsg;
    switch(TxState){
        case SEND_PACKET:{
        	PrepareBuffer(&Radio, messageToSend);
        	ConfigureRadioTx(&Radio, &radioConfiguration);
			Radio.SetTx(radioConfiguration.txTimeout);
        	printf("Sent = %d\n\r",index);
        	index++;
			HAL_GPIO_WritePin(SX_LED_TX_GPIO_Port, SX_LED_TX_Pin, GPIO_PIN_SET); // Turn LED On
        	TxState = WAIT_SEND_DONE;
       // 	HAL_Delay(0);
            break;
        }

        case WAIT_SEND_DONE:{
            if(radioFlags.txDone){
                // transmission successful, wait
                radioFlags.txDone = false;  // reset interrupted flag
                TxState = SEND_PACKET;
            }
            if(radioFlags.txTimeout){
                // transmission failed, try again!
                radioFlags.txTimeout = false;  // reset interrupted flag
                TxState = SEND_PACKET;

            }
            break;
        }
    }
}

void ConfigureRadioTx(SX126xHal *radio, RadioConfigurations_t *config){
    radio->SetDioIrqParams(config->irqTx, config->irqTx, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
}

void ConfigureRadioRx(SX126xHal *radio, RadioConfigurations_t *config){
    radio->SetDioIrqParams(config->irqRx, config->irqRx, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
}

void PrepareBuffer(SX126xHal *radio, const Messages_t *messageToSend){
    radio->SetPayload((uint8_t*)messageToSend, MESSAGE_SIZE);
}

void OnRxDone( void )
{
    HAL_GPIO_WritePin(SX_LED_RX_GPIO_Port, SX_LED_RX_Pin, GPIO_PIN_SET); // Turn LED On
    radioFlags.rxDone= true;
	Radio.GetPayload( RxBuffer, &RxBufferSize, RX_BUFFER_SIZE );
    HAL_GPIO_WritePin(SX_LED_RX_GPIO_Port, SX_LED_RX_Pin, GPIO_PIN_RESET); // Turn LED On
//	RunRXStateMachine();


}

void OnRxTimeout( void )
{
    radioFlags.rxTimeout = true;
}

void OnRxError( IrqErrorCode_t errCode )
{
    radioFlags.rxError = true;
}

void OnTxDone( void )
{
    radioFlags.txDone = true;
    HAL_GPIO_WritePin(SX_LED_TX_GPIO_Port, SX_LED_TX_Pin, GPIO_PIN_RESET); // Turn LED On
}

void OnTxTimeout( void )
{
    radioFlags.txTimeout = true;
    HAL_GPIO_WritePin(SX_LED_TX_GPIO_Port, SX_LED_TX_Pin, GPIO_PIN_RESET); // Turn LED On
}

void SetConfiguration(RadioConfigurations_t *config){
    config->irqRx = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT;
    config->irqTx = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
    config->rfFrequency = RF_FREQUENCY;
    config->rxTimeout = RX_TIMEOUT_US;
    config->txPower = TX_OUTPUT_POWER;
    config->txRampTime = RADIO_RAMP_10_US;
    config->packetType = PACKET_TYPE_LORA;
    config->modParams.PacketType = PACKET_TYPE_LORA;
    config->modParams.Params.LoRa.Bandwidth = LORA_BANDWIDTH;
    config->modParams.Params.LoRa.CodingRate = LORA_CODINGRATE;
    config->modParams.Params.LoRa.LowDatarateOptimize = LORA_LOWDATARATEOPTIMIZE;
    config->modParams.Params.LoRa.SpreadingFactor = LORA_SPREADING_FACTOR;
    config->packetParams.PacketType = PACKET_TYPE_LORA;
    config->packetParams.Params.LoRa.CrcMode = LORA_CRC_MODE;
    config->packetParams.Params.LoRa.HeaderType = LORA_HEADER_TYPE;
    config->packetParams.Params.LoRa.InvertIQ = LORA_IQ;
    config->packetParams.Params.LoRa.PayloadLength = BUFFER_SIZE;
    config->packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
}

void ChangeConfigurations(RadioConfigurations_t *config, RadioLoRaBandwidths_t bw, RadioLoRaCodingRates_t cr, RadioLoRaSpreadingFactors_t sf){
	config->modParams.Params.LoRa.Bandwidth = bw;
	config->modParams.Params.LoRa.CodingRate = cr;
    config->modParams.Params.LoRa.SpreadingFactor = sf;
}
void ConfigureGeneralRadio(SX126xHal *radio, RadioConfigurations_t *config){
    radio->SetPacketType(config->packetType);
    radio->SetPacketParams(&config->packetParams);
    radio->SetModulationParams(&config->modParams);
    radio->SetRfFrequency(config->rfFrequency);
    radio->SetTxParams(config->txPower, config->txRampTime);
    radio->SetInterruptMode();
}

/**
 *
  * @brief  EXTI line falling detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Falling_Callback could be implemented in the user file
   */
  if(GPIO_Pin == SX_DIO1_Pin){
	  Radio.InvokeHandler();
	  ConfigureRadioRx(&Radio, &radioConfiguration);
	  Radio.SetRx(radioConfiguration.rxTimeout);
  }
}

void GetRssiSnr(int8_t *rssi, int8_t *snr)
{
    PacketStatus_t pkt_stat;
    Radio.GetPacketStatus(&pkt_stat);
    *rssi = pkt_stat.Params.LoRa.RssiPkt;
    *snr = pkt_stat.Params.LoRa.SnrPkt;

}
