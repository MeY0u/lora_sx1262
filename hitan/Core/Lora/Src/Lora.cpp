#include "../Inc/Lora.h"

/*!
 * Radio events function pointer
 */
static RadioCallbacks_t RadioEvents = {
    .txDone = NULL,
    .rxDone = &OnRxDone,
    .rxPreambleDetect = NULL,
    .rxSyncWordDone = NULL,
    .rxHeaderDone = NULL,
    .txTimeout = NULL,
    .rxTimeout = &OnRxTimeout,
    .rxError = &OnRxError,
    .cadDone = NULL,
};

volatile AppStates_t RxState = RECEIVE_PACKET;

RadioConfigurations_t radioConfiguration;
RadioFlags_t radioFlags = {
    .txDone = false,
    .txTimeout = false,
};

const Messages_t *messageToSend = &PingMsg;

SX126xHal Radio(&hspi1, SX_SPI1_CS_GPIO_Port, SX_SPI1_CS_Pin,
				SX_BUSY_GPIO_Port, SX_BUSY_Pin, SX_DIO1_GPIO_Port, SX_DIO1_Pin,
				NULL, 0, NULL, 0, // Represent DIO2 and DIO3
				SX_RESET_GPIO_Port, SX_RESET_Pin, SX_Mode_FRx_Pin, SX_Mode_SX126X_Pin,
				SX_ANT_SW_GPIO_Port, SX_ANT_SW_Pin, &RadioEvents);

uint8_t RxBufferSize = RX_BUFFER_SIZE;
uint8_t RxBuffer[RX_BUFFER_SIZE];
int8_t RssiValue = 0;
int8_t SnrValue = 0;

void Lora_init()
{
	Radio.Init();
	SetConfiguration(&radioConfiguration);
	ConfigureGeneralRadio(&Radio, &radioConfiguration);
}

void Lora_Operation_RX()
{
	Lora_init();
	RxState = RECEIVE_PACKET;
	while(true){
		RunRXStateMachine();
	}
}

void RunRXStateMachine(){
	switch(RxState){
		case RECEIVE_PACKET:{
			ConfigureRadioRx(&Radio, &radioConfiguration);
			Radio.SetRx(radioConfiguration.rxTimeout);
			RxState = WAIT_RECEIVE_DONE;
			break;
		}
		case WAIT_RECEIVE_DONE:{
			if(radioFlags.rxDone == true){
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
	    	/*
	    	printf("Rssi : %u SNR: %u", RssiValue, SnrValue);
	    	for (size_t i = 0; i < RX_BUFFER_SIZE; i++) {
	    		printf("Element %zu: %u\n", i, RxBuffer[i]); // %u for unsigned int
	    	}
			*/
	    	RxState = RECEIVE_PACKET;
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
    printf("Rx done\n");
    HAL_GPIO_WritePin(SX_LED_RX_GPIO_Port, SX_LED_RX_Pin, GPIO_PIN_RESET); // Turn LED On
}

void OnRxTimeout( void )
{
    radioFlags.rxTimeout = true;
}

void OnRxError( IrqErrorCode_t errCode )
{
    radioFlags.rxError = true;
}

void SetConfiguration(RadioConfigurations_t *config){
    config->irqRx = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT;
    config->irqTx = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
    config->rfFrequency = RF_FREQUENCY;
    config->txTimeout = 50000;
    //config->rxTimeout = (uint32_t)(RX_TIMEOUT_US / 15.625);
    config->rxTimeout = 50000;
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
  }
}

void GetRssiSnr(int8_t *rssi, int8_t *snr)
{
    PacketStatus_t pkt_stat;
    Radio.GetPacketStatus(&pkt_stat);
    *rssi = pkt_stat.Params.LoRa.RssiPkt;
    *snr = pkt_stat.Params.LoRa.SnrPkt;
}
