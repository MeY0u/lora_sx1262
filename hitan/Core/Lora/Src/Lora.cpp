#include "../Inc/Lora.h"

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
    .rxTimeout = NULL,
    .rxError = NULL,
    .cadDone = NULL,
};

volatile AppStates_t State = SEND_PACKET;

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

bool proceed = true;

void Lora_init()
{
	Radio.Init();
	SetConfiguration(&radioConfiguration);
	ConfigureGeneralRadio(&Radio, &radioConfiguration);
}

void Lora_Operation_TX()
{
	Lora_init();
	// initialize transmitter
	printf("-->TX mode\n");
	State = SEND_PACKET;
	while(true){
		RunTXStateMachine();
	}
}

// let's keep the state machine because waiting for transmission to be done
// will take multiple instances of the state machine running. We could use a
// for loop, but the state machine seems g
void RunTXStateMachine(){
    switch(State){
        case SEND_PACKET:{
           if (proceed == true) {
                // wait for timer to say it's fine to proceed with transmission
                proceed = false;
                PrepareBuffer(&Radio, messageToSend);
                ConfigureRadioTx(&Radio, &radioConfiguration);
                /*
                uint32_t value = Radio.ReadReg(0x0889);
                value = value | 0x04;
                Radio.WriteReg(0x0889, value);
                 */
                Radio.SetTx(radioConfiguration.txTimeout);
                HAL_GPIO_WritePin(SX_LED_TX_GPIO_Port, SX_LED_TX_Pin, GPIO_PIN_SET); // Turn LED On
                State = WAIT_SEND_DONE;
            }
            break;
        }

        case WAIT_SEND_DONE:{
            if(radioFlags.txDone){
                // transmission successful, wait
                radioFlags.txDone = false;  // reset interrupted flag
                printf("TX done\r\n");
                State = SEND_PACKET;
                HAL_Delay(200);
            }
            if(radioFlags.txTimeout){
                // transmission failed, try again!
                radioFlags.txTimeout = false;  // reset interrupted flag
                printf("Tx Timeout\n\r" );
                State = SEND_PACKET;
            }
            break;
        }
    }
}

void ConfigureRadioTx(SX126xHal *radio, RadioConfigurations_t *config){
    radio->SetDioIrqParams(config->irqTx, config->irqTx, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
}

void PrepareBuffer(SX126xHal *radio, const Messages_t *messageToSend){
    radio->SetPayload((uint8_t*)messageToSend, MESSAGE_SIZE);
}

void OnTxDone( void )
{
	proceed = true;
    radioFlags.txDone = true;
    HAL_GPIO_WritePin(SX_LED_TX_GPIO_Port, SX_LED_TX_Pin, GPIO_PIN_RESET); // Turn LED On
}

void OnRxDone( void )
{
	proceed = true;
    radioFlags.rxDone= true;
    HAL_GPIO_WritePin(SX_LED_RX_GPIO_Port, SX_LED_RX_Pin, GPIO_PIN_RESET); // Turn LED On
}

void OnTxTimeout( void )
{
    radioFlags.txTimeout = true;
    proceed = true;
    HAL_GPIO_WritePin(SX_LED_TX_GPIO_Port, SX_LED_TX_Pin, GPIO_PIN_RESET); // Turn LED On
}

void SetConfiguration(RadioConfigurations_t *config){
    config->irqRx = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT;
    config->irqTx = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
    config->rfFrequency = RF_FREQUENCY;
    config->txTimeout = 50000;
    config->rxTimeout = (uint32_t)(RX_TIMEOUT_US / 15.625);
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
