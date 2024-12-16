#ifndef LORA_H
#define LORA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Sx126x_Lib/radio.hpp"
#include "Sx126x_Lib/sx126x.hpp"
#include "Sx126x_Lib/sx126x-hal.hpp"
#include "main.h"

/*
 *  Global variables declarations
 */
typedef enum
{
    SEND_PACKET,
    WAIT_SEND_DONE,
}AppStates_t;

typedef struct{
    RadioPacketTypes_t packetType;
    int8_t txPower;
    RadioRampTimes_t txRampTime;
    ModulationParams_t modParams;
    PacketParams_t packetParams;
    uint32_t rfFrequency;
    uint16_t irqTx;
    uint16_t irqRx;
    uint32_t txTimeout;
    uint32_t rxTimeout;
}RadioConfigurations_t;

typedef struct{
    bool rxDone;
    bool rxError;
    bool txDone;
    bool rxTimeout;
    bool txTimeout;
}RadioFlags_t;

extern SPI_HandleTypeDef hspi1;

#define MESSAGE_SIZE 4
typedef uint8_t Messages_t[MESSAGE_SIZE];
const Messages_t PingMsg = {'P', 'I', 'N', 'G'};

/* Set this flag to '1' to use the LoRa modulation or to '0' to use FSK modulation */
#define USE_MODEM_LORA  1
#define USE_MODEM_FSK   !USE_MODEM_LORA

#define RF_FREQUENCY                                    868000000 // Hz
#define TX_OUTPUT_POWER                                 22         // max 22 dBm

#define LORA_BANDWIDTH                              LORA_BW_500         // [0: 125 kHz,
                                                                  //  1: 250 kHz,
                                                                  //  2: 500 kHz,
                                                                  //  3: Reserved]
#define LORA_SPREADING_FACTOR                       LORA_SF7         // [SF7..SF12]
#define LORA_LOWDATARATEOPTIMIZE                    0
#define LORA_CODINGRATE                             LORA_CR_4_5         // [1: 4/5,
                                                                  //  2: 4/6,
                                                                  //  3: 4/7,
                                                                  //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        16         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_HEADER_TYPE                            LORA_PACKET_VARIABLE_LENGTH
#define LORA_FHSS_ENABLED                           false
#define LORA_NB_SYMB_HOP                            4
#define LORA_IQ                                     LORA_IQ_NORMAL
#define LORA_CRC_MODE                               LORA_CRC_OFF

#define RX_TIMEOUT_US 200000
#define BUFFER_SIZE                                     16        // Define the payload size here

void Lora_init( void );
void Lora_Operation_TX( void );
void PrepareBuffer(SX126xHal *radio, const Messages_t *messageToSend);
void ConfigureRadioTx(SX126xHal *radio, RadioConfigurations_t *config);
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( void );
void SetConfiguration(RadioConfigurations_t *config);
void ConfigureGeneralRadio(SX126xHal *radio, RadioConfigurations_t *config);
void RunTXStateMachine( void );

#ifdef __cplusplus
}
#endif

#endif
