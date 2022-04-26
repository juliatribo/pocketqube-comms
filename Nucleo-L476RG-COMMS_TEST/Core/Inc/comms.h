/*
 * comms.h
 *
 *  Created on: 24 feb. 2022
 *      Author: Daniel Herencia Ruiz
 */

#ifndef INC_COMMS_H_
#define INC_COMMS_H_


#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

#include <string.h>
#include <math.h>
#include "board.h"
#include "sx126x.h"
#include "radio.h"
#include "sx126x-board.h"



//#if defined( USE_BAND_868 )

#define RF_FREQUENCY                                868000000 // Hz

//#elif defined( USE_BAND_915 )

//#define RF_FREQUENCY                                915000000 // Hz

//#else
//    #error "Please define a frequency band in the compiler options."
//#endif

#define TX_OUTPUT_POWER                             22        // dBm

//#if defined( USE_MODEM_LORA )

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8//108       // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         100       // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define LORA_FIX_LENGTH_PAYLOAD_LEN                 19

/*
#elif defined( USE_MODEM_FSK )

#define FSK_FDEV                                    25e3      // Hz
#define FSK_DATARATE                                50e3      // bps
#define FSK_BANDWIDTH                               100e3     // Hz >> DSB in sx126x
#define FSK_AFC_BANDWIDTH                           100e3     // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#else
//    #error "Please define a modem in the compiler options."
#endif
*/

#define RX_TIMEOUT_VALUE                            4000
#define BUFFER_SIZE                                 64 // Define the payload size here




/*!
 *	CAD performance evaluation's parameters
 */
#define RX_FW       1
#define TX_FW       0   //TX_FW is only for test
#define FULL_DBG    1   //Active all traces




// Apps CAD timer
TimerEvent_t CADTimeoutTimer;
#define CAD_TIMER_TIMEOUT       1000        //Define de CAD timer's timeout here

TimerEvent_t RxAppTimeoutTimer;
#define RX_TIMER_TIMEOUT        4000        //Define de CAD timer's timeout here

//CAD parameters
#define CAD_SYMBOL_NUM          LORA_CAD_02_SYMBOL
#define CAD_DET_PEAK            22
#define CAD_DET_MIN             10
#define CAD_TIMEOUT_MS          2000
#define NB_TRY                  10

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );

/*!
 * \brief Function executed on Radio CAD Done event
 */
void OnCadDone( bool channelActivityDetected);

/*!
 * \brief Function configuring CAD parameters
 * \param [in]  cadSymbolNum   The number of symbol to use for CAD operations
 *                             [LORA_CAD_01_SYMBOL, LORA_CAD_02_SYMBOL,
 *                              LORA_CAD_04_SYMBOL, LORA_CAD_08_SYMBOL,
 *                              LORA_CAD_16_SYMBOL]
 * \param [in]  cadDetPeak     Limit for detection of SNR peak used in the CAD
 * \param [in]  cadDetMin      Set the minimum symbol recognition for CAD
 * \param [in]  cadTimeout     Defines the timeout value to abort the CAD activity
 */
void SX126xConfigureCad( RadioLoRaCadSymbols_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin , uint32_t cadTimeout);

/*!
 * \brief CAD timeout timer callback
 */
static void CADTimeoutTimeoutIrq( void );

/*!
 * \brief Rx timeout timer callback
 */
static void RxTimeoutTimerIrq( void );

/*!
 * \brief Average the collected RSSIs during CAD
 */
int8_t AverageCadRssi( void );

/*!
 * \brief Get the last good RSSI during CAD
 */
int8_t GetLastCadRssi( void );

/*!
 * \brief Display collected RSSIs each ms during CAD
 */
void DisplayCadRssivsTime( void );


void prueba( void );




#endif /* INC_COMMS_H_ */
