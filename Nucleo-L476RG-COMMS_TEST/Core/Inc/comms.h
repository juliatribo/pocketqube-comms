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

#include "flash.h"
#include "definitions.h"

#define RF_FREQUENCY                       	868000000 // Hz

#define TX_OUTPUT_POWER                     22        // dBm

#define LORA_BANDWIDTH                      0         // [0: 125 kHz,
													  //  1: 250 kHz,
													  //  2: 500 kHz,
													  //  3: Reserved]
#define LORA_SPREADING_FACTOR               11         // [SF7..SF12]
#define LORA_CODINGRATE                     1         // [1: 4/5,
													  //  2: 4/6,
													  //  3: 4/7,
													  //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                8//108    // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                 100       // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON          false
#define LORA_IQ_INVERSION_ON                false
#define LORA_FIX_LENGTH_PAYLOAD_LEN         19
#define WINDOW_SIZE							20

#define RX_TIMEOUT_VALUE                    4000
#define BUFFER_SIZE							100

#define TLE_PACKET_SIZE						66
#define TELEMETRY_PACKET_SIZE				34
#define CALIBRATION_PACKET_SIZE				96
#define CONFIG_PACKET_SIZE					30

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
#define CAD_DET_PEAK            20
#define CAD_DET_MIN             10
#define CAD_TIMEOUT_MS          2000
#define NB_TRY                  10



#define UPLINK_BUFFER_SIZE		100
#define ACK_PAYLOAD_LENGTH		5			//ACK payload data length
#define CONFIG_SIZE				13

//OTHER
#define ML 							(TELEMETRY_PACKET_SIZE + 3 + NPAR)
#define BLOCK_LENGTH_RS 			255
#define MIN_DISTANCE_RS 			32
#define MESSAGE_LENGTH_RS 			223

#define RATE_CON  					2
#define ORDER_CON 					7

#define BLOCK_ROW_INTER 			4
#define BLOCK_COL_INTER 			4

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


void StateMachine( void );

void txfunction( void );

void configuration(void);

void process_telecommand(uint8_t header, uint8_t info);

bool pin_correct(uint8_t pin_1, uint8_t pin_2);

void tx_beacon(void);

void comms_timmer(void);

void xTaskNotify(uint8_t noti);

void interleave(unsigned char *codeword, int size);

void deinterleave(unsigned char *codeword_interleaved, int size);

int encode (uint8_t* Buffer, uint8_t* conv_encoded);

void decode(uint8_t* data, size_t length);

#endif /* INC_COMMS_H_ */

