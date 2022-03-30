/*
 * comms.c
 *
 *  Created on: 23 feb. 2022
 *      Author: Daniel Herencia Ruiz
 */



/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: CAD performance evaluation test

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Benjamin Boulet
*/

#include "comms.h"




typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
    START_CAD,
}States_t;

typedef enum
{
    CAD_FAIL,
    CAD_SUCCESS,
    PENDING,
}CadRx_t;



uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

#if(RX_FW == TX_FW)
    #error "Please define only one firmware."
#endif
CadRx_t CadRx = CAD_FAIL;
bool PacketReceived = false;
bool RxTimeoutTimerIrqFlag = false;
uint16_t channelActivityDetectedCnt = 0;
uint16_t RxCorrectCnt = 0;
uint16_t RxErrorCnt = 0;
uint16_t RxTimeoutCnt = 0;
uint16_t SymbTimeoutCnt = 0;
int16_t RssiMoy = 0;
int8_t SnrMoy = 0;

/**
 * Main application entry point.
 */
void prueba( void )
{
    uint16_t PacketCnt = 0, i=0;
    float Per = 0.0;

    // Target board initialization
    BoardInitMcu( );
    BoardInitPeriph( );

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    RadioEvents.CadDone = OnCadDone;

    //Timer used to restart the CAD
    TimerInit( &CADTimeoutTimer, CADTimeoutTimeoutIrq );
    TimerSetValue( &CADTimeoutTimer, CAD_TIMER_TIMEOUT );

    //App timmer used to check the RX's end
    TimerInit( &RxAppTimeoutTimer, RxTimeoutTimerIrq );
    TimerSetValue( &RxAppTimeoutTimer, RX_TIMER_TIMEOUT );

    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );

//#if defined( USE_MODEM_LORA )

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
/*
#elif defined( USE_MODEM_FSK )

    Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                                  FSK_DATARATE, 0,
                                  FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, 0, 3000 );

    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                  0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                  0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                                  0, 0,false, true );

#else
//    #error "Please define a frequency band in the compiler options."
#endif
*/

    printf("\r\n=============================\r\n");
    printf("SX126X Cad Performance starts\r\n");
    printf("=============================\r\n\r\n");
#if(RX_FW == 1)
    SX126xConfigureCad( CAD_SYMBOL_NUM,
                        CAD_DET_PEAK,CAD_DET_MIN,
                        CAD_TIMEOUT_MS);            // Configure the CAD
                        Radio.StartCad( );          // do the config and lunch first CAD
#if(FULL_DBG)
    printf("CAD\r\n");
#endif
#else
    State = TX;
#endif

    while(  i < NB_TRY )
    {
        switch( State )
        {
            case RX_TIMEOUT:
            {
#if(FULL_DBG)
                printf( "RX Timeout\r\n");
#endif
                RxTimeoutCnt++;
                State = START_CAD;
                break;
            }
            case RX_ERROR:
            {
#if(FULL_DBG)
                printf( "RX Error\r\n");
#endif
                RxErrorCnt++;
                PacketReceived = false;
                State = START_CAD;
            break;
            }
            case RX:
            {
                if( PacketReceived == true )
                {
                    PacketReceived = false;     // Reset flag
                    if((Buffer[0]=='C') && (Buffer[1]=='A') && (Buffer[2]=='D'))
                    {
                        PacketCnt = (Buffer[4] << 8) + Buffer[5];   // ID packet
                        RxCorrectCnt++;         // Update RX counter
#if(FULL_DBG)
                        printf( "Rx Packet n %d\r\n", PacketCnt );
#endif
                    }
                State = START_CAD;
                }
                else
                {
                    if (CadRx == CAD_SUCCESS)
                    {
                        channelActivityDetectedCnt++;   // Update counter
#if(FULL_DBG)
                        printf( "Rxing\r\n");
#endif
                        RxTimeoutTimerIrqFlag = false;
                        TimerReset(&RxAppTimeoutTimer);	// Start the Rx's's Timer
                        Radio.Rx( RX_TIMEOUT_VALUE );   // CAD is detected, Start RX
                    }
                    else
                    {
                        TimerStart(&CADTimeoutTimer);   // Start the CAD's Timer
                    }
                    State = LOWPOWER;
                }
                break;
            }
            case TX:
            {
                printf("Send Packet n %d \r\n",PacketCnt);

                // Send the next frame
                Buffer[0] = 'C';
                Buffer[1] = 'A';
                Buffer[2] = 'D';
                Buffer[3] = '0';
                Buffer[4] = PacketCnt>>8;
                Buffer[5] = (uint8_t)PacketCnt ;

                if( PacketCnt == 0xFFFF)
                {
                    PacketCnt = 0;
                }
                else
                {
                    PacketCnt ++;
                }
                //Send Frame
                DelayMs( 1 );
                Radio.Send( Buffer, 6 );

                State = LOWPOWER;
                break;
            }
            case TX_TIMEOUT:
            {
                State = LOWPOWER;
                break;
            }
            case START_CAD:
            {
                i++;    // Update NbTryCnt
                TimerStop(&RxAppTimeoutTimer);  // Stop the Rx's Timer
                // Trace for debug
                if(CadRx == CAD_FAIL)
                {
#if(FULL_DBG)
                printf("No CAD detected\r\n");
#endif
                }
                CadRx = CAD_FAIL;           // Reset CAD flag
                DelayMs(randr(10,500));     //Add a random delay for the PER test
#if(FULL_DBG)
                printf("CAD %d\r\n",i);
#endif
                Radio.StartCad( );          //StartCad Again
                State = LOWPOWER;
            break;
            }
            case LOWPOWER:
            default:
                // Set low power
                break;
        }

        TimerLowPowerHandler( );
        // Process Radio IRQ
        Radio.IrqProcess( );
    }

    //Result
    Per =  100 - (((float)RxCorrectCnt / i) * 100);
    printf("CAD Performance Result :\r\n");
    printf("CAD SYMBOL = %d | Det_Peak = %d | Det_Min = %d \r\n",(int)pow(2,CAD_SYMBOL_NUM),CAD_DET_PEAK,CAD_DET_MIN);
    printf("Nb try :%d \r\n",i);
    printf("CAD detected : %d \r\nRX Correct : %d \r\n",channelActivityDetectedCnt,RxCorrectCnt);
    printf("RSSI moy : %d dBm\r\nSNR moy : %d \r\n",RssiMoy,SnrMoy);
//    printf("PER : %.1f \r\n",Per);
    printf("Nb RxError :%d \r\nNb RxTimout : %d \r\nNbSymbolTimeout : %d \r\n",RxErrorCnt,RxTimeoutCnt,SymbTimeoutCnt);
    printf("TEST END   \r\n \r\n");

    //Reset All results for next test
    Per = 0.0;
    RxCorrectCnt = 0;
    channelActivityDetectedCnt = 0;
    RxErrorCnt = 0;
    RxTimeoutCnt = 0;
    SymbTimeoutCnt = 0;
    i=0;
}

void OnTxDone( void )
{
    Radio.Standby( );
    State = TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Standby( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    PacketReceived = true;
    RssiMoy = (((RssiMoy * RxCorrectCnt) + RssiValue) / (RxCorrectCnt + 1));
    SnrMoy = (((SnrMoy * RxCorrectCnt) + SnrValue) / (RxCorrectCnt + 1));
    State = RX;
}

void OnTxTimeout( void )
{
    Radio.Standby( );
    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    Radio.Standby( );
    if( RxTimeoutTimerIrqFlag )
    {
        State = RX_TIMEOUT;
    }
    else
    {
#if(FULL_DBG)
        printf(".");
#endif
        Radio.Rx( RX_TIMEOUT_VALUE );   //  Restart Rx
        SymbTimeoutCnt++;               //  if we pass here because of Symbol Timeout
        State = LOWPOWER;
    }
}

void OnRxError( void )
{
    Radio.Standby( );
    State = RX_ERROR;
}

void OnCadDone( bool channelActivityDetected)
{
    Radio.Standby( );

    if( channelActivityDetected == true )
    {
        CadRx = CAD_SUCCESS;
    }
    else
    {
        CadRx = CAD_FAIL;
    }
    State = RX;
}

void SX126xConfigureCad( RadioLoRaCadSymbols_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin , uint32_t cadTimeout)
{
    SX126xSetDioIrqParams( 	IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED, IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED,
                            IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    SX126xSetCadParams( cadSymbolNum, cadDetPeak, cadDetMin, LORA_CAD_ONLY, ((cadTimeout * 1000) / 15.625 ));
}

static void CADTimeoutTimeoutIrq( void )
{
    Radio.Standby( );
    State = START_CAD;
}

static void RxTimeoutTimerIrq( void )
{
    RxTimeoutTimerIrqFlag = true;
}

