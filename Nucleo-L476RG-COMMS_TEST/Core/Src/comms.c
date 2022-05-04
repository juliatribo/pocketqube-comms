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

/*	THINGS TO REVISE IN THIS CODE
 * 	RF SWITCH connections are done on the opposite order
 * 	We have to program the DIO2 on the contrary
 *
 *
 *
 *
 *
 *
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
uint16_t txCounter = 0;
uint8_t Memory[MEMORY_SIZE] = {'H','O','L','A',',','S','O','C',' ','E','L',' ','D',
    		'A','N','I','E','L',' ','D','E',' ','C','O','M','M','S','.','T','H','I',
			'S',' ','I','S',' ','T','H','E',' ','1',' ','P','A','C','K','E','T','.',
			'H','O','L','A',',','S','O','C',' ','E','L',' ','D',
			'A','N','I','E','L',' ','D','E',' ','C','O','M','M','S','.','T','H','I',
			'S',' ','I','S',' ','T','H','E',' ','2',' ','P','A','C','K','E','T','.',
			'H','O','L','A',',','S','O','C',' ','E','L',' ','D',
			'A','N','I','E','L',' ','D','E',' ','C','O','M','M','S','.','T','H','I',
			'S',' ','I','S',' ','T','H','E',' ','3',' ','P','A','C','K','E','T','.',
			'H','O','L','A',',','S','O','C',' ','E','L',' ','D',
			'A','N','I','E','L',' ','D','E',' ','C','O','M','M','S','.','T','H','I',
			'S',' ','I','S',' ','T','H','E',' ','4',' ','P','A','C','K','E','T','.',
			'*','*','*','*','*','*','*','*','*','*','*','*','*','*','*','*','*','*',
};

uint8_t tx_non_stop = 1; //1 => yes ; 0 => not
uint8_t testRX = false;

/**
 * Main application entry point.
 */
void prueba( void )
{
    uint16_t PacketCnt = 0, i=0;
    float Per = 0.0;
    uint16_t bucleCounter = 0;
    uint16_t defaultCounter = 0;
    uint16_t rxCounter = 0;
    uint16_t cadCounter = 0;
    uint16_t failCADCounter = 0;
    uint16_t test_counter = 0;
    uint16_t tx_count = 0;

    uint8_t MemoryRX[MEMORY_RX_SIZE] = {'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
    		'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/'
    };

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
    State = TX;

    while(  i < NB_TRY )
    {
		//DelayMs( 300 );
    	bucleCounter = bucleCounter + 1;
    	/*
    	printf("Send Packet n %d \r\n",PacketCnt);
		txCounter = txCounter + 1;
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
		}<
		else
		{
			PacketCnt ++;
		}
		//Send Frame
		DelayMs( 100 );
		Radio.Send( Buffer, 6 );
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		DelayMs( 300 );*/
    	if (tx_non_stop == 1){
        	DelayMs( 300 );
    	}else{
    		DelayMs( 1 );
    	}
        switch( State )
        {
            case RX_TIMEOUT:
            {
#if(FULL_DBG)
                printf( "RX Timeout\r\n");
#endif
                RxTimeoutCnt++;
                //State = START_CAD;
                //Radio.Rx( RX_TIMEOUT_VALUE );	//Basic RX code
				//DelayMs(1);	//Basic RX code
                Radio.Standby;
				State = LOWPOWER;
                break;
            }
            case RX_ERROR:
            {
#if(FULL_DBG)
                printf( "RX Error\r\n");
#endif
                RxErrorCnt++;
                PacketReceived = false;
                //State = START_CAD;
                //Radio.Rx( RX_TIMEOUT_VALUE );	//Basic RX code
				//DelayMs(1);	//Basic RX code
                Radio.Standby;
				State = LOWPOWER;
            break;
            }
            case RX:
            {
            	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
                if( PacketReceived == true )
                {
                	if (testRX == 1){
                		test_counter = test_counter + 1;
            			for (uint8_t i=0; i<BUFFER_SIZE; i++){
            				MemoryRX[i + rxCounter*BUFFER_SIZE] = Buffer[i];
            			}
            			rxCounter = rxCounter + 1;
            			if (MEMORY_RX_SIZE < ( rxCounter*BUFFER_SIZE + BUFFER_SIZE )){
            				rxCounter = 0;
            			}
            			testRX = 0;
            			if (Buffer[0] == 'T' && Buffer[1] == 'X'){
            				//txNonStop = true;
            				//txNonStopNum = 1;
            				tx_non_stop = 1;
            			}
                	}

                    PacketReceived = false;     // Reset flag
                    //State = START_CAD;
                    //Radio.Rx( RX_TIMEOUT_VALUE );	//Basic RX code
					//DelayMs(1);	//Basic RX code
					State = LOWPOWER;
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
                        //Radio.Rx( RX_TIMEOUT_VALUE );   // CAD is detected, Start RX
                    }
                    else
                    {
                        TimerStart(&CADTimeoutTimer);   // Start the CAD's Timer
                    }
                    Radio.Rx( RX_TIMEOUT_VALUE );	//Basic RX code
					DelayMs(1);	//Basic RX code
                    State = LOWPOWER;
                }
                break;
            }
            case TX:
            {
                printf("Send Packet n %d \r\n",PacketCnt);
                // Send the next frame
                txfunction();
                tx_count = tx_count + 1;
                //Send Frame
                DelayMs( 1 );
                Radio.Send( Buffer, BUFFER_SIZE );
                //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

                State = LOWPOWER;
                break;
            }
            case TX_TIMEOUT:
            {
                State = LOWPOWER;
                break;
            }
            /*case START_CAD:
            {
            	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            	cadCounter = cadCounter + 1;
                i++;    // Update NbTryCnt
                TimerStop(&RxAppTimeoutTimer);  // Stop the Rx's Timer
                // Trace for debug
                if(CadRx == CAD_FAIL)
                {
                	failCADCounter = failCADCounter + 1;
#if(FULL_DBG)
                printf("No CAD detected\r\n");
#endif
                }
                CadRx = CAD_FAIL;           // Reset CAD flag
                //DelayMs(randr(10,500));     //Add a random delay for the PER test
#if(FULL_DBG)
                printf("CAD %d\r\n",i);
#endif
                Radio.StartCad( );          //StartCad Again
                //Radio.Rx( 0 );	//Basic RX code
				//DelayMs(1);	//Basic RX code
                State = LOWPOWER;
            break;
            }*/
            case LOWPOWER:
            default:
            	defaultCounter = defaultCounter + 1;
                //State = RX;
            	if (tx_non_stop == 1){
            		State = TX;
            	}else{
            		State = RX;
            	}
                // Set low power
                break;
        }

        TimerLowPowerHandler( );
        // Process Radio IRQ
        Radio.IrqProcess( );
    }

}

void txfunction( void ){
	for (uint8_t i = 0; i<BUFFER_SIZE; i=i+1){
		Buffer[i] = Memory[i+txCounter*BUFFER_SIZE];
	}
	txCounter = txCounter + 1;
	if (MEMORY_SIZE < ( txCounter*BUFFER_SIZE + BUFFER_SIZE )){
		txCounter = 0;
		tx_non_stop = 0;
	}
}

void OnTxDone( void )
{
    Radio.Standby( );
    if (tx_non_stop == 1){
        State = TX;
    } else{
        State = LOWPOWER;
    }
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Standby( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    uint8_t RXactual[BufferSize];
    memcpy( RXactual, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    PacketReceived = true;
    RssiMoy = (((RssiMoy * RxCorrectCnt) + RssiValue) / (RxCorrectCnt + 1));
    SnrMoy = (((SnrMoy * RxCorrectCnt) + SnrValue) / (RxCorrectCnt + 1));
    State = RX;
    testRX = 1;
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
//#if(FULL_DBG)
//        printf(".");
//#endif
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
    //SX126xSetCadParams( cadSymbolNum, cadDetPeak, cadDetMin, LORA_CAD_ONLY, ((cadTimeout * 1000) / 15.625 ));
    SX126xSetCadParams( cadSymbolNum, cadDetPeak, cadDetMin, LORA_CAD_RX, ((cadTimeout * 1000) / 15.625 ));
    //THE TOTAL CAD TIMEOUT CAN BE EQUAL TO RX TIMEOUT (IT SHALL NOT BE HIGHER THAN 4 SECONDS
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

