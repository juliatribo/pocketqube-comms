/*!
 * \file      comms.c
 *
 * \brief     Comms subsytem functions
 *
 *
 *
 * \code
 *
 * 				 _______    ______    ____    ____    ____    ____     ______
 * 				/ ______)  /  __  \  |    \  /    |  |    \  /    |   / _____)
 * 			   / /         | |  | |  |  \  \/  /  |  |  \  \/  /  |  ( (____
 *            ( (          | |  | |  |  |\    /|  |  |  |\    /|  |   \____ \
 *             \ \______   | |__| |  |  | \__/ |  |  |  | \__/ |  |   _____) )
 *              \_______)  \______/  |__|      |__|  |__|      |__|  (______/
 *
 *
 * \endcode
 *
 * \author    Daniel Herencia
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
			'S',' ','I','S',' ','T','H','E',' ',

			'1',' ','P','A','C','K','E','T','.','H','O','L','A',',','S','O','C',' ','E','L',' ','D',
			'A','N','I','E','L',' ','D','E',' ','C','O','M','M','S','.','T','H','I',

			'S',' ','I','S',' ','T','H','E',' ','2',' ','P','A','C','K','E','T','.',
			'H','O','L','A',',','S','O','C',' ','E','L',' ','D','A','N','I','E','L',' ','D','E',' ',

			'C','O','M','M','S','.','T','H','I','S',' ','I','S',' ','T','H','E',' ','3',' ','P','A','C','K','E','T','.',
			'H','O','L','A',',','S','O','C',' ','E','L',' ','D',

			'A','N','I','E','L',' ','D','E',' ','C','O','M','M','S','.','T','H','I',
			'S',' ','I','S',' ','T','H','E',' ','4',' ','P','A','C','K','E','T','.','*','*','*','*',

			'E','N','D','*','*','*','*','*','*','*','*','*','*','*',
};

uint8_t tx_non_stop = 1; //1 => yes ; 0 => not
uint8_t testRX = false;



uint8_t calib_packets = 0;			//Counter of the calibration packets received
uint8_t tle_packets = 0;			//Counter of the tle packets received
uint8_t telemetry_packets = 0;		//Counter of telemetry packets sent


/*
 * To avoid the variables being erased if a reset occurs, we have to store them in the Flash memory
 * Therefore, they have to be declared as a single-element array
 */
uint8_t count_packet[] = {0};		//To count how many packets have been sent (maximum WINDOW_SIZE)
uint8_t count_window[] = {0};		//To count the window number
uint8_t count_rtx[] = {0};			//To count the number of retransmitted packets

uint8_t i = 0;						//variable for loops
uint8_t j=0;						//variable for loops
uint8_t k=0;						//variable for loops

uint64_t ack;						//Information rx in the ACK (0 => ack, 1 => nack)
uint8_t nack_number;				//Number of the current packet to retransmit
bool nack;							//True when retransmission necessary
bool full_window;					//Stop & wait => to know when we reach the limit packet of the window
bool statemach = true;				//If true, comms workflow follows the state machine. This value should be controlled by OBC
									//Put true before activating the statemachine thread. Put false before ending comms thread


bool send_data = false;				//If true, the state machine send packets every airtime
bool send_telemetry = false;		//If true, we have to send telemetry packets instead of payload data
uint8_t num_telemetry = 0;			//Total of telemetry packets that have to be sent (computed when telecomand send telemetry received)
bool contingency = false;			//True if we are in contingency state => only receive



/**************************************************************************************
 *                                                                                    *
 * 	Function:  configuration		                                                  *
 * 	--------------------                                                              *
 * 	function to configure the transceiver and the transmission protocol parameters    *
 *                                                                                    *
 *  returns: nothing									                              *
 *                                                                                    *
 **************************************************************************************/

void configuration(void){

    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
    uint8_t count_packet1[] = {0};		//To count how many packets have been sent (maximum WINDOW_SIZE)
    uint8_t count_window1[] = {0};		//To count the window number
    uint8_t count_rtx1[] = {0};			//To count the number of retransmitted packets
    uint8_t count_packet2[] = {4};		//To count how many packets have been sent (maximum WINDOW_SIZE)
    uint8_t count_window2[] = {5};		//To count the window number
    uint8_t count_rtx2[] = {6};			//To count the number of retransmitted packets

    SX126xConfigureCad( CAD_SYMBOL_NUM,CAD_DET_PEAK,CAD_DET_MIN,CAD_TIMEOUT_MS);            // Configure the CAD

    /*
    uint32_t provaa = 3;
    uint32_t provaa2 = 0;
    //Write_Flash(0x08008200,&provaa,1);

    DelayMs( 300 );
    //Write_Flash( 0x08008200, &count_packet2, sizeof(count_packet));
    DelayMs( 300 );
    //Write_Flash( COUNT_WINDOW_ADDR , &count_window2 , sizeof(count_window) );
    DelayMs( 300 );
    //Write_Flash( COUNT_RTX_ADDR , &count_rtx2 , sizeof(count_rtx) );

    DelayMs( 300 );
    //Read_Flash( 0x08008200 , &provaa2 , 1 );	//Read from Flash count_packet
    DelayMs( 300 );
    //Read_Flash( COUNT_WINDOW_ADDR , &count_window1 , sizeof(count_window1) ); 	//Read from Flash count_window
	DelayMs( 300 );
	//Read_Flash( COUNT_RTX_ADDR , &count_rtx1 , sizeof(count_rtx1) ); 			//Read from Flash count_rtx

    DelayMs( 300 );
    //Read_Flash( 0x08008200 , &provaa2 , 1 );	//Read from Flash count_packet
    DelayMs( 300 );
    //Read_Flash( COUNT_WINDOW_ADDR , &count_window1 , sizeof(count_window1) ); 	//Read from Flash count_window
	DelayMs( 300 );
	//Read_Flash( COUNT_RTX_ADDR , &count_rtx1 , sizeof(count_rtx1) ); 			//Read from Flash count_rtx
	*/
}

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

    uint8_t MemoryTX[MEMORY_RX_SIZE] = {'/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/','/',
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

    configuration();

    Radio.StartCad( );          // do the config and lunch first CAD

    //State = TX;

    while(  i < NB_TRY )
    {
		//DelayMs( 300 );
    	bucleCounter = bucleCounter + 1;

    	if (tx_non_stop == 1){
        	DelayMs( 300 );
    	}else{
    		DelayMs( 1 );
    	}
    	//NO SE RX EL PRIMER PAQUETE EN EL CUBECELL. El ultimo no se recive en el stm32
        Radio.IrqProcess( );

        switch( State )
        {
            case RX_TIMEOUT:
            {
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
					test_counter = test_counter + 1;
					for (uint8_t i=0; i<BUFFER_SIZE; i++){
						MemoryRX[i + rxCounter*BUFFER_SIZE] = Buffer[i];
					}
					rxCounter = rxCounter + 1;
					if (MEMORY_RX_SIZE < ( rxCounter*BUFFER_SIZE + BUFFER_SIZE )){
						rxCounter = 0;
					}
					if (Buffer[0] == 'T' && Buffer[1] == 'X'){
						//txNonStop = true;
						//txNonStopNum = 1;
						tx_non_stop = 1;
						DelayMs(1000);
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
                // Send the next frame
                txfunction();
                //Send Frame
                DelayMs( 1 );
                Radio.Send( Buffer, BUFFER_SIZE );
                for (uint8_t i = 0; i<BUFFER_SIZE; i=i+1){
                	MemoryTX[i+tx_count*BUFFER_SIZE] = Buffer[i];
				}
                tx_count = tx_count + 1;
                if (tx_count*BUFFER_SIZE>MEMORY_RX_SIZE){
                	tx_count = 0;
                }
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
        //Radio.IrqProcess( );
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
    SX126xSetCadParams( cadSymbolNum, cadDetPeak, cadDetMin, LORA_CAD_RX, ((cadTimeout * 15.625) / 1000 ));
    //THE TOTAL CAD TIMEOUT CAN BE EQUAL TO RX TIMEOUT (IT SHALL NOT BE HIGHER THAN 4 SECONDS)
}

static void CADTimeoutTimeoutIrq( void )
{
    Radio.Standby( );
    State = LOWPOWER;
    //State = START_CAD;
    //State = RX;
}

static void RxTimeoutTimerIrq( void )
{
    RxTimeoutTimerIrqFlag = true;
}


/**************************************************************************************
 *                                                                                    *
 * 	Function:  process_telecommand                                                    *
 * --------------------                                                               *
 * 	processes the information contained in the packet depending on the telecommand    *
 * 	received																	      *
 *                                                                                    *
 *  header: number of telecommand			                                          *
 *  info: information contained in the received packet								  *
 *                                                                                    *
 *  returns: nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void process_telecommand(uint8_t header, uint8_t info) {
	switch(header) {
	case RESET2:
		HAL_NVIC_SystemReset();
		break;
	case NOMINAL:
		Write_Flash(NOMINAL_ADDR, &info, 1);
		break;
	case LOW:
		Write_Flash(LOW_ADDR, &info, 1);
		break;
	case CRITICAL:
		Write_Flash(CRITICAL_ADDR, &info, 1);
		break;
	case EXIT_LOW_POWER:{
		Write_Flash(EXIT_LOW_POWER_FLAG_ADDR, &info, 1);
		Write_Flash(EXIT_LOW_ADDR, TRUE, 1);
		break;
	}
	case SET_TIME:{
		uint8_t time[4];
		for (k=0; k<4; k++){
			time[k]=Buffer[k+1];
		}
		Write_Flash(TEMP_ADDR, &time, sizeof(time));
		break;
	}
	case SET_CONSTANT_KP:
		Write_Flash(KP_ADDR, &info, 1);
		break;
	case TLE:{
		uint8_t tle[UPLINK_BUFFER_SIZE-1];
		for (k=1; k<UPLINK_BUFFER_SIZE; k++){
			tle[k-1]=Buffer[k];
		}
		Write_Flash(TLE_ADDR + tle_packets*UPLINK_BUFFER_SIZE, &tle, sizeof(tle));
		tle_packets++;
		uint8_t integer_part = (uint8_t) 138/UPLINK_BUFFER_SIZE;
		if (tle_packets == integer_part+1){
			tle_packets = 0;
		}
		break;
	}
	case SET_GYRO_RES:
		/*4 possibles estats, rebrem 00/01/10/11*/
		Write_Flash(GYRO_RES_ADDR, &info, 1);
		break;
	case SEND_DATA:{
		if (!contingency){
			State = TX;
			send_data = true;
		}
		break;
	}
	case SEND_TELEMETRY:{
		if (!contingency){
			send_telemetry = true;
			num_telemetry = (uint8_t) 34/BUFFER_SIZE + 1; //cast to integer to erase the decimal part
			State = TX;
		}
		break;
	}
	case STOP_SENDING_DATA:{
		send_data = false;
		count_packet[0] = 0;
		break;
	}
	case ACK_DATA:{
		//check it
	 	 ack = ack & Buffer[1];
		 for(j=2; j<ACK_PAYLOAD_LENGTH; j++){
			 ack = (ack << 8*j) & Buffer[j];
		 }
		 count_window[0] = 0;
		 full_window = false;
		 if (ack != 0xFFFFFFFFFFFFFFFF){
			 nack = true;
		 }
		 State = TX;
		break;
	}
	case SET_SF_CR: {
		uint8_t SF;
		if (info == 0) SF = 7;
		else if (info == 1) SF = 8;
		else if (info == 2) SF = 9;
		else if (info == 3) SF = 10;
		else if (info == 4) SF = 11;
		else if (info == 5) SF = 12;
		Write_Flash(SF_ADDR, &SF, 1);
		/*4 cases (4/5, 4/6, 4/7,1/2), so we will receive and store 0, 1, 2 or 3*/
		Write_Flash(CRC_ADDR, &Buffer[2], 1);
		break;
	}
	case SEND_CALIBRATION:{	//Rx calibration
		uint8_t calib[UPLINK_BUFFER_SIZE-1];
		for (k=1; k<UPLINK_BUFFER_SIZE; k++){
			calib[k-1]=Buffer[k];
		}
		Write_Flash(CALIBRATION_ADDR, &calib, sizeof(calib));
		calib_packets = calib_packets + 1;
		uint8_t integer_part = (uint8_t) 138/UPLINK_BUFFER_SIZE;
		if(calib_packets == integer_part+1){
			calib_packets = 0;
		}
		break;
	}
	case TAKE_PHOTO:{
		/*GUARDAR TEMPS FOTO?*/
		Write_Flash(PAYLOAD_STATE_ADDR, TRUE, 1);
		Write_Flash(PL_TIME_ADDR, &info, 4);
		Write_Flash(PHOTO_RESOL_ADDR, &Buffer[5], 1);
		Write_Flash(PHOTO_COMPRESSION_ADDR, &Buffer[6], 1);
		break;
	}
	case TAKE_RF:{
		Write_Flash(PAYLOAD_STATE_ADDR, TRUE, 1);
		Write_Flash(PL_TIME_ADDR, &info, 8);
		Write_Flash(F_MIN_ADDR, &Buffer[9], 1);
		Write_Flash(F_MAX_ADDR, &Buffer[10], 1);
		Write_Flash(DELTA_F_ADDR, &Buffer[11], 1);
		Write_Flash(INTEGRATION_TIME_ADDR, &Buffer[12], 1);
		break;
	}
	case SEND_CONFIG:{
		uint8_t config[CONFIG_SIZE];
		Read_Flash(CONFIG_ADDR, &config, CONFIG_SIZE);
		Radio.Send( config, CONFIG_SIZE );
		break;
	}
	}
}

