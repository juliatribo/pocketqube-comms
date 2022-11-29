/*
 * comms.c
 *
 *  Created on: 23 feb. 2022
 *      Author: Daniel Herencia Ruiz
 * \code
 *
 *    _______    ______    ____    ____    ____    ____     ______
 *   / ______)  /  __  \  |    \  /    |  |    \  /    |   / _____)
 *  / /         | |  | |  |  \  \/  /  |  |  \  \/  /  |  ( (____
 * ( (          | |  | |  |  |\    /|  |  |  |\    /|  |   \____ \
 *  \ \______   | |__| |  |  | \__/ |  |  |  | \__/ |  |   _____) )
 *   \_______)  \______/  |__|      |__|  |__|      |__|  (______/
 *
 *
 * \endcode
 */


#include "comms.h"

/************  STATES  *************/

typedef enum                        //Possible States of the State Machine
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
    START_CAD,
}States_t;

typedef enum                        //CAD states
{
    CAD_FAIL,
    CAD_SUCCESS,
    PENDING,
}CadRx_t;

States_t State = LOWPOWER;          //Current state of the State Machine

/************  PACKETS  ************/

uint8_t Buffer[BUFFER_SIZE];        //Tx and Rx Buffer
uint8_t nack[WINDOW_SIZE];          //To store the last ack/nack received
uint8_t last_telecommand[BUFFER_SIZE]; //Last telecommand RX


/*************  FLAGS  *************/

uint8_t error_telecommand = false;  //To transmit an error packet
uint8_t tx_flag = false;            //To allow transmission
uint8_t send_data = false;          //To send data packets to the GS
uint8_t beacon_flag = false;        //To transmit the beacon
uint8_t nack_flag = false;          //Retransmission necessary
uint8_t tle_telecommand = false;    //True when TLE telecommand received
uint8_t telecommand_rx = false;     //To indicate that a telecommand has been received
uint8_t request_execution = false;  //To send the request execution packet
uint8_t protocol_timeout = false;   //True when the protocol timer ends
uint8_t reception_ack_mode = false; //True
uint8_t contingency = false;        //True if we are in contingency state (only RX allowed)
uint8_t ask_data = false;			//True if GS asks to ask another sat the data
uint8_t isGS = true;				//True if telecommand is sent by GS, false if telecommand is sent by another sat



/***********  COUNTERS  ***********/

uint8_t request_counter = 0;        //Number of request execution packets sent (to execute a telecommand order)
uint8_t packet_number = 0;          //Data packet number
uint8_t num_config = 0;             //Configuration packet number
uint8_t num_telemetry = 0;          //Telemetry packet number
uint8_t window_packet = 0;          //TX window number
uint8_t nack_counter;               //Position of the NACK array (packets already retransmitted)
uint8_t k=0;                        //Counter for loops
uint8_t protocol_timeout_counter = 0; //Number of times that the protocol timer (500 ms) has ended
                                      //This is used to have longer timers for higher SF


/********  LoRa PARAMETERS  ********/
uint8_t SF = LORA_SPREADING_FACTOR; //Spreading Factor
uint8_t CR = LORA_CODINGRATE;       //Coding Rate
uint16_t time_packets = 500;        //Time between data packets sent in ms


/*************  OTHER  *************/

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



// VARIABLES FROM OLD CODE
/*uint8_t calib_packets = 0;        //Counter of the calibration packets received
uint8_t tle_packets = 0;            //Counter of the tle packets received
uint8_t telemetry_packets = 0;      //Counter of telemetry packets sent
uint8_t count_packet[] = {0};       //To count how many packets have been sent (maximum WINDOW_SIZE)
uint8_t count_window[] = {0};       //To count the window number
uint8_t count_rtx[] = {0};          //To count the number of retransmitted packets
uint8_t i=0;                        //variable for loops
uint8_t j=0;                        //variable for loops*/
//uint64_t ack;                     //Information rx in the ACK (0 => ack, 1 => nack)
//uint8_t nack_number;              //Number of the current packet to retransmit
//bool nack;                        //True when retransmission necessary
//bool full_window;					//Stop & wait => to know when we reach the limit packet of the window
//bool statemach = true;            //If true, comms workflow follows the state machine. This value should be controlled by OBC
                                    //Put true before activating the statemachine thread. Put false before ending comms thread
//bool send_telemetry = false;      //If true, we have to send telemetry packets instead of payload data
//uint8_t contact_GS = false;       //To avoid TX beacon
//uint8_t packet_to_send;
//uint16_t rx_attemps_counter = 0;  //Instead of timeout with timers, counting iterations
//uint8_t rtx_confirms = 0;	//Maximum 3 retransmissions of execution request


/*************************************************************************
 *                                                                       *
 *  Function:  configuration                                             *
 *  --------------------                                                 *
 *  function to configure the transceiver and the protocol parameters    *
 *                                                                       *
 *  returns: nothing                                                     *
 *                                                                       *
 *************************************************************************/
void configuration(void){

	uint64_t read_variable; //Read flash function requieres variables of 64 bits


	/* Reads the SF, CR and time between packets variables from memory */
	/*Read_Flash(SF_ADDR, &read_variable, 1);
	memcpy(SF, read_variable, sizeof(SF));
	Read_Flash(CRC_ADDR, &read_variable, 1);
	memcpy(CR, read_variable, sizeof(CR));
	Read_Flash(COMMS_TIME_ADDR, &read_variable, 1);
	memcpy(time_packets, read_variable, sizeof(time_packets));*/
	SF = LORA_SPREADING_FACTOR;
	CR = LORA_CODINGRATE;
	time_packets = 500;

	/* Configuration of the LoRa frequency and TX and RX parameters */
    Radio.SetChannel( RF_FREQUENCY );

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, SF, CR,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, SF, CR, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
    /*
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );*/


    /* Configuration of the CAD parameters */
    SX126xConfigureCad( CAD_SYMBOL_NUM,CAD_DET_PEAK,CAD_DET_MIN,CAD_TIMEOUT_MS);
    Radio.StartCad( );      //To initialize the CAD process

    State = RX;             //To initialize in RX state
};


/*************************************************************************
 *                                                                       *
 *  Function:  stateMachine                                              *
 *  --------------------                                                 *
 *  communication process state machine                                  *
 *  States:                                                              *
 *  - RX_TIMEOUT: when the reception ends                                *
 *  - RX_ERROR: when an error in the reception process occurs            *
 *  - RX: when a packet has been received or to start a RX process       *
 *  - TX: to transmit a packet                                           *
 *  - TX_TIMEOUT: when the transmission ends                             *
 *  - LOWPOWER: when the transceiver is not transmitting nor receiving   *
 *                                                                       *
 *  returns: nothing                                                     *
 *                                                                       *
 *************************************************************************/
void StateMachine( void )
{
	/*
    uint16_t PacketCnt = 0, i=0;
    float Per = 0.0;
    uint16_t bucleCounter = 0;
    uint16_t defaultCounter = 0;
    uint16_t rxCounter = 0;
    uint16_t cadCounter = 0;
    uint16_t failCADCounter = 0;
    uint16_t test_counter = 0;
    uint16_t tx_count = 0;

    uint8_t compare_arrays = 0;
	*/
    /* Target board initialization*/
    BoardInitMcu( );
    BoardInitPeriph( );

    /* Radio initialization */
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    RadioEvents.CadDone = OnCadDone;

    /* Timer used to restart the CAD */
    TimerInit( &CADTimeoutTimer, CADTimeoutTimeoutIrq );
    TimerSetValue( &CADTimeoutTimer, CAD_TIMER_TIMEOUT );

    /* App timmer used to check the RX's end */
    TimerInit( &RxAppTimeoutTimer, RxTimeoutTimerIrq );
    TimerSetValue( &RxAppTimeoutTimer, RX_TIMER_TIMEOUT );

    Radio.Init( &RadioEvents );    //Initializes the Radio

    configuration();               //Configures the transceiver

    //while(  i < NB_TRY )
    while( 1 )                     //The only option to end the state machine is killing COMMS thread (by the OBC)
    {
		//DelayMs( 300 );
    	//bucleCounter = bucleCounter + 1;

    	/*if (tx_non_stop == 1){
    		DelayMs( 300 );
    	}else{
    		DelayMs( 1 );
    	}*/
    	DelayMs( 1 );
        Radio.IrqProcess( );       //Checks the interruptions
        //copy_state = State;
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
                if( PacketReceived == true )
                {
					//test_counter = test_counter + 1;
					/*for (uint8_t i=0; i<BUFFER_SIZE; i++){
						MemoryRX[i + rxCounter*BUFFER_SIZE] = Buffer[i];
					}
					rxCounter = rxCounter + 1;
					if (MEMORY_RX_SIZE < ( rxCounter*BUFFER_SIZE + BUFFER_SIZE )){
						rxCounter = 0;
					}*/
					/*if (Buffer[0] == 'T' && Buffer[1] == 'X'){
						//txNonStop = true;
						//txNonStopNum = 1;
						tx_non_stop = 1;
						DelayMs(1000);
					}*/
					if (pin_correct(Buffer[0], Buffer[1]))
					{
						State = LOWPOWER;
						if (Buffer[2] == TLE){
							Stop_timer_16();	//This is not necessary, put here for safety
							if (!tle_telecommand){	//First TLE packet
								tle_telecommand = true;
								State = RX;
								telecommand_rx = true;
							}
							else{	//Is the last TLE packet (there are 2)
								tle_telecommand = false;
								State = LOWPOWER;	//Line unnecessary
								telecommand_rx = false;
							}
							process_telecommand(Buffer[2], Buffer[3]);	//Saves the TLE
						}
						else if (telecommand_rx){	//Second telecommand RX consecutively
							//rx_attemps_counter = 0;
							if (Buffer[2] == last_telecommand[2]){	//Second telecommand received equal to the first CHANGE THIS TO CHECK THE WHOLE TELECOMMAND. USE VARIABLE compare_arrays
								//Buffer[2] == (SEND_DATA || SEND_TELEMETRY || ACK_DATA || SEND_CALIBRATION || SEND_CONFIG)
								Stop_timer_16();
								if (Buffer[2] == SEND_DATA || Buffer[2] == SEND_TELEMETRY || Buffer[2] == ACK_DATA || Buffer[2] == SEND_CALIBRATION || Buffer[2] == SEND_CONFIG || Buffer[2] == ASK_DATA){
									telecommand_rx = false;
									process_telecommand(Buffer[2], Buffer[3]);
								}
								else {
									request_execution = true;
									State = TX;
									DelayMs(300);
								}
							}
							else if(Buffer[2] == ACK){	//Order execution ACK
								//rx_attemps_counter = 0;
								Stop_timer_16();
								request_counter = 0;
								request_execution = false;
								reception_ack_mode = false;
								telecommand_rx = false;
								process_telecommand(last_telecommand[2], last_telecommand[3]);
								State = RX;
							}
							else{	//Second telecommand received different from the first
							    State = TX;
							    telecommand_rx = false;
							    error_telecommand = true;
							    Stop_timer_16();
							    DelayMs(10);
							}
						}
						else{	//First telecommand RX
							memcpy( last_telecommand, Buffer, BUFFER_SIZE );
							last_telecommand[0] = MISSION_ID;	//To avoid retransmitting the PIN
							last_telecommand[1] = POCKETQUBE_ID;
							tle_telecommand = false;
							telecommand_rx = true;
							State = RX;
							//rx_attemps_counter = 0;
							DelayMs(10);
							Start_timer_16();
						}
					}
					else{	//Pin not correct. If pin not correct it is assumed that the packet comes from another source. The protocol continues ignoring it
					    State = TX;
					    error_telecommand = true;
					    Stop_timer_16();
					    DelayMs(500);
					}
                    PacketReceived = false;     // Reset flag
                }
                else	//If packet not received, restart reception process
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

                	if (reception_ack_mode){
                		reception_ack_mode = false;
                		DelayMs( 300 );
                	}
                }
                break;
            }
            case TX:
            {
            	/* TO TEST TELECOMMANDS */
                State = LOWPOWER;
            	if (error_telecommand){	//Send error message
            		uint8_t packet_to_send[] = {MISSION_ID,POCKETQUBE_ID,ERROR};
            		Radio.Send(packet_to_send,sizeof(packet_to_send));
            		DelayMs(100);
            		Radio.Send(packet_to_send,sizeof(packet_to_send));
                    error_telecommand = false;
            	} else if (request_execution){	//Send request for execute telecommand order
            		//packet_to_send = last_telecommand[2];
            		//Radio.Send(packet_to_send,1);
            		DelayMs(100);
            		Radio.Send(last_telecommand,3);	//TEST ONLY SENDING ONE REQUEST (NORMALLY PACKETS ARE TX IN PAIRS
            		//Radio.Send(last_telecommand,sizeof(last_telecommand));	//TEST ONLY SENDING ONE REQUEST (NORMALLY PACKETS ARE TX IN PAIRS
            		DelayMs(100);
            		Radio.Send(last_telecommand,3);	//TEST ONLY SENDING ONE REQUEST (NORMALLY PACKETS ARE TX IN PAIRS
            		//Radio.Send(last_telecommand,sizeof(last_telecommand));	//Better here or iterate another time and return to TX?
            		request_counter++;
            		reception_ack_mode = true;
            		State = RX;
            		//rx_attemps_counter = 0;
            		Stop_timer_16();
            		Start_timer_16();
            		//TimerStart(&CADTimeoutTimer);
            		//Radio.Rx( RX_TIMEOUT_VALUE );
            		//PacketReceived = false;
            	}
            	else if(ask_data){

            		Buffer[0] = MISSION_ID;	//Satellite ID
					Buffer[1] = POCKETQUBE_ID;	//Poquetcube ID (there are at least 3)
					Buffer[2] =  SEND_DATA; //ID del telecommand de send_data

					Radio.Send( Buffer, BUFFER_SIZE );
					DelayMs( (uint16_t) time_packets*3/5 );
					Radio.Send( Buffer, BUFFER_SIZE );
					ask_data = false;
					State = RX;

            	}else if (tx_flag){	//Send data
                    //txfunction( );
            		uint64_t read_photo[12];
            		uint8_t transformed[96];
            		if (window_packet < WINDOW_SIZE){
            			if (nack_flag){
            				if (nack[nack_counter] != 0){
            					Flash_Read_Data(DATA_ADDR + nack[nack_counter]*96, &read_photo, sizeof(read_photo));
                    			Buffer[2] = nack[nack_counter];	//Number of the retransmitted packet
                    			nack_counter++;
            				} else{ //When all packets have been retransmitted, we continue with the next one
            					nack_flag = false;
            					nack_counter = 0;
                				Flash_Read_Data(DATA_ADDR + packet_number*96, &read_photo, sizeof(read_photo));
                    			Buffer[2] = packet_number;	//Number of the packet
                    			packet_number++;
            				}
            			} else {
            				Flash_Read_Data(DATA_ADDR + packet_number*96, &read_photo, sizeof(read_photo));
                			Buffer[2] = packet_number;	//Number of the packet
                			packet_number++;
            			}
            			Buffer[0] = MISSION_ID;	//Satellite ID
            			Buffer[1] = POCKETQUBE_ID;	//Poquetcube ID (there are at least 3)
            			memcpy(&transformed, read_photo, sizeof(transformed));
            			for (uint8_t i=3; i<BUFFER_SIZE-1; i++){
            				Buffer[i] = transformed[i-3];
            			}
            			Buffer[BUFFER_SIZE-1] = 0xFF;	//Final of the packet indicator
            			window_packet++;
            			State = TX;
                        //DelayMs( 300 );
            			DelayMs( (uint16_t) time_packets*3/5 );
                        Radio.Send( Buffer, BUFFER_SIZE );
            		} else{
            			//tx_non_stop = false;
            			tx_flag = false;
            			send_data = false;
            			window_packet = 0;
            			State = RX;
            		}
                    //DelayMs( 200 );
            		DelayMs( (uint16_t) time_packets*2/5 );
            	}

            	else if (beacon_flag){
					//uint8_t packet_to_send[] = {MISSION_ID,POCKETQUBE_ID,BEACON};
					//Radio.Send(packet_to_send,sizeof(packet_to_send));
					//DelayMs(100);
					//Radio.Send(packet_to_send,sizeof(packet_to_send));
					//beacon_flag = false;

					uint64_t read_telemetry[5];
					uint8_t transformed[TELEMETRY_PACKET_SIZE];	//Maybe is better to use 40 bytes, as multiple of 8
					            		if (!contingency){
					            			Flash_Read_Data(TELEMETRY_ADDR, &read_telemetry, sizeof(read_telemetry));
					            			Buffer[0] = MISSION_ID;	//Satellite ID
					            			Buffer[1] = POCKETQUBE_ID;	//Poquetcube ID (there are at least 3)
					           				Buffer[2] = num_telemetry;	//Number of the packet
					        				memcpy(&transformed, read_telemetry, sizeof(transformed));
					            			for (uint8_t i=3; i<TELEMETRY_PACKET_SIZE+3; i++){
					            				Buffer[i] = transformed[i-3];
					            			}
					            			Buffer[TELEMETRY_PACKET_SIZE+3] = 0xFF;	//Final of the packet indicator
					            			num_telemetry++;
					            		    DelayMs( 300 );
					            		    Radio.Send( Buffer, TELEMETRY_PACKET_SIZE+4 );
					            			State = RX;
					            		}

					            		beacon_flag = false;

            	}

            	/* TO TEST PROTOCOL AND SWITCHING BETWEEN STATES
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
                */
            	DelayMs( 1 );
                break;
            }
            case TX_TIMEOUT:
            {
                State = LOWPOWER;
                break;
            }
            case LOWPOWER:
            default:
            	//defaultCounter = defaultCounter + 1;
                //State = RX;
            	if (error_telecommand || tx_flag){
					State = TX;
				}
            	else if (reception_ack_mode || tle_telecommand){
            		State = RX;
            	}
            	else if (telecommand_rx){	//We have received at least one telecommand
            		if (request_execution ){	//In this case we have to TX request or wait for ACK
            			if (request_counter >= 3){	//If 3 request have been sent, we send an error message
            				request_execution = false;
            				error_telecommand = true;
            				telecommand_rx = false;
            				State = TX;
            				request_counter=0;
            				//rx_attemps_counter=0;
            				Stop_timer_16();
            			} else if (protocol_timeout){	//TX another request execution
            				protocol_timeout = false;
            				State = TX;
            			} else {	//Iterate till 500 ms approx
            				//rx_attemps_counter++;
            				State = RX;
            			}
            			/*State = TX;*/// IN THE CASE OF RETRANSMISSIONS OF REQUEST
            			//TimerStart(&CADTimeoutTimer);
            			//Radio.Rx( RX_TIMEOUT_VALUE );
            			//DelayMs(500);
            			//if (!PacketReceived){	//If in 500 ms the RX IRQ does not jump, we go to TX the request or the error
            			//	State = TX;
            			//}
            		} else{	//We want to Rx the second telecommand
            			//TimerStart(&CADTimeoutTimer);
            			//Radio.Rx( RX_TIMEOUT_VALUE );
						//DelayMs(500);
            			//rx_attemps_counter++;

            			/*Check the 160 value with the whole code and multithread, because maybe induce a delay*/
            			if (protocol_timeout){	//With this value all 2nd telecommand that arrive before 650 ms are received. 700 ms or more error packet is send
            				PacketReceived = true;
            				protocol_timeout = false;
            				//rx_attemps_counter = 0;
            				DelayMs(1);
            				Stop_timer_16();
            			}
						State = RX; //If Timeout passes and the 2nd telecommand is not received, goes to RX and will process the first, as if the second has been RX
						//PacketReceived = true;
					}
            	}
            	else if (beacon_flag){
            		State = TX;
            	}
            	else{
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

/*************************************************************************
 *                                                                       *
 *  Function:  tx_beacon                                                 *
 *  --------------------                                                 *
 *  Activates the flags to TX the beacon                                 *
 *                                                                       *
 *  returns: nothing                                                     *
 *                                                                       *
 *************************************************************************/
void tx_beacon(void){
	if (State == RX || State == LOWPOWER){
		if (!reception_ack_mode && !tle_telecommand && !telecommand_rx){
			State = TX;
			beacon_flag = true;
		}
	}
}

/*************************************************************************
 *                                                                       *
 *  Function:  OnTxDone                                                  *
 *  --------------------                                                 *
 *  Activate the flag to indicate that the RX timeout has finished       *
 *                                                                       *
 *  returns: nothing                                                     *
 *                                                                       *
 *************************************************************************/
void comms_timmer(void){
	if (protocol_timeout_counter >= SF - 7){	//Timeout proportional to SF (high SF require more time)
		protocol_timeout = true;
		protocol_timeout_counter = 0;
	} else{
		protocol_timeout_counter = protocol_timeout_counter + 1;
	}
}

/*
void txfunction( void ){
	uint64_t read_photo[12];
	uint8_t transformed[96];
	if (window_packet < WINDOW_SIZE){
		Flash_Read_Data(SAVE_PHOTO+packet_number*96, &read_photo, sizeof(read_photo));
		Buffer[0] = MISSION_ID;	//Satellite ID
		Buffer[1] = POCKETQUBE_ID;	//Poquetcube ID (there are at least 3)
		Buffer[2] = packet_number;	//Number of the packet
		memcpy(&transformed, read_photo, sizeof(transformed));
		for (uint8_t i=3; i<BUFFER_SIZE-1; i++){
			Buffer[i] = transformed[i-3];
		}
		Buffer[BUFFER_SIZE-1] = 0xFF;	//Final of the packet indicator
		packet_number++;
		window_packet++;
		State = TX;
	} else{
		tx_non_stop = false;
		tx_flag = false;
		send_data = false;
		window_packet = 0;
		State = RX;
	}
	//for (uint8_t i = 0; i<BUFFER_SIZE; i=i+1){
	//	Buffer[i] = Memory[i+txCounter*BUFFER_SIZE];
	//}
	//txCounter = txCounter + 1;
	//if (MEMORY_SIZE < ( txCounter*BUFFER_SIZE + BUFFER_SIZE )){
	//	txCounter = 0;
	//	tx_non_stop = 0;
	//}
}*/

/*************************************************************************
 *                                                                       *
 *  Function:  OnTxDone                                                  *
 *  --------------------                                                 *
 *  when the transmission finish correctly                               *
 *                                                                       *
 *  returns: nothing                                                     *
 *                                                                       *
 *************************************************************************/
void OnTxDone( void )
{
    Radio.Standby( );
    if (tx_flag == 1){
        State = TX;
    } else{
        State = LOWPOWER;
    }
}

/*************************************************************************
 *                                                                       *
 *  Function:  OnRxDone                                                  *
 *  --------------------                                                 *
 *  processes the information when the reception has been done correctly *
 *  calculates the rssi and snr                                          *
 *                                                                       *
 *  payload: information received                                        *
 *  size: size of the payload                                            *
 *  rssi: rssi value                                                     *
 *  snr: snr value                                                       *
 *                                                                       *
 *  returns: nothing                                                     *
 *                                                                       *
 *************************************************************************/
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Standby( );
    uint16_t BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    //uint8_t RXactual[BufferSize];
    //memcpy( RXactual, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    PacketReceived = true;
    RssiMoy = (((RssiMoy * RxCorrectCnt) + RssiValue) / (RxCorrectCnt + 1));
    SnrMoy = (((SnrMoy * RxCorrectCnt) + SnrValue) / (RxCorrectCnt + 1));
    State = RX;
    //testRX = 1;
}

/*************************************************************************
 *                                                                       *
 *  Function:  OnTxTimeout                                               *
 *  --------------------                                                 *
 *  to process transmission timeout                                      *
 *                                                                       *
 *  returns: nothing                                                     *
 *                                                                       *
 *************************************************************************/
void OnTxTimeout( void )
{
    Radio.Standby( );
    State = TX_TIMEOUT;
}

/*************************************************************************
 *                                                                       *
 *  Function:  OnRxTimeout                                               *
 *  --------------------                                                 *
 *  to process reception timeout                                         *
 *                                                                       *
 *  returns: nothing                                                     *
 *                                                                       *
 *************************************************************************/
void OnRxTimeout( void )
{
    Radio.Standby( );
    //State = TX;
    //error_telecommand=true;
    State = RX;
    //if (request_execution){
    	//State = TX;
    //}
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

/*************************************************************************
 *                                                                       *
 *  Function:  OnRxErro                                                  *
 *  --------------------                                                 *
 *  function called when a reception error occurs                        *
 *                                                                       *
 *  returns: nothing                                                     *
 *                                                                       *
 *************************************************************************/
void OnRxError( void )
{
    Radio.Standby( );
    State = RX_ERROR;
}

/*************************************************************************
 *                                                                       *
 *  Function:  OnCadDone                                                 *
 *  --------------------                                                 *
 *  Function to check if the CAD has been done correctly or not          *
 *                                                                       *
 *  channelActivityDetected: boolean that contains the CAD flat          *
 *                                                                       *
 *  returns: nothing                                                     *
 *                                                                       *
 *************************************************************************/
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

/*************************************************************************
 *                                                                       *
 *  Function:  SX126xConfigureCad                                        *
 *  --------------------                                                 *
 *  Function Configure the Channel Activity Detection parameters and IRQ *
 *                                                                       *
 *  returns: nothing                                                     *
 *                                                                       *
 *************************************************************************/
void SX126xConfigureCad( RadioLoRaCadSymbols_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin , uint32_t cadTimeout)
{
    SX126xSetDioIrqParams( 	IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED, IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED,
                            IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    //SX126xSetCadParams( cadSymbolNum, cadDetPeak, cadDetMin, LORA_CAD_ONLY, ((cadTimeout * 1000) / 15.625 ));
    SX126xSetCadParams( cadSymbolNum, cadDetPeak, cadDetMin, LORA_CAD_RX, ((cadTimeout * 15.625) / 1000 ));
    //THE TOTAL CAD TIMEOUT CAN BE EQUAL TO RX TIMEOUT (IT SHALL NOT BE HIGHER THAN 4 SECONDS)
}

/*************************************************************************
 *                                                                       *
 *  Function:  CADTimeoutTimeoutIrq                                      *
 *  --------------------                                                 *
 *  Function called automatically when a CAD IRQ occurs                  *
 *                                                                       *
 *  returns: nothing                                                     *
 *                                                                       *
 *************************************************************************/
static void CADTimeoutTimeoutIrq( void )
{
    Radio.Standby( );
    //if (request_execution){
    //	State = TX;
    //} else{
	State = LOWPOWER;
    //}
    //State = START_CAD;
    //State = RX;
}

/*************************************************************************
 *                                                                       *
 *  Function:  RxTimeoutTimerIrq                                         *
 *  --------------------                                                 *
 *  Function called automatically when a Timeout interruption occurs     *
 *                                                                       *
 *  returns: nothing                                                     *
 *                                                                       *
 *************************************************************************/
static void RxTimeoutTimerIrq( void )
{
    RxTimeoutTimerIrqFlag = true;
}

/*************************************************************************
 *                                                                       *
 *  Function:  pin_correct                                               *
 *  --------------------                                                 *
 *  check if the pin in the telecommand is correct                       *
 *                                                                       *
 *  pin_1: first byte of the pin                                         *
 *  pin_2: second byte of the pin                                        *
 *                                                                       *
 *  returns: true if correct                                             *
 *                                                                       *
 *************************************************************************/
bool pin_correct(uint8_t pin_1, uint8_t pin_2) {
	if (pin_1 == PIN1 && pin_2 == PIN2){
		isGS = true;
		return true;
	}
	else if (pin_1 == MISSION_ID && pin_2 == POQUETQUBE_ID2){
		isGS = false;
		return true;
	}
	return false;
}


/*************************************************************************
 *                                                                       *
 *  Function:  process_telecommand                                       *
 *  --------------------                                                 *
 *  processes the information contained in the packet depending on       *
 *  the telecommand received                                             *
 *                                                                       *
 *  header: number of telecomman                                         *
 *  info: information contained in the received packet                   *
 *                                                                       *
 *  returns: nothing                                                     *
 *                                                                       *
 *************************************************************************/
void process_telecommand(uint8_t header, uint8_t info) {
	uint64_t info_write;	//Flash_Write_Data functions requires uint64_t variables (64 bits) or arrays
	switch(header) {
	case RESET2:{
		HAL_NVIC_SystemReset();
		break;
	}
	case NOMINAL:{
		info_write = info;
		Flash_Write_Data(NOMINAL_ADDR, &info_write, 1);
		//xTaskNotify("Task OBC", NOMINAL_NOTI, eSetBits); //Notification to OBC
		break;
	}
	case LOW:{
		info_write = info;
		Flash_Write_Data(LOW_ADDR, &info_write, 1);
		//xTaskNotify("Task OBC", LOW_NOTI, eSetBits); //Notification to OBC
		break;
	}
	case CRITICAL:{
		info_write = info;
		Flash_Write_Data(CRITICAL_ADDR, &info_write, 1);
		//xTaskNotify("Task OBC", CRITICAL_NOTI, eSetBits); //Notification to OBC
		break;
	}
	case EXIT_LOW_POWER:{
		//xTaskNotify("Task OBC", EXIT_LOW_POWER_NOTI, eSetBits); //Notification to OBC
		break;
	}
	case EXIT_CONTINGENCY:{
		//xTaskNotify("Task OBC", EXIT_CONTINGENCY_NOTI, eSetBits); //Notification to OBC
		break;
	}
	case EXIT_SUNSAFE:{
		//xTaskNotify("Task OBC", EXIT_SUNSAFE_NOTI, eSetBits); //Notification to OBC
		break;
	}
	case SET_TIME:{
		uint8_t time[4];
		for (k=0; k<4; k++){
			time[k]=Buffer[k+3];
		}
		Flash_Write_Data(TIME_ADDR, &time, sizeof(time));
		//xTaskNotify("Task OBC", SETTIME_NOTI, eSetBits); //Notification to OBC
		break;
	}
	case SET_CONSTANT_KP:{
		info_write = info;
		Flash_Write_Data(KP_ADDR, &info_write, 1);
		//xTaskNotify("Task OBC", CTEKP_NOTI, eSetBits); //Notification to OBC
		break;
	}
	case TLE:{
		uint8_t tle[TLE_PACKET_SIZE];
		for (k=0; k<TLE_PACKET_SIZE; k++){
			tle[k]=Buffer[k+3];
		}
		if (tle_telecommand){
			Flash_Write_Data(TLE_ADDR1, &tle, sizeof(tle));
			//tle_packets++;
		} else{
			Flash_Write_Data(TLE_ADDR2, &tle, sizeof(tle));
			//tle_packets = 0;
		}
		//xTaskNotify("Task OBC", TLE_NOTI, eSetBits); //Notification to OBC
		break;
		//For high SF 3 packets will be needed and the code should be adjusted
	}
	case SET_GYRO_RES:{
		info_write = info;
		Flash_Write_Data(GYRO_RES_ADDR, &info_write, 1);
		//xTaskNotify("Task OBC", GYRORES_NOTI, eSetBits); //Notification to OBC
		break;
	}
	case SEND_DATA:{
		if (!contingency){
			tx_flag = true;	//Activates TX flag
			State = TX;
			send_data = true;
		}
		break;
	}
	case SEND_TELEMETRY:{
		uint64_t read_telemetry[5];
		uint8_t transformed[TELEMETRY_PACKET_SIZE];	//Maybe is better to use 40 bytes, as multiple of 8
		if (!contingency){
			Flash_Read_Data(TELEMETRY_ADDR, &read_telemetry, sizeof(read_telemetry));
			Buffer[0] = MISSION_ID;	//Satellite ID
			Buffer[1] = POCKETQUBE_ID;	//Poquetcube ID (there are at least 3)
			Buffer[2] = num_telemetry;	//Number of the packet
			memcpy(&transformed, read_telemetry, sizeof(transformed));
			for (uint8_t i=3; i<TELEMETRY_PACKET_SIZE+3; i++){
				Buffer[i] = transformed[i-3];
			}
			Buffer[TELEMETRY_PACKET_SIZE+3] = 0xFF;	//Final of the packet indicator
			num_telemetry++;
            DelayMs( 300 );
            Radio.Send( Buffer, TELEMETRY_PACKET_SIZE+4 );
			State = RX;
		}
		break;
	}
	case ASK_DATA:{
		if (!contingency){
			State = TX;
			ask_data = true; 	// activate ask others flag
		}
		break;
	}
	case STOP_SENDING_DATA:{
		tx_flag = false;	//Activates TX flag
		State = RX;
		send_data = false;
		break;
	}
	case ACK_DATA:{
		if (!contingency && info != 0){
			nack_flag = true;
			memcpy(&nack, Buffer[3], sizeof(nack));
			tx_flag = true;	//Activates TX flag
			State = TX;
			send_data = true;
		}
		break;
	}
	case SET_SF_CR: {
		if (info == 0) SF = 7;
		else if (info == 1) SF = 8;
		else if (info == 2) SF = 9;
		else if (info == 3) SF = 10;
		else if (info == 4) SF = 11;
		else if (info == 5) SF = 12;
		info_write = SF;
		Flash_Write_Data(SF_ADDR, &info_write, 1);
		/*4 cases (4/5, 4/6, 4/7,1/2), so we will receive and store 0, 1, 2 or 3*/
		info_write = Buffer[4];
		Flash_Write_Data(CRC_ADDR, &info_write, 1);
		DelayMs(10);
		configuration();
		break;
	}
	case SEND_CALIBRATION:{
		/* CALIBRATION PACKET RECEIVED */
		uint8_t calibration_packet[CALIBRATION_PACKET_SIZE];
		for (k=0; k<CALIBRATION_PACKET_SIZE; k++){
			calibration_packet[k]=Buffer[k+3];
		}
		Flash_Write_Data(CALIBRATION_ADDR, &calibration_packet, sizeof(calibration_packet));
		//xTaskNotify("Task OBC", CALIBRATION_NOTI, eSetBits); //Notification to OBC
		break;
		//For high SF 2 packets will be needed and the code should be adjusted
	}
	case CHANGE_TIMEOUT:{
		memcpy(&time_packets, Buffer[3], 2);
		Flash_Write_Data(COMMS_TIME_ADDR, &time_packets, sizeof(time_packets));
		break;
	}
	case TAKE_PHOTO:{
		Flash_Write_Data(PL_TIME_ADDR, &Buffer[3], 4);
		info_write = Buffer[7];
		Flash_Write_Data(PHOTO_RESOL_ADDR, &info_write, 1);
		info_write = Buffer[8];
		Flash_Write_Data(PHOTO_COMPRESSION_ADDR, &info_write, 1);
		//xTaskNotify("Task OBC", TAKEPHOTO_NOTI, eSetBits); //Notification to OBC
		break;
	}
	case TAKE_RF:{
		Flash_Write_Data(PL_TIME_ADDR, &Buffer[3], 8);
		info_write = Buffer[11];
		Flash_Write_Data(F_MIN_ADDR, &info_write, 1);
		info_write = Buffer[12];
		Flash_Write_Data(F_MAX_ADDR, &info_write, 1);
		info_write = Buffer[13];
		Flash_Write_Data(DELTA_F_ADDR, &info_write, 1);
		info_write = Buffer[14];
		Flash_Write_Data(INTEGRATION_TIME_ADDR, &info_write, 1);
		//xTaskNotify("Task OBC", TAKERF_NOTI, eSetBits); //Notification to OBC
		break;
	}
	case SEND_CONFIG:{
		uint64_t read_config[4];
		uint8_t transformed[CONFIG_PACKET_SIZE];	//CONFIG_PACKET_SIZE=30
		if (!contingency){
			Flash_Read_Data(CONFIG_ADDR, &read_config, sizeof(read_config));
			Buffer[0] = MISSION_ID;	//Satellite ID
			Buffer[1] = POCKETQUBE_ID;	//Poquetcube ID (there are at least 3)
			Buffer[2] = num_config;	//Number of the packet
			memcpy(&transformed, read_config, sizeof(transformed));
			for (uint8_t i=3; i<CONFIG_PACKET_SIZE+3; i++){
				Buffer[i] = transformed[i-3];
			}
			Buffer[CONFIG_PACKET_SIZE+3] = 0xFF;	//Final of the packet indicator
			num_config++;
            DelayMs( 300 );
            Radio.Send( Buffer, CONFIG_PACKET_SIZE+4 );
			State = RX;
		}
		break;
	}
	default:{
		State = TX;
		error_telecommand = true;
	}
	}
}
