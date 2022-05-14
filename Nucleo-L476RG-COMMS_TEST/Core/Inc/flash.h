/*!
 * \file      flash.c
 *
 * \brief     It contains all the functions to read from / write in the flash memory
 * 			  It also contains the different addresses of all the variables
 *
 *
 * \created on: 15/11/2021
 *
 * \author    Pol Simon
 *
 * \author    David Reiss
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include <stdint.h>
#include <stdbool.h>

#define INIT_ADDR_TEST				0x08000800

#define PHOTO_ADDR 					0x08020000
#define PAYLOAD_STATE_ADDR 			0x08008000	//CHECK THIS => IS THE SAME AS CURRENT_STATE_ADDRESS
#define COMMS_STATE_ADDR 			0x08008001
#define DETUMBLE_STATE_ADDR 		0x08008004
#define CURRENT_STATE_ADDR			0x08008000
#define PREVIOUS_STATE_ADDR			0x08008001
#define DEPLOYMENT_STATE_ADDR 		0x08008002
#define DEPLOYMENTRF_STATE_ADDR 	0x08008003
#define NOMINAL_ADDR 				0x08008004
#define LOW_ADDR 					0x08008005
#define CRITICAL_ADDR 				0x08008006
#define EXIT_LOW_ADDR 				0x08008007
#define SET_TIME_ADDR				0x08008008 	//4 bytes (GS -> RTC)
#define RTC_TIME_ADDR				0x0800800C	//4 bytes (RTC -> Unix) Get from the RTC
//CONFIGURATION ADDRESSES
#define CONFIG_ADDR 				0x08008010
#define KP_ADDR 					0x08008010
#define GYRO_RES_ADDR 				0x08008011
#define SF_ADDR 					0x08008012
#define CRC_ADDR 					0x08008013
#define PHOTO_RESOL_ADDR 			0x08008014
#define PHOTO_COMPRESSION_ADDR 		0x08008015
#define F_MIN_ADDR 					0x08008016
#define F_MAX_ADDR 					0x08008018
#define DELTA_F_ADDR 				0x0800801A
#define INTEGRATION_TIME_ADDR 		0x0800801C

#define TLE_ADDR 					0x08008020 // 138 bytes
#define EXIT_LOW_POWER_FLAG_ADDR 	0x080080AA

//CALIBRATION ADDRESSES
#define CALIBRATION_ADDR			0x080080AB
#define MAGNETO_MATRIX_ADDR			0x080080AB // 35 bytes
#define MAGNETO_OFFSET_ADDR			0x080080CF // 11 bytes
#define GYRO_POLYN_ADDR 			0x080080DB // 23 bytes
#define PHOTODIODES_OFFSET_ADDR 	0x080080F3 // 12 bytes

//TELEMETRY ADDRESSES
#define TELEMETRY_ADDR				0x08008100
#define TEMP_ADDR 					0x08008100
#define VOLTAGE_ADDR 				0x08008108
#define CURRENT_ADDR 				0x08008109
#define BATT_LEVEL_ADDR 			0x0800810A

//TIME ADDR
#define PL_TIME_ADDR 				0x08008110 	//4 bytes (GS -> PAYLOAD CAMERA)
#define PL_RF_TIME_ADDR				0x0800811C	//8 bytes (GS -> PAYLOAD RF)

//COMMS CONFIGURATION ADDRESSES
//TODO: How many bytes are needed for each one ?
#define COUNT_PACKET_ADDR 			0x08008200
#define COUNT_WINDOW_ADDR 			0x08008201
#define COUNT_RTX_ADDR 				0x08008202

#define TEST_ADDRESS 				0x0800D000



uint32_t Flash_Write_Data (uint32_t StartSectorAddress, uint64_t *Data, uint16_t numberofbytes);

void Write_Flash(uint32_t StartSectorAddress, uint64_t *Data, uint16_t numberofbytes);

void Flash_Read_Data (uint32_t StartSectorAddress, uint64_t *RxBuf, uint16_t numberofbytes);

void Check_Redundancy(uint32_t Address, uint64_t *RxDef, uint16_t numberofbytes);

void Read_Flash(uint32_t StartSectorAddress, uint64_t *RxBuf, uint16_t numberofbytes);

#endif /* INC_FLASH_H_ */



