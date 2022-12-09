/*
 * utils.h
 *
 *  Created on: 23 nov. 2022
 *      Author: NilRi
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

/****************************
 * GENERAL NOTIFICATIONS	*
 ****************************/

// TELECOMMANDS
#define FLAGS				0b000000

#define RESET_NOTI			0b000001

#define NOMINAL_NOTI 		0b000010

#define LOW_NOTI			0b000011

#define CRITICAL_NOTI 		0b000100

#define EXITLOWPOWER_NOTI	0b000101

#define EXITCONTINGENCY_NOTI 0b000110

#define EXITSUNSAFE_NOTI    0b000111

#define SETTIME_NOTI 		0b001000

#define CTEKP_NOTI			0b001001

#define TLE_NOTI			0b001010

#define GYRORES_NOTI		0b001011

#define SENDDATA_NOTI       0b001100

#define SENDTELEMETRY_NOTI  0b001101

#define STOPSENDINGDATA_NOTI 0b001110

#define ACKDATA_NOTI        0b001111

#define SETSFCR_NOTI        0b010000

#define CALIBRATION_NOTI	0b010001

#define CHANGE_TIMEOUT_NOTI 0b010010

#define TAKEPHOTO_NOTI		0b010011

#define TAKERF_NOTI			0b0010100

#define SENDCONFIG_NOTI     0b010101


// TELEMETRY

// Notification: Contingency state
// To: COMMS, ADCS
// From: OBC
#define CONTINGENCY_NOTI	0b010110

// Notification: SUNSAFE state = KILL THE THREAD
// To: COMMS, ADCS, PAYLOAD
// From:OBC
#define SUNSAFE_NOTI		0b010111
// Notification: Wake up a task that is suspended (sleep)
// To: COMMS, ADCS, PAYLOAD
// From:OBC
#define WAKEUP_NOTI			0b011000

// Notification: We are in the contact region of the GS
// To: COMMS
// From:ADCS
#define GS_NOTI				0b011001
// Notification: We are NOT in the contact region of the GS
// To: COMMS
// From:ADCS
#define NOTGS_NOTI			0b011010


/************************
 * PAYLOAD NOTIFICATIONS	*
 ************************/
// Notification: Take photo
// To: PAYLOAD_CAMERA
// From: COMMS
#define TAKEPHOTO_NOTI		0b011011

// Notification: Photo taken
// To: COMMS
// From: PAYLOAD_CAMERA
#define DONEPHOTO_NOTI		0b011100
// Notification: RF taken
// To: COMMS
// From: PAYLOAD_RF
#define DONERF_NOTI			0b011101
// Notification: The Payload camera is not working
// To: OBC -> COMMS
// From: PAYLOAD_CAMERA
#define PAYLOAD_ERROR_NOTI  0b011101


/************************
 * ADCS NOTIFICATIONS	*
 ************************/

// Notification: Start pointing to the Earth
// To: ADCS
// From: COMMS-> OBC
#define POINTING_NOTI		0b011110

// Notification: Detumble the satellite
// To: ADCS
// From: COMMS-> OBC
#define DETUMBLING_NOTI		0b011111

// Notification: Rotate the satellite
// To: ADCS
// From: COMMS-> OBC
#define ROTATE_NOTI			0b100000

// Notification: Pointing done
// To: PL
// From: ADCS
#define POINTING_DONE_NOTI		0b100001

// Notification: Stop pointing
// To: ADCS
// From: OBC
#define STOP_POINTING_NOTI		0b100010
#endif /* INC_UTILS_H_ */
