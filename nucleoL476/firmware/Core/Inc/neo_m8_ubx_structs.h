/*
 * neo_m8_ubx_structs.h
 *
 *  Created on: Jul 17, 2018
 *      Author: alexis
 */

#ifndef NEO_M8_UBX_STRUCTS_H_
#define NEO_M8_UBX_STRUCTS_H_

#include <stdint.h>
#include <stdbool.h>

/* See page 134 of document UBX-13003221 - R15 for more precisions */

/*
 *  • Every Frame starts with a 2-byte Preamble consisting of two synchronization characters: 0xB5 0x62.
 *	• A 1-byte Message Class field follows. A Class is a group of messages that are related to each other.
 *	• A 1-byte Message ID field defines the message that is to follow.
 *	• A 2-byte Length field follows. The length is defined as being that of the payload only. It does not include
 *		the Preamble, Message Class, Message ID, Length, or CRC fields. The number format of the length field is a
 *		Little-Endian unsigned 16-bit integer.
 *	• The Payload field contains a variable number of bytes.
 *	• The two 1-byte CK_A and CK_B fields hold a 16-bit checksum whose calculation is defined below. This
 *		concludes the Frame.
 */



#define UBX_SYNC_CHAR1	0xB5
#define UBX_SYNC_CHAR2	0x62
#define UBX_PAYLOAD_LENGTH 92

/* UBX Class IDs */
/* See UBX-13003221 - R15 Early Production Information, Page 137 of 386 for details */
#define UBX_MSG_CLASS_ACK	0x05		/* Ack/Nak Messages: Acknowledge or Reject messages to CFG input messages */
#define UBX_MSG_CLASS_AID	0x0B		/* AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input */
#define UBX_MSG_CLASS_CFG	0x06		/* Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc. */
#define UBX_MSG_CLASS_ESF	0x10		/* External Sensor Fusion Messages: External Sensor Measurements and Status Information */
#define UBX_MSG_CLASS_HNR	0x28		/* High Rate Navigation Results Messages: High rate time, position, speed, heading */
#define UBX_MSG_CLASS_INF	0x04		/* Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice */
#define UBX_MSG_CLASS_LOG	0x21		/* Logging Messages: Log creation, deletion, info and retrieval */
#define UBX_MSG_CLASS_MGA   0x13		/* Multiple GNSS Assistance Messages: Assistance data for various GNSS */
#define UBX_MSG_CLASS_MON	0x0A 	/* Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status */
#define UBX_MSG_CLASS_NAV 	0x01		/* Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used */
#define UBX_MSG_CLASS_RXM	0x02 	/* Receiver Manager Messages: Satellite Status, RTC Status */
#define UBX_MSG_CLASS_SEC	0x27		/* Security Feature Messages */
#define UBX_MSG_CLASS_TIM	0x0D		/* Timing Messages: Time Pulse Output, Time Mark Results */
#define UBX_MSG_CLASS_UPD	0x09		/* Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc. */

/* UBX Class ACK */
#define UBX_MSG_ID_ACK_ACK 	0x01
#define UBX_MSG_ID_ACK_NAK 	0x00

/* UBX Class AID */
#define UBX_MSG_ID_AID_ALM 	0x30
#define UBX_MSG_ID_AID_AOP 	0x33
#define UBX_MSG_ID_AID_EPH 	0x31
#define UBX_MSG_ID_AID_HUI 	0x02
#define UBX_MSG_ID_AID_INI 	0x01

/* UBX Class CFG */
#define UBX_MSG_ID_CFG_ANT 			0x13
#define UBX_MSG_ID_CFG_BATCH			0x93
#define UBX_MSG_ID_CFG_CFG 			0x09
#define UBX_MSG_ID_CFG_DAT 			0x06
#define UBX_MSG_ID_CFG_DGNSS 		0x70
#define UBX_MSG_ID_CFG_DOSC 			0x61
#define UBX_MSG_ID_CFG_DYNSEED		0x85
#define UBX_MSG_ID_CFG_ESRC 			0x60
#define UBX_MSG_ID_CFG_FIXSEED 		0x84
#define UBX_MSG_ID_CFG_GEOFENCE	 	0x69
#define UBX_MSG_ID_CFG_GNSS 			0x3E
#define UBX_MSG_ID_CFG_HNR    		0x5C
#define UBX_MSG_ID_CFG_INF 			0x02
#define UBX_MSG_ID_CFG_ITFM 			0x39
#define UBX_MSG_ID_CFG_LOGFILTER 	0x47
#define UBX_MSG_ID_CFG_MSG			0x01
#define UBX_MSG_ID_CFG_NAV5 			0x24
#define UBX_MSG_ID_CFG_NAVX5 		0x23
#define UBX_MSG_ID_CFG_NMEA 		    0x17
#define UBX_MSG_ID_CFG_ODO 			0x1E
#define UBX_MSG_ID_CFG_PM2 			0x3B
#define UBX_MSG_ID_CFG_PMS 			0x86
#define UBX_MSG_ID_CFG_PRT 			0x00
#define UBX_MSG_ID_CFG_PWR 			0x57
#define UBX_MSG_ID_CFG_RATE 			0x08
#define UBX_MSG_ID_CFG_RINV 			0x34
#define UBX_MSG_ID_CFG_RST 			0x04
#define UBX_MSG_ID_CFG_RXM 			0x11
#define UBX_MSG_ID_CFG_SBAS 			0x16
#define UBX_MSG_ID_CFG_SMGR 			0x62
#define UBX_MSG_ID_CFG_TMODE2 		0x3D
#define UBX_MSG_ID_CFG_TMODE3 		0x71
#define UBX_MSG_ID_CFG_TP5 			0x31
#define UBX_MSG_ID_CFG_TXSLOT 		0x53
#define UBX_MSG_ID_CFG_USB 			0x1B

//todo: add esf, hnr, inf, log, mga, rxm, sec, tim, and upd sections

/* UBX Class NAV */
#define UBX_MSG_ID_NAV_AOPSTATUS 	0x60
#define UBX_MSG_ID_NAV_ATT 			0x05
#define UBX_MSG_ID_NAV_CLOCK 		0x22
#define UBX_MSG_ID_NAV_DGPS 			0x31
#define UBX_MSG_ID_NAV_DOP 			0x04
#define UBX_MSG_ID_NAV_EOE 			0x61
#define UBX_MSG_ID_NAV_GEOFENCE 		0x39
#define UBX_MSG_ID_NAV_HPPOSECEF 	0x13
#define UBX_MSG_ID_NAV_HPPOSLLH 		0x14
#define UBX_MSG_ID_NAV_ODO 			0x09
#define UBX_MSG_ID_NAV_ORB 			0x34
#define UBX_MSG_ID_NAV_POSECEF 		0x01
#define UBX_MSG_ID_NAV_POSLLH 		0x02
#define UBX_MSG_ID_NAV_PVT 			0x07
#define UBX_MSG_ID_NAV_RELPOSNED 	0x3C
#define UBX_MSG_ID_NAV_RESETODO		0x10 /* This is a command */
#define UBX_MSG_ID_NAV_SAT 			0x35
#define UBX_MSG_ID_NAV_SBAS 			0x32
#define UBX_MSG_ID_NAV_SOL 			0x06
#define UBX_MSG_ID_NAV_STATUS 		0x03
#define UBX_MSG_ID_NAV_SVINFO 		0x30
#define UBX_MSG_ID_NAV_SVIN 			0x3B
#define UBX_MSG_ID_NAV_TIMEBDS 		0x24
#define UBX_MSG_ID_NAV_TIMEGAL 		0x25
#define UBX_MSG_ID_NAV_TIMEGLO 		0x23
#define UBX_MSG_ID_NAV_TIMEGPS 		0x20
#define UBX_MSG_ID_NAV_TIMELS 		0x26
#define UBX_MSG_ID_NAV_TIMEUTC 		0x21
#define UBX_MSG_ID_NAV_VELECEF 		0x11
#define UBX_MSG_ID_NAV_VELNED 		0x12

struct UbxFrame {
	uint8_t syncChar1;
	uint8_t syncChar2;
	uint8_t class;
	uint8_t id;
	uint16_t length;
	char payload[UBX_PAYLOAD_LENGTH];
	uint8_t checksumA;
	uint8_t checksumB;
};

/* See the page 309 of UBX-13003221 - R15 for details */
typedef enum {
	PSM_NOT_ACTIVE 		= 0,
	PSM_ENABLED 	   		= 1,
	PSM_ACQUISITION 		= 2,
	PSM_TRACKING			= 3,
	PSM_OPT_TRACKING 	= 4,
	PSM_INACTIVE			= 5
} ubx_power_save_mode;

/* See the page 309 of UBX-13003221 - R15 for details */
typedef enum {
	NO_CARRIER_PHASE_RANGE_SOLUTION = 0,
	FLOAT_SOLUTION					= 1,
	FIXED_SOLUTION					= 2
} ubx_carrier_phase_range;


/* See the page 307 of UBX-13003221 - R15 for the units and scalings */
typedef struct {
	unsigned long 	iTOW;		/*(ms) GPS time of week of the navigation epoch. */
	unsigned short 	year;		/* Year (UTC) */
	unsigned char   month;		/* Month, range 1..12 (UTC) */
	unsigned char   day;			/* Day of month, range 1..31 (UTC) */
	unsigned char   hour;		/* Hour of day, range 0..23 (UTC) */
	unsigned char 	min;			/* Minute of hour, range 0..59 (UTC) */
	unsigned char 	sec;			/* Seconds of minute, range 0..60 (UTC) */
	unsigned char   valid; 	 	/* Validity flags */
	unsigned long 	tAcc;		/* GPS time of week of the navigation epoch. */
	signed   long  	nano;		/* Fraction of second, range -1e9 .. 1e9 (UTC) */
	unsigned char   fixType;		/* GNSSfix Type: 0: no fix ; 1: dead reckoning only 2: 2D-fix
												 3: 3D-fix   4: GNSS + dead reckoning combined
												 5: time only fix */
	unsigned char 	flags;		/* Fix status flags (see graphic below) */
	unsigned char	flags2;		/* Additional flags (see graphic below) */
	unsigned char	numSV;		/* Number of satellites used in Nav Solution */
	signed long		lon;			/* Longitude */
	signed long		lat;			/* Latitude (deg) */
	signed long		height;		/* Height above ellipsoid */
	signed long		hMSL;		/* Height above mean sea level */
	signed long		hAcc;		/* Horizontal accuracy estimate */
	unsigned long	vAcc;		/* Vertical accuracy estimate */
	signed long		velN;		/* NED north velocity */
	signed long		velE;		/* NED east velocity */
	signed long		velD;	 	/* NED down velocity */
	signed long 		gSpeed;		/* Ground Speed (2-D) */
	signed long		headMot;		/* Heading of motion (2-D) */
	unsigned long	sAcc;		/* Speed accuracy estimate */
	unsigned long 	headAcc;		/* Heading accuracy estimate (both motion and vehicle) */
	unsigned short	pDOP;		/* Position DOP */
/*  unsigned char	reserved;*/ /* Byte offset 78 is reserved, this is only here to indicate it */
	signed long		headVeh;		/* Heading of vehicle (2-D) */
	signed short		magDec;		/* Magnetic declination */
	unsigned short	magAcc;		/* Magnetic declination accuracy */

	/* Bitfield valid */
	bool				validDate;		/* 1 = valid UTC Date (see Time Validity section for details) */
	bool				validTime;		/* 1 = valid UTC Time of Day (see Time Validity section for details) */
	bool				fullyResolved;	/* 1 = UTC Time of Day has been fully resolved (no seconds uncertainty) */
	bool 			validMag;		/* 1 = valid Magnetic declination */

	/*	Bitfield flags */
	bool 			gnssFixOK;			/* 1 = valid fix (i.e within DOP & accuracy masks) */
	bool				diffSoln;			/* 1 = differential corrections were applied */
	bool				headVehValid;		/* 1 = heading of vehicle is valid */
	ubx_power_save_mode		psmState;	/* Power Save Mode state (see Power Management) */
	ubx_carrier_phase_range	carrSoln;	/* Carrier phase range solution status */


	/* Bitfield flags2 */
	bool 			confirmedAvai;		/* 1 = information about UTC Date and Time of Day validity confirmation is available */
	bool 			confirmedDate;		/* 1 = UTC Date validity could be confirmed (see Time Validity section for details) */
	bool				confirmedTime;		/* 1 = UTC Time of Day could be confirmed (see Time Validity section for details) */

} ubx_nav_pvt_msg_t;

/* See the page 296 of UBX-13003221 - R15 for the units and scalings */
typedef struct {
	unsigned long  iTOW;			/*(ms) GPS time of week of the navigation epoch. */
	unsigned char version;		/* Message version (0 for this version) */
	/* unsigned char reserved1[3]; */

	signed long 	  roll;			/* Vehicle roll. */
	signed long 	  pitch;			/* Vehicle pitch */
	signed long 	  heading;		/* Vehicle heading */

	unsigned long accRoll;		/* Vehicle roll accuracy */
	unsigned long accPitch;		/* Vehicle pitch accuracy */
	unsigned long accHeading;	/* Vehicle heading accuracy */

}ubx_nav_att_msg_t;

/* See the page 298 and 299 of UBX-13003221 - R15 for the units and scalings */
/* Description Dilution of precision */
/* Comment:
 * 	• DOP values are dimensionless.
 *  • All DOP values are scaled by a factor of 100. If the unit transmits a value of e.g. 156, the
		DOP value is 1.56
 */
typedef struct {
	unsigned long  iTOW;		/*(ms) GPS time of week of the navigation epoch. */
	unsigned short gDOP;		/* Geometric DOP */
	unsigned short pDOP;		/* Position DOP */
	unsigned short tDOP;		/* Time DOP */
	unsigned short vDOP;		/* Vertical DOP */
	unsigned short hDOP;		/* Horizontal DOP */
	unsigned short nDOP;		/* Northing DOP */
	unsigned short eDOP;		/* Easting DOP */
}ubx_nav_dop_msg_t;


/* NAV-ODO: Odometer Solution */
/* See the page 302 of UBX-13003221 - R15 for the units and scalings */
/* Comment:
 * 	This message outputs the traveled distance since last reset (see NAV-RESETODO) together
 *  with an associated estimated accuracy and the total cumulated ground distance (can only
 *  be reset by a cold start of the receiver).
 */
typedef struct {
	unsigned char  version; 		/* Message version (0 for this version) */
/* unsigned char reserved1[3]; */
	unsigned long  iTOW;			 /*(ms) GPS time of week of the navigation epoch. */
	unsigned long  distance;		 /* Ground distance since last reset */
	unsigned long totalDistance; /* Total cumulative ground distance */
	unsigned long distanceStd;	 /* Ground distance accuracy (1-sigma) */

}ubx_nav_odo_msg_t;



#endif /* NEO_M8_UBX_STRUCTS_H_ */
