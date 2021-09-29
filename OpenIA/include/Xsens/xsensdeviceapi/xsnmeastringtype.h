
//  ==> COPYRIGHT (C) 2019 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE <==
//  WARNING: COPYRIGHT (C) 2019 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE. ALL RIGHTS RESERVED.
//  THIS FILE AND THE SOURCE CODE IT CONTAINS (AND/OR THE BINARY CODE FILES FOUND IN THE SAME
//  FOLDER THAT CONTAINS THIS FILE) AND ALL RELATED SOFTWARE (COLLECTIVELY, "CODE") ARE SUBJECT
//  TO AN END USER LICENSE AGREEMENT ("AGREEMENT") BETWEEN XSENS AS LICENSOR AND THE AUTHORIZED
//  LICENSEE UNDER THE AGREEMENT. THE CODE MUST BE USED SOLELY WITH XSENS PRODUCTS INCORPORATED
//  INTO LICENSEE PRODUCTS IN ACCORDANCE WITH THE AGREEMENT. ANY USE, MODIFICATION, COPYING OR
//  DISTRIBUTION OF THE CODE IS STRICTLY PROHIBITED UNLESS EXPRESSLY AUTHORIZED BY THE AGREEMENT.
//  IF YOU ARE NOT AN AUTHORIZED USER OF THE CODE IN ACCORDANCE WITH THE AGREEMENT, YOU MUST STOP
//  USING OR VIEWING THE CODE NOW, REMOVE ANY COPIES OF THE CODE FROM YOUR COMPUTER AND NOTIFY
//  XSENS IMMEDIATELY BY EMAIL TO INFO@XSENS.COM. ANY COPIES OR DERIVATIVES OF THE CODE (IN WHOLE
//  OR IN PART) IN SOURCE CODE FORM THAT ARE PERMITTED BY THE AGREEMENT MUST RETAIN THE ABOVE
//  COPYRIGHT NOTICE AND THIS PARAGRAPH IN ITS ENTIRETY, AS REQUIRED BY THE AGREEMENT.
//  
//  THIS SOFTWARE CAN CONTAIN OPEN SOURCE COMPONENTS WHICH CAN BE SUBJECT TO 
//  THE FOLLOWING GENERAL PUBLIC LICENSES:
//  ==> Qt GNU LGPL version 3: http://doc.qt.io/qt-5/lgpl.html <==
//  ==> LAPACK BSD License:  http://www.netlib.org/lapack/LICENSE.txt <==
//  ==> StackWalker 3-Clause BSD License: https://github.com/JochenKalmbach/StackWalker/blob/master/LICENSE <==
//  ==> Icon Creative Commons 3.0: https://creativecommons.org/licenses/by/3.0/legalcode <==
//  

#ifndef XSNMEASTRINGTYPE_H
#define XSNMEASTRINGTYPE_H

/*!	\addtogroup enums Global enumerations
	@{
*/
//! NMEA string types
enum XsNmeaStringType {
	 XNST_None		= 0x0000
	,XNST_HCHDM		= 0x0001 //!< NMEA string with Magnetic Heading
	,XNST_HCHDG		= 0x0002 //!< NMEA string with Heading and Magnetic Variation
	,XNST_TSS2		= 0x0004 //!< Proprietry string with Heading, Heave, Roll and Pitch
	,XNST_PHTRO		= 0x0008 //!< Proprietry NMEA string with Pitch and Roll
	,XNST_PRDID		= 0x0010 //!< Proprietry NMEA string with Pitch, Roll and Heading
	,XNST_EM1000	= 0x0020 //!< Binary format suitable for use with Simrad EM1000 mulitibeam sounders with Roll, Pitch, Heave and Heading
	,XNST_PSONCMS	= 0x0040 //!< NMEA string with Xsens Compass Motion Sensor information
	,XNST_HCMTW		= 0x0080 //!< NMEA string with (water) Temperature
	,XNST_HEHDT		= 0x0100 //!< NMEA string with True Heading
	,XNST_HEROT		= 0x0200 //!< NMEA string with Rate of Turn
	,XNST_GPGGA		= 0x0400 //!< NMEA string with Global Positioning system fix data
	,XNST_PTCF		= 0x0800 //!< NMEA string with motion data
	,XNST_XSVEL		= 0x1000 //!< Proprietry NMEA string with velocity data
	,XNST_GPZDA		= 0x2000 //!< NMEA string with date and time
	,XNST_GPRMC		= 0x4000 //!< NMEA string with recommended minimum specific GPS/Transit data
};
/*! @} */
typedef enum XsNmeaStringType XsNmeaStringType;

#endif // file guard
