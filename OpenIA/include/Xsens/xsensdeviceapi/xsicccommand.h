
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

#ifndef XSICCCOMMAND_H
#define XSICCCOMMAND_H

#include <xstypes/pstdint.h>

/*! \brief ICC (Inrun Compass Calibration) commands.
	\details To be used inside XMID_IccCommand and XMID_IccCommandAck messages
*/
enum XsIccCommand
{
	XIC_StartRepMotion		= 0x00,	//!< Indicate to ICC the start of representative motion
	XIC_StopRepMotion		= 0x01,	//!< Indicate to ICC the end of representative motion
	XIC_StoreResults		= 0x02,	//!< Update the stored magnetometer calibration using the ICC estimated calibration values
	XIC_RepMotionState		= 0x03,	//!< Retrieve the current state of the representative motion
	XIC_Status				= 0x04	//!< Retrieve the current ICC status
};
typedef enum XsIccCommand XsIccCommand;

/*! \brief ICC status flag
	\details Used for status fields in XMID_IccCommand and XMID_IccCommandAck messages
*/
enum XsIccStatusFlag
{
	XISF_ddtWarning		= 0x01,	//!< Indicates magnetic disturbance
	XISF_notEnoughData	= 0x02,	//!< Indicates data during representative motion does not have enough observability for an estimate
	XISF_OutputStable	= 0x10,	//!< Indicates the ICC output is stable and used by the filter
	XISF_RepMoActive	= 0x20	//!< Indicates ICC is recording a representative motion
};
typedef enum XsIccStatusFlag XsIccStatusFlag;

#endif
