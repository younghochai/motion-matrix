
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

#ifndef XSCOMCALLBACKOPTIONS_H
#define XSCOMCALLBACKOPTIONS_H

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Xda options, used to control the callback enabled in the COM object
	\details These options are used to specify whether the COM object should call
	a specific callback. All the callbacks are disabled by default in order
	to prevent memory leakage from unflushed buffers.
*/
enum XsComCallbackOptions {
		XSC_None					= 0			//!< all calbacks are disabled
		, XSC_LivePacket			= 0x0001	//!< live packet callback enable
		, XSC_LivePackets			= 0x0002	//!< live packets callback enable
		, XSC_BufferedPacket		= 0x0004	//!< buffered packet callback enable
		, XSC_BufferedPackets		= 0x0008	//!< buffered packets callback enable
		, XSC_Packet				= 0x0010	//!< automatic live/buffered packet callback enable
		, XSC_Packets				= 0x0020	//!< automatic live/buffered packets callback enable
		, XSC_RecordedPacket		= 0x0040	//!< recorded packet callback enable
		, XSC_RecordedPackets		= 0x0080	//!< recorded packets callback enable
		, XSC_MessageReceived		= 0x0100	//!< Message received callback enable
		, XSC_MessageSent			= 0x0200	//!< Message sent callback enable
		, XSC_All					= 0x03FF	//!< all calbacks are enabled
};
/*! @} */
typedef enum XsComCallbackOptions XsComCallbackOptions;

#ifdef __cplusplus
//! \brief Logical OR operator for XsComCallbackOptions values
inline XsComCallbackOptions operator | (XsComCallbackOptions a, XsComCallbackOptions b)
{
	return (XsComCallbackOptions) ((int)a | (int)b);
}
//! \brief Logical AND operator for XsComCallbackOptions values
inline XsComCallbackOptions operator & (XsComCallbackOptions a, XsComCallbackOptions b)
{
	return (XsComCallbackOptions) ((int)a & (int)b);
}
//! \brief Logical XOR operator for XsComCallbackOptions values
inline XsComCallbackOptions operator ^ (XsComCallbackOptions a, XsComCallbackOptions b)
{
	return (XsComCallbackOptions) ((int)a ^ (int)b);
}
//! \brief Logical NEG operator for XsComCallbackOptions values
inline XsComCallbackOptions operator ~ (XsComCallbackOptions a)
{
	return (XsComCallbackOptions) (~(int)a);
}

#endif
#endif