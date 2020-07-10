
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

#ifndef XSOPTION_H
#define XSOPTION_H

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Xda options, used to control the kind of data processing done by XDA
	\details These options are used to specify whether XDA should compute certain kinds of data from
	available other data. XsOptions can be logically ORed together
*/
//AUTO namespace xscontroller {
enum XsOption {
	  XSO_None = 0			//!< No option
	, XSO_Calibrate = 0x0001	//!< Compute calibrated inertial data from raw data and temperature
	, XSO_Orientation = 0x0002	//!< Compute orientation, the orientation is typically only computed in one stream. If not specified the system will decide: when reading from file it will use XSO_OrientationInBufferedStream, otherwise XSO_OrientationInLiveStream.

	, XSO_KeepLastLiveData = 0x0004	//!< Keep the last available live data in a cache so XsDevice::lastAvailableLiveData will work
	, XSO_RetainLiveData = 0x0008	//!< Keep the live data in a cache so it can be accessed through XsDevice::getDataPacketByIndex or XsDevice::takeFirstDataPacketInQueue. This option is mutually exclusive with XSO_RetainBufferedData. If both are set, XSO_RetainBufferedData will be used.
	, XSO_RetainBufferedData = 0x0010	//!< Keep the buffered data in a cache so it can be accessed through XsDevice::getDataPacketByIndex or XsDevice::takeFirstDataPacketInQueue. This option is mutually exclusive with XSO_RetainLiveData. If both are set, XSO_RetainBufferedData will be used.
	, XSO_OrientationInLiveStream = 0x0020	//!< Compute orientation in the live stream. This is no longer (since version 4.9.2) mutually exclusive with XSO_OrientationInBufferedStream, but unless they're both explicitly set, the system will choose only one stream to use. Please note that this option does not do anything unless XSO_Orientation is also enabled.
	, XSO_OrientationInBufferedStream = 0x0040	//!< Compute orientation in the buffered stream. This is no longer (since version 4.9.2) mutually exclusive with XSO_OrientationInLiveStream, but unless they're both explicitly set, the system will choose only one stream to use. Please note that this option does not do anything unless XSO_Orientation is also enabled.

	, XSO_ApplyOrientationResetToCalData = 0x0080	//!< Apply orientation reset to calibrated acc, gyr and mag (object reset only) and heading reset to free acc. Default is enabled for the MTw family, disabled for others

	, XSO_SkipDataBundling = 0x2000	//!< When set, the onAll...DataAvailable callbacks will not be called by the master device. This prevents some internal buffering.
	, XSO_ExpectNoRetransmissionsInFile = 0x4000	//!< When set and reading a file, missing data is immediately treated as unavailable. The default behaviour is to read further in the file to see if the data was retransmitted.
	, XSO_Reserved = 0x8000		//!< Reserved for internal use

	, XSO_All					= 0x807F	//!< All options, note that setting 'all options' is not valid, but it is useful for clearing all options
};

/*! @} */
typedef enum XsOption XsOption;

//AUTO }

#ifdef __cplusplus
//! \brief Logical OR operator for XsOption values
inline XsOption operator | (XsOption a, XsOption b)
{
	return (XsOption) ((int)a | (int)b);
}
//! \brief Logical AND operator for XsOption values
inline XsOption operator & (XsOption a, XsOption b)
{
	return (XsOption) ((int)a & (int)b);
}
//! \brief Logical XOR operator for XsOption values
inline XsOption operator ^ (XsOption a, XsOption b)
{
	return (XsOption) ((int)a ^ (int)b);
}
//! \brief Logical NEG operator for XsOption values
inline XsOption operator ~ (XsOption a)
{
	return (XsOption) (~(int)a);
}
//! \brief Return the option with mutually exclusive values 'fixed'
inline XsOption XsOption_purify(XsOption a)
{
	if ((a & XSO_RetainLiveData) && (a & XSO_RetainBufferedData))
		a = a & ~XSO_RetainLiveData;

	return a;
}
#endif

#endif
