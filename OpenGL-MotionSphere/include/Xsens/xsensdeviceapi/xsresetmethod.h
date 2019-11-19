
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

#ifndef XSRESETMETHOD_H
#define XSRESETMETHOD_H

/*!	\addtogroup enums Global enumerations
	@{
*/

//AUTO namespace xscontroller {
/*! \brief Orientation reset type. */
enum XsResetMethod {
	XRM_StoreAlignmentMatrix	= 0,				//!< \brief Store the current object alignment matrix to persistent memory
	XRM_Heading					= 1,				//!< \brief Reset the heading (yaw)
	XRM_Object					= 3,				//!< \brief Reset the attitude (roll, pitch), same as XRM_Inclination
	XRM_Inclination				= XRM_Object,		//!< \brief Reset the inclination (roll, pitch), same as XRM_Object
	XRM_Alignment				= 4,				//!< \brief Reset heading and attitude \details This effectively combines the XRM_Heading and XRM_Object
	XRM_Global					= XRM_Alignment,	//!< \brief Reset the full orientation of the device \details Obsolete. Use XRM_Alignment instead.
	XRM_DefaultHeading			= 5,				//!< \brief Revert to default behaviour for heading, undoes XRM_Heading
	XRM_DefaultInclination		= 6,				//!< \brief Revert to default behaviour for inclination, undoes XRM_Inclination
	XRM_DefaultAlignment		= 7,				//!< \brief Revert to default behaviour for heading and inclination, undoes any reset
	XRM_None										//!< \brief No reset planned
};
/*! @} */
typedef enum XsResetMethod XsResetMethod;
//AUTO }

#endif
