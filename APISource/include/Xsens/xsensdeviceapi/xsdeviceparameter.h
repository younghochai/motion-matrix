
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

#ifndef XSDEVICEPARAMETER_H
#define XSDEVICEPARAMETER_H

#include "xsdeviceparameteridentifier.h"
#include <xstypes/pstdint.h>

#ifdef __cplusplus
extern "C" {
struct XsDeviceParameter;
} // extern "C"
#endif

/*!
 *	\class XsDeviceParameter
 *	\brief Class to set and retrieve parameters from a XsDevice object.
 */
struct XsDeviceParameter
{
#ifdef __cplusplus
	//!	\brief Constructor, initializes the object with a parameter \a id.
	explicit XsDeviceParameter(XsDeviceParameterIdentifier id)
		: m_id(id)
		, m_value(0)
	{
	}

	//!	\brief Constructor, initializes the object with a parameter \a id and desired \a value.
	explicit XsDeviceParameter(XsDeviceParameterIdentifier id, int value)
		: m_id(id)
		, m_value(static_cast<uint32_t>(value))
	{
	}

	//!	\brief Returns the current parameter identifier.
	XsDeviceParameterIdentifier id() const
	{
		return m_id;
	}

	/*!
	 *	\brief Returns the stored parameter value.
	 *	\returns The parameter value in the desired type.
	 */
	template<typename T>
	T getValue() const;


	/*!
	 *	\brief Sets the parameter value.
	 *	\param value: the desired parameter value.
	 */
	template<typename T>
	void setValue(T value);

private:
#endif
	XsDeviceParameterIdentifier m_id;
	int m_value;
};


#ifdef __cplusplus
template<>
//! \copydoc XsDeviceParameter::getValue
inline bool XsDeviceParameter::getValue<bool>() const
{
	return m_value > 0;
}

//! \copydoc XsDeviceParameter::getValue
template<>
inline uint8_t XsDeviceParameter::getValue<uint8_t>() const
{
	return static_cast<uint8_t>(m_value);
}

//! \copydoc XsDeviceParameter::getValue
template<>
inline uint16_t XsDeviceParameter::getValue<uint16_t>() const
{
	return static_cast<uint16_t>(m_value);
}

//! \copydoc XsDeviceParameter::getValue
template<>
inline uint32_t XsDeviceParameter::getValue<uint32_t>() const
{
	return static_cast<uint32_t>(m_value);
}

//! \copydoc XsDeviceParameter::getValue
template<>
inline int XsDeviceParameter::getValue<int>() const
{
	return m_value;
}

//! \copydoc XsDeviceParameter::setValue
template<>
inline void XsDeviceParameter::setValue<bool>(bool value)
{
	m_value = value ? 1 :0;
}

//! \copydoc XsDeviceParameter::setValue
template<>
inline void XsDeviceParameter::setValue<uint8_t>(uint8_t value)
{
	m_value = static_cast<int>(value);
}

//! \copydoc XsDeviceParameter::setValue
template<>
inline void XsDeviceParameter::setValue<uint16_t>(uint16_t value)
{
	m_value = static_cast<int>(value);
}

//! \copydoc XsDeviceParameter::setValue
template<>
inline void XsDeviceParameter::setValue<uint32_t>(uint32_t value)
{
	m_value = static_cast<int>(value);
}

//! \copydoc XsDeviceParameter::setValue
template<>
inline void XsDeviceParameter::setValue<int>(int value)
{
	m_value = value;
}
#endif

typedef struct XsDeviceParameter XsDeviceParameter;

#endif
