
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

#ifndef XSDEVICE_REF_H
#define XSDEVICE_REF_H

#ifdef __cplusplus

#include "xsdevice.h"

/*! \brief A references counted class for XsDevice.
	This class can be used to have a reference counted pointer to an XsDevice object as
	returned by XsControl.
	When used, XsDevice pointer will only be deleted when its reference count is zero.
	Enabling classes which use the device pointer to terminate gracefully before the pointer
	is deleted.
*/
class XsDeviceRef
{
public:
	/*! \brief Empty constructor
	*/
	XsDeviceRef() : m_device(0) {}

	/*! \brief Constructor accepting XsDevice pointer
		This calls XsDevices' addRef
	*/
	XsDeviceRef(XsDevice* device) : m_device(device)
	{
		assert(device == 0 || device->refCounter() > 0);
		addRef();
	}

	/*! \brief Copy constructor
	*/
	XsDeviceRef(const XsDeviceRef& deviceRef)
	{
		m_device = deviceRef.m_device;
		addRef();
	}

	/*! \brief Destructor
		This calls XsDevices' removeRef
	*/
	~XsDeviceRef(void)
	{
		removeRef();
	}

	/*! \brief Structure dereference. Return m_device pointer
		\returns XsDevice pointer
	*/
	XsDevice *operator->() const { return m_device; }

	/*! \brief Equal to operator. Compare device pointers
		\param other The pointer to compare device pointer to
		\returns true if the pointers are equal
	*/
	bool operator==(XsDeviceRef const& other) const {return m_device == other.m_device;}

	/*! \brief Not equal to operator. Compare device pointers
		\param other The pointer to compare device pointer to
		\returns true if the pointers are not equal
	*/
	bool operator!=(XsDeviceRef const& other) const {return m_device != other.m_device;}

	/*! \brief Equal to operator. Compare device pointers
		\param other The pointer to compare device pointer to
		\returns true if the pointers are equal
	*/
	bool operator==(const XsDevice* other) const {return m_device == other;}

	/*! \brief Not equal to operator. Compare device pointers
		\param other The pointer to compare device pointer to
		\returns true if the pointers are not equal
	*/
	bool operator!=(const XsDevice* other) const {return m_device != other;}
	/*! \brief Indirection operator. Return device pointer */
	operator XsDevice *() {return m_device; }
	/*! \brief Logical negation operator */
	bool operator!() const {return !m_device;}

	/*! \brief Call XsDevices' addRef
	*/
	void addRef(void)
	{
		if (m_device)
			m_device->addRef();
	}

	/*! \brief Call XsDevices' removeRef
	*/
	void removeRef(void)
	{
		if (m_device)
			m_device->removeRef();
	}

private:
//! \protectedsection
	XsDevice* m_device;	//!< The referenced XsDevice
};

#endif

#endif // fileguard
