
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

#ifndef XSDEVICECONFIGURATION_H
#define XSDEVICECONFIGURATION_H

#include "xscontrollerconfig.h"
#include <xstypes/pstdint.h>
#include <xstypes/xsbusid.h>
#include <xstypes/xstypedefs.h>
#ifdef __cplusplus
#include <xstypes/xsexception.h>
#endif

struct MtwInfo;
struct XsDeviceConfiguration;
struct XsMessage;
struct XsDeviceId;

#ifdef __cplusplus
extern "C" {
#else
#define XSDEVICEINFO_INITIALIZER = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define XSDEVICECONFIGURATION_INITIALIZER = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#endif

XDA_DLL_API void XsDeviceConfiguration_construct(struct XsDeviceConfiguration* thisPtr);
XDA_DLL_API void XsDeviceConfiguration_assign(struct XsDeviceConfiguration* thisPtr, XsSize numberOfDevices, const struct XsDeviceConfiguration* src);
XDA_DLL_API void XsDeviceConfiguration_destruct(struct XsDeviceConfiguration* thisPtr);
XDA_DLL_API void XsDeviceConfiguration_copy(struct XsDeviceConfiguration* copy, struct XsDeviceConfiguration const* src);
XDA_DLL_API int  XsDeviceConfiguration_empty(const struct XsDeviceConfiguration* thisPtr);
XDA_DLL_API void XsDeviceConfiguration_readFromMessage(struct XsDeviceConfiguration* thisPtr, const struct XsMessage* msg);
XDA_DLL_API void XsDeviceConfiguration_writeToMessage(const struct XsDeviceConfiguration* thisPtr, struct XsMessage* msg);
XDA_DLL_API XsSize XsDeviceConfiguration_findDevice(const struct XsDeviceConfiguration* thisPtr, const struct XsDeviceId* deviceId);

#ifdef __cplusplus
} // extern "C"
#endif

/*! \brief %Device information for MT devices in an XsDeviceConfiguration. */
struct XsMtDeviceConfiguration {
	uint32_t	m_deviceId;			/*!< \brief This device ID */
	uint16_t	m_dataLength;			/*!< \brief The total length of the data */
	uint16_t	m_outputMode;			/*!< \brief The legacy output mode \sa XsOutputMode */
	uint16_t	m_filterProfile;		/*!< \brief The currently chosen filter profile */
	uint8_t		m_fwRevMajor;			/*!< \brief The major version of the firmware */
	uint8_t		m_fwRevMinor;			/*!< \brief The minor version of the firmware */
	uint8_t		m_fwRevRevision;		/*!< \brief The revision version of the firmware */
	uint8_t		m_filterType;			/*!< \brief The filter type */
	uint8_t		m_filterMajor;			/*!< \brief The filter major version */
	uint8_t		m_filterMinor;			/*!< \brief The filter minor version */
};

typedef struct XsMtDeviceConfiguration XsMtDeviceConfiguration;

/*! \brief Device information for the main device in an XsDeviceConfiguration. */
struct XsMasterDeviceConfiguration {
	uint32_t		m_masterDeviceId;	/*!< \brief The master device ID */
	uint16_t		m_samplingPeriod;	/*!< \brief The sampling period */
	uint16_t		m_outputSkipFactor;	/*!< \brief The output skip factor */
	uint16_t		m_syncInMode;		/*!< \brief The sync-in mode */
	uint16_t		m_syncInSkipFactor;	/*!< \brief The sync-in skip factor */
	uint32_t		m_syncInOffset;		/*!< \brief The sync-in offset */
	uint8_t			m_date[8];		/*!< \brief The date */
	uint8_t			m_time[8];		/*!< \brief The time */
	uint8_t			m_reservedForHost[32];	/*!< \brief Reserved space */
	uint8_t			m_reservedForClient[32];/*!< \brief Reserved space */
};
typedef struct XsMasterDeviceConfiguration XsMasterDeviceConfiguration;

#ifdef __cplusplus
/*! \class XsDeviceConfigurationException
	Exception class thrown when an exception occured inside the XsDeviceConfiguration
*/
class XsDeviceConfigurationException : public XsException {
public:
	XsDeviceConfigurationException() : XsException("Invalid device configuration") {}
};
#endif

/*! \brief Structure containing a full device configuration as returned by the ReqConfig message. */
struct XsDeviceConfiguration {
#ifdef __cplusplus
	/*! \brief Constructor

	  \param numberOfDevs : The number of devices for which memory should be allocated in the XsDeviceConfiguration

	  \sa XsDeviceConfiguration_construct
	*/
	explicit XsDeviceConfiguration(uint16_t numberOfDevs = 0)
		: m_numberOfDevices(0)
		, m_deviceInfo(0)
	{
		memset(this, 0, sizeof(XsDeviceConfiguration));
		if (numberOfDevs)
			XsDeviceConfiguration_assign(this, numberOfDevs, 0);
	}

	/*! \brief Copy constructor
	  \param other the object to copy
	  \sa XsDeviceConfiguration_copy
	 */
	XsDeviceConfiguration(const XsDeviceConfiguration& other)
		: m_numberOfDevices(0)
		, m_deviceInfo(0)
	{
		memset(this, 0, sizeof(XsDeviceConfiguration));
		XsDeviceConfiguration_copy(this, &other);
	}

	/*! \brief Assign \a other to this device configuaration

	  \param other the object to copy

	  \returns a const reference to this object
	  \sa XsDeviceConfiguration_copy
	 */
	inline const XsDeviceConfiguration& operator = (const XsDeviceConfiguration& other)
	{
		if (this != &other)
			XsDeviceConfiguration_copy(this, &other);
		return *this;
	}

	/*! \brief Destructor \sa XsDeviceConfiguration_destruct */
	inline ~XsDeviceConfiguration()
	{
		XsDeviceConfiguration_destruct(this);
	}

	/*! \brief Clears and frees data \sa XsDeviceConfiguration_destruct */
	inline void clear()
	{
		XsDeviceConfiguration_destruct(this);
	}

	/*! \brief Test if this object is empty
	  \returns true if this object is empty
	*/
	inline bool empty() const
	{
		return m_numberOfDevices == 0;
	}

	/*! \brief \copybrief XsDeviceConfiguration_readFromMessage

	   \param msg the message to read the device configuration from

	   \sa XsDeviceConfiguration_readFromMessage
	 */
	inline void readFromMessage(const XsMessage &msg)
	{
		XsDeviceConfiguration_readFromMessage(this, &msg);
	}

	/*! \brief \copybrief XsDeviceConfiguration_writeToMessage

	   \param msg the message to write the device configuration to

	   \sa XsDeviceConfiguration_writeToMessage
	 */
	inline void writeToMessage(XsMessage& msg) const
	{
		XsDeviceConfiguration_writeToMessage(this, &msg);
	}

	/*! \brief Return true if this contains device info for \a deviceId

	  \param deviceId the device ID to find in this configuration
	  \returns true if the device is present in the configuration
	*/
	inline bool containsDevice(const XsDeviceId& deviceId)
	{
		return XsDeviceConfiguration_findDevice(this, &deviceId) != 0;
	}

	/*! \brief The deviceInfo for the \a deviceId
	  \param deviceId the device ID to identify with
	  \returns the deviceInfo structure for \a deviceId
	*/
	inline XsMtDeviceConfiguration& deviceInfo(const XsDeviceId& deviceId)
	{
		XsSize busId = XsDeviceConfiguration_findDevice(this, &deviceId);
		if (busId == 0)
			throw XsDeviceConfigurationException();

		return deviceInfo(busId);
	}

	/*! \brief The device info for the device at \a busId

	  \param busId the bus ID of the device for which to return data for

	  \returns a reference to the device configuration for the device at \a busId
	*/
	inline XsMtDeviceConfiguration& deviceInfo(XsSize busId)
	{
		if (!m_numberOfDevices)
			throw XsDeviceConfigurationException();

		if (busId == XS_BID_MASTER)
			return m_deviceInfo[0];

		if (busId > m_numberOfDevices)
			throw XsDeviceConfigurationException();

		return m_deviceInfo[busId-1];
	}

	/*! \brief The device info for the device at \a busId

	  \param busId the bus ID of the device for which to return data for

	  \returns a const reference to the device configuration for the device at \a busId
	*/
	inline const XsMtDeviceConfiguration& deviceInfo(XsSize busId) const
	{
		if (!m_numberOfDevices)
			throw XsDeviceConfigurationException();

		if (busId == XS_BID_MASTER)
			return m_deviceInfo[0];

		if (busId > m_numberOfDevices)
			throw XsDeviceConfigurationException();

		return m_deviceInfo[busId-1];
	}


	/*! \brief The device info for the master device

	  \returns a reference to the device configuration for the master device
	*/
	inline XsMasterDeviceConfiguration& masterInfo()
	{
		return m_masterInfo;
	}

	/*! \brief The device info for the master device

	  \returns a const reference to the device configuration for the master device
	*/
	inline const XsMasterDeviceConfiguration& masterInfo() const
	{
		return m_masterInfo;
	}

	/*! \brief Set the number of devices to \a count
	  \param count the new number of devices to allocate for
	 */
	inline void setNumberOfDevices(XsSize count)
	{
		XsDeviceConfiguration_assign(this, count, 0);
	}

	/*! \brief The current number of devices
	  \returns the number of devices
	 */
	inline XsSize numberOfDevices() const
	{
		return (XsSize) m_numberOfDevices;
	}

	/*! \brief \copybrief numberOfDevices
	  \copydetails numberOfDevices
	*/
	inline XsSize deviceCount() const
	{
		return numberOfDevices();
	}

private:
//! \protectedsection
#endif
	XsMasterDeviceConfiguration m_masterInfo;	//!< \brief The master info
	const uint16_t	m_numberOfDevices;		//!< \brief The currently allocated number of devices
	XsMtDeviceConfiguration* const m_deviceInfo;	//!< \brief The list of device infos
};

typedef struct XsDeviceConfiguration XsDeviceConfiguration;

#endif
