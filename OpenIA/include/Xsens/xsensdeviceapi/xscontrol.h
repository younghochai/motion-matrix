
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

#ifndef GUARD_XSCONTROL_H_TMP
#define GUARD_XSCONTROL_H_TMP
#include <xstypes/pstdint.h>
#include <xstypes/xstypedefs.h>
#include <xstypes/xsexception.h>
#include <xstypes/xsstring.h>
#include <xstypes/xsportinfo.h>
#include <xstypes/xsportinfoarray.h>
#include <xstypes/xssyncsetting.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xsmessage.h>
#include <xstypes/xsresultvalue.h>
#include <xstypes/xsbaud.h>
#include <xstypes/xsxbusmessageid.h>
#include <xstypes/xsdeviceidarray.h>
#include <xstypes/xsintlist.h>
#include <xstypes/xssyncsettingarray.h>
#include "xsdevice.h"
#include <xstypes/xsfilepos.h>
#include <xscontroller/xsfilterprofilearray.h>
#include <xscontroller/xsoption.h>
#include <xscontroller/xsdeviceconfiguration.h>
#include <xscontroller/xsfilterprofile.h>
#include <xscontroller/xsdeviceptrarray.h>
#ifdef __cplusplus
extern "C" {
#endif
/*! \addtogroup cinterface C Interface
	@{ */
struct XsControl;
typedef struct XsControl XsControl;
XDA_DLL_API struct XsControl* XsControl_construct(void);
XDA_DLL_API void XsControl_destruct(struct XsControl* thisPtr);/*!< \copydoc XsControl::~XsControl()*/
XDA_DLL_API void XsControl_flushInputBuffers(struct XsControl* thisPtr);/*!< \copydoc XsControl::flushInputBuffers()*/
XDA_DLL_API XsString* XsControl_resultText(XsString* returnValue, XsResultValue resultCode);/*!< \copydoc XsControl::resultText(XsResultValue)*/
XDA_DLL_API void XsControl_clearHardwareError(struct XsControl* thisPtr);/*!< \copydoc XsControl::clearHardwareError()*/
XDA_DLL_API void XsControl_close(struct XsControl* thisPtr);/*!< \copydoc XsControl::close()*/
XDA_DLL_API int XsControl_openPort(struct XsControl* thisPtr, const XsString* portname, XsBaudRate baudrate, uint32_t timeout, int detectRs485);/*!< \copydoc XsControl::openPort(const XsString&,XsBaudRate,uint32_t,bool)*/
XDA_DLL_API int XsControl_openPort_1(struct XsControl* thisPtr, XsPortInfo* portinfo, uint32_t timeout, int detectRs485);/*!< \copydoc XsControl::openPort(XsPortInfo&,uint32_t,bool)*/
XDA_DLL_API int XsControl_openPortWithCredentials(struct XsControl* thisPtr, XsPortInfo* portinfo, const XsString* id, const XsString* key, uint32_t timeout);/*!< \copydoc XsControl::openPortWithCredentials(XsPortInfo&,const XsString&,const XsString&,uint32_t)*/
XDA_DLL_API int XsControl_openCustomPort(struct XsControl* thisPtr, int channelId, uint32_t channelLatency, int detectRs485);/*!< \copydoc XsControl::openCustomPort(int,uint32_t,bool)*/
XDA_DLL_API int XsControl_openImarPort_internal(struct XsControl* thisPtr, const XsString* portname, XsBaudRate baudrate, int imarType, uint32_t timeout);/*!< \private*/
XDA_DLL_API int XsControl_openPort_2(struct XsControl* thisPtr, int portNr, XsBaudRate baudrate, uint32_t timeout, int detectRs485);/*!< \copydoc XsControl::openPort(int,XsBaudRate,uint32_t,bool)*/
XDA_DLL_API void XsControl_closePort(struct XsControl* thisPtr, const XsString* portname);/*!< \copydoc XsControl::closePort(const XsString&)*/
XDA_DLL_API void XsControl_closePort_1(struct XsControl* thisPtr, const XsDeviceId* deviceId);/*!< \copydoc XsControl::closePort(const XsDeviceId&)*/
XDA_DLL_API void XsControl_closePort_2(struct XsControl* thisPtr, const XsPortInfo* portinfo);/*!< \copydoc XsControl::closePort(const XsPortInfo&)*/
XDA_DLL_API void XsControl_closeCustomPort(struct XsControl* thisPtr, int channelId);/*!< \copydoc XsControl::closeCustomPort(int)*/
XDA_DLL_API void XsControl_closePort_3(struct XsControl* thisPtr, int portNr);/*!< \copydoc XsControl::closePort(int)*/
XDA_DLL_API void XsControl_closePort_4(struct XsControl* thisPtr, XsDevice* device);/*!< \copydoc XsControl::closePort(XsDevice*)*/
XDA_DLL_API XsPortInfo* XsControl_customPortInfo(const struct XsControl* thisPtr, XsPortInfo* returnValue, int channelId);/*!< \copydoc XsControl::customPortInfo(int) const*/
XDA_DLL_API int XsControl_openLogFile(struct XsControl* thisPtr, const XsString* filename);/*!< \copydoc XsControl::openLogFile(const XsString&)*/
XDA_DLL_API XsResultValue XsControl_lastResult(const struct XsControl* thisPtr);/*!< \copydoc XsControl::lastResult() const*/
XDA_DLL_API XsString* XsControl_lastResultText(const struct XsControl* thisPtr, XsString* returnValue);/*!< \copydoc XsControl::lastResultText() const*/
XDA_DLL_API XsResultValue XsControl_lastHardwareError(const struct XsControl* thisPtr);/*!< \copydoc XsControl::lastHardwareError() const*/
XDA_DLL_API XsDeviceId* XsControl_lastHardwareErrorDeviceId(const struct XsControl* thisPtr, XsDeviceId* returnValue);/*!< \copydoc XsControl::lastHardwareErrorDeviceId() const*/
XDA_DLL_API int XsControl_deviceCount(const struct XsControl* thisPtr);/*!< \copydoc XsControl::deviceCount() const*/
XDA_DLL_API int XsControl_mainDeviceCount(const struct XsControl* thisPtr);/*!< \copydoc XsControl::mainDeviceCount() const*/
XDA_DLL_API XsDeviceIdArray* XsControl_mainDeviceIds(const struct XsControl* thisPtr, XsDeviceIdArray* returnValue);/*!< \copydoc XsControl::mainDeviceIds() const*/
XDA_DLL_API int XsControl_mtCount(const struct XsControl* thisPtr);/*!< \copydoc XsControl::mtCount() const*/
XDA_DLL_API XsDeviceIdArray* XsControl_mtDeviceIds(const struct XsControl* thisPtr, XsDeviceIdArray* returnValue);/*!< \copydoc XsControl::mtDeviceIds() const*/
XDA_DLL_API XsDeviceIdArray* XsControl_deviceIds(const struct XsControl* thisPtr, XsDeviceIdArray* returnValue);/*!< \copydoc XsControl::deviceIds() const*/
XDA_DLL_API XsDevice* XsControl_getDeviceFromLocationId(const struct XsControl* thisPtr, uint16_t locationId);/*!< \copydoc XsControl::getDeviceFromLocationId(uint16_t) const*/
XDA_DLL_API XsDeviceId* XsControl_dockDeviceId(const struct XsControl* thisPtr, XsDeviceId* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsControl::dockDeviceId(const XsDeviceId&) const*/
XDA_DLL_API int XsControl_isDeviceWireless(const struct XsControl* thisPtr, const XsDeviceId* deviceId);/*!< \copydoc XsControl::isDeviceWireless(const XsDeviceId&) const*/
XDA_DLL_API int XsControl_isDeviceDocked(const struct XsControl* thisPtr, const XsDeviceId* deviceId);/*!< \copydoc XsControl::isDeviceDocked(const XsDeviceId&) const*/
XDA_DLL_API int XsControl_loadFilterProfiles(struct XsControl* thisPtr, const XsString* filename);/*!< \copydoc XsControl::loadFilterProfiles(const XsString&)*/
XDA_DLL_API XsOption XsControl_enabledOptions(const struct XsControl* thisPtr);/*!< \copydoc XsControl::enabledOptions() const*/
XDA_DLL_API XsOption XsControl_disabledOptions(const struct XsControl* thisPtr);/*!< \copydoc XsControl::disabledOptions() const*/
XDA_DLL_API void XsControl_setOptions(struct XsControl* thisPtr, XsOption enable, XsOption disable);/*!< \copydoc XsControl::setOptions(XsOption,XsOption)*/
XDA_DLL_API void XsControl_setOptionsForce(struct XsControl* thisPtr, XsOption enabled);/*!< \copydoc XsControl::setOptionsForce(XsOption)*/
XDA_DLL_API int XsControl_setInitialPositionLLA(struct XsControl* thisPtr, const XsVector* lla);/*!< \copydoc XsControl::setInitialPositionLLA(const XsVector&)*/
XDA_DLL_API XsDevice* XsControl_device(const struct XsControl* thisPtr, const XsDeviceId* deviceId);/*!< \copydoc XsControl::device(const XsDeviceId&) const*/
XDA_DLL_API XsDevicePtrArray* XsControl_mainDevices(const struct XsControl* thisPtr, XsDevicePtrArray* returnValue);/*!< \copydoc XsControl::mainDevices() const*/
XDA_DLL_API XsDevice* XsControl_broadcast(const struct XsControl* thisPtr);/*!< \copydoc XsControl::broadcast() const*/
XDA_DLL_API void XsControl_transmissionReceived(struct XsControl* thisPtr, int channelId, const XsByteArray* data);/*!< \copydoc XsControl::transmissionReceived(int,const XsByteArray&)*/
XDA_DLL_API void XsControl_clearCallbackHandlers(struct XsControl* thisPtr, int chain);/*!< \copydoc XsControl::clearCallbackHandlers(bool)*/
XDA_DLL_API void XsControl_addCallbackHandler(struct XsControl* thisPtr, XsCallbackPlainC* cb, int chain);/*!< \copydoc XsControl::addCallbackHandler(XsCallbackPlainC*,bool)*/
XDA_DLL_API void XsControl_removeCallbackHandler(struct XsControl* thisPtr, XsCallbackPlainC* cb, int chain);/*!< \copydoc XsControl::removeCallbackHandler(XsCallbackPlainC*,bool)*/
XDA_DLL_API void XsControl_gotoConfig(struct XsControl* thisPtr);/*!< \copydoc XsControl::gotoConfig()*/
XDA_DLL_API void XsControl_gotoMeasurement(struct XsControl* thisPtr);/*!< \copydoc XsControl::gotoMeasurement()*/
XDA_DLL_API XsResultValue XsControl_startRestoreCommunication(struct XsControl* thisPtr, const XsString* portName);/*!< \copydoc XsControl::startRestoreCommunication(const XsString&)*/
XDA_DLL_API void XsControl_stopRestoreCommunication(struct XsControl* thisPtr);/*!< \copydoc XsControl::stopRestoreCommunication()*/
/*! @} */
#ifdef __cplusplus
} // extern "C"
struct XsControl {
	//! \brief Construct a new XsControl* object. Clean it up with the destruct() function or delete the object
	inline static XsControl* construct(void)
	{
		return XsControl_construct();
	}

	//! \brief Destruct a XsControl object and free all memory allocated for it
	inline void destruct(void)
	{
		XsControl_destruct(this);
	}

	inline void flushInputBuffers(void)
	{
		XsControl_flushInputBuffers(this);
	}

	inline static XsString resultText(XsResultValue resultCode)
	{
		XsString returnValue;
		return *XsControl_resultText(&returnValue, resultCode);
	}

	inline void clearHardwareError(void)
	{
		XsControl_clearHardwareError(this);
	}

	inline void close(void)
	{
		XsControl_close(this);
	}

	inline bool openPort(const XsString& portname, XsBaudRate baudrate, uint32_t timeout = 0, bool detectRs485 = false)
	{
		return 0 != XsControl_openPort(this, &portname, baudrate, timeout, detectRs485);
	}

	inline bool openPort(XsPortInfo& portinfo, uint32_t timeout = 0, bool detectRs485 = false)
	{
		return 0 != XsControl_openPort_1(this, &portinfo, timeout, detectRs485);
	}

	inline bool openPortWithCredentials(XsPortInfo& portinfo, const XsString& id, const XsString& key, uint32_t timeout = 0)
	{
		return 0 != XsControl_openPortWithCredentials(this, &portinfo, &id, &key, timeout);
	}

	inline bool openCustomPort(int channelId, uint32_t channelLatency, bool detectRs485 = false)
	{
		return 0 != XsControl_openCustomPort(this, channelId, channelLatency, detectRs485);
	}

	/*! \private*/
	inline bool openImarPort_internal(const XsString& portname, XsBaudRate baudrate, int imarType, uint32_t timeout = 0)
	{
		return 0 != XsControl_openImarPort_internal(this, &portname, baudrate, imarType, timeout);
	}

	inline bool openPort(int portNr, XsBaudRate baudrate, uint32_t timeout = 0, bool detectRs485 = false)
	{
		return 0 != XsControl_openPort_2(this, portNr, baudrate, timeout, detectRs485);
	}

	inline void closePort(const XsString& portname)
	{
		XsControl_closePort(this, &portname);
	}

	inline void closePort(const XsDeviceId& deviceId)
	{
		XsControl_closePort_1(this, &deviceId);
	}

	inline void closePort(const XsPortInfo& portinfo)
	{
		XsControl_closePort_2(this, &portinfo);
	}

	inline void closeCustomPort(int channelId)
	{
		XsControl_closeCustomPort(this, channelId);
	}

	inline void closePort(int portNr)
	{
		XsControl_closePort_3(this, portNr);
	}

	inline void closePort(XsDevice* device)
	{
		XsControl_closePort_4(this, device);
	}

	inline XsPortInfo customPortInfo(int channelId) const
	{
		XsPortInfo returnValue;
		return *XsControl_customPortInfo(this, &returnValue, channelId);
	}

	inline bool openLogFile(const XsString& filename)
	{
		return 0 != XsControl_openLogFile(this, &filename);
	}

	inline XsResultValue lastResult(void) const
	{
		return XsControl_lastResult(this);
	}

	inline XsString lastResultText(void) const
	{
		XsString returnValue;
		return *XsControl_lastResultText(this, &returnValue);
	}

	inline XsResultValue lastHardwareError(void) const
	{
		return XsControl_lastHardwareError(this);
	}

	inline XsDeviceId lastHardwareErrorDeviceId(void) const
	{
		XsDeviceId returnValue;
		return *XsControl_lastHardwareErrorDeviceId(this, &returnValue);
	}

	inline int deviceCount(void) const
	{
		return XsControl_deviceCount(this);
	}

	inline int mainDeviceCount(void) const
	{
		return XsControl_mainDeviceCount(this);
	}

	inline XsDeviceIdArray mainDeviceIds(void) const
	{
		XsDeviceIdArray returnValue;
		return *XsControl_mainDeviceIds(this, &returnValue);
	}

	inline int mtCount(void) const
	{
		return XsControl_mtCount(this);
	}

	inline XsDeviceIdArray mtDeviceIds(void) const
	{
		XsDeviceIdArray returnValue;
		return *XsControl_mtDeviceIds(this, &returnValue);
	}

	inline XsDeviceIdArray deviceIds(void) const
	{
		XsDeviceIdArray returnValue;
		return *XsControl_deviceIds(this, &returnValue);
	}

	inline XsDevice* getDeviceFromLocationId(uint16_t locationId) const
	{
		return XsControl_getDeviceFromLocationId(this, locationId);
	}

	inline XsDeviceId dockDeviceId(const XsDeviceId& deviceId) const
	{
		XsDeviceId returnValue;
		return *XsControl_dockDeviceId(this, &returnValue, &deviceId);
	}

	inline bool isDeviceWireless(const XsDeviceId& deviceId) const
	{
		return 0 != XsControl_isDeviceWireless(this, &deviceId);
	}

	inline bool isDeviceDocked(const XsDeviceId& deviceId) const
	{
		return 0 != XsControl_isDeviceDocked(this, &deviceId);
	}

	inline bool loadFilterProfiles(const XsString& filename)
	{
		return 0 != XsControl_loadFilterProfiles(this, &filename);
	}

	inline XsOption enabledOptions(void) const
	{
		return XsControl_enabledOptions(this);
	}

	inline XsOption disabledOptions(void) const
	{
		return XsControl_disabledOptions(this);
	}

	inline void setOptions(XsOption enable, XsOption disable)
	{
		XsControl_setOptions(this, enable, disable);
	}

	inline void setOptionsForce(XsOption enabled)
	{
		XsControl_setOptionsForce(this, enabled);
	}

	inline bool setInitialPositionLLA(const XsVector& lla)
	{
		return 0 != XsControl_setInitialPositionLLA(this, &lla);
	}

	inline XsDevice* device(const XsDeviceId& deviceId) const
	{
		return XsControl_device(this, &deviceId);
	}

	inline XsDevicePtrArray mainDevices(void) const
	{
		XsDevicePtrArray returnValue;
		return *XsControl_mainDevices(this, &returnValue);
	}

	inline XsDevice* broadcast(void) const
	{
		return XsControl_broadcast(this);
	}

	inline void transmissionReceived(int channelId, const XsByteArray& data)
	{
		XsControl_transmissionReceived(this, channelId, &data);
	}

	inline void clearCallbackHandlers(bool chain = true)
	{
		XsControl_clearCallbackHandlers(this, chain);
	}

	inline void addCallbackHandler(XsCallbackPlainC* cb, bool chain = true)
	{
		XsControl_addCallbackHandler(this, cb, chain);
	}

	inline void removeCallbackHandler(XsCallbackPlainC* cb, bool chain = true)
	{
		XsControl_removeCallbackHandler(this, cb, chain);
	}

	inline void gotoConfig(void)
	{
		XsControl_gotoConfig(this);
	}

	inline void gotoMeasurement(void)
	{
		XsControl_gotoMeasurement(this);
	}

	inline XsResultValue startRestoreCommunication(const XsString& portName)
	{
		return XsControl_startRestoreCommunication(this, &portName);
	}

	inline void stopRestoreCommunication(void)
	{
		XsControl_stopRestoreCommunication(this);
	}

	//! \brief Destructor, calls destruct() function to clean up object
	~XsControl()
	{
		XsControl_destruct(this);
	}

	//! \brief overloaded delete operator to allow user to use delete instead of calling destruct() function
	void operator delete (void*)
	{
	}

private:
	XsControl(); // Default constructor not implemented to prevent faulty memory allocation, use construct function instead
#ifndef SWIG
	void* operator new (size_t); //!< \brief new operator not implemented to prevent faulty memory allocation by user, use construct() function instead
	void* operator new[] (size_t); //!< \brief array new operator not implemented to prevent faulty memory allocation by user, use construct() function instead
	void operator delete[] (void*); //!< \brief array delete operator not implemented to prevent faulty memory deallocation by user, use destruct() function instead
#endif
};
#endif // __cplusplus
#endif // GUARD_XSCONTROL_H_TMP
