
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

#ifndef XSFILTERPROFILE_H
#define XSFILTERPROFILE_H

#include "xscontrollerconfig.h"
#include <xstypes/xsstring.h>
#include <xstypes/pstdint.h>

#ifdef __cplusplus
extern "C" {
#else
#define XSFILTERPROFILE_INITIALIZER {0, 0, {0}, 0, 0, 0}
#endif

struct XsFilterProfile;
XDA_DLL_API void XsFilterProfile_toString(struct XsFilterProfile* thisPtr, XsString *out);
XDA_DLL_API int XsFilterProfile_empty(struct XsFilterProfile* thisPtr);
XDA_DLL_API void XsFilterProfile_swap(struct XsFilterProfile* a, struct XsFilterProfile* b);
#ifdef __cplusplus
}
#endif

#define XS_MAX_FILTERPROFILES			254
#define XS_LEN_FILTERPROFILELABEL_TERM	(20+1)
#define XS_MAX_FILTERPROFILES_IN_MT		5

struct XsFilterProfile
{
#ifdef __cplusplus
	/*! \brief Construct a filter profile object

	   \param type_ the profile type
	   \param version_ the profile version
	   \param label_ the profile name
	   \param filterType_ the filter type this profile is for
	   \param filterMajor_ the major version of the compatible filter
	   \param filterMinor_ the minor version of the compatible filter
	 */
	explicit XsFilterProfile(uint8_t type_ = 0, uint8_t version_ = 0, const char* label_ = 0, char filterType_ = 0, uint8_t filterMajor_ = 0, uint8_t filterMinor_ = 0)
		: m_type(type_)
		, m_version(version_)
		, m_filterType(filterType_)
		, m_filterMajor(filterMajor_)
		, m_filterMinor(filterMinor_)
	{
		setLabel(label_);
	}

	/*! \brief Copy constructor for a filter profile object
		\param other the filter profile object to construct a copy of
	*/
	XsFilterProfile(const XsFilterProfile& other)
		: m_type(other.m_type)
		, m_version(other.m_version)
		, m_filterType(other.m_filterType)
		, m_filterMajor(other.m_filterMajor)
		, m_filterMinor(other.m_filterMinor)
	{
		setLabel(other.m_label);
	}

	/*! \brief Destroy the filter profile */
	~XsFilterProfile() {}

	/*! \brief \copybrief XsFilterProfile_empty
	  \returns true if the filter profile is empty
	  \sa XsFilterProfile_empty
	*/
	inline bool empty()
	{
		return (0 != XsFilterProfile_empty(this));
	}

	/*! \brief \copybrief XsFilterProfile_toString
	  \returns a string representation of this filter profile
	  \sa XsFilterProfile_toString
	*/
	inline XsString toString()
	{
		XsString out;
		XsFilterProfile_toString(this, &out);
		return out;
	}

	/*! \brief The filter profile type */
	inline uint8_t type() const { return m_type; }

	/*! \brief The filter profile version */
	inline uint8_t version() const { return m_version; }

	/*! \brief The filter profile name */
	inline const char* label() const { return m_label; }

	/*! \brief The filter type this filter profile is for */
	inline uint8_t filterType() const { return m_filterType; }

	/*! \brief The major version of the compatible filter */
	inline uint8_t filterMajor() const { return m_filterMajor; }

	/*! \brief The minor version of the compatible filter */
	inline uint8_t filterMinor() const { return m_filterMinor; }

	/*! \brief Set the type of the filter profile to \a type_
	  \param type_ the new type of the filter profile
	*/
	inline void setType(uint8_t type_)
	{
		m_type = type_;
	}

	/*! \brief Set the version of the filter profile to \a version_
	  \param version_ the new label of the filter profile
	*/
	inline void setVersion(uint8_t version_)
	{
		m_version = version_;
	}

	/*! \brief Set the label of the filter profile \a label_
	  \param label_ the new label of the filter profile
	*/
	inline void setLabel(const char* label_)
	{
		if (!label_ || label_[0] == 0)
		{
			m_label[0] = 0;
		}
		else
		{
			int i = 0;
			for (; i < XS_LEN_FILTERPROFILELABEL_TERM-1; ++i)
			{
				if (label_[i] == '\0' || label_[i] == ' ')
					break;
				m_label[i] = label_[i];
			}
			m_label[i] = 0;
		}
	}

	/*! \brief Set the filter type of this filter profile to \a filterType_
	  \param filterType_ the new filter type
	*/
	inline void setFilterType(char filterType_)
	{
		m_filterType = filterType_;
	}

	/*! \brief Set the filter version of this filter profile to \a major_, \a minor_
	  \param major_ the major version number
	  \param minor_ the minor version number
	*/
	inline void setFilterVersion(uint8_t major_, uint8_t minor_)
	{
		m_filterMajor = major_;
		m_filterMinor = minor_;
	}

	/*! \brief Swap the contents with \a other
	*/
	inline void swap(XsFilterProfile& other)
	{
		XsFilterProfile_swap(this, &other);
	}

	/*! \brief Return true if the filter profile type and version are identical to those of \a other */
	inline bool operator == (const XsFilterProfile& other) const
	{
		return m_filterMajor == other.m_filterMajor && m_filterMinor == other.m_filterMinor;
	}

protected:
#endif

	uint8_t m_type;								//!< The type of the filter profile. When set to 255 in an operation, the 'current' filter profile is used.
	uint8_t m_version;							//!< The version of the filter profile.
	char m_label[XS_LEN_FILTERPROFILELABEL_TERM];	//!< The label of the filter profile.
	char m_filterType;							//!< The type of the XKF filter this filter profile is intended for '3': XKF-3, '6': XKF-6. \note The value is a character, so XKF-3 is '3', which is hex 0x33
	uint8_t m_filterMajor;						//!< The major version of the XKF filter this filter profile is intended for
	uint8_t m_filterMinor;						//!< The minor version of the XKF filter this filter profile is intended for
};

typedef struct XsFilterProfile XsFilterProfile;

#endif
