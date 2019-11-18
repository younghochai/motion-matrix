
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

#ifndef XSFILTERPROFILEARRAY_H
#define XSFILTERPROFILEARRAY_H

#include "xscontrollerconfig.h"
#include <xstypes/xsarray.h>

#ifdef __cplusplus
#include "xsfilterprofile.h"
extern "C" {
#endif

extern XsArrayDescriptor const XDA_DLL_API g_xsFilterProfileArrayDescriptor;

#ifndef __cplusplus
#define XSFILTERPROFILEARRAY_INITIALIZER	XSARRAY_INITIALIZER(&g_xsFilterProfileArrayDescriptor)

struct XsFilterProfile;

XSARRAY_STRUCT(XsFilterProfileArray, struct XsFilterProfile);
typedef struct XsFilterProfileArray XsFilterProfileArray;

XDA_DLL_API void XsFilterProfileArray_construct(XsFilterProfileArray* thisPtr, XsSize count, struct XsFilterProfile const* src);
#else
} // extern "C"
#endif

#ifdef __cplusplus
struct XsFilterProfileArray : public XsArrayImpl<XsFilterProfile, g_xsFilterProfileArrayDescriptor, XsFilterProfileArray> {
	//! \brief Constructs an XsFilterProfileArray
	inline explicit XsFilterProfileArray(XsSize sz = 0, XsFilterProfile const* src = 0)
		 : ArrayImpl(sz, src)
	{
	}

	//! \brief Constructs an XsFilterProfileArray as a copy of \a other
	inline XsFilterProfileArray(XsFilterProfileArray const& other)
		 : ArrayImpl(other)
	{
	}

	//! \brief Constructs an XsFilterProfileArray that references the data supplied in \a ref
	inline explicit XsFilterProfileArray(XsFilterProfile* ref, XsSize sz, XsDataFlags flags = XSDF_None)
		: ArrayImpl(ref, sz, flags)
	{
	}

#ifndef XSENS_NOITERATOR
	//! \brief Constructs an XsFilterProfileArray with the array bound by the supplied iterators \a beginIt and \a endIt
	template <typename Iterator>
	inline XsFilterProfileArray(Iterator beginIt, Iterator endIt)
		: ArrayImpl(beginIt, endIt)
	{
	}
#endif
};
#endif
#endif
