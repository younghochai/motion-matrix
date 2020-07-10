
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

#ifndef XSDEVICEPTRLIST_H
#define XSDEVICEPTRLIST_H

#include "xsdeviceptrarray.h"
#define XsDevicePtrList XsDevicePtrArray

#ifndef __cplusplus

#define XSDEVICEPTRLIST_INITIALIZER		XSDEVICEPTRARRAY_INITIALIZER

#define XsDevicePtrList_assign(thisPtr, size, src)		XsArray_assign(thisPtr, size, src)
#define XsDevicePtrList_construct(thisPtr, size, src)	XsDevicePtrArray_construct(thisPtr, size, src)
#define XsDevicePtrList_destruct(thisPtr)				XsArray_destruct(thisPtr)
#define XsDevicePtrList_copy(thisPtr, copy)				XsArray_copy(copy, thisPtr)
#define XsDevicePtrList_append(thisPtr, other)			XsArray_append(thisPtr, other)
#define XsDevicePtrList_popFront(thisPtr, count)		XsArray_erase(thisPtr, 0, count)
#define XsDevicePtrList_popBack(thisPtr, count)			XsArray_erase(thisPtr, (XsSize)-1, count)
#define XsDevicePtrList_swap(a, b)						XsArray_swap(a, b)
#define XsDevicePtrList_erase(thisPtr, index, count)	XsArray_erase(thisPtr, index, count)

#endif
#endif
