
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef XSMUTEX_H
#define XSMUTEX_H

#include <xstypes/xstypesconfig.h>

#ifdef XSENS_WINDOWS
#include <Windows.h>
#else
#include <pthread.h>
#endif

#ifndef __cplusplus
typedef struct XsMutex XsMutex;
#endif

struct XsMutex;

/*! \addtogroup cinterface C Interface
	@{
*/
/*! \brief Create a mutex */
static inline void XsMutex_construct(struct XsMutex* thisPtr);
/*! \brief Destroy a mutex */
static inline void XsMutex_destruct(struct XsMutex* thisPtr);
/*! \brief Lock the mutex
	\return non-zero on success, zero otherwise
*/
static inline int XsMutex_lock(struct XsMutex* thisPtr);
/*! \brief Try locking the mutex
	\return non-zero on success, zero otherwise
*/
static inline int XsMutex_tryLock(struct XsMutex* thisPtr);
/*! \brief Unlock a previously locked mutex
	\return non-zero on success, zero otherwise
*/
static inline int XsMutex_unlock(struct XsMutex* thisPtr);
/*! @} */

/*! \brief A platform independent mutex implementation

\details A mutex is used to protect data from being accessed by multiple threads at the same time.

This mutex is always recursive, which means that a single thread can
lock the same mutex multiple times. Of course it is important to unlock
the mutex just as many times before other threads can access the
protected data again.

XsMutex relies heavily on code inlining. The net result will be that any
dependent code will call platform code directly.

Typical usage in C++:
\code{.cpp}
class MyData
{
public:
	MyData() : m_value(0) {}

	void setValue(int newValue);
	int value() const;

private:
	int m_value;
	mutable XsMutex m_mutex;
};

void MyData::setValue(int newValue)
{
	m_mutex.lock();
	m_value = newValue;
	m_mutex.unlock();
}

int MyData::value() const
{
	m_mutex.lock();
	int v = m_value;
	m_mutex.unlock();
	return v;
}
\endcode


Typical usage in C:
\code{.c}
struct MyData
{
	int value;
	struct XsMutex mutex;
};

struct MyData d;
d.value = 0;
XsMutex_construct(&d.mutex);

XsMutex_lock(&d.mutex);
d.value = 3;
XsMutex_unlock(&d.mutex);

XsMutex_destruct(&d.mutex);
\endcode
*/
struct XsMutex
{
#ifdef __cplusplus
public:
	/*! \brief Create a mutex */
	inline XsMutex()
	{
		XsMutex_construct(this);
	}

	/*! \brief Destroy a mutex */
	inline ~XsMutex()
	{
		XsMutex_destruct(this);
	}

	/*! \brief Lock the mutex
		\return true on success
	*/
	inline bool lock()
	{
		return XsMutex_lock(this) != 0;
	}

	/*! \brief Try locking the mutex
		\return true on success
	*/
	inline bool tryLock()
	{
		return XsMutex_tryLock(this) != 0;
	}

	/*! \brief Unlock a previously locked mutex
		\return true on success
	*/
	inline bool unlock()
	{
		return XsMutex_unlock(this) != 0;
	}
private:
	friend inline void XsMutex_construct(struct XsMutex*);
	friend inline void XsMutex_destruct(struct XsMutex*);
	friend inline int XsMutex_lock(struct XsMutex*);
	friend inline int XsMutex_tryLock(struct XsMutex*);
	friend inline int XsMutex_unlock(struct XsMutex*);
#endif
#ifdef XSENS_WINDOWS
	CRITICAL_SECTION m_mutex;
#else
	pthread_mutexattr_t m_attr;
	pthread_mutex_t m_mutex;
#endif
};

#ifdef XSENS_WINDOWS
static inline void XsMutex_construct(struct XsMutex* thisPtr)
{
	InitializeCriticalSection(&thisPtr->m_mutex);
}

static inline void XsMutex_destruct(struct XsMutex* thisPtr)
{
	DeleteCriticalSection(&thisPtr->m_mutex);
}

static inline int XsMutex_lock(struct XsMutex* thisPtr)
{
	EnterCriticalSection(&thisPtr->m_mutex);
	return TRUE;
}

static inline int XsMutex_tryLock(struct XsMutex* thisPtr)
{
	return TryEnterCriticalSection(&thisPtr->m_mutex) == 1;
}

static inline int XsMutex_unlock(struct XsMutex* thisPtr)
{
	LeaveCriticalSection(&thisPtr->m_mutex);
	return TRUE;
}
#else
static inline void XsMutex_construct(struct XsMutex* thisPtr)
{
	pthread_mutexattr_init(&thisPtr->m_attr);
	pthread_mutexattr_settype(&thisPtr->m_attr, PTHREAD_MUTEX_RECURSIVE);
	pthread_mutex_init(&thisPtr->m_mutex, &thisPtr->m_attr);
}

static inline void XsMutex_destruct(struct XsMutex* thisPtr)
{
	pthread_mutex_destroy(&thisPtr->m_mutex);
	pthread_mutexattr_destroy(&thisPtr->m_attr);
}

static inline int XsMutex_lock(struct XsMutex* thisPtr)
{
	return pthread_mutex_lock(&thisPtr->m_mutex) == 0;
}

static inline int XsMutex_tryLock(struct XsMutex* thisPtr)
{
	return pthread_mutex_trylock(&thisPtr->m_mutex) == 0;
}

static inline int XsMutex_unlock(struct XsMutex* thisPtr)
{
	return pthread_mutex_unlock(&thisPtr->m_mutex) == 0;
}
#endif

#ifdef __cplusplus
/*! \brief A RAII object that ensures mutexes are unlocked when leaving scope
 *
 * When using this object, it is advised not to touch the mutex we're working
 * on. The internal states may mismatch and result in undefined behavior.
 */
struct XsMutexLocker
{
public:
	/*! \brief Create a mutex locker and lock the mutex */
	explicit XsMutexLocker(XsMutex& m) :
		m_mutex(m),
		m_locked(false)
	{
		lock();
	}

	/*! \brief Destroy the mutex locker -- unlock the mutex */
	~XsMutexLocker()
	{
		unlock();
	}

	/*! \brief Lock the mutex if it isn't already locked */
	inline bool lock()
	{
		if (!m_locked)
			return (m_locked = m_mutex.lock());
		return true;
	}

	/*! \brief Unlock the mutex if it isn't already unlocked */
	inline bool unlock()
	{
		if (m_locked)
			return !(m_locked = !m_mutex.unlock());
		return true;
	}

	/*! \brief Try to lock the mutex */
	inline bool tryLock()
	{
		if (!m_locked)
			return (m_locked = m_mutex.tryLock());
		return true;
	}

private:
	XsMutex& m_mutex;
	bool m_locked;
};
#endif

#endif
