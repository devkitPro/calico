// SPDX-License-Identifier: ZPL-2.1
// SPDX-FileCopyrightText: Copyright fincs, devkitPro
#include <calico/types.h>
#include <calico/system/mutex.h>
#include <calico/system/condvar.h>
#include <calico/system/thread.h>
#include <errno.h>
#include <malloc.h>
#include <sys/iosupport.h>

#if defined(__NDS__)
#include "../nds/transfer.h"

MK_INLINE time_t _getUnixTime(void)
{
	return s_transferRegion->unix_time;
}

#else

MK_INLINE time_t _getUnixTime(void)
{
	return 0;
}

#endif

struct __pthread_t {
	Thread base;
};

// Dummy symbol referenced by crt0 so that this object file is pulled in by the linker
const u32 __newlib_syscalls = 0xdeadbeef;

struct _reent* __SYSCALL(getreent)(void)
{
	return (struct _reent*)threadGetSelf()->impure;
}

int* __errno(void)
{
	static int s_defaultErrno;
	struct _reent* r = __SYSCALL(getreent)();
	return r ? &r->_errno : &s_defaultErrno;
}

int __SYSCALL(gettod_r)(struct _reent* ptr, struct timeval* tp, struct timezone* tz)
{
	if (tp) {
		tp->tv_sec = _getUnixTime();
		tp->tv_usec = 0;
	}

	if (tz) {
		tz->tz_minuteswest = 0;
		tz->tz_dsttime = 0;
	}

	return 0;
}

MK_CODE32 int __SYSCALL(nanosleep)(const struct timespec* req, struct timespec* rem)
{
	threadSleep((unsigned)req->tv_sec*1000000U + (unsigned)req->tv_nsec/1000U);
	return 0;
}

void __SYSCALL(lock_init)(_LOCK_T* lock)
{
	*lock = (_LOCK_T){0};
}

void __SYSCALL(lock_acquire)(_LOCK_T* lock)
{
	mutexLock((Mutex*)lock);
}

int __SYSCALL(lock_try_acquire)(_LOCK_T* lock)
{
	return mutexTryLock((Mutex*)lock) ? 0 : 1;
}

void __SYSCALL(lock_release)(_LOCK_T* lock)
{
	mutexUnlock((Mutex*)lock);
}

void __SYSCALL(lock_init_recursive)(_LOCK_RECURSIVE_T* lock)
{
	*lock = (_LOCK_RECURSIVE_T){0};
}

void __SYSCALL(lock_acquire_recursive)(_LOCK_RECURSIVE_T* lock)
{
	rmutexLock((RMutex*)lock);
}

int __SYSCALL(lock_try_acquire_recursive)(_LOCK_RECURSIVE_T* lock)
{
	return rmutexTryLock((RMutex*)lock) ? 0 : 1;
}

void __SYSCALL(lock_release_recursive)(_LOCK_RECURSIVE_T* lock)
{
	rmutexUnlock((RMutex*)lock);
}

int __SYSCALL(cond_signal)(_COND_T* cond)
{
	condvarSignal((CondVar*)cond);
	return 0;
}

int __SYSCALL(cond_broadcast)(_COND_T* cond)
{
	condvarBroadcast((CondVar*)cond);
	return 0;
}

int __SYSCALL(cond_wait)(_COND_T* cond, _LOCK_T* lock, uint64_t timeout_ns)
{
	if (timeout_ns != UINT64_MAX) {
		return ETIMEDOUT;
	}

	condvarWait((CondVar*)cond, (Mutex*)lock);
	return 0;
}

int __SYSCALL(cond_wait_recursive)(_COND_T* cond, _LOCK_RECURSIVE_T* lock, uint64_t timeout_ns)
{
	if (timeout_ns != UINT64_MAX) {
		return ETIMEDOUT;
	}

	RMutex* r = (RMutex*)lock;
	u32 counter_backup = r->counter;
	r->counter = 0;

	condvarWait((CondVar*)cond, &r->mutex);

	r->counter = counter_backup;
	return 0;
}

static size_t extraTls = 0;

void threadSetPthreadExtraTls(const size_t size)
{
	// make sure it's 8-byte aligned
	extraTls = (size + 7) &~ 7;
}

int __SYSCALL(thread_create)(struct __pthread_t** thread, void* (*func)(void*), void* arg, void* stack_addr, size_t stack_size)
{
	if (((uptr)stack_addr & 7) || (stack_size & 7)) {
		return EINVAL;
	}

	if (!stack_size) {
		stack_size = 8*1024;
	}

	size_t struct_sz = (sizeof(struct __pthread_t) + 7) &~ 7;

	size_t needed_sz = struct_sz + threadGetLocalStorageSize() + extraTls;
	if (!stack_addr) {
		needed_sz += stack_size;
	}

	*thread = _malloc_r(__SYSCALL(getreent)(), needed_sz); // malloc align is 2*sizeof(void*); which is already 8
	if (!*thread) {
		return ENOMEM;
	}

	void* stack_top;
	if (stack_addr) {
		stack_top = (u8*)stack_addr + stack_size;
	} else {
		stack_top = (u8*)*thread + needed_sz;
	}

	Thread* t = &(*thread)->base;
	threadPrepare(t, (ThreadFunc)func, arg, stack_top, THREAD_MIN_PRIO);
	threadAttachLocalStorage(t, (u8*)*thread + struct_sz);
	threadStart(t);

	return 0;
}

void* __SYSCALL(thread_join)(struct __pthread_t* thread)
{
	void* rc = (void*)threadJoin(&thread->base);
	_free_r(__SYSCALL(getreent)(), thread);
	return rc;
}

void __SYSCALL(thread_exit)(void* value)
{
	threadExit((int)value);
}

struct __pthread_t* __SYSCALL(thread_self)(void)
{
	return (struct __pthread_t*)threadGetSelf();
}
