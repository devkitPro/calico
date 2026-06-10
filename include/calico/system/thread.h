// SPDX-License-Identifier: ZPL-2.1
// SPDX-FileCopyrightText: Copyright fincs, devkitPro
#pragma once
#include "../types.h"
#include "../arm/common.h"
#include "irq.h"
#include "tick.h"

/*! @addtogroup thread

	Calico provides a simple preemptive multithreading system. Each thread is
	assigned a priority level and has its own execution state, allowing software
	to be better structured by having different processing tasks each in its own
	thread. Threads may wait on events such as interrupts. Calico ensures at all
	times that the currently running thread is always the highest priority thread
	that can run. If any event occurs that causes a higher priority thread to
	become runnable, Calico will preempt the current thread. Unlike in common PC
	operating systems, Calico presently does not implement timeslicing to share
	the CPU between threads of the same priority; meaning it is necessary to
	explicitly yield control of the CPU if any such threads exist.

	While Calico provides its own threading API, it is also possible to use standard
	threading APIs such as POSIX threads or C++ threads. These APIs are in fact
	preferred when working with cross platform projects.

	@{
*/

MK_EXTERN_C_START

typedef struct Thread Thread;

//! List of blocked threads, used as a building block for synchronization primitives
typedef struct ThrListNode {
	Thread* next; //!< @private
	Thread* prev; //!< @private
} ThrListNode;

//! @private
typedef enum ThrStatus {
	ThrStatus_Uninitialized,
	ThrStatus_Finished,
	ThrStatus_Running,
	ThrStatus_Waiting,
	ThrStatus_WaitingOnMutex,
} ThrStatus;

#define THREAD_MAX_PRIO 0x00 //!< Maximum priority value of a thread
#define THREAD_MIN_PRIO 0x3f //!< Minimum priority value of a thread

#define MAIN_THREAD_PRIO 0x1c //!< Default priority value of the main thread

//! Thread entrypoint type
typedef int (* ThreadFunc)(void* arg);

//! Data structure containing all management information for a thread
struct Thread {
	ArmContext ctx;      //!< CPU context structure

	void* tp;            //!< Virtual thread-local segment register
	void* impure;        //!< Pointer to per-thread C standard library state

	Thread* next;        //!< @private
	ThrStatus status;    //!< @private
	u8 prio;             //!< Current thread priority (including inheritance)
	u8 baseprio;         //!< Nominal thread priority (not including inheritance)
	u8 pause;            //!< @private

	ThrListNode waiters; //!< @private

	union {
		// Data for waiting threads
		struct {
			ThrListNode link;
			ThrListNode* queue;
			u32 token;
		};
		// Data for finished threads
		struct {
			int rc;
		};
	}; //!< @private
};

//! @private
typedef struct ThrSchedState {
	Thread *cur;
	Thread *deferred;
	Thread *first;
	IrqMask irqWaitMask;
	ThrListNode irqWaitList;
#if MK_IRQ_NUM_HANDLERS > 32
	IrqMask irqWaitMask2;
	ThrListNode irqWaitList2;
#endif
} ThrSchedState;

/*! @name Thread initialization and synchronization

	This group of functions is used to set up a thread, and wait for it to finish.

	@{
*/

/*! @brief Initializes a new @ref Thread @p t with the specified settings
	@param[in] entrypoint Function that will be called when the thread starts executing
	@param[in] arg Argument to pass to @p entrypoint
	@param[in] stack_top Initial value of the stack pointer (must be 8-byte aligned)
	@param[in] prio Numeric priority level of the thread.
	Higher numerical values correspond to lower thread priority, and viceversa.
	The lowest priority is @ref THREAD_MIN_PRIO, while the highest is @ref THREAD_MAX_PRIO.

	Example usage:
	@code
	alignas(8) static u8 s_myThreadStack[2048];
	static Thread s_myThread;

	//...

	threadPrepare(&s_myThread, myThreadFunc, NULL, &s_myThreadStack[sizeof(s_myThreadStack)], MAIN_THREAD_PRIO);
	threadAttachLocalStorage(&s_myThread, NULL);
	threadStart(&s_myThread);
	@endcode

	@note 16 bytes of stack space are always reserved to support calling BIOS routines.
*/
void threadPrepare(Thread* t, ThreadFunc entrypoint, void* arg, void* stack_top, u8 prio);

//! @brief Returns the required size for thread-local storage (8-byte aligned) @see threadAttachLocalStorage
size_t threadGetLocalStorageSize(void);

// Sets an extra amount of thread-local storage to be allocated for threads created using
// system calls (POSIX threads, C threads, C++ std::thread). Will be 8-byte aligned.
void threadSetPthreadExtraTls(const size_t size);

/*! @brief Attaches thread-local storage to a @ref Thread @p t
	@param[in] storage 8-byte aligned memory buffer to use as thread-local storage,
	or NULL to consume thread stack memory instead

	Thread-local storage includes book-keeping information used by the C standard
	library, as well as any objects marked as `thread_local` anywhere in the binary.
	The required amount of space can be obtained using @ref threadGetLocalStorageSize().
	If a thread is started without attached local storage, it is illegal to call
	functions from the C standard library or use `thread_local` objects.
*/
void threadAttachLocalStorage(Thread* t, void* storage);

//! @brief Starts or unpauses the @ref Thread @p t
void threadStart(Thread* t);

//! @brief Pauses the @ref Thread @p t
void threadPause(Thread* t);

//! @brief Changes the priority of @ref Thread @p t to @p prio
void threadSetPrio(Thread* t, u8 prio);

/*! @brief Waits for the @ref Thread @p t to finish executing
	@returns Result code returned by the entrypoint function, or passed to @ref threadExit.
*/
int threadJoin(Thread* t);

//! @}

/*! @name Basic thread operations

	This group of functions apply to the currently running thread.

	@{
*/

//! @brief Returns the pointer to the currently running thread
MK_INLINE Thread* threadGetSelf(void)
{
	extern ThrSchedState __sched_state;
	return __sched_state.cur;
}

/*! @brief Relinquishes control of the CPU to other runnable threads
	@note In practice, yielding is only useful for sharing the CPU between threads
	of the same priority. Higher priority threads are always guaranteed to preempt
	the current thread when they become runnable, and lower priority threads cannot
	be yielded to at all.
*/
void threadYield(void);

/*! @brief Waits for any of the specified interrupts to occur
	@param[in] next_irq
		If false, immediately returns when the interrupt(s) have already occurred.
		If true, waits for the next instance of the interrupt(s) to occur instead.
	@param[in] mask Bitmask specifying which interrupts to wait for.
	@returns Bitmask indicating which interrupt(s) have occurred.
	@note This function has the same semantics as the IntrWait routine in the GBA/DS BIOS.
*/
u32  threadIrqWait(bool next_irq, IrqMask mask);

#if MK_IRQ_NUM_HANDLERS > 32
//! @brief Same as @ref threadIrqWait, but for extended interrupts.
u32  threadIrqWait2(bool next_irq, IrqMask mask);
#endif

#if defined(IRQ_VBLANK)

/*! @brief Waits for the next VBlank interrupt to occur
	@note This function has the same semantics as the VBlankIntrWait routine in the GBA/DS BIOS.
*/
MK_INLINE void threadWaitForVBlank(void)
{
	threadIrqWait(true, IRQ_VBLANK);
}

#endif

//! @brief Exits the current thread with @p rc as the result code
void threadExit(int rc) MK_NORETURN;

//! @}

/*! @name Thread blocking and unblocking

	This group of functions is used to implement synchronization between threads.

	@{
*/

/*! @brief Blocks the current thread into the given @p queue, using the specified @p token.

	The thread will be woken up when any of the threadUnblock<em>XX</em> functions are called,
	and the @p token depending condition is met. The following conditions are supported:
	- **By value**: unblocks threads whose @p token exactly matches the reference value (`==` equals operator)
		@see threadUnblockOneByValue, threadUnblockAllByValue
	- **By mask**: unblocks threads whose @p token has one or more bits in common with the reference mask (`&` bitwise-and operator)
		@see threadUnblockOneByMask, threadUnblockAllByMask

	@return 0 if threadBlockCancel was called, 1 if the thread was unblocked by value, the matched bits if the thread was unblocked by mask.
*/
MK_EXTERN32 u32 threadBlock(ThrListNode* queue, u32 token);

//! @brief Unblocks at most one thread in the @p queue matching the specified @p ref value @see threadBlock
MK_EXTERN32 void threadUnblockOneByValue(ThrListNode* queue, u32 ref);
//! @brief Unblocks at most one thread in the @p queue matching the specified @p ref mask @see threadBlock
MK_EXTERN32 void threadUnblockOneByMask(ThrListNode* queue, u32 ref);
//! @brief Unblocks all threads in the @p queue matching the specified @p ref value @see threadBlock
MK_EXTERN32 void threadUnblockAllByValue(ThrListNode* queue, u32 ref);
//! @brief Unblocks all threads in the @p queue matching the specified @p ref mask @see threadBlock
MK_EXTERN32 void threadUnblockAllByMask(ThrListNode* queue, u32 ref);

//! @brief Removes thread @p t from the specified @p queue
MK_EXTERN32 void threadBlockCancel(ThrListNode* queue, Thread* t);

//! @}

/*! @name Thread sleeping

	This group of functions allow the current thread to sleep for specified durations.

	@{
*/

//! @brief Pauses the thread for the specified time interval expressed in ticks @see ticksFromUsec
void threadSleepTicks(u32 ticks);

//! @brief Starts a @ref TickTask based timer with the specified period in ticks @see ticksFromHz
void threadTimerStartTicks(TickTask* task, u32 period_ticks);

/*! @brief Waits for the specified timer
	@warning The timer must have been previously started with
	@ref threadTimerStart() or @ref threadTimerStartTicks(). It is invalid to call this function
	with an arbitrary @ref TickTask.
*/
void threadTimerWait(TickTask* task);

//! @brief Pauses the thread for the specified time interval in microseconds @see threadSleepTicks
MK_INLINE void threadSleep(u32 usec)
{
	threadSleepTicks(ticksFromUsec(usec));
}

//! @brief Starts a @ref TickTask based timer with the specified period in Hz @see threadTimerStartTicks
MK_INLINE void threadTimerStart(TickTask* task, u32 period_hz)
{
	threadTimerStartTicks(task, ticksFromHz(period_hz));
}

//! @}

//! @brief Returns true if thread @p t is valid
MK_CONSTEXPR bool threadIsValid(Thread* t)
{
	return t->status != ThrStatus_Uninitialized;
}

MK_EXTERN_C_END

//! @}
