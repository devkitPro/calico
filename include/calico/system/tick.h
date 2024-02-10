#pragma once
#include "../types.h"
#include "sysclock.h"

/*! @addtogroup tick
	@{
*/

#define TICK_FREQ (SYSTEM_CLOCK/64)

MK_EXTERN_C_START

typedef struct TickTask TickTask;
typedef void (* TickTaskFn)(TickTask*);

struct TickTask {
	TickTask* next;
	u32 target;
	u32 period;
	TickTaskFn fn;
};

MK_CONSTEXPR u32 ticksFromUsec(u32 us)
{
	return (us * (u64)TICK_FREQ) / 1000000;
}

MK_CONSTEXPR u32 ticksFromHz(u32 hz)
{
	return (TICK_FREQ + hz/2) / hz;
}

void tickInit(void);
u64 tickGetCount(void);
void tickTaskStart(TickTask* t, TickTaskFn fn, u32 delay_ticks, u32 period_ticks);
void tickTaskStop(TickTask* t);

MK_EXTERN_C_END

//! @}
