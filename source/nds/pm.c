#include <calico/types.h>
#include <calico/arm/common.h>
#include <calico/system/irq.h>
#include <calico/system/thread.h>
#include <calico/system/mailbox.h>
#include <calico/nds/env.h>
#include <calico/nds/system.h>
#include <calico/nds/pxi.h>
#include <calico/nds/keypad.h>
#include <calico/nds/pm.h>
#include "pxi/reset.h"
#include "pxi/pm.h"

#if defined(ARM9)
#include <calico/arm/cp15.h>
#include <calico/arm/cache.h>
#elif defined(ARM7)
#include <calico/nds/arm7/pmic.h>
#include <calico/nds/arm7/i2c.h>
#include <calico/nds/arm7/mcu.h>
#endif

#include <sys/iosupport.h>

#define PM_FLAG_RESET_ASSERTED (1U << 0)
#define PM_FLAG_RESET_PREPARED (1U << 1)
#define PM_FLAG_SLEEP_ALLOWED  (1U << 2)

typedef struct PmState {
	PmEventCookie* cookie_list;
	u32 flags;
} PmState;

static PmState s_pmState;

static Thread s_pmPxiThread;
static alignas(8) u8 s_pmPxiThreadStack[0x200];

MEOW_NOINLINE static void _pmCallEventHandlers(PmEvent event)
{
	for (PmEventCookie* c = s_pmState.cookie_list; c; c = c->next) {
		c->handler(c->user, event);
	}
}

static void _pmResetPxiHandler(void* user, u32 msg)
{
	switch (pxiResetGetType(msg)) {
		default:
			break;

		case PxiResetMsgType_Reset:
		case PxiResetMsgType_Abort:
			s_pmState.flags |= PM_FLAG_RESET_ASSERTED;
			break;
	}
}

static int _pmPxiThreadMain(void* arg)
{
	// Set up PXI mailbox
	Mailbox mb;
	u32 mb_slots[4];
	mailboxPrepare(&mb, mb_slots, sizeof(mb_slots)/sizeof(u32));

	// Register PXI channels
	pxiSetHandler(PxiChannel_Reset, _pmResetPxiHandler, NULL);
	pxiSetMailbox(PxiChannel_Power, &mb);

	// Handle PXI messages
	for (;;) {
		u32 msg = mailboxRecv(&mb);
		PxiPmMsgType type = pxiPmGetType(msg);

		switch (type) {
			default: break;

#if defined(ARM9)

			//...

#elif defined(ARM7)

			case PxiPmMsg_GetBatteryState:
				pxiReply(PxiChannel_Power, pmGetBatteryState());
				break;

#endif

		}
	}

	return 0;
}

#if defined(ARM7)

MEOW_CODE32 MEOW_EXTERN32 MEOW_NOINLINE MEOW_NORETURN static void _pmJumpToBootstub(void)
{
	// Remap WRAM_A to the location used by DSi-enhanced (hybrid) apps
	// This is needed for compatibility with libnds
	if (systemIsTwlMode()) {
		MEOW_REG(u32, IO_MBK_MAP_A) = 0x00403000;
	}

	// Jump to ARM7 entrypoint
	((void(*)(void))g_envAppNdsHeader->arm7_entrypoint)();
	for (;;); // just in case
}

#endif

void __SYSCALL(exit)(int rc)
{
	MEOW_DUMMY(rc); // TODO: Do something useful with this

	// Call event handlers
	_pmCallEventHandlers(PmEvent_OnReset);

	// Assert reset on the other processor
	pxiSend(PxiChannel_Reset, pxiResetMakeMsg(PxiResetMsgType_Reset));

	// Wait for reset to be asserted on us
	while (!(s_pmState.flags & PM_FLAG_RESET_ASSERTED)) {
		threadSleep(1000);
	}

#if defined(ARM9)

	// Disable all interrupts
	armIrqLockByPsr();
	irqLock();

	// Hang indefinitely if there is no jump target
	if (!pmHasResetJumpTarget()) {
		for (;;);
	}

	// Flush caches
	armDCacheFlushAll();
	armICacheInvalidateAll();

	// Disable PU and caches
	u32 cp15_cr;
	__asm__ __volatile__ ("mrc p15, 0, %0, c1, c0, 0" : "=r" (cp15_cr));
	cp15_cr &= ~(CP15_CR_PU_ENABLE | CP15_CR_DCACHE_ENABLE | CP15_CR_ICACHE_ENABLE | CP15_CR_ROUND_ROBIN);
	__asm__ __volatile__ ("mcr p15, 0, %0, c1, c0, 0" :: "r" (cp15_cr));

	// Jump to ARM9 bootstub entrypoint
	g_envNdsBootstub->arm9_entrypoint();

#elif defined(ARM7)

	// On DSi, if the user started to press the power button, wait for the
	// MCU to figure out the desired command (reset/shutdown). Note that
	// we will only see reset requests, as shutdown handling takes effect
	// immediately (see explanation in pmInit).
	if (systemIsTwlMode()) {
		while (mcuGetPwrBtnState() == McuPwrBtnState_Begin) {
			threadSleep(1000);
		}
	}

	// If there is no jump target, reset or shut down the DS
	if (!pmHasResetJumpTarget()) {
		if (systemIsTwlMode()) {
			// Issue reset through MCU
			mcuIssueReset();
		} else {
			// Use PMIC to shut down the DS
			pmicIssueShutdown();
		}
	}

	// Disable interrupts
	armIrqLockByPsr();
	irqLock();

	// Perform PXI sync sequence
	REG_PXI_SYNC = PXI_SYNC_SEND(1);
	while (PXI_SYNC_RECV(REG_PXI_SYNC) != 1);
	REG_PXI_SYNC = PXI_SYNC_SEND(0);
	while (PXI_SYNC_RECV(REG_PXI_SYNC) != 0);

	// Clear PXI FIFO
	while (!(REG_PXI_CNT & PXI_CNT_RECV_EMPTY)) {
		MEOW_DUMMY(REG_PXI_RECV);
	}

	// Jump to new ARM7 entrypoint
	_pmJumpToBootstub();

#endif
}

void pmInit(void)
{
	s_pmState.flags = PM_FLAG_SLEEP_ALLOWED;

#if defined(ARM9)
	//...
#elif defined(ARM7)
	// Set up soft-reset key combination
	REG_KEYCNT = KEY_SELECT | KEY_START | KEY_R | KEY_L | KEYCNT_IRQ_ENABLE | KEYCNT_IRQ_AND;
	irqSet(IRQ_KEYPAD, pmPrepareToReset);
	irqEnable(IRQ_KEYPAD);

	if (systemIsTwlMode()) {
		// Initialize DSi MCU, and register interrupt handlers for it.
		// Power button:
		//   Begin pressing: we issue a reset request, which is later handled
		//     by the application main loop.
		//   Reset (aka tap): handled by __SYSCALL(exit) (refer to it)
		//   Shutdown (aka hold): immediately shut down (see below explanation)

		// For safety reasons, immediately shut down the DSi upon receiving a
		// power button shutdown/battery empty message. Note that a timeframe
		// exists between the power button being physically pushed and released
		// (see McuReg_PwrBtnHoldDelay), which seems to be intended to allow the
		// running application to finish writing any unsaved data to disk/etc.

		mcuInit();
		mcuIrqSet(MCU_IRQ_PWRBTN_SHUTDOWN|MCU_IRQ_BATTERY_EMPTY, (McuIrqHandler)mcuIssueShutdown);
		mcuIrqSet(MCU_IRQ_PWRBTN_BEGIN, (McuIrqHandler)pmPrepareToReset);
	}
#endif

	// Bring up PXI thread
	threadPrepare(&s_pmPxiThread, _pmPxiThreadMain, NULL, &s_pmPxiThreadStack[sizeof(s_pmPxiThreadStack)], 0x01);
	threadStart(&s_pmPxiThread);

	// Wait for the other CPU to bring up their PXI thread
	pxiWaitRemote(PxiChannel_Power);
}

void pmAddEventHandler(PmEventCookie* cookie, PmEventFn handler, void* user)
{
	if (!handler) {
		return;
	}

	IrqState st = irqLock();
	cookie->next = s_pmState.cookie_list;
	cookie->handler = handler;
	cookie->user = user;
	s_pmState.cookie_list = cookie;
	irqUnlock(st);
}

void pmRemoveEventHandler(PmEventCookie* cookie)
{
	// TODO
}

bool pmShouldReset(void)
{
	return s_pmState.flags & (PM_FLAG_RESET_ASSERTED|PM_FLAG_RESET_PREPARED);
}

void pmPrepareToReset(void)
{
	IrqState st = irqLock();
	s_pmState.flags |= PM_FLAG_RESET_PREPARED;
	irqUnlock(st);
}

bool pmIsSleepAllowed(void)
{
	return s_pmState.flags & PM_FLAG_SLEEP_ALLOWED;
}

void pmSetSleepAllowed(bool allowed)
{
	IrqState st = irqLock();
	if (allowed) {
		s_pmState.flags |= PM_FLAG_SLEEP_ALLOWED;
	} else {
		s_pmState.flags &= ~PM_FLAG_SLEEP_ALLOWED;
	}
	irqUnlock(st);
}

bool pmHasResetJumpTarget(void)
{
	return g_envNdsBootstub->magic == ENV_NDS_BOOTSTUB_MAGIC;
}

void pmClearResetJumpTarget(void)
{
	g_envNdsBootstub->magic = 0;
}

bool pmMainLoop(void)
{
	// TODO: Sleep mode handling
	return !pmShouldReset();
}
