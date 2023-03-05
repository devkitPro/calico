#pragma once
#include <calico/nds/pxi.h>
#include <calico/nds/pm.h>

typedef enum PxiPmMsgType {

	PxiPmMsg_GetBatteryState = 0,
	PxiPmMsg_Sleep           = 1,
	PxiPmMsg_Wakeup          = 2,

} PxiPmMsgType;

MEOW_CONSTEXPR u32 pxiPmMakeMsg(PxiPmMsgType type, unsigned imm)
{
	return (type & 0xff) | ((imm & 0xff) << 8);
}

MEOW_CONSTEXPR PxiPmMsgType pxiPmGetType(u32 msg)
{
	return (PxiPmMsgType)(msg & 0xff);
}

MEOW_CONSTEXPR unsigned pxiPmGetImmediate(u32 msg)
{
	return (msg >> 8) & 0xff;
}