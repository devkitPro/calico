#pragma once
#include "types.h"

MEOW_EXTERN_C_START

typedef struct MwlMlmeCallbacks {
	void (* onBssInfo)(WlanBssDesc* bssInfo, WlanBssExtra* bssExtra, unsigned rssi);
	u32 (* onScanEnd)(void);
	void (* onJoinEnd)(bool ok);
	void (* onAuthEnd)(unsigned status);
	void (* onAssocEnd)(unsigned status);
	void (* onStateLost)(MwlStatus new_class, unsigned reason);
} MwlMlmeCallbacks;

MwlMlmeCallbacks* mwlMlmeGetCallbacks(void);
bool mwlMlmeScan(WlanBssScanFilter const* filter, unsigned ch_dwell_time);
bool mwlMlmeJoin(WlanBssDesc const* bssInfo, unsigned timeout);
bool mwlMlmeAuthenticate(unsigned timeout);
bool mwlMlmeAssociate(unsigned timeout);
bool mwlMlmeDeauthenticate(void);

MEOW_EXTERN_C_END
