// SPDX-License-Identifier: ZPL-2.1
// SPDX-FileCopyrightText: Copyright fincs, devkitPro
#include <calico/types.h>
#include <calico/system/thread.h>
#include <calico/system/mutex.h>
#include <calico/dev/tmio.h>
#include <calico/dev/sdio.h>
#include <calico/dev/ar6k.h>
#include <calico/dev/wpa.h>
#include <calico/nds/bios.h>
#include <calico/nds/env.h>
#include <calico/nds/ndma.h>
#include <calico/nds/arm7/gpio.h>
#include <calico/nds/arm7/i2c.h>
#include <calico/nds/arm7/mcu.h>
#include <calico/nds/arm7/twlwifi.h>
#include <string.h>

//#define TWLWIFI_DEBUG

#ifdef TWLWIFI_DEBUG
#include <calico/system/dietprint.h>
#else
#define dietPrint(...) ((void)0)
#endif

#define SDIO_BLOCK_SZ_WORDS (SDIO_BLOCK_SZ/4)

static TmioCtl s_sdioCtl;
static u32 s_sdioCtlBuf[2];
static Thread s_sdioThread;
alignas(8) static u8 s_sdioThreadStack[1024];
static Thread s_sdioIrqThread;
alignas(8) static u8 s_sdioIrqThreadStack[2048];
static Thread s_wpaSupplicantThread;
alignas(8) static u8 s_wpaSupplicantThreadStack[2048];

static SdioCard s_sdioCard;
static Ar6kDev s_ar6kDev;
alignas(4) static u8 s_ar6kWorkBuf[AR6K_WORK_BUF_SIZE];
static WpaState s_wpaState;
static u8 s_wpaIeBuf[sizeof(WlanIeHdr) + 0xFF];

static const u8 s_macAny[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

static struct {
	WlanBssDesc* bssTable;
	unsigned bssCount;
	bool has_bssid_filter;
	u8 bssid_filter[6];
	TwlWifiScanCompleteFn cb;
	void* user;
} s_scanVars;

static struct {
	bool wpa_wait;
	TwlWifiAssocFn cb;
	void* user;
} s_assocVars;

static void _sdioIrqHandler(void)
{
	tmioIrqHandler(&s_sdioCtl);
}

static void _twlwifiSetWifiReset(bool reset)
{
	i2cLock();
	unsigned reg = i2cReadRegister8(I2cDev_MCU, McuReg_WifiLed);
	reg &= ~(1U<<4);
	if (reset) {
		reg |= 1U<<4;
	}
	i2cWriteRegister8(I2cDev_MCU, McuReg_WifiLed, reg);
	i2cUnlock();
}

static bool _twlwifiGetWifiReset(void)
{
	i2cLock();
	unsigned reg = i2cReadRegister8(I2cDev_MCU, McuReg_WifiLed);
	i2cUnlock();
	return (reg & (1U<<4)) != 0;
}

MK_INLINE void _twlwifiSetupDma(unsigned ch,
	uptr src, NdmaMode srcmode, uptr dst, NdmaMode dstmode,
	u32 unit_words, u32 total_words, u32 cnt)
{
	REG_NDMAxSAD(ch) = src;
	REG_NDMAxDAD(ch) = dst;
	REG_NDMAxBCNT(ch) = 0;
	REG_NDMAxTCNT(ch) = total_words;
	REG_NDMAxWCNT(ch) = unit_words;
	REG_NDMAxCNT(ch) =
		NDMA_DST_MODE(dstmode) |
		NDMA_SRC_MODE(srcmode) |
		NDMA_BLK_WORDS(unit_words) |
		cnt;
}

static void _twlwifiSdioDma(TmioCtl* ctl, TmioTx* tx)
{
	if (tx->status & TMIO_STAT_CMD_BUSY) {
		// DMA Start
		if (tx->type & TMIO_CMD_TX_READ) {
			_twlwifiSetupDma(1,
				ctl->fifo_base, NdmaMode_Fixed, (uptr)tx->user, NdmaMode_Increment,
				SDIO_BLOCK_SZ_WORDS, tx->num_blocks*SDIO_BLOCK_SZ_WORDS,
				NDMA_TIMING(NdmaTiming_Tmio1) | NDMA_TX_MODE(NdmaTxMode_Timing) | NDMA_START);
		} else {
			_twlwifiSetupDma(1,
				(uptr)tx->user, NdmaMode_Increment, ctl->fifo_base, NdmaMode_Fixed,
				SDIO_BLOCK_SZ_WORDS, tx->num_blocks*SDIO_BLOCK_SZ_WORDS,
				NDMA_TIMING(NdmaTiming_Tmio1) | NDMA_TX_MODE(NdmaTxMode_Timing) | NDMA_START);
		}
	} else if (tx->status == 0) {
		// DMA End (OK)
		ndmaBusyWait(1);
	} else {
		// DMA End (Error)
		REG_NDMAxCNT(1) = 0;
	}
}

static void _twlwifiOnBssInfo(Ar6kDev* dev, Ar6kWmiBssInfoHdr* bssInfo, NetBuf* pPacket)
{
	if (!s_scanVars.bssTable) {
		return;
	}

	if (s_scanVars.has_bssid_filter && memcmp(bssInfo->bssid, s_scanVars.bssid_filter, 6) != 0) {
		return;
	}

	// Add new entry or find existing entry to overwrite
	unsigned rssi = bssInfo->snr >= 0 ? bssInfo->snr : 0;
	WlanBssDesc* desc = wlanFindOrAddBss(s_scanVars.bssTable, &s_scanVars.bssCount, bssInfo->bssid, rssi);
	if (!desc) {
		return;
	}

	// Parse the beacon
	wlanParseBeacon(desc, NULL, pPacket);
	memcpy(desc->bssid, bssInfo->bssid, 6);
	desc->rssi = rssi;
	desc->channel = wlanFreqToChannel(bssInfo->channel_mhz);
}

static void _twlwifiOnScanComplete(Ar6kDev* dev, int status)
{
	dietPrint("[TWLWIFI] Scan complete (%d)\n", status);

	WlanBssDesc* bss_table = s_scanVars.bssTable;
	if (bss_table) {
		s_scanVars.bssTable = NULL;
		if (s_scanVars.cb) {
			s_scanVars.cb(s_scanVars.user, bss_table, s_scanVars.bssCount);
		}
	}
}

static void _twlwifiOnAssoc(Ar6kDev* dev, Ar6kWmiEvtConnected* info)
{
	dietPrint("[TWLWIFI] Associated\n");

	u8* ie_data = info->assoc_info + info->beacon_ie_len + 4;
	unsigned remsize = info->assoc_req_len - 4;

	WlanIeHdr* ie = wlanFindRsnOrWpaIe(ie_data, remsize);
	if (ie) {
		unsigned ie_len = sizeof(WlanIeHdr) + ie->len;
		memcpy(s_wpaIeBuf, ie, ie_len);
		s_wpaState.ie_len = ie_len;
	} else {
		s_wpaState.ie_len = 0;
	}

	if (s_assocVars.cb && !s_assocVars.wpa_wait) {
		s_assocVars.cb(s_assocVars.user, true, 0);
	}
}

static void _twlwifiOnDisassoc(Ar6kDev* dev, Ar6kWmiEvtDisconnected* info)
{
	dietPrint("[TWLWIFI] Disassociated\n");
	dietPrint("  IEEE reason = %u\n", info->reason_ieee);
	dietPrint("  Reason = %u\n", info->reason);

	// XX: The ar6k only stops trying to connect when the host issues a WMI
	// disconnect command. In order to avoid leaving the ar6k in a dirty
	// state, explicitly issue a disconnect when we are booted out.
	if (info->reason != 3 && !ar6kWmiDisconnect(dev)) {
		return;
	}

	if (s_assocVars.cb) {
		s_assocVars.cb(s_assocVars.user, false, info->reason_ieee);
		s_assocVars.cb = NULL;
	}
}

MK_WEAK void _netbufRx(NetBuf* pPacket)
{
	netbufFree(pPacket);
}

static void _twlwifiRx(Ar6kDev* dev, NetBuf* pPacket)
{
	NetMacHdr* machdr = (NetMacHdr*)netbufGet(pPacket);
	unsigned ethertype = __builtin_bswap16(machdr->len_or_ethertype_be);

	if (ethertype == NetEtherType_EAPOL) {
		// EAPOL -> forward package to WPA supplicant
		if (!wpaEapolRx(&s_wpaState, pPacket)) {
			dietPrint("[RX] WPA busy, dropping packet\n");
			netbufFree(pPacket);
		}
	} else {
		// Regular packet
		_netbufRx(pPacket);
	}
}

static NetBuf* _twlwifiWpaAllocPacket(WpaState* st, size_t len)
{
	return netbufAlloc(
		sizeof(Ar6kHtcFrameHdr) + sizeof(Ar6kWmiDataHdr) + sizeof(NetMacHdr) + sizeof(NetLlcSnapHdr),
		len, NetBufPool_Tx);
}

static void _twlwifiWpaTx(WpaState* st, NetBuf* pPacket)
{
	twlwifiTx(pPacket);
}

static void _twlwifiWpaInstallKey(WpaState* st, bool is_group, unsigned slot, unsigned len)
{
	// XX: wmiSyncronize should happen around key change

	Ar6kWmiCipherKey key;
	key.index = slot;
	key.type = len > WPA_AES_BLOCK_LEN ? Ar6kWmiCipherType_TKIP : Ar6kWmiCipherType_AES;
	key.usage = is_group ? AR6K_WMI_CIPHER_USAGE_GROUP : AR6K_WMI_CIPHER_USAGE_PAIRWISE;
	key.length = len;
	key.op_ctrl = AR6K_WMI_KEY_OP_INIT_TSC | AR6K_WMI_KEY_OP_INIT_RSC;

	WpaKey* inkey = is_group ? &st->gtk : &st->ptk.tk;
	memcpy(&key.key[0], inkey->main, 0x10);

	if (key.type == Ar6kWmiCipherType_TKIP) {
		// Fill in extra fields for TKIP
		memcpy(&key.key[0x10], inkey->rx, 8);
		memcpy(&key.key[0x18], inkey->tx, 8);
	}

	// Fill in RSC
	if (is_group) {
		memcpy(key.rsc, st->rsc, WPA_RSC_LEN);
	} else {
		memset(key.rsc, 0, WPA_RSC_LEN);
	}

	ar6kWmiSimpleCmdWithParam8(&s_ar6kDev, Ar6kWmiCmdId_Synchronize, 0);

	if (!ar6kWmiAddCipherKey(&s_ar6kDev, &key)) {
		dietPrint("[TWLWIFI] %s key%u fail\n", is_group ? "group" : "pairwise", slot);
	}

	ar6kWmiSimpleCmdWithParam8(&s_ar6kDev, Ar6kWmiCmdId_Synchronize, 0);

	if (is_group) {
		dietPrint("[TWLWIFI] WPA negotiated!\n");
		if (s_assocVars.cb && s_assocVars.wpa_wait) {
			s_assocVars.cb(s_assocVars.user, true, 0);
			s_assocVars.wpa_wait = false;
		}
	}
}

bool twlwifiInit(void)
{
	// XX: Below sequence does a full reset of the Atheros hardware, clearing
	// the loaded firmware in the process. We obviously do not want that.
	//_twlwifiSetWifiReset(false);
	//threadSleep(1000);

	// Ensure Atheros is selected and powered on
	gpioSetWlModule(GpioWlModule_Atheros);
	if (!_twlwifiGetWifiReset()) {
		_twlwifiSetWifiReset(true);
		threadSleep(1000);
	}

	// Initialize TMIO host controller used for SDIO
	if (!tmioInit(&s_sdioCtl, MM_IO + IO_TMIO1_BASE, MM_IO + IO_TMIO1_FIFO, s_sdioCtlBuf, sizeof(s_sdioCtlBuf)/sizeof(u32))) {
		dietPrint("[TWLWIFI] TMIO init failed\n");
		return false;
	}

	threadPrepare(&s_sdioThread, (ThreadFunc)tmioThreadMain, &s_sdioCtl, &s_sdioThreadStack[sizeof(s_sdioThreadStack)], 0x10);
	threadStart(&s_sdioThread);

	irqSet2(IRQ2_TMIO1, _sdioIrqHandler);
	irqEnable2(IRQ2_TMIO1);

	// Initialize the SDIO card interface
	if (!sdioCardInit(&s_sdioCard, &s_sdioCtl, 0)) {
		dietPrint("[TWLWIFI] SDIO init failed\n");
		goto _tmioCleanup;
	}

	// Initialize DMA for SDIO
	s_sdioCard.dma_cb = _twlwifiSdioDma;

	// Initialize the Atheros wireless device
	if (!ar6kDevInit(&s_ar6kDev, &s_sdioCard, s_ar6kWorkBuf)) {
		dietPrint("[TWLWIFI] AR6K init failed\n");
		goto _tmioCleanup;
	}

	// Start the Atheros interrupt thread
	threadPrepare(&s_sdioIrqThread, (ThreadFunc)ar6kDevThreadMain, &s_ar6kDev, &s_sdioIrqThreadStack[sizeof(s_sdioIrqThreadStack)], 0x11);
	threadStart(&s_sdioIrqThread);

	// Wait for WMI to be ready
	if (!ar6kWmiStartup(&s_ar6kDev)) {
		dietPrint("[TWLWIFI] AR6K WMI startup fail\n");
		goto _ar6kCleanup;
	}

	// Prepare and start the WPA supplicant thread
	wpaPrepare(&s_wpaState);
	threadPrepare(&s_wpaSupplicantThread, (ThreadFunc)wpaSupplicantThreadMain, &s_wpaState,
		&s_wpaSupplicantThreadStack[sizeof(s_wpaSupplicantThreadStack)], MAIN_THREAD_PRIO+0x10);
	threadStart(&s_wpaSupplicantThread);

	dietPrint("[TWLWIFI] Init complete\n");

	// Set callbacks
	s_ar6kDev.cb_onBssInfo = _twlwifiOnBssInfo;
	s_ar6kDev.cb_onScanComplete = _twlwifiOnScanComplete;
	s_ar6kDev.cb_onAssoc = _twlwifiOnAssoc;
	s_ar6kDev.cb_onDisassoc = _twlwifiOnDisassoc;
	s_ar6kDev.cb_rx = _twlwifiRx;
	s_wpaState.cb_alloc_packet = _twlwifiWpaAllocPacket;
	s_wpaState.cb_tx = _twlwifiWpaTx;
	s_wpaState.cb_install_key = _twlwifiWpaInstallKey;
	s_wpaState.ie_data = s_wpaIeBuf;

	// Copy wireless interface settings
	memcpy(g_envExtraInfo->wlmgr_macaddr, s_ar6kDev.macaddr, 6);
	g_envExtraInfo->wlmgr_channel_mask = s_ar6kDev.wmi_channel_mask;
	g_envExtraInfo->wlmgr_hdr_headroom_sz = sizeof(Ar6kHtcFrameHdr) + sizeof(Ar6kWmiDataHdr) + sizeof(NetLlcSnapHdr);

	return true;

_ar6kCleanup:
	ar6kDevThreadCancel(&s_ar6kDev);
	threadJoin(&s_sdioIrqThread);
_tmioCleanup:
	tmioThreadCancel(&s_sdioCtl);
	irqDisable2(IRQ2_TMIO1);
	threadJoin(&s_sdioThread);
	return false;
}

void twlwifiExit(void)
{
	twlwifiDisassociate();
	wpaFinalize(&s_wpaState);
	threadJoin(&s_wpaSupplicantThread);
	ar6kWmiHostExitNotify(&s_ar6kDev);
	ar6kDevThreadCancel(&s_ar6kDev);
	threadJoin(&s_sdioIrqThread);
	tmioThreadCancel(&s_sdioCtl);
	irqDisable2(IRQ2_TMIO1);
	threadJoin(&s_sdioThread);
}

bool twlwifiStartScan(WlanBssDesc* out_table, WlanBssScanFilter const* filter, TwlWifiScanCompleteFn cb, void* user)
{
	u32 channel_mask = s_ar6kDev.wmi_channel_mask;
	bool is_active = false;
	bool has_bssid_filter = false;

	if (s_scanVars.bssTable) {
		return false; // already scanning
	}

	// Validate filter if specified
	if (filter) {
		channel_mask &= filter->channel_mask;
		if (!channel_mask || filter->target_ssid_len > WLAN_MAX_SSID_LEN) {
			return false;
		}
		if (memcmp(filter->target_bssid, s_macAny, 6) != 0) {
			has_bssid_filter = true;
		}
	}

	Ar6kWmiProbedSsid probed_ssid = { 0 };
	if (filter && filter->target_ssid_len) {
		is_active = true;
		probed_ssid.mode = Ar6kWmiSsidProbeMode_Specific;
		probed_ssid.ssid_len = filter->target_ssid_len;
		memcpy(probed_ssid.ssid, filter->target_ssid, filter->target_ssid_len);
	}
	if (!ar6kWmiSetProbedSsid(&s_ar6kDev, &probed_ssid)) {
		return false;
	}

	if (!ar6kWmiSetChannelParams(&s_ar6kDev, 0, channel_mask)) {
		return false;
	}

	Ar6kWmiScanParams scan_params = { 0 };
	u32 dwell_time_ms = is_active ? 30 : 105; // shorter scan time for active scans
	scan_params.fg_start_period_secs = UINT16_MAX;
	scan_params.fg_end_period_secs = UINT16_MAX;
	scan_params.bg_period_secs = UINT16_MAX;
	scan_params.minact_chdwell_time_ms = dwell_time_ms;
	scan_params.maxact_chdwell_time_ms = dwell_time_ms;
	scan_params.pas_chdwell_time_ms = dwell_time_ms;
	//scan_params.short_scan_ratio = 0;
	scan_params.scan_ctrl_flags = AR6K_WMI_SCAN_CONNECT | (is_active ? AR6K_WMI_SCAN_ACTIVE : 0);
	if (!ar6kWmiSetScanParams(&s_ar6kDev, &scan_params)) {
		return false;
	}

	if (!ar6kWmiSetBssFilter(&s_ar6kDev, is_active ? Ar6kWmiBssFilter_ProbedSsid : Ar6kWmiBssFilter_All, 0)) {
		return false;
	}

	s_scanVars.bssTable = out_table;
	s_scanVars.bssCount = 0;
	s_scanVars.has_bssid_filter = has_bssid_filter;
	s_scanVars.cb = cb;
	s_scanVars.user = user;
	if (has_bssid_filter) {
		memcpy(s_scanVars.bssid_filter, filter->target_bssid, 6);
	}

	if (!ar6kWmiStartScan(&s_ar6kDev, Ar6kWmiScanType_Long, 20)) {
		s_scanVars.bssTable = NULL;
		return false;
	}

	return true;
}

bool twlwifiIsScanning(void)
{
	return s_scanVars.bssTable != NULL;
}

bool twlwifiAssociate(WlanBssDesc const* bss, WlanAuthData const* auth, TwlWifiAssocFn cb, void* user)
{
	static const Ar6kWmiProbedSsid dummy_probed_ssid = { 0 };
	static const Ar6kWmiScanParams dummy_scan_params = {
		.fg_start_period_secs = UINT16_MAX,
		.fg_end_period_secs = UINT16_MAX,
		.bg_period_secs = UINT16_MAX,
		.maxact_chdwell_time_ms = 200,
		.pas_chdwell_time_ms = 200,
		.short_scan_ratio = 0,
		.scan_ctrl_flags = AR6K_WMI_SCAN_CONNECT | AR6K_WMI_SCAN_ACTIVE | AR6K_WMI_SCAN_ROAM,
		.minact_chdwell_time_ms = 200,
		._pad = 0,
		.max_dfsch_act_time_ms = 0,
	};

	static const Ar6kWmiPstreamConfig pstream_params = {
		.min_service_int_msec = 20,
		.max_service_int_msec = 20,
		.inactivity_int_msec  = 9999999,
		.suspension_int_msec  = (u32)-1,
		.srv_start_time       = 0,
		.min_data_rate_bps    = 83200,
		.mean_data_rate_bps   = 83200,
		.peak_data_rate_bps   = 83200,
		.max_burst_size       = 0,
		.delay_bound          = 0,
		.min_phy_rate_bps     = 6000000,
		.sba                  = 0x2000,
		.medium_time          = 0,
		.nominal_msdu         = 0x80d0,
		.max_msdu             = 0xd0,
		.traffic_class        = 0,
		.traffic_dir          = Ar6kWmiTrafficDir_Bidir,
		.rx_queue_num         = 0xff,
		.traffic_type         = Ar6kWmiTrafficType_Periodic,
		.voice_ps_cap         = Ar6kWmiVoicePSCap_DisableForThisAC,
		.tsid                 = 5,
		.user_prio            = 0,
	};

	if (bss->auth_type != WlanBssAuthType_Open && !auth) {
		return false;
	}

	if (!ar6kWmiSetBssFilter(&s_ar6kDev, Ar6kWmiBssFilter_None, 0)) {
		return false;
	}

	if (!ar6kWmiSetProbedSsid(&s_ar6kDev, &dummy_probed_ssid)) {
		return false;
	}

	if (!ar6kWmiSetScanParams(&s_ar6kDev, &dummy_scan_params)) {
		return false;
	}

	if (!ar6kWmiSetChannelParams(&s_ar6kDev, 0, 0)) {
		return false;
	}

	if (!ar6kWmiSetBitRate(&s_ar6kDev, Ar6kWmiBitRate_Auto, Ar6kWmiBitRate_1Mbps, Ar6kWmiBitRate_1Mbps)) {
		return false;
	}

	if (!ar6kWmiSetFrameRate(&s_ar6kDev, 4, 10, ~(1U<<Ar6kWmiBitRate_11Mbps))) { // PS-Poll
		return false;
	}

	ar6kWmiSimpleCmdWithParam8(&s_ar6kDev, Ar6kWmiCmdId_Synchronize, 0);

	if (!ar6kWmiSetPowerMode(&s_ar6kDev, Ar6kWmiPowerMode_MaxPerformance)) {
		return false;
	}

	ar6kWmiSimpleCmdWithParam8(&s_ar6kDev, Ar6kWmiCmdId_Synchronize, 0);

	if (!ar6kWmiCreatePstream(&s_ar6kDev, &pstream_params)) {
		return false;
	}

	if (!ar6kWmiSetWscStatus(&s_ar6kDev, false)) { // WPS related?
		return false;
	}

	if (!ar6kWmiSetDiscTimeout(&s_ar6kDev, 2)) {
		return false;
	}

	Ar6kWmiConnectParams conn_params = { 0 };
	conn_params.network_type = Ar6kWmiNetworkType_Infrastructure;
	conn_params.ssid_len = bss->ssid_len;
	memcpy(conn_params.ssid, bss->ssid, bss->ssid_len);
	conn_params.channel_mhz = wlanChannelToFreq(bss->channel);
	memcpy(conn_params.bssid, bss->bssid, 6);

	s_assocVars.wpa_wait = false;
	s_assocVars.cb = cb;
	s_assocVars.user = user;

	switch (bss->auth_type) {
		default:
		case WlanBssAuthType_Open: {
			// Set params
			conn_params.auth_mode_ieee = Ar6kWmiAuthModeIeee_Open;
			conn_params.auth_mode = Ar6kWmiAuthMode_Open;
			conn_params.pairwise_cipher_type = Ar6kWmiCipherType_None;
			conn_params.group_cipher_type = Ar6kWmiCipherType_None;
			break;
		}

		case WlanBssAuthType_WEP_40:
		case WlanBssAuthType_WEP_104:
		case WlanBssAuthType_WEP_128: {
			// Install WEP key (slot 0)
			static const u8 key_length[] = { WLAN_WEP_40_LEN, WLAN_WEP_104_LEN, WLAN_WEP_128_LEN };
			Ar6kWmiCipherKey key = { 0 };
			key.type = Ar6kWmiCipherType_WEP;
			key.usage = AR6K_WMI_CIPHER_USAGE_GROUP | AR6K_WMI_CIPHER_USAGE_TX;
			key.length = key_length[bss->auth_type-WlanBssAuthType_WEP_40];
			memcpy(key.key, auth->wep_key, key.length);
			key.op_ctrl = AR6K_WMI_KEY_OP_INIT_TSC | AR6K_WMI_KEY_OP_INIT_RSC;
			if (!ar6kWmiAddCipherKey(&s_ar6kDev, &key)) {
				return false;
			}

			// Install dummy WEP keys (slots 1..3)
			key.usage = AR6K_WMI_CIPHER_USAGE_GROUP;
			memset(key.key, 0, key.length);
			for (unsigned i = 1; i < 4; i ++) {
				key.index = i;
				if (!ar6kWmiAddCipherKey(&s_ar6kDev, &key)) {
					return false;
				}
			}

			// Set params
			conn_params.auth_mode_ieee = Ar6kWmiAuthModeIeee_Shared;
			conn_params.auth_mode = Ar6kWmiAuthMode_Open;
			conn_params.pairwise_cipher_type = Ar6kWmiCipherType_WEP;
			conn_params.pairwise_key_len = key.length;
			conn_params.group_cipher_type = Ar6kWmiCipherType_WEP;
			conn_params.group_key_len = key.length;
			break;
		}

		case WlanBssAuthType_WPA_PSK_TKIP:
		case WlanBssAuthType_WPA2_PSK_TKIP:
		case WlanBssAuthType_WPA_PSK_AES:
		case WlanBssAuthType_WPA2_PSK_AES: {
			static const u8 authtype[] = { Ar6kWmiAuthType_WPA_PSK, Ar6kWmiAuthType_WPA2_PSK, Ar6kWmiAuthType_WPA_PSK, Ar6kWmiAuthType_WPA2_PSK };
			static const u8 cipher[] = { Ar6kWmiCipherType_TKIP, Ar6kWmiCipherType_TKIP, Ar6kWmiCipherType_AES, Ar6kWmiCipherType_AES };
			s_assocVars.wpa_wait = true;

			// Set params
			conn_params.auth_mode_ieee = Ar6kWmiAuthModeIeee_Open;
			conn_params.auth_mode = authtype[bss->auth_type-WlanBssAuthType_WPA_PSK_TKIP];
			conn_params.pairwise_cipher_type = cipher[bss->auth_type-WlanBssAuthType_WPA_PSK_TKIP];
			conn_params.group_cipher_type = cipher[bss->auth_type-WlanBssAuthType_WPA_PSK_TKIP];

			// Reset WPA supplicant state with the new PSK
			wpaReset(&s_wpaState, auth->wpa_psk);
			break;
		}
	}

	if (!ar6kWmiSetKeepAlive(&s_ar6kDev, 0)) {
		return false;
	}

	if (!ar6kWmiConnect(&s_ar6kDev, &conn_params)) {
		return false;
	}

	return true;
}

bool twlwifiDisassociate(void)
{
	static const Ar6kWmiScanParams dummy_scan_params = {
		.fg_start_period_secs = UINT16_MAX,
		.fg_end_period_secs = UINT16_MAX,
		.bg_period_secs = UINT16_MAX,
		.maxact_chdwell_time_ms = 200,
		.pas_chdwell_time_ms = 200,
		.short_scan_ratio = 0,
		.scan_ctrl_flags = AR6K_WMI_SCAN_CONNECT,
		.minact_chdwell_time_ms = 200,
		._pad = 0,
		.max_dfsch_act_time_ms = 0,
	};

	if (!ar6kWmiSetScanParams(&s_ar6kDev, &dummy_scan_params)) {
		return false;
	}

	if (!ar6kWmiSetPowerMode(&s_ar6kDev, Ar6kWmiPowerMode_Recommended)) {
		return false;
	}

	if (!ar6kWmiDisconnect(&s_ar6kDev)) {
		return false;
	}

	return true;
}

bool twlwifiTx(NetBuf* pPacket)
{
	bool rc = ar6kWmiTx(&s_ar6kDev, pPacket);
	netbufFree(pPacket);
	return rc;
}
