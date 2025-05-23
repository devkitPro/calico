// SPDX-License-Identifier: ZPL-2.1
// SPDX-FileCopyrightText: Copyright fincs, devkitPro
#include <calico/types.h>
#include <calico/system/irq.h>
#include <calico/nds/mm.h>
#include <calico/nds/io.h>
#include <calico/nds/system.h>
#include <calico/nds/dma.h>
#include <calico/nds/irq.h>
#include <calico/nds/env.h>
#include <calico/nds/lcd.h>
#include <calico/nds/pm.h>
#include <calico/nds/arm9/vram.h>

MK_WEAK void systemUserStartup(void)
{
	// Nothing
}

MK_WEAK void systemStartup(void)
{
	// Clear video display registers
	REG_POWCNT = POWCNT_LCD | POWCNT_2D_GFX_A | POWCNT_2D_GFX_B | POWCNT_LCD_SWAP;
	dmaStartFill32(3, (void*)(MM_IO + IO_GFX_A), 0, IO_TVOUTCNT-IO_DISPCNT);
	dmaBusyWait(3);
	dmaStartFill32(3, (void*)(MM_IO + IO_GFX_B), 0, IO_TVOUTCNT-IO_DISPCNT);
	dmaBusyWait(3);

	// Unmap VRAM
	REG_VRAMCNT_ABCD = 0;
	REG_VRAMCNT_EF   = 0;
	REG_VRAMCNT_G    = 0;
	REG_VRAMCNT_HI   = 0;

	// Wait for the LCDs to be initialized on DSi
	if (systemIsTwlMode()) {
		while (!(REG_DISPSTAT & DISPSTAT_LCD_READY_TWL));
	}

	// Configure LCD interrupts (enabling VBlank)
	lcdSetIrqMask(DISPSTAT_IE_ALL, DISPSTAT_IE_VBLANK);
	irqEnable(IRQ_VBLANK);

	// Initialize PM
	pmInit();

	// Call user initialization function
	systemUserStartup();
}
