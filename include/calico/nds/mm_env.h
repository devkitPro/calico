#pragma once
#if !defined(__NDS__)
#error "This header file is only for NDS"
#endif

// Optional parameters for DSi autoload
#define MM_ENV_TWL_AUTOLOAD_PARAM 0x2000000

// DSi autoload
#define MM_ENV_TWL_AUTOLOAD       0x2000300

// DSi user settings (loaded from NAND)
#define MM_ENV_TWL_USER_SETTINGS  0x2000400

// DSi Atheros wireless firmware information
#define MM_ENV_TWL_WLFIRM_INFO    0x20005e0

// DSi hardware info (sourced from HWINFO_N.dat)
#define MM_ENV_TWL_HWINFO_N       0x2000600

// Extended DSi autoload, implemented by some homebrew entrypoints
#define MM_ENV_TWL_AUTOLOAD_EXT   0x2000800

// Homebrew bootstub area, used to implement return-to-hbmenu
#define MM_ENV_HB_BOOTSTUB        0x2ff4000
#define MM_ENV_HB_BOOTSTUB_SZ     0x8000 // 32kb

// DSi ROM header for the currently inserted gamecard
#define MM_ENV_CARD_TWL_HEADER    0x2ffc000

// Free area
#define MM_ENV_FREE_D000          0x2ffd000
#define MM_ENV_FREE_D000_SZ       0x7b0

// DSi system software version information
#define MM_ENV_TWL_VERSION_INFO   0x2ffd7b0

// DSi eMMC information
#define MM_ENV_TWL_NAND_INFO      0x2ffd7bc

// DSi known title list
#define MM_ENV_TWL_TITLE_LIST     0x2ffd800

// DSi device mount list
#define MM_ENV_TWL_DEVICE_LIST    0x2ffdc00

// DSi ROM header for the currently running application
#define MM_ENV_APP_TWL_HEADER     0x2ffe000

// Free area
#define MM_ENV_FREE_F000          0x2fff000
#define MM_ENV_FREE_F000_SZ       0xa80 // 2.625kb

// NDS ROM header for the currently inserted gamecard
#define MM_ENV_CARD_NDS_HEADER    0x2fffa80

// Free area
#define MM_ENV_FREE_FBE0          0x2fffbe0
#define MM_ENV_FREE_FBE0_SZ       0x60

// Application boot indicator + info (used by DS Download Play)
#define MM_ENV_BOOT_PARAM         0x2fffc40
#define MM_ENV_BOOT_PARAM_SZ      0x40

// User settings (loaded from NVRAM)
#define MM_ENV_USER_SETTINGS      0x2fffc80

// Free area
#define MM_ENV_FREE_FD00          0x2fffd00
#define MM_ENV_FREE_FD00_SZ       0x68

// DSi hardware info (sourced from HWINFO_S.dat)
#define MM_ENV_TWL_HWINFO_S       0x2fffd68

// Debug exception stack
#define MM_ENV_DEBUG_STACK_BOTTOM 0x2fffd80
#define MM_ENV_DEBUG_STACK_TOP    0x2fffdf0

// SCFG registers backup
#define MM_ENV_TWL_SCFG_BACKUP    0x2fffdf0

// NDS ROM header for the currently running application
#define MM_ENV_APP_NDS_HEADER     0x2fffe00

// Argument header
#define MM_ENV_ARGV_HEADER        0x2fffe70

// Free area
#define MM_ENV_FREE_FF60          0x2ffff60
#define MM_ENV_FREE_FF60_SZ       0x9c

// Special address used as an I/O port to control the main RAM chip
#define MM_ENV_MAIN_MEM_CMD       0x2fffffe