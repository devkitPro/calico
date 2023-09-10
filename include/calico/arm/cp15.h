#pragma once

#define ARM_ICACHE_SZ     0x2000
#define ARM_DCACHE_SZ     0x1000
#define ARM_CACHE_LINE_SZ 32
#define ARM_DCACHE_SETS_LOG2 2
#define ARM_DCACHE_SETS (1 << ARM_DCACHE_SETS_LOG2)

#define CP15_CR_PU_ENABLE     (1<<0)
#define CP15_CR_DCACHE_ENABLE (1<<2)
#define CP15_CR_SB1           (0xF<<3)
#define CP15_CR_BIG_ENDIAN    (1<<7)
#define CP15_CR_ICACHE_ENABLE (1<<12)
#define CP15_CR_ALT_VECTORS   (1<<13)
#define CP15_CR_ROUND_ROBIN   (1<<14)
#define CP15_CR_DISABLE_TBIT  (1<<15)
#define CP15_CR_DTCM_ENABLE   (1<<16)
#define CP15_CR_DTCM_LOAD     (1<<17)
#define CP15_CR_ITCM_ENABLE   (1<<18)
#define CP15_CR_ITCM_LOAD     (1<<19)

#define CP15_TCM_4K   (0b00011 << 1)
#define CP15_TCM_8K   (0b00100 << 1)
#define CP15_TCM_16K  (0b00101 << 1)
#define CP15_TCM_32K  (0b00110 << 1)
#define CP15_TCM_64K  (0b00111 << 1)
#define CP15_TCM_128K (0b01000 << 1)
#define CP15_TCM_256K (0b01001 << 1)
#define CP15_TCM_512K (0b01010 << 1)
#define CP15_TCM_1M   (0b01011 << 1)
#define CP15_TCM_2M   (0b01100 << 1)
#define CP15_TCM_4M   (0b01101 << 1)
#define CP15_TCM_8M   (0b01110 << 1)
#define CP15_TCM_16M  (0b01111 << 1)
#define CP15_TCM_32M  (0b10000 << 1)
#define CP15_TCM_64M  (0b10001 << 1)
#define CP15_TCM_128M (0b10010 << 1)
#define CP15_TCM_256M (0b10011 << 1)

#define CP15_PU_ENABLE 1

#define CP15_PU_4K   (0b01011 << 1)
#define CP15_PU_8K   (0b01100 << 1)
#define CP15_PU_16K  (0b01101 << 1)
#define CP15_PU_32K  (0b01110 << 1)
#define CP15_PU_64K  (0b01111 << 1)
#define CP15_PU_128K (0b10000 << 1)
#define CP15_PU_256K (0b10001 << 1)
#define CP15_PU_512K (0b10010 << 1)
#define CP15_PU_1M   (0b10011 << 1)
#define CP15_PU_2M   (0b10100 << 1)
#define CP15_PU_4M   (0b10101 << 1)
#define CP15_PU_8M   (0b10110 << 1)
#define CP15_PU_16M  (0b10111 << 1)
#define CP15_PU_32M  (0b11000 << 1)
#define CP15_PU_64M  (0b11001 << 1)
#define CP15_PU_128M (0b11010 << 1)
#define CP15_PU_256M (0b11011 << 1)
#define CP15_PU_512M (0b11100 << 1)
#define CP15_PU_1G   (0b11101 << 1)
#define CP15_PU_2G   (0b11110 << 1)
#define CP15_PU_4G   (0b11111 << 1)

#define CP15_PU_PERM_NONE    0
#define CP15_PU_PERM_PRIV_RW 1
#define CP15_PU_PERM_RW_R    2
#define CP15_PU_PERM_RW      3
#define CP15_PU_PERM_PRIV_RO 5
#define CP15_PU_PERM_RO      6
