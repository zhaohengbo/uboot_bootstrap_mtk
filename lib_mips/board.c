/*
 * (C) Copyright 2003
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <command.h>
#include <malloc.h>
#include <devices.h>
#include <version.h>
#include <net.h>
#include <environment.h>
#include <asm/mipsregs.h>
#include <rt_mmap.h>
#include <spi_api.h>
#include <nand_api.h>
#include "LzmaWrapper.h"

DECLARE_GLOBAL_DATA_PTR;
#undef DEBUG

#define SDRAM_CFG1_REG RALINK_SYSCTL_BASE + 0x0304

#ifdef DEBUG
   #define DATE      "05/25/2006"
   #define VERSION   "v0.00e04"
#endif
#if ( ((CFG_ENV_ADDR+CFG_ENV_SIZE) < CFG_MONITOR_BASE) || \
      (CFG_ENV_ADDR >= (CFG_MONITOR_BASE + CFG_MONITOR_LEN)) ) || \
    defined(CFG_ENV_IS_IN_NVRAM)
#define	TOTAL_MALLOC_LEN	(CFG_MALLOC_LEN + CFG_ENV_SIZE)
#else
#define	TOTAL_MALLOC_LEN	CFG_MALLOC_LEN
#endif
#define ARGV_LEN  128

#if defined (RT6855A_ASIC_BOARD) || defined(RT6855A_FPGA_BOARD)	
static int watchdog_reset();
#endif

extern int timer_init(void);

extern void  rt2880_eth_halt(struct eth_device* dev);

extern void setup_internal_gsw(void); 
//extern void pci_init(void);

extern int incaip_set_cpuclk(void);
extern int do_bootm (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
extern int do_tftpb (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
extern int do_mem_cp ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
extern int flash_sect_protect (int p, ulong addr_first, ulong addr_last);
int flash_sect_erase (ulong addr_first, ulong addr_last);
int get_addr_boundary (ulong *addr);
extern int do_reset(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
extern void input_value(u8 *str);
#if defined (RT6855_ASIC_BOARD) || defined (RT6855_FPGA_BOARD) || \
    defined (MT7620_ASIC_BOARD) || defined (MT7620_FPGA_BOARD)
extern void rt_gsw_init(void);
#elif defined (RT6855A_ASIC_BOARD) || defined (RT6855A_FPGA_BOARD) 
extern void rt6855A_gsw_init(void);
#elif defined (RT3883_ASIC_BOARD) && defined (MAC_TO_MT7530_MODE)
extern void rt3883_gsw_init(void);
#else
extern void rt305x_esw_init(void);
#endif
extern void LANWANPartition(void);

extern struct eth_device* 	rt2880_pdev;

extern ulong uboot_end_data;
extern ulong uboot_end;
#ifdef RALINK_HTTP_UPGRADE_FUN
extern int NetUipLoop;
#endif
#if defined (RALINK_USB ) || defined (MTK_USB)
extern int usb_stor_curr_dev;
#endif

unsigned char load_addr_str[9];
ulong monitor_flash_len;

const char version_string[] =
	U_BOOT_VERSION" (" __DATE__ " - " __TIME__ ")";


static int auto_load = 0;


unsigned long mips_cpu_feq;
unsigned long mips_bus_feq;


/*
 * Begin and End of memory area for malloc(), and current "brk"
 */
static ulong mem_malloc_start;
static ulong mem_malloc_end;
static ulong mem_malloc_brk;

/*
 * The Malloc area is immediately below the monitor copy in DRAM
 */
static void mem_malloc_init(ulong dest_addr)
{
	/* ulong dest_addr = BOOTSTRAP_CFG_MONITOR_BASE + gd->reloc_off; */
	mem_malloc_end = dest_addr;
	mem_malloc_start = dest_addr - TOTAL_MALLOC_LEN;
	mem_malloc_brk = mem_malloc_start;

	memset((void *)mem_malloc_start, 0, mem_malloc_end - mem_malloc_start);
}

void *malloc(unsigned int size)
{
	if (size < (mem_malloc_end - mem_malloc_start)) {
		mem_malloc_start += size;
		return (void *)(mem_malloc_start - size);
	}

	return NULL;
}

void free(void *src)
{
	return;
}

/*
 * Begin and End of memory area for malloc(), and current "brk"
 */
static ulong mem_malloc_start;
static ulong mem_malloc_end;
static ulong mem_malloc_brk;

static char  file_name_space[ARGV_LEN];

#define read_32bit_cp0_register_with_select1(source)            \
({ int __res;                                                   \
        __asm__ __volatile__(                                   \
        ".set\tpush\n\t"                                        \
        ".set\treorder\n\t"                                     \
        "mfc0\t%0,"STR(source)",1\n\t"                          \
        ".set\tpop"                                             \
        : "=r" (__res));                                        \
        __res;})

#if defined (CONFIG_DDR_CAL)
__attribute__((nomips16)) void dram_cali(void);
#endif

static int init_func_ram (void)
{

#ifdef	CONFIG_BOARD_TYPES
	int board_type = gd->board_type;
#else
	int board_type = 0;	/* use dummy arg */
#endif
	//puts ("DRAM:  ");

/*init dram config*/
#ifdef RALINK_DDR_OPTIMIZATION
#ifdef ON_BOARD_DDR2
/*optimize ddr parameter*/
{	
	u32 tDDR;
	tDDR = RALINK_REG(DDR_CFG0_REG);

        tDDR &= 0xf0780000; 
	tDDR |=  RAS_VALUE << RAS_OFFSET;
	tDDR |=  TRFC_VALUE << TRFC_OFFSET;
	tDDR |=  TRFI_VALUE << TRFI_OFFSET;
	RALINK_REG(DDR_CFG0_REG) = tDDR;
}
#endif
#endif


	if ((gd->ram_size = initdram (board_type)) > 0) {
		//print_size (gd->ram_size, "\n");
		return (0);  
	}
	//puts ("*** failed ***\n");

	return (1);
}

static int init_baudrate (void)
{
	//uchar tmp[64]; /* long enough for environment variables */
	//int i = getenv_r ("baudrate", tmp, sizeof (tmp));
	//kaiker 
	gd->baudrate = CONFIG_BAUDRATE;
/*
	gd->baudrate = (i > 0)
			? (int) simple_strtoul (tmp, NULL, 10)
			: CONFIG_BAUDRATE;
*/
	return (0);
}


/*
 * Breath some life into the board...
 *
 * The first part of initialization is running from Flash memory;
 * its main purpose is to initialize the RAM so that we
 * can relocate the monitor code to RAM.
 */

/*
 * All attempts to come up with a "common" initialization sequence
 * that works for all boards and architectures failed: some of the
 * requirements are just _too_ different. To get rid of the resulting
 * mess of board dependend #ifdef'ed code we now make the whole
 * initialization sequence configurable to the user.
 *
 * The requirements for any new initalization function is simple: it
 * receives a pointer to the "global data" structure as it's only
 * argument, and returns an integer return code, where 0 means
 * "continue" and != 0 means "fatal error, hang the system".
 */

//  
__attribute__((nomips16)) void board_init_f(ulong bootflag)
{
	gd_t gd_data, *id;
	bd_t *bd;  
	//init_fnc_t **init_fnc_ptr;
	ulong addr, addr_sp, len = (ulong)&uboot_end - CFG_MONITOR_BASE;
	ulong *s;
	u32 value;
	void (*ptr)(void);
	u32 fdiv = 0, step = 0, frac = 0, i;

#if defined RT6855_FPGA_BOARD || defined RT6855_ASIC_BOARD || \
    defined MT7620_FPGA_BOARD || defined MT7620_ASIC_BOARD
	value = le32_to_cpu(*(volatile u_long *)(RALINK_SPI_BASE + 0x10));
	value &= ~(0x7);
	value |= 0x2;
	*(volatile u_long *)(RALINK_SPI_BASE + 0x10) = cpu_to_le32(value);	
#elif defined MT7621_FPGA_BOARD || defined MT7628_FPGA_BOARD
	value = le32_to_cpu(*(volatile u_long *)(RALINK_SPI_BASE + 0x3c));
	value &= ~(0xFFF);
	*(volatile u_long *)(RALINK_SPI_BASE + 0x3c) = cpu_to_le32(value);	
#elif defined MT7621_ASIC_BOARD
	value = le32_to_cpu(*(volatile u_long *)(RALINK_SPI_BASE + 0x3c));
	value &= ~(0xFFF);
	value |= 5; //work-around 3-wire SPI issue (3 for RFB, 5 for EVB)
	*(volatile u_long *)(RALINK_SPI_BASE + 0x3c) = cpu_to_le32(value);	
#elif  defined MT7628_ASIC_BOARD
	value = le32_to_cpu(*(volatile u_long *)(RALINK_SPI_BASE + 0x3c));
	value &= ~(0xFFF);
	value |= 8;
	*(volatile u_long *)(RALINK_SPI_BASE + 0x3c) = cpu_to_le32(value);	
#endif

#if defined(MT7620_FPGA_BOARD) || defined(MT7620_ASIC_BOARD)
/* Adjust CPU Freq from 60Mhz to 600Mhz(or CPLL freq stored from EE) */
	value = RALINK_REG(RT2880_SYSCLKCFG_REG);
	fdiv = ((value>>8)&0x1F);
	step = (unsigned long)(value&0x1F);
	while(step < fdiv) {
		value = RALINK_REG(RT2880_SYSCLKCFG_REG);
		step = (unsigned long)(value&0x1F) + 1;
		value &= ~(0x1F);
		value |= (step&0x1F);
		RALINK_REG(RT2880_SYSCLKCFG_REG) = value;
		udelay(10);
	};	
#elif defined(MT7628_ASIC_BOARD)
	value = RALINK_REG(RALINK_DYN_CFG0_REG);
	fdiv = (unsigned long)((value>>8)&0x0F);
	if ((CPU_FRAC_DIV < 1) || (CPU_FRAC_DIV > 10))
	frac = (unsigned long)(value&0x0F);
	else
		frac = CPU_FRAC_DIV;
	i = 0;
	
	while(frac < fdiv) {
		value = RALINK_REG(RALINK_DYN_CFG0_REG);
		fdiv = ((value>>8)&0x0F);
		fdiv--;
		value &= ~(0x0F<<8);
		value |= (fdiv<<8);
		RALINK_REG(RALINK_DYN_CFG0_REG) = value;
		udelay(500);
		i++;
		value = RALINK_REG(RALINK_DYN_CFG0_REG);
		fdiv = ((value>>8)&0x0F);
		//frac = (unsigned long)(value&0x0F);
	}	
#elif defined (MT7621_ASIC_BOARD)
#if (MT7621_CPU_FREQUENCY!=50)
	value = RALINK_REG(RALINK_CUR_CLK_STS_REG);
	fdiv = ((value>>8)&0x1F);
	frac = (unsigned long)(value&0x1F);

	i = 0;
	
	while(frac < fdiv) {
		value = RALINK_REG(RALINK_DYN_CFG0_REG);
		fdiv = ((value>>8)&0x0F);
		fdiv--;
		value &= ~(0x0F<<8);
		value |= (fdiv<<8);
		RALINK_REG(RALINK_DYN_CFG0_REG) = value;
		udelay(500);
		i++;
		value = RALINK_REG(RALINK_CUR_CLK_STS_REG);
		fdiv = ((value>>8)&0x1F);
		frac = (unsigned long)(value&0x1F);
	}
	
#endif
#if ((MT7621_CPU_FREQUENCY!=50) && (MT7621_CPU_FREQUENCY!=500))
	//change CPLL from GPLL to MEMPLL
	value = RALINK_REG(RALINK_CLKCFG0_REG);
	value &= ~(0x3<<30);
	value |= (0x1<<30);
	RALINK_REG(RALINK_CLKCFG0_REG) = value;	
#endif
#endif

#ifdef CONFIG_PURPLE
	void copy_code (ulong); 
#endif
	//*pio_mode = 0xFFFF;

	/* Pointer is writable since we allocated a register for it.
	 */
	gd = &gd_data;
	/* compiler optimization barrier needed for GCC >= 3.4 */
	__asm__ __volatile__("": : :"memory");
	
		
	memset ((void *)gd, 0, sizeof (gd_t));

#if defined (RT6855A_ASIC_BOARD) || defined(RT6855A_FPGA_BOARD)	
	watchdog_reset();
#endif
	timer_init();
	//env_init();		/* initialize environment */
	init_baudrate();		/* initialze baudrate settings */
	serial_init();		/* serial communications setup */
	
	//console_init_f();
#if (TEXT_BASE == 0xBFC00000) || (TEXT_BASE == 0xBF000000) || (TEXT_BASE == 0xBC000000)
#if defined (CONFIG_DDR_CAL)	
	ptr = dram_cali;
	ptr = (void*)((u32)ptr & ~(1<<29));
	(*ptr)();
#endif	
#endif
	//display_banner();		/* say that we are here */
	//checkboard();
	init_func_ram(); 

	/* reset Frame engine */
	value = le32_to_cpu(*(volatile u_long *)(RALINK_SYSCTL_BASE + 0x0034));
	udelay(100);    
#if defined (RT2880_FPGA_BOARD) || defined (RT2880_ASIC_BOARD)
	value |= (1 << 18);
#elif defined (MT7621_FPGA_BOARD) || defined (MT7621_ASIC_BOARD)
	/* nothing */
	//value |= (1<<26 | 1<<25 | 1<<24); /* PCIE de-assert for E3 */
#else
	//2880 -> 3052 reset Frame Engine from 18 to 21
	value |= (1 << 21);
#endif
	*(volatile u_long *)(RALINK_SYSCTL_BASE + 0x0034) = cpu_to_le32(value);	
#if defined (RT2880_FPGA_BOARD) || defined (RT2880_ASIC_BOARD)
	value &= ~(1 << 18);
#elif defined (MT7621_FPGA_BOARD) || defined (MT7621_ASIC_BOARD)
	/* nothing */
#else
	value &= ~(1 << 21);
#endif
	*(volatile u_long *)(RALINK_SYSCTL_BASE + 0x0034) = cpu_to_le32(value);	
	udelay(200);

	/*
	 * Now that we have DRAM mapped and working, we can
	 * relocate the code and continue running from DRAM.
	 */
	addr = CFG_SDRAM_BASE + gd->ram_size;

	/* round down to next 4 kB limit.
	 */
	addr &= ~(4096 - 1); 
   
	/* Reserve memory for U-Boot code, data & bss
	 * round down to next 16 kB limit
	 */
	addr -= len;
	addr &= ~(16 * 1024 - 1);
	 /* Reserve memory for malloc() arena.
	 */
	addr_sp = addr - TOTAL_MALLOC_LEN;
	/*
	 * (permanently) allocate a Board Info struct
	 * and a permanent copy of the "global" data
	 */
	addr_sp -= sizeof(bd_t);
	bd = (bd_t *)addr_sp;
	gd->bd = bd;
	
	addr_sp -= sizeof(gd_t);
	id = (gd_t *)addr_sp;
	
 	/* Reserve memory for boot params.
	 */
	addr_sp -= CFG_BOOTPARAMS_LEN;
	bd->bi_boot_params = addr_sp;

	/*
	 * Finally, we set up a new (bigger) stack.
	 *
	 * Leave some safety gap for SP, force alignment on 16 byte boundary
	 * Clear initial stack frame
	 */
	addr_sp -= 16;
	addr_sp &= ~0xF;
	s = (ulong *)addr_sp;
	*s-- = 0;
	*s-- = 0;
	addr_sp = (ulong)s;

	/*
	 * Save local variables to board info struct
	 */
	bd->bi_memstart	= CFG_SDRAM_BASE;	/* start of  DRAM memory */
	bd->bi_memsize	= gd->ram_size;		/* size  of  DRAM memory in bytes */
	bd->bi_baudrate	= gd->baudrate;		/* Console Baudrate */

	memcpy (id, (void *)gd, sizeof (gd_t));

	/* On the purple board we copy the code in a special way
	 * in order to solve flash problems
	 */
#ifdef CONFIG_PURPLE
	copy_code(addr);
#endif
	
	memcpy((void *)addr,(void *)CFG_MONITOR_BASE,(unsigned long)&uboot_end_data - CFG_MONITOR_BASE);
	
	extern void flush_cache (ulong start_addr, ulong size);
	
	flush_cache(addr,(unsigned long)&uboot_end_data - CFG_MONITOR_BASE);
	
	serial_puts("mtk_uboot_bootstrap by zhaoxiaowei\n");
#if defined(CFG_RUN_CODE_IN_RAM)
	/* 
	 * tricky: relocate code to original TEXT_BASE
	 * for ICE souce level debuggind mode 
	 */	
	relocate_code (addr_sp, id, /*TEXT_BASE*/ addr);	
#else
	relocate_code (addr_sp, id, addr);
#endif

	/* NOTREACHED - relocate_code() does not return */
}

/************************************************************************
 *
 * This is the next part if the initialization sequence: we are now
 * running from RAM and have a "normal" C environment, i. e. global
 * data can be written, BSS has been cleared, the stack size in not
 * that critical any more, etc.
 *
 ************************************************************************
 */

gd_t gd_data;
 
__attribute__((nomips16)) void board_init_r (gd_t *id, ulong dest_addr)
{
	int i;
	ulong addr;
	ulong data, len;
	image_header_t header;
	image_header_t *hdr = &header;
	unsigned int destLen;
	int (*fn)(int);
	
	/* Initialize malloc() area */
	mem_malloc_init(dest_addr);
	
	addr = (ulong)((char *)(CFG_MONITOR_BASE + ((ulong)&uboot_end_data - dest_addr)));
	memmove(&header, (char *)addr, sizeof(image_header_t));

	if (ntohl(hdr->ih_magic) != IH_MAGIC)
	{
		serial_puts("bad magic!\n");
		hang();
	}

	data = addr + sizeof(image_header_t);
	len = ntohl(hdr->ih_size);

	/*
	 * If we've got less than 4 MB of malloc() space,
	 * use slower decompression algorithm which requires
	 * at most 2300 KB of memory.
	 */
	destLen = 0x0;
	
	serial_puts("delzma second bootloader\n");

#ifdef CONFIG_LZMA
	i = lzma_inflate((unsigned char *)data, len, (unsigned char*)ntohl(hdr->ih_load), (int *)&destLen);

	if (i != LZMA_RESULT_OK)
	{
		serial_puts("lzma failed!\n");
		hang();
	}
#endif

	fn = (void *)ntohl(hdr->ih_load);
	flush_cache((ulong)ntohl(hdr->ih_load), (ulong)destLen);
	
	serial_puts("starting second bootloader\n");

	(*fn)(id->ram_size);

	hang();
	/* NOTREACHED - no way out of command loop except booting */
}


void hang (void)
{
	serial_puts ("### ERROR ### Please RESET the board ###\n");
	for (;;);
}

#if defined (RT6855A_ASIC_BOARD) || defined(RT6855A_FPGA_BOARD)	
static int watchdog_reset()
{
	unsigned int word;
	unsigned int i;

	/* check if do watch dog reset */
	if ((RALINK_REG(RALINK_HIR_REG) & 0xffff0000) == 0x40000) {
		if (!(RALINK_REG(0xbfb00080) >> 31)) {
			/* set delay counter */
			RALINK_REG(RALINK_TIMER5_LDV) = 1000;
			/* enable watch dog timer */
			word = RALINK_REG(RALINK_TIMER_CTL);
			word |= ((1 << 5) | (1 << 25));
			RALINK_REG(RALINK_TIMER_CTL) = word;
			while(1);
		}
	}

	return 0;
}
#endif

#if defined (CONFIG_DDR_CAL)
#include <asm-mips/mipsregs.h>
#include <asm-mips/cacheops.h>
#include <asm/mipsregs.h>
#include <asm/cache.h>

#if defined (CONFIG_TINY_UBOOT)
__attribute__((nomips16))
#endif
static inline void cal_memcpy(void* src, void* dst, unsigned int size)
{
	int i;
	unsigned char* psrc = (unsigned char*)src, *pdst=(unsigned char*)dst;
	for (i = 0; i < size; i++, psrc++, pdst++)
		(*pdst) = (*psrc);
	return;
}
#if defined (CONFIG_TINY_UBOOT)
__attribute__((nomips16))
#endif
static inline void cal_memset(void* src, unsigned char pat, unsigned int size)
{
	int i;
	unsigned char* psrc = (unsigned char*)src;
	for (i = 0; i < size; i++, psrc++)
		(*psrc) = pat;
	return;
}

#define pref_op(hint,addr)						\
	 __asm__ __volatile__(						\
	"       .set    push                                    \n"	\
	"       .set    noreorder                               \n"	\
	"       pref   %0, %1                                  \n"	\
	"       .set    pop                                     \n"	\
	:								\
	: "i" (hint), "R" (*(unsigned char *)(addr)))	
		
#define cache_op(op,addr)						\
	 __asm__ __volatile__(						\
	"       .set    push                                    \n"	\
	"       .set    noreorder                               \n"	\
	"       .set    mips3\n\t                               \n"	\
	"       cache   %0, %1                                  \n"	\
	"       .set    pop                                     \n"	\
	:								\
	: "i" (op), "R" (*(unsigned char *)(addr)))	

__attribute__((nomips16)) static void inline cal_invalidate_dcache_range(ulong start_addr, ulong stop)
{
	unsigned long lsize = CONFIG_SYS_CACHELINE_SIZE;
	unsigned long addr = start_addr & ~(lsize - 1);
	unsigned long aend = (stop - 1) & ~(lsize - 1);

	while (1) {
		cache_op(HIT_INVALIDATE_D, addr);
		if (addr == aend)
			break;
		addr += lsize;
	}
}	

#if defined (CONFIG_TINY_UBOOT)
__attribute__((nomips16))
#endif
static void inline cal_patgen(unsigned long* start_addr, unsigned int size, unsigned bias)
{
	int i = 0;
	for (i = 0; i < size; i++)
		start_addr[i] = ((ulong)start_addr+i+bias);
		
	return;	
}	

#define NUM_OF_CACHELINE	128
#define MIN_START	6
#define MIN_FINE_START	0xF
#define MAX_START 7
#define MAX_FINE_START	0x0
#define cal_debug debug
								
#define HWDLL_FIXED	1
#if defined (HWDLL_FIXED)								
#define DU_COARSE_WIDTH	16
#define DU_FINE_WIDTH 16
#define C2F_RATIO 8
#define HWDLL_AVG	1
#define HWDLL_LV	1
//#define HWDLL_HV	1
#define HWDLL_MINSCAN	1
#endif

#define MAX_TEST_LOOP   8								
								
__attribute__((nomips16)) void dram_cali(void)
{
#if defined(ON_BOARD_64M_DRAM_COMPONENT)
	#define DRAM_BUTTOM 0x800000	
#endif
#if defined(ON_BOARD_128M_DRAM_COMPONENT)
	#define DRAM_BUTTOM 0x1000000	
#endif	
#if defined(ON_BOARD_256M_DRAM_COMPONENT)
	#define DRAM_BUTTOM 0x2000000	
#endif
#if defined(ON_BOARD_512M_DRAM_COMPONENT)
	#define DRAM_BUTTOM 0x4000000	
#endif
#if defined(ON_BOARD_1024M_DRAM_COMPONENT)
	#define DRAM_BUTTOM 0x8000000	
#endif
#if defined(ON_BOARD_2048M_DRAM_COMPONENT)
	#define DRAM_BUTTOM 0x10000000
#endif

	unsigned int * nc_addr = 0xA0000000+DRAM_BUTTOM-0x0400;
	unsigned int * c_addr = 0x80000000+DRAM_BUTTOM-0x0400;
	unsigned int min_coarse_dqs[2];
	unsigned int max_coarse_dqs[2];
	unsigned int min_fine_dqs[2];
	unsigned int max_fine_dqs[2];
	unsigned int coarse_dqs[2];
	unsigned int fine_dqs[2];
	unsigned int min_dqs[2];
	unsigned int max_dqs[2];
	unsigned int total_min_comp_dqs[2];
	unsigned int total_max_comp_dqs[2];
	unsigned int avg_min_cg_comp_dqs[2];
	unsigned int avg_max_cg_comp_dqs[2];
	unsigned int avg_min_fg_comp_dqs[2];
	unsigned int avg_max_fg_comp_dqs[2];
	unsigned int min_comp_dqs[2][MAX_TEST_LOOP];
	unsigned int max_comp_dqs[2][MAX_TEST_LOOP];
	unsigned int reg = 0, ddr_cfg2_reg = 0, dqs_dly_reg = 0;
	unsigned int reg_avg = 0, reg_with_dll = 0, hw_dll_reg = 0;
	int ret = 0;
	int flag = 0, min_failed_pos[2], max_failed_pos[2], min_fine_failed_pos[2], max_fine_failed_pos[2];
	int i,j, k;
	int dqs = 0;
	unsigned int min_coarse_dqs_bnd, min_fine_dqs_bnd, coarse_dqs_dll, fine_dqs_dll;
	unsigned int reg_minscan = 0;
	unsigned int avg_cg_dly[2],avg_fg_dly[2];
	unsigned int g_min_coarse_dqs_dly[2], g_min_fine_dqs_dly[2];
#if defined(MT7628_FPGA_BOARD) || defined(MT7628_ASIC_BOARD)		
	//unsigned int cid = (RALINK_REG(RALINK_SYSCTL_BASE+0xC)>>16)&0x01;
	unsigned int cid = 0;
#endif

#if (NUM_OF_CACHELINE > 40)
#else	
	unsigned int cache_pat[8*40];
#endif	
	u32 value, test_count = 0;;
	u32 fdiv = 0, step = 0, frac = 0;

	value = RALINK_REG(RALINK_DYN_CFG0_REG);
	fdiv = (unsigned long)((value>>8)&0x0F);
	if ((CPU_FRAC_DIV < 1) || (CPU_FRAC_DIV > 10))
	frac = (unsigned long)(value&0x0F);
	else
		frac = CPU_FRAC_DIV;
	i = 0;
	
	while(frac < fdiv) {
		value = RALINK_REG(RALINK_DYN_CFG0_REG);
		fdiv = ((value>>8)&0x0F);
		fdiv--;
		value &= ~(0x0F<<8);
		value |= (fdiv<<8);
		RALINK_REG(RALINK_DYN_CFG0_REG) = value;
		udelay(500);
		i++;
		value = RALINK_REG(RALINK_DYN_CFG0_REG);
		fdiv = ((value>>8)&0x0F);
	}	
#if (NUM_OF_CACHELINE > 40)
#else
	cal_memcpy(cache_pat, dram_patterns, 32*6);
	cal_memcpy(cache_pat+32*6, line_toggle_pattern, 32);
	cal_memcpy(cache_pat+32*6+32, pattern_ken, 32*13);
#endif

#if defined (HWDLL_LV)
#if defined (HWDLL_HV)
	RALINK_REG(RALINK_RGCTRL_BASE+0x108) = 0x01300;
	mdelay(100);
#else
	//RALINK_REG(RALINK_RGCTRL_BASE+0x108) = 0x0F00;//0x0d00;//0x0b00;
#endif
	//cal_debug("\nSet [0x10001108]=%08X\n",RALINK_REG(RALINK_RGCTRL_BASE+0x108));
	//mdelay(100);
#endif
	
#if defined(MT7628_FPGA_BOARD) || defined(MT7628_ASIC_BOARD)	
	RALINK_REG(RALINK_MEMCTRL_BASE+0x10) &= ~(0x1<<4);
#else
	RALINK_REG(RALINK_MEMCTRL_BASE+0x18) = &= ~(0x1<<4);
#endif
	ddr_cfg2_reg = RALINK_REG(RALINK_MEMCTRL_BASE+0x48);
	dqs_dly_reg = RALINK_REG(RALINK_MEMCTRL_BASE+0x64);
	RALINK_REG(RALINK_MEMCTRL_BASE+0x48)&=(~((0x3<<28)|(0x3<<26)));

	total_min_comp_dqs[0] = 0;
	total_min_comp_dqs[1] = 0;
	total_max_comp_dqs[0] = 0;
	total_max_comp_dqs[1] = 0;
TEST_LOOP:
	min_coarse_dqs[0] = MIN_START;
	min_coarse_dqs[1] = MIN_START;
	min_fine_dqs[0] = MIN_FINE_START;
	min_fine_dqs[1] = MIN_FINE_START;
	max_coarse_dqs[0] = MAX_START;
	max_coarse_dqs[1] = MAX_START;
	max_fine_dqs[0] = MAX_FINE_START;
	max_fine_dqs[1] = MAX_FINE_START;
	min_failed_pos[0] = 0xFF;
	min_fine_failed_pos[0] = 0;
	min_failed_pos[1] = 0xFF;
	min_fine_failed_pos[1] = 0;
	max_failed_pos[0] = 0xFF;
	max_fine_failed_pos[0] = 0;
	max_failed_pos[1] = 0xFF;
	max_fine_failed_pos[1] = 0;
	dqs = 0;

	// Add by KP, DQS MIN boundary
	reg = RALINK_REG(RALINK_MEMCTRL_BASE+0x20);
	coarse_dqs_dll = (reg & 0xF00) >> 8;
	fine_dqs_dll = (reg & 0xF0) >> 4;
	if (coarse_dqs_dll<=8)
		min_coarse_dqs_bnd = 8 - coarse_dqs_dll;
	else
		min_coarse_dqs_bnd = 0;
	
	if (fine_dqs_dll<=8)
		min_fine_dqs_bnd = 8 - fine_dqs_dll;
	else
		min_fine_dqs_bnd = 0;
	// DQS MIN boundary
 
DQS_CAL:
	flag = 0;
	j = 0;

	for (k = 0; k < 2; k ++)
	{
		unsigned int test_dqs, failed_pos = 0;
		if (k == 0)
			test_dqs = MAX_START;
		else
			test_dqs = MAX_FINE_START;	
		flag = 0;
		do 
		{	
			flag = 0;
			for (nc_addr = 0xA0000000; nc_addr < (0xA0000000+DRAM_BUTTOM-NUM_OF_CACHELINE*32); nc_addr+=((DRAM_BUTTOM>>6)+1*0x400))
			{
				RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = 0x00007474;
				wmb();
				c_addr = (unsigned int*)((ulong)nc_addr & 0xDFFFFFFF);
				cal_memset(((unsigned char*)c_addr), 0x1F, NUM_OF_CACHELINE*32);
#if (NUM_OF_CACHELINE > 40)
				cal_patgen(nc_addr, NUM_OF_CACHELINE*8, 3);
#else			
				cal_memcpy(((unsigned char*)nc_addr), ((unsigned char*)cache_pat), NUM_OF_CACHELINE*32);
#endif			
				
				if (dqs > 0)
					RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = 0x00000074|(((k==1) ? max_coarse_dqs[dqs] : test_dqs)<<12)|(((k==0) ? 0xF : test_dqs)<<8);
				else
					RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = 0x00007400|(((k==1) ? max_coarse_dqs[dqs] : test_dqs)<<4)|(((k==0) ? 0xF : test_dqs)<<0);
				wmb();
				
				cal_invalidate_dcache_range(((unsigned char*)c_addr), ((unsigned char*)c_addr)+NUM_OF_CACHELINE*32);
				wmb();
				for (i = 0; i < NUM_OF_CACHELINE*8; i ++)
				{
					if (i % 8 ==0)
						pref_op(0, &c_addr[i]);
				}		
				for (i = 0; i < NUM_OF_CACHELINE*8; i ++)
				{
#if (NUM_OF_CACHELINE > 40)
					if (c_addr[i] != (ulong)nc_addr+i+3)
#else				
					if (c_addr[i] != cache_pat[i])
#endif				
					{	
						flag = -1;
						failed_pos = i;
						goto MAX_FAILED;
					}
				}
			}
MAX_FAILED:
			if (flag==-1)
			{
				break;
			}
			else
				test_dqs++;
		}while(test_dqs<=0xF);
		
		if (k==0)
		{	
			max_coarse_dqs[dqs] = test_dqs;
			max_failed_pos[dqs] = failed_pos;
		}
		else
		{	
			test_dqs--;
	
			if (test_dqs==MAX_FINE_START-1)
			{
				max_coarse_dqs[dqs]--;
				max_fine_dqs[dqs] = 0xF;	
			}
			else
			{	
				max_fine_dqs[dqs] = test_dqs;
			}
			max_fine_failed_pos[dqs] = failed_pos;
		}	
	}	

	for (k = 0; k < 2; k ++)
	{
		unsigned int test_dqs, failed_pos = 0;
		if (k == 0)
			test_dqs = MIN_START;
		else
			test_dqs = MIN_FINE_START;	
		flag = 0;
		do 
		{
			for (nc_addr = 0xA0000000; nc_addr < (0xA0000000+DRAM_BUTTOM-NUM_OF_CACHELINE*32); (nc_addr+=(DRAM_BUTTOM>>6)+1*0x480))
			{
				RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = 0x00007474;
				wmb();
				c_addr = (unsigned int*)((ulong)nc_addr & 0xDFFFFFFF);
				RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = 0x00007474;
				wmb();
				cal_memset(((unsigned char*)c_addr), 0x1F, NUM_OF_CACHELINE*32);
#if (NUM_OF_CACHELINE > 40)
				cal_patgen(nc_addr, NUM_OF_CACHELINE*8, 1);
#else			
				cal_memcpy(((unsigned char*)nc_addr), ((unsigned char*)cache_pat), NUM_OF_CACHELINE*32);
#endif				
				if (dqs > 0)
					RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = 0x00000074|(((k==1) ? min_coarse_dqs[dqs] : test_dqs)<<12)|(((k==0) ? 0x0 : test_dqs)<<8);
				else
					RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = 0x00007400|(((k==1) ? min_coarse_dqs[dqs] : test_dqs)<<4)|(((k==0) ? 0x0 : test_dqs)<<0);			
				wmb();
				cal_invalidate_dcache_range(((unsigned char*)c_addr), ((unsigned char*)c_addr)+NUM_OF_CACHELINE*32);
				wmb();
				for (i = 0; i < NUM_OF_CACHELINE*8; i ++)
				{
					if (i % 8 ==0)
						pref_op(0, &c_addr[i]);
				}	
				for (i = 0; i < NUM_OF_CACHELINE*8; i ++)
				{
#if (NUM_OF_CACHELINE > 40)
					if (c_addr[i] != (ulong)nc_addr+i+1)	
#else				
					if (c_addr[i] != cache_pat[i])
#endif
					{
						flag = -1;
						failed_pos = i;
						goto MIN_FAILED;
					}
				}
			}
	
MIN_FAILED:
	
			if (k==0)
			{	
				if ((flag==-1)||(test_dqs==min_coarse_dqs_bnd))
				{
					break;
				}
				else
					test_dqs--;
					
				if (test_dqs < min_coarse_dqs_bnd)
					break;	
			}
			else
			{
				if (flag==-1)
				{
					test_dqs++;
					break;
				}
				else if (test_dqs==min_fine_dqs_bnd)
				{
					break;
				}
				else
				{	
					test_dqs--;                    
				}
				
				if (test_dqs < min_fine_dqs_bnd)
					break;
				
			}
		}while(test_dqs>=0);
		
		if (k==0)
		{	
			min_coarse_dqs[dqs] = test_dqs;
			min_failed_pos[dqs] = failed_pos;
		}
		else
		{	
			if (test_dqs==MIN_FINE_START+1)
			{
				min_coarse_dqs[dqs]++;
				min_fine_dqs[dqs] = 0x0;	
			}
			else
			{	
				min_fine_dqs[dqs] = test_dqs;
			}
			min_fine_failed_pos[dqs] = failed_pos;
		}
	}

	min_comp_dqs[dqs][test_count] = (8-min_coarse_dqs[dqs])*8+(8-min_fine_dqs[dqs]);
	total_min_comp_dqs[dqs] += min_comp_dqs[dqs][test_count];
	max_comp_dqs[dqs][test_count] = (max_coarse_dqs[dqs]-8)*8+(max_fine_dqs[dqs]-8);
	total_max_comp_dqs[dqs] += max_comp_dqs[dqs][test_count];

	if (max_comp_dqs[dqs][test_count]+ min_comp_dqs[dqs][test_count] <=(2*8))
	{
		reg_minscan = 0x18180000;
		reg_with_dll = 0x88880000;
		g_min_coarse_dqs_dly[0] = g_min_coarse_dqs_dly[1] = 0;
		g_min_fine_dqs_dly[0] = g_min_fine_dqs_dly[1] = 0;
		hw_dll_reg = RALINK_REG(RALINK_MEMCTRL_BASE+0x20);
		goto FINAL_SETUP;
	}	

	if (dqs==0)
	{
		dqs = 1;	
		goto DQS_CAL;
	}

	for (i=0 ; i < 2; i++)
	{
		unsigned int temp;
		coarse_dqs[i] = (max_coarse_dqs[i] + min_coarse_dqs[i])>>1; 
		temp = (((max_coarse_dqs[i] + min_coarse_dqs[i])%2)*4)  +  ((max_fine_dqs[i] + min_fine_dqs[i])>>1);
		if (temp >= 0x10)
		{
		   coarse_dqs[i] ++;
		   fine_dqs[i] = (temp-0x10) +0x8;
		}
		else
		{
			fine_dqs[i] = temp;
		}
	}
	reg = (coarse_dqs[1]<<12)|(fine_dqs[1]<<8)|(coarse_dqs[0]<<4)|fine_dqs[0];
	
#if defined(MT7628_FPGA_BOARD) || defined(MT7628_ASIC_BOARD)
	if (cid == 1)
		RALINK_REG(RALINK_MEMCTRL_BASE+0x10) &= ~(0x1<<4);
#else
	RALINK_REG(RALINK_MEMCTRL_BASE+0x18) = &= ~(0x1<<4);
#endif
	if (cid == 1) {
		RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = reg;
		RALINK_REG(RALINK_MEMCTRL_BASE+0x48) = ddr_cfg2_reg;
	}
#if defined(MT7628_FPGA_BOARD) || defined(MT7628_ASIC_BOARD)	
	if (cid == 1)
		RALINK_REG(RALINK_MEMCTRL_BASE+0x10) |= (0x1<<4);
#else
	RALINK_REG(RALINK_MEMCTRL_BASE+0x18) |= (0x1<<4);
#endif

	test_count++;


FINAL:
#if defined(MT7628_FPGA_BOARD) || defined(MT7628_ASIC_BOARD)	
	if (cid==1)
#endif	
	{	
		//for (j = 0; j < 2; j++)	
			//cal_debug("[%02X%02X%02X%02X]",min_coarse_dqs[j],min_fine_dqs[j], max_coarse_dqs[j],max_fine_dqs[j]);
		//cal_debug("\nDDR Calibration DQS reg = %08X\n",reg);
		goto EXIT;
	}
	if (test_count < MAX_TEST_LOOP)
		goto TEST_LOOP;

	for (j = 0; j < 2; j++)	
	{
		unsigned int min_count = MAX_TEST_LOOP;
		unsigned int max_count = MAX_TEST_LOOP;
		
		unsigned int tmp_min_comp_dqs = total_min_comp_dqs[j]>>3;
		unsigned int tmp_total_min_comp_dqs = total_min_comp_dqs[j];
		
		unsigned int tmp_max_comp_dqs = total_max_comp_dqs[j]>>3;
		unsigned int tmp_total_max_comp_dqs = total_max_comp_dqs[j];
		
		for (k = 0; k < MAX_TEST_LOOP; k++)
		{
			int diff_min = ((tmp_min_comp_dqs-min_comp_dqs[j][k]) > 0) ? (tmp_min_comp_dqs-min_comp_dqs[j][k]) : (min_comp_dqs[j][k]-tmp_min_comp_dqs);
			int diff_max = ((tmp_max_comp_dqs-max_comp_dqs[j][k]) > 0) ? (tmp_max_comp_dqs-max_comp_dqs[j][k]) : (max_comp_dqs[j][k]-tmp_max_comp_dqs);

			if (diff_min > 5)
			{
				//cal_debug("remove the %d min comp dqs %d (%d)\n" ,k ,min_comp_dqs[j][k],tmp_min_comp_dqs);
				tmp_total_min_comp_dqs-= min_comp_dqs[j][k];
				tmp_total_min_comp_dqs += tmp_min_comp_dqs;
				min_count--;
			}
			if (diff_max > 5)
			{
				//cal_debug("remove the %d (diff=%d) max comp dqs %d (%d)\n" ,k ,diff_max,max_comp_dqs[j][k],tmp_max_comp_dqs);
				tmp_total_max_comp_dqs-= max_comp_dqs[j][k];
				tmp_total_max_comp_dqs += tmp_max_comp_dqs;
				max_count--;
			}
		}	
		tmp_min_comp_dqs = tmp_total_min_comp_dqs>>3;
		avg_min_cg_comp_dqs[j] = 8-(tmp_min_comp_dqs>>3);
		avg_min_fg_comp_dqs[j] = 8-(tmp_min_comp_dqs&0x7);
		
		tmp_max_comp_dqs = tmp_total_max_comp_dqs>>3;
		avg_max_cg_comp_dqs[j] = 8+(tmp_max_comp_dqs>>3);
		avg_max_fg_comp_dqs[j] = 8+(tmp_max_comp_dqs&0x7);
		
	}
	//cal_debug("\n\r");
	//for (j = 0; j < 2; j++)	
	//		cal_debug("[%02X%02X%02X%02X]", avg_min_cg_comp_dqs[j],avg_min_fg_comp_dqs[j], avg_max_cg_comp_dqs[j],avg_max_fg_comp_dqs[j]);
	
	for (i=0 ; i < 2; i++)
	{
		unsigned int temp;
		coarse_dqs[i] = (avg_max_cg_comp_dqs[i] + avg_min_cg_comp_dqs[i])>>1; 
		temp = (((avg_max_cg_comp_dqs[i] + avg_min_cg_comp_dqs[i])%2)*4)  +  ((avg_max_fg_comp_dqs[i] + avg_min_fg_comp_dqs[i])>>1);
		if (temp >= 0x10)
		{
		   coarse_dqs[i] ++;
		   fine_dqs[i] = (temp-0x10) +0x8;
		}
		else
		{
			fine_dqs[i] = temp;
		}
	}

	reg = (coarse_dqs[1]<<12)|(fine_dqs[1]<<8)|(coarse_dqs[0]<<4)|fine_dqs[0];
	
#if defined (HWDLL_FIXED)
/* Read DLL HW delay */
{
	unsigned int sel_fine[2],sel_coarse[2];
	int sel_mst_coarse, sel_mst_fine;
	unsigned int avg_cg_dly[2],avg_fg_dly[2];
	
	hw_dll_reg = RALINK_REG(RALINK_MEMCTRL_BASE+0x20);
	sel_mst_coarse = (hw_dll_reg >> 8) & 0x0F;
	sel_mst_fine = (hw_dll_reg >> 4) & 0x0F;	
	for (j = 0; j < 2; j++)
	{	
		int cg_dly_adj, fg_dly_adj,sel_fine_tmp,sel_coarse_tmp;

		cg_dly_adj = coarse_dqs[j];
		fg_dly_adj = fine_dqs[j]; 	
		
		sel_fine_tmp = ((sel_mst_fine + fg_dly_adj) < 8 ) ? 0 : (sel_mst_fine + fg_dly_adj - 8);
		sel_coarse_tmp = ((sel_mst_coarse + cg_dly_adj) > DU_COARSE_WIDTH -1 + 8) ? DU_COARSE_WIDTH-1 : \
			  ((sel_mst_coarse + cg_dly_adj) < 8) ? 0 : sel_mst_coarse + cg_dly_adj -8;
		
		if (sel_fine_tmp > (DU_FINE_WIDTH-1)) {
			if (sel_coarse_tmp < (DU_COARSE_WIDTH-1)) {
				sel_fine[j] = sel_fine_tmp - C2F_RATIO;
				sel_coarse[j] = 	sel_coarse_tmp + 1;
			}
			else {
				sel_fine[j] = DU_FINE_WIDTH-1;
				sel_coarse[j] = 	sel_coarse_tmp;
			}
		}
		else if (sel_fine_tmp < 0){
			if (sel_coarse_tmp > 0) {
				sel_fine[j] = sel_fine_tmp + C2F_RATIO;
				sel_coarse[j] = 	sel_coarse_tmp - 1;
			}
			else {
				//saturate
				sel_fine[j] = 0;
				sel_coarse[j] = 	sel_coarse_tmp;
			}
		}
		else {
			sel_fine[j] = sel_fine_tmp;
			sel_coarse[j] = 	sel_coarse_tmp;
		}
		if ((sel_fine[j] & 0xf) != sel_fine[j])
		{
			sel_fine[j] = sel_fine[j] & 0xf;
		}
		if ((sel_coarse[j] & 0xf) != sel_coarse[j])
		{
			sel_coarse[j] = sel_coarse[j] & 0xf;
		}
	}
	reg_with_dll = (sel_coarse[1]<<28)|(sel_fine[1]<<24)|(sel_coarse[0]<<20)|(sel_fine[0]<<16);
	
#if defined(HWDLL_AVG)
	for (j = 0; j < 2; j++)
	{
		unsigned int avg;
		int min_coarse_dqs_dly,min_fine_dqs_dly; 
		min_coarse_dqs_dly = sel_mst_coarse - (8 - min_coarse_dqs[j]);
		min_fine_dqs_dly = sel_mst_fine - (8 -min_fine_dqs[j]);
		min_coarse_dqs_dly = (min_coarse_dqs_dly < 0) ? 0 : min_coarse_dqs_dly;
		min_fine_dqs_dly = (min_fine_dqs_dly < 0) ? 0 : min_fine_dqs_dly;
		
		
		avg_cg_dly[j] = ((min_coarse_dqs_dly<<1) + (sel_coarse[j]<<1))>>1;
		avg_cg_dly[j] = avg_cg_dly[j]&0x01 ? ((avg_cg_dly[j]>>1)+1) : (avg_cg_dly[j]>>1);
			
		avg_fg_dly[j] = ((min_fine_dqs_dly<<1) + (sel_fine[j]<<1))>>1;
		avg_fg_dly[j] = avg_fg_dly[j]&0x01 ? ((avg_fg_dly[j]>>1)+1) : (avg_fg_dly[j]>>1);
		
		g_min_coarse_dqs_dly[j] = min_coarse_dqs_dly;
		g_min_fine_dqs_dly[j] = min_fine_dqs_dly;
		if ((avg_cg_dly[j] & 0xf) != avg_cg_dly[j])
		{
			avg_cg_dly[j] = avg_cg_dly[j] & 0xf;
		}
		if ((avg_fg_dly[j] & 0xf) != avg_fg_dly[j])
		{
			avg_fg_dly[j] = avg_fg_dly[j] & 0xf;
		}
	}
	reg_avg = (avg_cg_dly[1]<<28)|(avg_fg_dly[1]<<24)|(avg_cg_dly[0]<<20)|(avg_fg_dly[0]<<16);
#endif

#if defined (HWDLL_MINSCAN)
{
	unsigned int min_scan_cg_dly[2], min_scan_fg_dly[2], adj_dly[2], reg_scan;
	
	RALINK_REG(RALINK_RGCTRL_BASE+0x108) = 0x01300;

	k=9583000;
	do {k--; }while(k>0);
		
	RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = reg_with_dll;
	RALINK_REG(RALINK_MEMCTRL_BASE+0x68) |= (0x1<<4);
	
	for (j = 0; j < 2; j++)
	{
		min_scan_cg_dly[j] = 0;
		min_scan_fg_dly[j] = 0;
	
		do
		{	
				int diff_dly;
				for (nc_addr = 0xA0000000; nc_addr < (0xA0000000+DRAM_BUTTOM-NUM_OF_CACHELINE*32); nc_addr+=((DRAM_BUTTOM>>6)+1*0x400))
				{
					
					RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = reg_with_dll;
					wmb();
					c_addr = (unsigned int*)((ulong)nc_addr & 0xDFFFFFFF);
					cal_memset(((unsigned char*)c_addr), 0x1F, NUM_OF_CACHELINE*32);
#if (NUM_OF_CACHELINE > 40)
					cal_patgen(nc_addr, NUM_OF_CACHELINE*8, 2);
#else			
					cal_memcpy(((unsigned char*)nc_addr), ((unsigned char*)cache_pat), NUM_OF_CACHELINE*32);
#endif			
					if (j == 0)
						reg_scan = (reg_with_dll&0xFF000000)|(min_scan_cg_dly[j]<<20)|(min_scan_fg_dly[j]<<16);
					else		
						reg_scan = (reg_with_dll&0x00FF0000)|(min_scan_cg_dly[j]<<28)|(min_scan_fg_dly[j]<<24);	
					
					RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = reg_scan;
					wmb();		
					cal_invalidate_dcache_range(((unsigned char*)c_addr), ((unsigned char*)c_addr)+NUM_OF_CACHELINE*32);
					wmb();

					for (i = 0; i < NUM_OF_CACHELINE*8; i ++)
					{
						if (i % 8 ==0)
							pref_op(0, &c_addr[i]);
					}		
					for (i = 0; i < NUM_OF_CACHELINE*8; i ++)
					{
#if (NUM_OF_CACHELINE > 40)
						if (c_addr[i] != (ulong)nc_addr+i+2)
#else				
						if (c_addr[i] != cache_pat[i])
#endif				
						{
							goto MINSCAN_FAILED;
						}
					}
				}	
				diff_dly = (avg_cg_dly[j]*8 + avg_fg_dly[j])-(min_scan_cg_dly[j]*8+min_scan_fg_dly[j]);
				//if (diff_dly < 0)
					//cal_debug("diff_dly=%d\n",diff_dly);
					
				if (diff_dly < 6)
					adj_dly[j] = (avg_cg_dly[j]*8 + avg_fg_dly[j]) + (6 - diff_dly);
				else
					adj_dly[j] = (avg_cg_dly[j]*8 + avg_fg_dly[j]);

				break;
MINSCAN_FAILED:
				min_scan_fg_dly[j] ++;
				if (min_scan_fg_dly[j] > 8)
				{	
					min_scan_fg_dly[j] = 0;
					min_scan_cg_dly[j]++;
					if ((min_scan_cg_dly[j]*8+min_scan_fg_dly[j]) >= (avg_cg_dly[j]*8 + avg_fg_dly[j]))
					{
						if (j==0)
							adj_dly[0] = ((reg_with_dll>>20) &0x0F)*8 + ((reg_with_dll>>16) &0x0F);
						else
							adj_dly[1] = ((reg_with_dll>>28) &0x0F)*8 + ((reg_with_dll>>24) &0x0F);				
						break;
					}	
				}
		}while(1);		
	} /* dqs loop */
	{
		unsigned int tmp_cg_dly[2],tmp_fg_dly[2];
		for (j = 0; j < 2; j++)
		{
			if (adj_dly[j]==(avg_cg_dly[j]*8+avg_fg_dly[j]))
			{
				tmp_cg_dly[j] = avg_cg_dly[j];
				tmp_fg_dly[j] = avg_fg_dly[j];
			}
			else
			{
				tmp_cg_dly[j] = adj_dly[j]>>3;
				tmp_fg_dly[j] = adj_dly[j]&0x7;
			}
		}		
		reg_minscan = (tmp_cg_dly[1]<<28) | (tmp_fg_dly[1]<<24) | (tmp_cg_dly[0]<<20) | (tmp_fg_dly[0]<<16);
	}
}

#endif /* HWDLL_MINSCAN */

#if defined (HWDLL_LV)
	RALINK_REG(RALINK_RGCTRL_BASE+0x108) = 0x0f00;

	k=9583000;
	do {k--; }while(k>0);
#endif			

FINAL_SETUP:
#if (defined(HWDLL_AVG) && (!defined(HWDLL_MINSCAN)))
	RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = (reg_avg&0xFFFF0000)|((reg_with_dll>>16)&0x0FFFF);		
#elif defined(HWDLL_MINSCAN)
	if ((reg_minscan >> 24) > ((hw_dll_reg>>4)&0xFF))
	{
		reg_minscan &= 0xFFFFFF;
		reg_minscan |= (((hw_dll_reg>>4)&0xFF) << 24);
		//printf("overwrite %x\n", reg_minscan);
	}
	if (((reg_minscan >> 16) & 0xFF) > ((hw_dll_reg>>4)&0xFF))
	{
		reg_minscan &= 0xFF00FFFF;
		reg_minscan |= (((hw_dll_reg>>4)&0xFF) << 16);
		//printf("overwrite %x\n", reg_minscan);
	}
	RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = (reg_minscan&0xFFFF0000)|((reg_with_dll>>16)&0x0FFFF);		
#else	
	RALINK_REG(RALINK_MEMCTRL_BASE+0x64) = (reg_with_dll&0xFFFF0000)|(reg&0x0FFFF);
#endif		
	
	RALINK_REG(RALINK_MEMCTRL_BASE+0x68) |= (0x1<<4);
	//cal_debug("\n\r");
	//for (j = 0; j < 2; j++)	
			//cal_debug("[%02X%02X%02X%02X]", avg_min_cg_comp_dqs[j],avg_min_fg_comp_dqs[j], avg_max_cg_comp_dqs[j],avg_max_fg_comp_dqs[j]);

	//cal_debug("[%04X%02X%02X][%08X][00%04X%02X]\n", reg&0x0FFFF,\
	//					(g_min_coarse_dqs_dly[0]<<4)|g_min_coarse_dqs_dly[0], \
	//					(g_min_coarse_dqs_dly[1]<<4)|g_min_coarse_dqs_dly[1], \
	//					RALINK_REG(RALINK_MEMCTRL_BASE+0x64),
	//					(reg_avg&0xFFFF0000)>>16,
	//					(hw_dll_reg>>4)&0x0FF
	//					);
	//cal_debug("DU Setting Cal Done\n");
	RALINK_REG(RALINK_MEMCTRL_BASE+0x48) = ddr_cfg2_reg;
#if defined(MT7628_FPGA_BOARD) || defined(MT7628_ASIC_BOARD)	
	RALINK_REG(RALINK_MEMCTRL_BASE+0x10) |= (0x1<<4);
#else
	RALINK_REG(RALINK_MEMCTRL_BASE+0x18) |= (0x1<<4);
#endif

#endif	
}

EXIT:

	return ;
}
#endif /* #defined (CONFIG_DDR_CAL) */
