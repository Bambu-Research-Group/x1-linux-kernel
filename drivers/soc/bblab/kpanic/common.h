#ifndef __RK_DWMMC_COMMON__H
#define __RK_DWMMC_COMMON__H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/kernel.h>
#include <linux/bitops.h>

#include <linux/printk.h>
#include <linux/delay.h>

#include <asm/cacheflush.h>
#include <asm/div64.h>
#include <asm/delay.h>

#include <clocksource/arm_arch_timer.h>



typedef unsigned char		uchar;
typedef volatile unsigned long	vu_long;
typedef volatile unsigned short vu_short;
typedef volatile unsigned char	vu_char;



//#define  CONFIG_ARCH_ROCKCHIP
//#define CONFIG_SYS_CACHELINE_SIZE   64
//#define ARCH_DMA_MINALIGN	CONFIG_SYS_CACHELINE_SIZE


#define CONFIG_SPL_LIBCOMMON_SUPPORT

#define CONFIG_MMC_TINY     0
#define CONFIG_DM_MMC       0
#define CONFIG_BLK          0


//#define CONFIG_IS_ENABLED(option) defined(CONFIG_##option)
#define CONFIG_IS_ENABLED(option) (CONFIG_##option)

#define ROUND(a,b)		(((a) + (b) - 1) & ~((b) - 1))

#define LOG2(x) (((x & 0xaaaaaaaa) ? 1 : 0) + ((x & 0xcccccccc) ? 2 : 0) + \
		 ((x & 0xf0f0f0f0) ? 4 : 0) + ((x & 0xff00ff00) ? 8 : 0) + \
		 ((x & 0xffff0000) ? 16 : 0))
#define LOG2_INVALID(type) ((type)((sizeof(type)<<3)-1))


/* Wrapper for do_div(). Doesn't modify dividend and returns
 * the result, not reminder.
 */
static inline uint64_t lldiv(uint64_t dividend, uint32_t divisor)
{
	uint64_t __res = dividend;
	do_div(__res, divisor);
	return(__res);
}


#define debug(fmt, ...)			\
			pr_info(pr_fmt(fmt), ##__VA_ARGS__);	\


#define ASSERT(X)							\
do {									\
	if (unlikely(!(X))) {						\
		pr_err("\n");						\
		pr_err("Assertion failed\n");		\
		BUG();							\
	}								\
} while (0)

ulong get_timer(ulong base);


void *memalign(size_t align, size_t bytes);
void free(void *ptr);

#endif // __RK_DWMMC_COMMON__H
