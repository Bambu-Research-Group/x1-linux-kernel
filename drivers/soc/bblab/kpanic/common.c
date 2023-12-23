#include <linux/printk.h>
#include "common.h"


ulong get_timer(ulong base)
{
	u64 count;
	u32 rate;

	count = arch_timer_read_counter();
	rate = arch_timer_get_rate();

	do_div(count, rate / 1000);

	return count - base; // ms
}

void *memalign(size_t align, size_t bytes)
{
    ASSERT(0);
    return NULL;
}

void free(void *ptr)
{
    ASSERT(0);
}


