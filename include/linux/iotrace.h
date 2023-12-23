#ifndef _IOTRACE_H_
#define _IOTRACE_H_

#include <asm/atomic.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>

enum iotrace_flags {
	IOT_8 = 0,
	IOT_16,
	IOT_32,

	IOT_READ = 0 << 3,
	IOT_WRITE = 1 << 3,
};

__attribute__ ((__used__)) void iotrace_add_record(int flags, \
		volatile const void *ptr, u32 value);
#endif /* _IOTRACE_H_ */
