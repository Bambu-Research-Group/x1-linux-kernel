#ifndef __BBL_KPANIC__H
#define __BBL_KPANIC__H

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/kmsg_dump.h>
#include <linux/pagemap.h>
#include <linux/zlib.h>
#include <linux/uaccess.h>

#include <linux/err.h>
#include <linux/version.h>
#include <linux/pstore.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/compiler.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <linux/blkdev.h>
#include <linux/file.h>
#include <linux/mount.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/notifier.h>
#include <linux/debugfs.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/preempt.h>
#include <linux/fb.h>
#include <linux/kmsg_dump.h>
#include <linux/file.h>


#define KPANIC_INVALID_OFFSET 0xFFFFFFFF

struct panic_header {
	u32 magic;
#define PANIC_MAGIC 0xdeadf00d

	u32 version;
#define PHDR_VERSION   0x01

	u64 minidump_offset;
	u64 minidump_length;

	u64 console_offset;
	u64 console_length;

	u64 threads_offset;
	u64 threads_length;
};

struct kpanic_context {
	struct panic_header curr;
	void *flash;
	u32 flash_block_len;

	void * bounce;
	void * kmsg_buffer;

	const char * blkdev;
	u64 start; // bytes
	u64 end;   // bytes
	u64 partition_start; //sector
	u64 partition_size;  //sectors

	void __iomem *emmc_base;
	void __iomem *cru_base;

	// proc control entry
	struct proc_dir_entry	*kpanic_minidump;
	struct proc_dir_entry	*kpanic_console;
	struct proc_dir_entry	*kpanic_threads;
	struct proc_dir_entry 	*kpanic_mmc;

};


#endif // __BBL_KPANIC__H
