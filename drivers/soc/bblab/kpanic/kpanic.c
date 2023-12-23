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
#include <linux/nmi.h>

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


#include "rockchip_dw_mmc.h"
#include "mmc.h"
#include "blk.h"

#include "kpanic.h"



static struct kpanic_context oops_ctx;

static struct work_struct proc_removal_work;
static DEFINE_MUTEX(drv_mutex);

static int in_panic = 0;



static int kpanic_mmc_init(void)
{
	struct kpanic_context *ctx = &oops_ctx;
	struct mmc *mmc;

	printk("kpanic mmc_init ...\n");

	mmc_initialize();
	rockchip_dwmmc_probe(ctx);
	init_mmc_device(0, true);

	mmc = find_mmc_device(0);
	if (!mmc) {
		printk("no mmc device at slot 0\n");
		return -1;
	}

	if(0 == mmc->read_bl_len){
		printk("mmc->read_bl_len == 0\n");
		return -1;
	}

	// block size in blocks
	if(0 == mmc->block_dev.lba){
		printk("mmc->block_dev.lba == 0\n");
		mmc->block_dev.lba = 0xEE000;
		return -1;
	}

	ctx->flash = mmc;
	ctx->flash_block_len = mmc->write_bl_len;

	printk(KERN_INFO"kpanic start mmc address is %llu to %llu\n", ctx->start, ctx->end);

	return 0;
}

static void kpanic_eraseflash(void)
{
	struct kpanic_context *ctx = &oops_ctx;
	struct file *file = NULL;
	unsigned int len;
	loff_t offset = 0;

	file = filp_open(ctx->blkdev, O_RDWR, 0);
	if (IS_ERR_OR_NULL(file)) {
		printk("%s open panic file failed error =%ld .\n", __func__, PTR_ERR(file));
	} else {
		memset(ctx->bounce, 0, 512);
		len = kernel_write(file, ctx->bounce, 512, &offset); // just clear the panic header
		filp_close(file, NULL);
	}
}

static int kpanic_writeflashpage(loff_t to,
				 const u_char *buf)
{
	struct kpanic_context *ctx = &oops_ctx;
	struct mmc *mmc = (struct mmc *)ctx->flash;
	int rc;
	unsigned long wlen;
	//lbaint_t blkstart, blkcnt;

	if (!ctx->start) {
		printk(KERN_EMERG"kpanic could not get mmc start address\n");
		return 0;
	}

	wlen = to + ctx->start;
	if (wlen > ctx->end)
		return 0;

	wlen = wlen >> 9;

	if (to == KPANIC_INVALID_OFFSET) {
		printk(KERN_EMERG "kpanic: write to invalid address\n");
		return 0;
	}

	/*rc = blk_dwrite(mmc_get_blk_desc(mmc), blkstart, blkcnt, buf);*/
	rc = mmc->block_dev.block_write(mmc_get_blk_desc(mmc), wlen, 1, buf);
	if (rc != 1) {
		printk(KERN_EMERG
		       "%s: Error writing data to mmc (%d)\n",
		       __func__, rc);
		return 0;
	}

	return mmc->write_bl_len;
}

static int kpanic_readflashpage(off_t offset, char *buffer,
				 int count)
{
	struct file *file = NULL;
	struct kpanic_context *ctx = &oops_ctx;
	size_t len;
	loff_t pos = offset;

	file = filp_open(ctx->blkdev, O_RDONLY, 0);
	if (IS_ERR_OR_NULL(file)) {
		pr_err("%s open panic file failed error = %ld .\n", __func__, PTR_ERR(file));
		return -EINVAL;
	} else {
		len = kernel_read(file, buffer, count, &pos);
		filp_close(file, NULL);
	}

	return len;
}


/*
 * Writes the contents of the console to the specified offset in flash.
 * Returns number of bytes written
 */
static int kpanic_write_console(unsigned int off)
{
	struct kpanic_context *ctx = &oops_ctx;
	int saved_oip;
	int idx = 0;
	int rc, rc2, end;
	unsigned int last_chunk = 0;
	unsigned int remain = 0, dump_len;
	size_t len = 0;
	struct kmsg_dumper kpanic_dumper;

	kmsg_dump_rewind_nolock(&kpanic_dumper);
	kpanic_dumper.active = true;

	while (!last_chunk) {
		saved_oip = oops_in_progress;
		oops_in_progress = 1;

		dump_len = 0;
		end = 0;

		if(remain < ctx->flash_block_len){
			while (kmsg_dump_get_line(&kpanic_dumper, false,
				ctx->kmsg_buffer + dump_len + remain,
				PAGE_SIZE - dump_len - remain, &len)) {
				end = 1;
				dump_len += len;
				if((dump_len + remain) >= ctx->flash_block_len){
					break;
				}
			}
		} else {
			end = 1;
		}

		if (!end)
			break;

		if(dump_len + remain >= ctx->flash_block_len){
			memcpy(ctx->bounce, ctx->kmsg_buffer, ctx->flash_block_len);
			remain = dump_len + remain - ctx->flash_block_len;
			memcpy(ctx->kmsg_buffer, ctx->kmsg_buffer + ctx->flash_block_len, remain);
			rc = ctx->flash_block_len;
		} else {
			memcpy(ctx->bounce, ctx->kmsg_buffer, dump_len + remain);
			rc = dump_len + remain;
			remain = 0;
		}

		if (rc != ctx->flash_block_len)
			last_chunk = rc;

		oops_in_progress = saved_oip;
		if (rc <= 0)
			break;
		if (rc != ctx->flash_block_len)
			memset(ctx->bounce + rc, 0, ctx->flash_block_len - rc);

		rc2 = kpanic_writeflashpage(off, ctx->bounce);
		if (rc2 <= 0) {
			printk(KERN_EMERG
			       "kpanic: mmc write failed (%d)\n", rc2);
			return idx;
		}
		if (!last_chunk)
			idx += rc2;
		else
			idx += last_chunk;
		off += rc2;
	}
	return idx;
}

static int kpanic(struct notifier_block *this, unsigned long event,
			void *ptr)
{
	struct kpanic_context *ctx = &oops_ctx;
	struct panic_header *hdr = (struct panic_header *) ctx->bounce;
	int minidump_offset = 0;
	int minidump_len = 0;
	int console_offset = 0;
	int console_len = 0;
#if 0
	int threads_offset = 0;
	int threads_len = 0;
#endif
	int rc;

	if (in_panic)
		return NOTIFY_DONE;
	in_panic = 1;
#ifdef CONFIG_PREEMPT
	preempt_count_inc();
#endif

	touch_softlockup_watchdog();
	//touch_watchdog();

	kpanic_mmc_init();

	if (!ctx->flash)
		goto out;

	if (ctx->curr.magic) {
		printk(KERN_EMERG "Crash partition in use!\n");
		goto out;
	}

	minidump_offset = ctx->flash_block_len;

#if 0
	/*
	 * Write out the minidump
	 */
	minidump_len = kpanic_write_minidump(minidump_offset);
	if (minidump_len < 0) {
		printk(KERN_EMERG "Error writing minidump to panic log! (%d)\n",
		       minidump_len);
		minidump_len = 0;
	}
#endif

	/*
	 * Write out the console
	 */
	console_offset = ALIGN(minidump_offset + minidump_len, ctx->flash_block_len);
	console_len = kpanic_write_console(console_offset);
	if (console_len < 0) {
		printk(KERN_EMERG "Error writing console to panic log! (%d)\n",
		       console_len);
		console_len = 0;
	}

	/*
	 * Write out all threads
	 */
#if 0
	threads_offset = ALIGN(console_offset + console_len, ctx->flash_block_len);
	if (!threads_offset)
		threads_offset = ctx->flash_block_len;

	syslog_print_all(NULL, 0, true);
	show_state_filter(0);
	threads_len = kpanic_write_console(threads_offset);
	if (threads_len < 0) {
		printk(KERN_EMERG "Error writing threads to panic log! (%d)\n",
		       threads_len);
		threads_len = 0;
	}
#endif

	/*
	 * Finally write the panic header
	 */
	memset(ctx->bounce, 0, PAGE_SIZE);
	hdr->magic = PANIC_MAGIC;
	hdr->version = PHDR_VERSION;

#if 0
	hdr->minidump_offset = minidump_offset;
	hdr->minidump_length = minidump_len;
#endif

	hdr->console_offset = console_offset;
	hdr->console_length = console_len;
#if 0
	hdr->threads_offset = threads_offset;
	hdr->threads_length = threads_len;
#endif

	rc = kpanic_writeflashpage(0, ctx->bounce);
	if (rc <= 0) {
		printk(KERN_EMERG "kpanic: Header write failed (%d)\n",
		       rc);
		goto out;
	}

	printk(KERN_EMERG "kpanic: Panic dump sucessfully written to flash\n");

out:

#ifdef CONFIG_PREEMPT
	preempt_count_dec();
#endif

	in_panic = 0;
	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call	= kpanic,
};

static int panic_dbg_get(void *data, u64 *val)
{
	kpanic(NULL, 0, NULL);
	return 0;
}

static int panic_dbg_set(void *data, u64 val)
{
	BUG();
	return -1;
}

DEFINE_SIMPLE_ATTRIBUTE(panic_dbg_fops, panic_dbg_get, panic_dbg_set, "%llu\n");



static ssize_t kpanic_proc_read(struct file *file, char __user *buffer,
                        size_t count, loff_t *ppos)
{
	struct kpanic_context *ctx = &oops_ctx;
	void *dat = PDE_DATA(file_inode(file));
	size_t file_length;
	off_t file_offset;
	char * buff_temp = NULL;
	size_t ret = -EFAULT;
	int len = -EINVAL;
	int cnt;

	buff_temp = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if(!buff_temp)
		goto out_free;

	if (!count)
		goto out_free;

	mutex_lock(&drv_mutex);

	switch ((long) dat) {
	case 0:	/* kpanic_minidump */
		file_length = ctx->curr.minidump_length;
		file_offset = ctx->curr.minidump_offset;
		break;
	case 1:	/* kpanic_console */
		file_length = ctx->curr.console_length;
		file_offset = ctx->curr.console_offset;
		break;
#if 0
	case 2:	/* kpanic_threads */
		file_length = ctx->curr.threads_length;
		file_offset = ctx->curr.threads_offset;
		break;
#endif
	default:
		pr_err("Bad dat (%ld)\n", (long) dat);
		len = -EINVAL;
		goto out;
	}

	if((int)*ppos >= file_length) {
		len = 0;
		goto out;
	}

	cnt = min(file_length - (size_t)*ppos, min((size_t)count, (size_t)PAGE_SIZE));
	len = kpanic_readflashpage(file_offset + (int)*ppos, buff_temp, cnt);
	if (len < 0) {
		len = -EIO;
		goto out;
	}

	ret = copy_to_user(buffer, buff_temp, len);
	len = len - ret;
	*ppos += len;

out:
	mutex_unlock(&drv_mutex);
out_free:
	kfree(buff_temp);

	return len;
}

static void kpanic_remove_proc_work(struct work_struct *work)
{
	struct kpanic_context *ctx = &oops_ctx;

	mutex_lock(&drv_mutex);
	kpanic_eraseflash();
	memset(&ctx->curr, 0, sizeof(struct panic_header));
	if (ctx->kpanic_minidump) {
		remove_proc_entry("kpanic_minidump", NULL);
		ctx->kpanic_minidump = NULL;
	}
	if (ctx->kpanic_console) {
		remove_proc_entry("kpanic_console", NULL);
		ctx->kpanic_console = NULL;
	}
	if (ctx->kpanic_threads) {
		remove_proc_entry("kpanic_threads", NULL);
		ctx->kpanic_threads = NULL;
	}
	mutex_unlock(&drv_mutex);
}

static ssize_t kpanic_proc_write(struct file *file, const char __user *buffer,
                        size_t count, loff_t *ppos)
{
	schedule_work(&proc_removal_work);
	return count;
}


static const struct file_operations kpanic_console_operations = {
        .owner = THIS_MODULE,
        .read  = kpanic_proc_read,
        .write = kpanic_proc_write,
};

static void kpanic_proc_init(void)
{
	struct kpanic_context *ctx = &oops_ctx;
	struct panic_header *hdr = ctx->bounce;
	int proc_entry_created = 0;

	if (hdr->magic == 0) {
		printk(KERN_INFO "kpanic: magic is zero \n");
		return;
	}

	if (hdr->magic != PANIC_MAGIC) {
		printk(KERN_INFO "kpanic: No panic data available\n");
		kpanic_eraseflash();
		return;
	}

	if (hdr->version != PHDR_VERSION) {
		printk(KERN_INFO "kpanic: Version mismatch (%d != %d)\n",
		       hdr->version, PHDR_VERSION);
		kpanic_eraseflash();
		return;
	}

	memcpy(&ctx->curr, hdr, sizeof(struct panic_header));

	printk(KERN_INFO "kpanic: minidump(%llx, %llx) console(%llx, %llx) threads(%llx, %llx)\n",
		hdr->minidump_offset,hdr->minidump_length,
	       hdr->console_offset, hdr->console_length,
	       hdr->threads_offset, hdr->threads_length);

	if (hdr->minidump_length) {
		ctx->kpanic_minidump = proc_create_data("kpanic_minidump", S_IFREG | S_IRUGO, NULL,
			&kpanic_console_operations, (void *)0);

		if (!ctx->kpanic_minidump)
			printk(KERN_ERR "%s: failed creating procfile\n",
			       __func__);
		else {
			proc_set_size(ctx->kpanic_minidump, hdr->minidump_length);
			proc_entry_created = 1;
		}
	}

	if (hdr->console_length) {
		ctx->kpanic_console = proc_create_data("kpanic_console", S_IFREG | S_IRUGO, NULL,
			&kpanic_console_operations, (void *)1);
		if (!ctx->kpanic_console)
			printk(KERN_ERR "%s: failed creating procfile\n",
			       __func__);
		else {
			proc_set_size(ctx->kpanic_console, hdr->console_length);
			proc_entry_created = 1;
		}
	}
#if 0
	if (hdr->threads_length) {
		ctx->kpanic_threads = proc_create_data("kpanic_threads", S_IFREG | S_IRUGO, NULL,
			&kpanic_console_operations, (void *)2);

		if (!ctx->kpanic_threads)
			printk(KERN_ERR "%s: failed creating procfile\n",
			       __func__);
		else {
			proc_set_size(ctx->kpanic_threads, hdr->threads_length);
			proc_entry_created = 1;
		}
	}
#endif

	if (!proc_entry_created)
		kpanic_eraseflash();
}


static ssize_t kpanic_mmc_proc_write(struct file *file, const char __user *buffer,
				size_t count, loff_t *ppos)
{
	struct kpanic_context *ctx = &oops_ctx;
	struct file *kpanic_file = NULL;
	size_t len = 0;
	loff_t offset = 0;

	kpanic_file = filp_open(ctx->blkdev, O_RDONLY, 0);
	if (IS_ERR_OR_NULL(kpanic_file)) {
		printk("%s open panic file failed error =%ld .\n", __func__, PTR_ERR(kpanic_file));
		return PTR_ERR(kpanic_file);
	} else {
		len = kernel_read(kpanic_file, ctx->bounce, 512, &offset);
		filp_close(kpanic_file, NULL);
	}

	if (len != 512) {
		printk(KERN_WARNING
		       "kpanic: %s read hdr error\n", __func__);
	}

	kpanic_proc_init();

	return count;
}

static const struct file_operations kpanic_mmc_operations = {
        .owner = THIS_MODULE,
        .read  = NULL,
        .write = kpanic_mmc_proc_write,
};

static void kpanic_mmc_create_proc(void)
{
	struct kpanic_context *ctx = &oops_ctx;

	ctx->kpanic_mmc = proc_create_data("kpanic_mmc", S_IFREG | S_IRUGO, NULL,
		&kpanic_mmc_operations, (void *)1);

	if (!ctx->kpanic_mmc)
			printk(KERN_ERR "%s: failed creating procfile\n",
			       __func__);
	else
		proc_set_size(ctx->kpanic_mmc, 1);
}

static int kpanic_parse_dt_size(struct platform_device *pdev,
				 const char *propname, u32 *value)
{
	u32 val32 = 0;
	int ret;

	ret = of_property_read_u32(pdev->dev.of_node, propname, &val32);
	if (ret < 0 && ret != -EINVAL) {
		dev_err(&pdev->dev, "failed to parse property %s: %d\n",
			propname, ret);
		return ret;
	}

	if (val32 > INT_MAX) {
		dev_err(&pdev->dev, "%s %u > INT_MAX\n", propname, val32);
		return -EOVERFLOW;
	}

	*value = val32;
	return 0;
}

static int kpanic_parse_dt(struct platform_device *pdev)
{
	struct device_node *of_node = pdev->dev.of_node;
	struct resource *res;
	struct kpanic_context *ctx = dev_get_drvdata(&pdev->dev);
	u32 value;
	int ret;

	dev_info(&pdev->dev, "using Device Tree\n");

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "emmc");
	if (!res) {
		dev_err(&pdev->dev,
			"failed to locate emmc resource\n");
		return -EINVAL;
	}
	ctx->emmc_base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR_OR_NULL(ctx->emmc_base))
		return PTR_ERR(ctx->emmc_base);

	dev_info(&pdev->dev, "emmc base 0x%x\n", (u32)ctx->emmc_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cru");
	if (!res) {
		dev_err(&pdev->dev,
			"failed to locate cru resource\n");
		return -EINVAL;
	}
	ctx->cru_base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR_OR_NULL(ctx->cru_base))
		return PTR_ERR(ctx->cru_base);

	dev_info(&pdev->dev, "cru base 0x%x\n", (u32)ctx->cru_base);

#define parse_size(name, field) {					\
		ret = kpanic_parse_dt_size(pdev, name, &value);	\
		if (ret < 0)						\
			return ret;					\
		field = value;						\
	}

	parse_size("partition-start", ctx->partition_start);
	parse_size("partition-size", ctx->partition_size);

#undef parse_size

	ctx->start = ctx->partition_start << 9;
	ctx->end = (ctx->partition_start + ctx->partition_size) << 9;
	ctx->end -= 1;

	if (of_property_read_string(of_node, "partition-name", &ctx->blkdev)) {
		pr_err("kpanic read blkdev faile\n");
		return -ENODATA;
	}

	return 0;
}


static int kpanic_probe(struct platform_device *pdev)
{
	struct kpanic_context *ctx = &oops_ctx;
	int err = -EINVAL;

	platform_set_drvdata(pdev, ctx);

	err = kpanic_parse_dt(pdev);
	if (err < 0)
		goto fail_out;

	kpanic_mmc_create_proc();

	atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);

	debugfs_create_file("kpanic", 0644, NULL, NULL, &panic_dbg_fops);

	ctx->bounce = (void *) __get_free_page(GFP_KERNEL);
	oops_ctx.kmsg_buffer = (void *) __get_free_page(GFP_KERNEL);
	INIT_WORK(&proc_removal_work, kpanic_remove_proc_work);

	printk(KERN_INFO "kernel panic handler initialized\n");

	return 0;

fail_out:
	return err;
}

static int kpanic_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id dt_match[] = {
	{ .compatible = "kpanic" },
	{}
};

static struct platform_driver kpanic_driver = {
	.probe		= kpanic_probe,
	.remove		= kpanic_remove,
	.driver		= {
		.name		= "kpanic",
		.of_match_table	= dt_match,
	},
};

static int __init kpanic_init(void)
{
	int ret;

	ret = platform_driver_register(&kpanic_driver);
	if (ret != 0)
        pr_err("register kpanic driver failed, ret=%d\n", ret);
    else
        pr_info("register kpanic driver ok\n");

	return ret;
}
postcore_initcall(kpanic_init);

static void __exit kpanic_exit(void)
{
	platform_driver_unregister(&kpanic_driver);
}
module_exit(kpanic_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BLK Oops/Panic logger/driver");
