#include <linux/types.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/pagemap.h>
#include <linux/zlib.h>
#include <linux/uaccess.h>
#include <linux/err.h>
#include <linux/version.h>
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
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <linux/preempt.h>
#include <linux/fb.h>
#include <linux/file.h>
#include <linux/sysfs.h>
#include <linux/iotrace.h>

#define IOTRACE_MAGIC (0xdeadb12f)
#define ONE_RECORD_MSG_LEN (26)
#define ONE_RECORD_LEN (sizeof(struct iotrace_record))
#define IOTRACE_ALIGN(x) ((x) - ((x) % ONE_RECORD_LEN))
#define IOTRACE_DEBUG (1)

struct iotrace_record {
	enum iotrace_flags flags;
	uint32_t addr;
	uint32_t value;
};

struct iotrace_header {
	uint32_t magic;
	uint32_t start;
	uint32_t end;
	uint32_t offset[4];
} iotrace_header;

struct iotrace_context {
	struct iotrace_header *header;
	struct iotrace_header init_header;
	struct proc_dir_entry *iotrace_dump_proc;
	uint32_t record_msg_length;
	uint32_t dumped_cnt;
	uint32_t part_total_cnt;
	uint32_t part_base[4];
	bool enable;
};

static struct iotrace_context gctx = {
	.enable = false,
};

static DEFINE_MUTEX(drv_mutex);

#if 0
static uint32_t invalid_cnt = 0;
static bool is_record_valid(uint32_t addr) {
	struct iotrace_record* temp = (struct iotrace_record*)addr;

	if (temp->addr == 0) {
		invalid_cnt++;
		return false;
	} else {
		return true;
	}
}
#endif

static size_t load_to_buffer(char* buffer, uint32_t addr) {
	struct iotrace_record* temp = (struct iotrace_record*)addr;
	char *rw;
	char *bit_width;

	rw = (temp->flags >> 3) & 0x00000001 ? "w" : "r";
	if ((temp->flags & 0x00000003) == IOT_8) {
		bit_width = "08";
	} else if ((temp->flags & 0x00000003) == IOT_16) {
		bit_width = "16";
	} else if ((temp->flags & 0x00000003) == IOT_32) {
		bit_width = "32";
	} else {
		bit_width = "NA";
	}

	return sprintf(buffer, "%s%s_0x%08x_0x%08x\n", \
			rw, bit_width,temp->addr, temp->value);
}

static ssize_t iotrace_dump(struct file *file, char __user *buffer,
		size_t count, loff_t *ppos)
{
	size_t ret = -EFAULT;
	struct iotrace_context *ctx = &gctx;
	struct iotrace_header* header = ctx->header;
	char* buff_temp = NULL;
	unsigned int addr = 0;
	int len = 0;
	int cnt = 0;

	buff_temp = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buff_temp) {
		pr_err("iotrace buffer kmalloc failed.\n");
		goto out_free;
	}

	mutex_lock(&drv_mutex);

	if((int)*ppos >= ctx->record_msg_length) {
		len = 0;
		goto out;
	}

	cnt = min(ctx->record_msg_length - (size_t)*ppos, min((size_t)count, (size_t)PAGE_SIZE));

	for (addr = header->start + ctx->dumped_cnt * ONE_RECORD_LEN; \
			addr < header->end; addr += ONE_RECORD_LEN) {
		if (PAGE_SIZE - len < ONE_RECORD_MSG_LEN)
			break;

		ctx->dumped_cnt++;

#if 0
		if (!is_record_valid(addr))
			continue;
#endif

		len += load_to_buffer(buff_temp + len, addr);
	}

	ret = copy_to_user(buffer, buff_temp, len);
	len = len - ret;
	*ppos += len;

	if (ctx->dumped_cnt >= ctx->part_total_cnt * 4) {
		pr_err("iotrace dump done, dumped_cnt: %d.\n", ctx->dumped_cnt);
		ctx->dumped_cnt = 0;
		//invalid_cnt = 0;
	}

out:
	mutex_unlock(&drv_mutex);
out_free:
	kfree(buff_temp);

	return len;
}

static const struct file_operations iotrace_dump_operations = {
        .owner = THIS_MODULE,
        .read  = iotrace_dump,
        .write = NULL,
};

static ssize_t iotrace_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iotrace_context* ctx = &gctx;

	return sprintf(buf, "%d\n", ctx->enable);
}

static ssize_t iotrace_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct iotrace_context* ctx = &gctx;
	uint32_t val = 0;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret || val > 1)
		return -EINVAL;

	ctx->enable = val;

	if (ctx->enable) {
		ctx->record_msg_length = 0;
		ctx->dumped_cnt = 0;
		memset((void*)(ctx->init_header.start), 0, ctx->init_header.end - ctx->init_header.start);
		memcpy(ctx->header, &ctx->init_header, sizeof(iotrace_header));
	}

	return count;
}

static DEVICE_ATTR(iotrace_enable, S_IRUSR | S_IWUSR, iotrace_enable_show, iotrace_enable_store);

#if IOTRACE_DEBUG
static ssize_t iotrace_test_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	uint32_t val = 0;
	void* __iomem vaddr;
	int ret;

	ret = kstrtouint(buf, 16, &val);
	if (ret) {
		dev_err(dev, "value is invalid\n");
		return -EINVAL;
	}

	vaddr = memremap(val, 0x16, MEMREMAP_WB);
	if (IS_ERR_OR_NULL(vaddr)) {
		dev_err(dev, "mmap failed\n");
		return PTR_ERR(vaddr);
	}

	dev_info(dev, "paddr %#x, vaddr %#x, value %#x\n", \
			val, (uint32_t)vaddr, readl(vaddr));

	return count;
}

static DEVICE_ATTR(iotrace_test, S_IWUSR, NULL, iotrace_test_store);
#endif

static ssize_t iotrace_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iotrace_context* ctx = &gctx;

	return sprintf(buf, "cpu0_offset: %d, " \
			"cpu1_offset: %d, " \
			"cpu2_offset: %d, " \
			"cpu3_offset: %d\n", \
			ctx->header->offset[0] % ctx->part_total_cnt, \
			ctx->header->offset[1] % ctx->part_total_cnt + ctx->part_total_cnt, \
			ctx->header->offset[2] % ctx->part_total_cnt + ctx->part_total_cnt * 2, \
			ctx->header->offset[3] % ctx->part_total_cnt + ctx->part_total_cnt * 3);
}

static DEVICE_ATTR(iotrace_offset, S_IRUSR, iotrace_offset_show, NULL);

static int iotrace_sysfs_register(struct device *dev)
{
	int ret = 0;

	ret = device_create_file(dev, &dev_attr_iotrace_enable);
	if (ret) {
		dev_err(dev, "create iotrace enable attr error\n");
		goto err;
	}
#if IOTRACE_DEBUG
	ret = device_create_file(dev, &dev_attr_iotrace_test);
	if (ret) {
		dev_err(dev, "create iotrace test attr error\n");
		goto err;
	}
#endif

	ret = device_create_file(dev, &dev_attr_iotrace_offset);
	if (ret) {
		dev_err(dev, "create iotrace offset attr error\n");
		goto err;
	}

err:
	return ret;
}

void iotrace_disable(void) {
	struct iotrace_context* ctx = &gctx;
	ctx->enable = false;
}
EXPORT_SYMBOL(iotrace_disable);

static int iotrace_parse_dt(struct platform_device *pdev)
{
	struct device_node *mem;
	struct resource res;
	void* __iomem start;
	int err, i;

	dev_info(&pdev->dev, "using Device Tree\n");

	mem = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!mem) {
		dev_err(&pdev->dev, "not find \"memory-region\" property\n");
		return -EINVAL;
	}

	err = of_address_to_resource(mem, 0, &res);
	if (err < 0) {
		dev_err(&pdev->dev,
				"failed to parse memory resource\n");
		return -EINVAL;
	}

	start = ioremap_nocache(res.start, resource_size(&res));
	if (IS_ERR_OR_NULL(start))
		return PTR_ERR(start);

	gctx.header = (struct iotrace_header*)start;
	gctx.init_header.magic = IOTRACE_MAGIC;
	gctx.init_header.start = (uint32_t)start + sizeof(iotrace_header);
	gctx.part_total_cnt = (IOTRACE_ALIGN(resource_size(&res) - sizeof(iotrace_header))) / ONE_RECORD_LEN / 4;
	gctx.init_header.end = gctx.init_header.start + gctx.part_total_cnt * ONE_RECORD_LEN * 4;

	for (i = 0; i < 4; i++) {
		gctx.init_header.offset[i] = 0;
		gctx.part_base[i] = gctx.init_header.start + i * gctx.part_total_cnt * ONE_RECORD_LEN;
		dev_info(&pdev->dev, "part_base[%d]: %#x, offset[%d]: %#x\n", \
				i, gctx.part_base[i], i , gctx.init_header.offset[i]);
	}

	dev_info(&pdev->dev, "reserve start phy: %#x, size: %#x, \n" \
            "iotrace buffer start vaddr: %#x, buffer end vaddr: %#x, buffer size %#x, total cnt is %d ,part_cnt is %d", \
            res.start, resource_size(&res), gctx.init_header.start, \
			gctx.init_header.end, gctx.init_header.end - gctx.init_header.start, \
			gctx.part_total_cnt * 4, gctx.part_total_cnt);

	return 0;
}

static uint32_t iotrace_v2p(uint32_t vaddr) {
	uint32_t paddr = 0, offset = 0;
	pgd_t *pgdp;
	pud_t *pudp;
	pmd_t *pmdp;
	pte_t pte;

	pgdp = pgd_offset_k(vaddr);
	if (unlikely(pgd_none(*pgdp))) {
		pr_err("iotrace pgdp is none, vaddr is %#x\n", vaddr);
		goto err;
	}

	pudp = pud_offset(pgdp, vaddr);
	if (unlikely(pud_none(*pudp))) {
		pr_err("iotrace pudp is none, vaddr is %#x\n", vaddr);
		goto err;
	}

	pmdp = pmd_offset(pudp, vaddr);
	if (unlikely(pmd_none(*pmdp))) {
		pr_err("iotrace pmdp is none, vaddr is %#x\n", vaddr);
		goto err;
	}

	pte = *pte_offset_kernel(pmdp, vaddr);
	if (unlikely(!pte_present(pte))) {
		pr_err("iotrace pte is not present, vaddr is %#x\n", vaddr);
		goto err;
	}

	paddr = (uint32_t)pte_pfn(pte) << PAGE_SHIFT;
	offset = vaddr & ~PAGE_MASK;

err:
	return paddr | offset;
}

void iotrace_add_record(int flags, volatile const void *ptr, uint32_t value)
{
	struct iotrace_record *rec;
	struct iotrace_context *ctx = &gctx;
	struct iotrace_header *header = ctx->header;
	int cpu;
	uint32_t offset;
	uint32_t phy_addr;

	if (unlikely(!ctx->enable))
		return;

	phy_addr = (uint32_t)iotrace_v2p((uint32_t)ptr);
#if 0
	if (((phy_addr & 0xffff0000) == 0xfeff0000) || \
			((phy_addr & 0xfff00000) == 0xffe00000) || \
			((phy_addr & 0xffff0000) == 0xff570000) || \
			((phy_addr & 0xffff0000) == 0xffb00000) || \
			((phy_addr & 0xffff0000) == 0xffb30000))
        return;
#endif

	cpu = smp_processor_id();
	offset = header->offset[cpu]++ % ctx->part_total_cnt;
	rec = (struct iotrace_record*)(ctx->part_base[cpu] + offset * ONE_RECORD_LEN);
	rec->flags = flags;
	rec->addr = phy_addr;
	rec->value = value;
}

static int current_records(struct platform_device *pdev, struct iotrace_context *ctx) {
	struct iotrace_header *header;
	int count = 0;
	uint32_t addr;

	if (!ctx) {
		dev_err(&pdev->dev, "context ptr is null\n");
		return 0;
	}

	header = ctx->header;

	for (addr = header->start; addr < header->end; addr += ONE_RECORD_LEN) {
		count++;
	}

	return count;
}

static int iotrace_probe(struct platform_device *pdev)
{
	struct iotrace_context *ctx = &gctx;
	int err = -EINVAL;
	int count = 0;

	err = iotrace_parse_dt(pdev);
	if (err < 0)
		goto fail_out;

	err = iotrace_sysfs_register(&pdev->dev);
	if (err) {
		dev_err(&pdev->dev, "sysfs failed to register\n");
		goto fail_out;
	}

	if (ctx->header->magic != IOTRACE_MAGIC) {
		dev_info(&pdev->dev, "no magic, do not has records\n");
		return 0;
	}

	ctx->iotrace_dump_proc = proc_create_data("iotrace_dump", S_IFREG | S_IRUGO, NULL,
			&iotrace_dump_operations, (void *)0);

	if (!ctx->iotrace_dump_proc)
		dev_err(&pdev->dev, "failed creating proc file\n");
	else {
		count = current_records(pdev, ctx);
		ctx->record_msg_length = count * ONE_RECORD_MSG_LEN;
		ctx->dumped_cnt = 0;
		dev_info(&pdev->dev, "store %d Bytes message now, records cnt %d\n", \
				ctx->record_msg_length, count);
		if (ctx->record_msg_length)
			proc_set_size(ctx->iotrace_dump_proc, ctx->record_msg_length);
	}

	dev_info(&pdev->dev, "handler initialized\n");

	return 0;

fail_out:
	return err;
}

static int iotrace_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id dt_match[] = {
	{ .compatible = "iotrace" },
	{}
};

static struct platform_driver iotrace_driver = {
	.probe		= iotrace_probe,
	.remove		= iotrace_remove,
	.driver		= {
		.name		= "iotrace",
		.of_match_table	= dt_match,
	},
};

static int __init iotrace_init(void)
{
	int ret;

	ret = platform_driver_register(&iotrace_driver);
	if (ret != 0)
		pr_err("register iotrace driver failed, ret=%d\n", ret);
	else
		pr_info("register iotrace driver ok\n");

	return ret;
}
postcore_initcall(iotrace_init);

static void __exit iotrace_exit(void)
{
	platform_driver_unregister(&iotrace_driver);
}
module_exit(iotrace_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BBL IOTRACE DRIVER");
