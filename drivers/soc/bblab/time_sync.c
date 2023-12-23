#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/of_platform.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

struct time_sync_data
{
	struct device *dev;
	int gpio_key;
	int irq_gpio_key;
};

static irqreturn_t time_sync_irq_handler(int irq, void *dev_id);
static u64 trigger_time = 0;

static ssize_t time_sync_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s: %lld ns.\n", "last sync time is", trigger_time);
}

static ssize_t time_sync_trigger(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	time_sync_irq_handler(0, NULL);
	return count;
}

static DEVICE_ATTR(lastest_triggered_time, S_IRUSR | S_IWUSR, time_sync_get, time_sync_trigger);

static int time_sync_sysfs_register(struct device *dev)
{
	return device_create_file(dev, &dev_attr_lastest_triggered_time);
}

static irqreturn_t time_sync_irq_handler(int irq, void *dev_id)
{
	trigger_time = ktime_get_ns();

	return IRQ_HANDLED;
}

static int time_sync_open(struct inode *inode, struct file *file) {
	return 0;
}

static int time_sync_release(struct inode *inode, struct file *file) {
	return 0;
}

#define GET_LOCAL_TIME	_IOR('k', 0x80, int64_t)
static long time_sync_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret;

	switch (cmd) {
		case GET_LOCAL_TIME:
			ret = copy_to_user((void __user*)arg,
					&trigger_time, sizeof(u64)) ? -EFAULT : 0;
			break;
		default:
			return -EINVAL;
	}

	return ret;
}

static const struct file_operations time_sync_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.unlocked_ioctl	= time_sync_ioctl,
	.open		= time_sync_open,
	.release	= time_sync_release,
};

static struct miscdevice time_sync_dev = {
	.minor		= BBL_TIME_SYNC,
	.name		= "time_sync",
	.fops		= &time_sync_fops,
};

static int time_sync_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct time_sync_data *data = NULL;
	int gpio = 0;
	int irq = (-1);

	data = devm_kzalloc(&pdev->dev, sizeof(struct time_sync_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = &pdev->dev;
	dev_set_drvdata ( data->dev, data );

	gpio = of_get_named_gpio(pdev->dev.of_node, "time_sync_gpio", 0);
	if (gpio < 0)
		return -EINVAL;

	data->gpio_key = gpio;

	gpio_direction_input(data->gpio_key);
	irq = gpio_to_irq(data->gpio_key);
	data->irq_gpio_key = irq;
	ret = devm_request_any_context_irq(&pdev->dev, data->irq_gpio_key,
			time_sync_irq_handler, IRQF_TRIGGER_FALLING,
			dev_name(&pdev->dev), data);
	if (ret)
		return ret;

	ret = time_sync_sysfs_register(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "sysfs time_sync failed to register\n");
		return ret;
	}

	if (misc_register(&time_sync_dev)) {
		return -ENODEV;
	}

	return ret;
}

static int time_sync_remove(struct platform_device *dev)
{
	struct time_sync_data *data;

	data = dev_get_drvdata(&dev->dev);

	return 0;
}

static const struct of_device_id time_sync_of_match[] = {
	{ .compatible = "bblab,time_sync" },
	{},
};

MODULE_DEVICE_TABLE(of, time_sync_of_match);

static struct platform_driver time_sync_driver =
{
	.driver = {
		.name = "time_sync",
		.of_match_table = of_match_ptr(time_sync_of_match),
	},
	.probe = time_sync_probe,
	.remove = time_sync_remove,
};

module_platform_driver(time_sync_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Bblab time sync driver");
