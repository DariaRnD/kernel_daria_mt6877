#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>
#include <linux/semaphore.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>
#include <linux/input.h>
#if defined(CONFIG_PM_WAKELOCKS)
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h>
#endif
#include <linux/time.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#define COMMON_NODE_DEVNAME    "common_node_dev"
enum node_idx{
	GESTURE,
//	SUPERTORCH,
/*
  new item ,add here
*/
	MAX
};
static char *func_node_name[] ={
	[GESTURE]  = "GESTURE" ,
//	[SUPERTORCH]  = "Supertorch" ,
/*
  new item ,add here
*/
};
struct node_dev_info{
	char  name[10];
	unsigned char state;
	void(*set)(unsigned char on_off);	
};

struct common_node{
	struct node_dev_info node_array[MAX];
};

static struct common_node* local_common_node;
/*************************************************************************************************************************************************/
/*
  new item ,add here
*/
/*
static ssize_t supertorch_show(struct device *dev,struct device_attribute*attr, char *buf)
{
    int count;
    count = sprintf(buf, "Node Func: %s, state = %d\n",local_common_node->node_array[SUPERTORCH].name,local_common_node->node_array[SUPERTORCH].state);
    return count;
}
 
static ssize_t supertorch_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    int error;
	unsigned int temp;
    error = kstrtouint(buf, 10, &temp);
	if(error < 0)
		goto err;
	if(local_common_node->node_array[SUPERTORCH].set){
		local_common_node->node_array[SUPERTORCH].state = temp;
		local_common_node->node_array[SUPERTORCH].set(temp);
	}else
		printk("node func %s is null!! \r\n",local_common_node->node_array[SUPERTORCH].name);
err:
	return count;
}
static DEVICE_ATTR(supertorch, S_IRUGO|S_IWUSR,supertorch_show, supertorch_store);
*/
static ssize_t gesture_show(struct device *dev,struct device_attribute*attr, char *buf)
{
    int count;
    count = sprintf(buf, "Node Func: %s, state = %s\n",local_common_node->node_array[GESTURE].name,local_common_node->node_array[GESTURE].state?"On":"Off");
    return count;
}
 
static ssize_t gesture_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    int error;
	unsigned int temp;
    error = kstrtouint(buf, 10, &temp);
	if(error < 0)
		goto err;
	if(local_common_node->node_array[GESTURE].set){
		local_common_node->node_array[GESTURE].state = temp;
		local_common_node->node_array[GESTURE].set(temp);
	}else
		printk("node func %s is null!! \r\n",local_common_node->node_array[GESTURE].name);
err:
	return count;
}
static DEVICE_ATTR(gesture, S_IRUGO|S_IWUSR, gesture_show, gesture_store);

static const struct attribute *common_node_event_attr[] = {
        &dev_attr_gesture.attr,
	//	&dev_attr_supertorch.attr,
/*
  new item ,add here
*/
        NULL,
};
/*************************************************************************************************************************************************/
static const struct attribute_group common_node_event_attr_group = {
        .attrs = (struct attribute **) common_node_event_attr,
};

void prize_common_node_register(char* name,void(*set)(unsigned char on_off))
{
	int i;
	for(i = 0;i < MAX;i++)
	{
		if(strcmp(name,func_node_name[i]) == 0)
		{
			local_common_node->node_array[i].set = set;
		}
	}
}
EXPORT_SYMBOL(prize_common_node_register);

static void common_node_array_init(void)
{
	int i;
	for(i = 0;i < MAX;i++)
	{
		strcpy(local_common_node->node_array[i].name,func_node_name[i]);
		local_common_node->node_array[i].state = 0;
		local_common_node->node_array[i].set = NULL;
	}
}
static int common_node_probe(struct platform_device *pdev){

    int ret = 0;
 	local_common_node = devm_kzalloc(&pdev->dev, sizeof(struct common_node), GFP_KERNEL);
	common_node_array_init();
	if (local_common_node) {
		ret = sysfs_create_group(&pdev->dev.kobj, &common_node_event_attr_group);
		if(ret < 0) {
			printk(KERN_ERR "common_node:sysfs_create_group fail\r\n");
			return ret;
		}
	} else {
		printk(KERN_ERR "common_node:device_create fail\r\n");
	}
	printk("[%s] ok!!!\n",__func__);
    return 0;
}

static int common_node_remove(struct platform_device *pdev){
	printk("[common_node_dev]:common_node_remove begin!\n");
	printk("[common_node_dev]:common_node_remove Done!\n");
    
	return 0;
}

const struct of_device_id common_node_of_match[] = {
	{ .compatible = "coosea,common_node", },
	{},
};

struct platform_device common_node_device = {
	.name		= COMMON_NODE_DEVNAME,
	.id			= -1,
};

static struct platform_driver common_node_driver = {
	.remove = common_node_remove,
	.probe = common_node_probe,
	.driver = {
			.name = COMMON_NODE_DEVNAME,
			.owner = THIS_MODULE,
			.of_match_table = common_node_of_match,
	},
};

static int __init common_node_init(void)
{
	int ret = 0;
	
	printk("COOSEA common_node common_node_init\n");
	
	ret = platform_driver_register(&common_node_driver);
	if (ret < 0)
		printk("common_node : common_node_init failed ret:%d\n", ret);
	return 0;
}

static void __exit common_node_exit(void)
{
	printk("COOSEA common_node common_node_exit \n");
	platform_driver_unregister(&common_node_driver);

}

module_init(common_node_init);
module_exit(common_node_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("COOSEA COMMON_NODE driver");
MODULE_AUTHOR("Liao Jie<liaojie@cooseagroup.com>");