#include <linux/input/tp_common.h>
#include <linux/kernel.h>

struct kobject *touchpanel_kobj;

#define TS_RO_FOPS(type)                                               \
    int tp_common_set_##type##_ops(struct tp_common_ops *ops)          \
    {                                                                  \
        static struct kobj_attribute kattr =                           \
            __ATTR(type, S_IRUGO, NULL, NULL);                         \
        WARN(ops->store, "RO attribute with store method");         \
        WARN(!ops->show, "RO attribute without show method");       \
        kattr.show = ops->show;                                        \
        return sysfs_create_file(touchpanel_kobj, &kattr.attr);        \
    }

#define TS_RW_FOPS(type)                                               \
    int tp_common_set_##type##_ops(struct tp_common_ops *ops)          \
    {                                                                  \
        static struct kobj_attribute kattr =                           \
            __ATTR(type, (S_IWUSR | S_IRUGO), NULL, NULL);             \
        WARN(!ops->show, "RW attribute without show method");       \
        WARN(!ops->store, "RW attribute without store method");     \
        kattr.show = ops->show;                                        \
        kattr.store = ops->store;                                      \
        return sysfs_create_file(touchpanel_kobj, &kattr.attr);        \
    }

TS_RO_FOPS(double_tap_pressed)
TS_RO_FOPS(single_tap_pressed)
TS_RO_FOPS(fod_pressed)

TS_RW_FOPS(double_tap_enabled)
TS_RW_FOPS(single_tap_enabled)
TS_RW_FOPS(fod_enabled)

static int __init tp_common_init(void)
{
	touchpanel_kobj = kobject_create_and_add("touchpanel", NULL);
	if (!touchpanel_kobj)
		return -ENOMEM;

	return 0;
}

core_initcall(tp_common_init);
