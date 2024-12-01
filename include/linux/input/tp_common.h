#include <linux/kobject.h>

#define TS_DEFINE_OPS(type) \
    int tp_common_set_##type##_ops(struct tp_common_ops *ops);

extern struct kobject *touchpanel_kobj;

struct tp_common_ops {
	ssize_t (*show)(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf);
	ssize_t (*store)(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count);
};

TS_DEFINE_OPS(double_tap_pressed)
TS_DEFINE_OPS(single_tap_pressed)
TS_DEFINE_OPS(fod_pressed)

TS_DEFINE_OPS(double_tap_enabled)
TS_DEFINE_OPS(single_tap_enabled)
TS_DEFINE_OPS(fod_enabled)
