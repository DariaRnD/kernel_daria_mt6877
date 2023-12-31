/***********************************************************
 *  版权所有 (C) 2015-2020, 深圳市铂睿智恒科技有限公司 
 *
 *  文件名称: bat_det_device.c
 *  内容摘要: bat_det driver for bat_det device
 *  当前版本: V1.0
 *  作    者: 丁俊
 *  完成日期: 2015-04-10
 *  修改记录: 
 *  修改日期: 
 *  版本号  :
 *  修改人  :
 ***********************************************************/

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
//#include <linux/switch.h>
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

/*prize add by liuxuhui for sbat charger ---20221013---start*/
#include <linux/power_supply.h>
/*prize add by liuxuhui for sbat charger ---20221013---end*/

//#include <mach/mt_typedefs.h>
//#include <mach/mt_reg_base.h>
//#include <mach/irqs.h>
//#include <accdet_custom.h>
//#include <accdet_custom_def.h>

//#include <cust_eint.h>
//#include <cust_gpio_usage.h>

//#include <mach/mt_gpio.h>
//#include <mach/eint.h>

//#include <linux/mtgpio.h>
#include <linux/gpio.h>
#include <linux/input.h>
#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)
#include <linux/iio/consumer.h>
#endif

/*----------------------------------------------------------------------
static variable defination
----------------------------------------------------------------------*/
#define BAT_DET_DEVNAME    "bat_det_dev"

#define EN_DEBUG

#if defined(EN_DEBUG)
		
#define TRACE_FUNC 	printk("[bat_det_dev] function: %s, line: %d \n", __func__, __LINE__);

#define BAT_DET_DEBUG  printk
#else

#define TRACE_FUNC(x,...)

#define BAT_DET_DEBUG(x,...)
#endif

#define  BAT_DET_OK   0
#define  BAT_DET_NO    1

//#define BAT_DET_SWITCH_EINT        CUST_EINT_BAT_DET_1_NUM
//#define BAT_DET_SWITCH_DEBOUNCE    CUST_EINT_BAT_DET_1_DEBOUNCE_CN		/* ms */
//#define BAT_DET_SWITCH_TYPE        CUST_EINT_BAT_DET_1_TYPE           /*EINTF_TRIGGER_LOW*/
//#define BAT_DET_SWITCH_SENSITIVE   MT_LEVEL_SENSITIVE

/****************************************************************/
/*******static function defination                             **/
/****************************************************************/
//static dev_t g_bat_det_devno;
//static struct cdev *g_bat_det_cdev = NULL;
//static struct class *bat_det_class = NULL;
//static struct device *bat_det_nor_device = NULL;
//static struct input_dev *bat_det_input_dev;
//static struct kobject *g_bat_det_sys_device;
static volatile int cur_bat_det_status = BAT_DET_OK;
//prize modified by huarui, update with kernel version 20200408 start
#if defined(CONFIG_PM_WAKELOCKS)
struct wakeup_source bat_det_lock;
#else
struct wake_lock bat_det_lock;
#endif
//prize modified by huarui, update with kernel version 20200408 end
//static int bat_det_key_event = 0;
unsigned int bat_det_irq = 1;
unsigned int bat_det_eint_type;
static u32 gpiopin,gpio_deb;
//u32 ints[2] = {0, 0};
/*prize add by liuxuhui for sbat charger ---20221013---start*/
static u32 gpio_sbat_dcdc_en;
static u32 gpio_sbat_chg_en;
static u32 gpio_sbat_cs_eint;
static int is_sbat_dcdc_en = 0;
struct workqueue_struct*  sbat_chg_workqueue;
struct delayed_work sbat_chg_work;
/*prize add by liuxuhui for sbat charger ---20221013---end*/
static struct pinctrl *bat_det_pinctrl;
static struct pinctrl_state *bat_det_eint_default;
static struct pinctrl_state *bat_det_eint_as_int;


static const struct of_device_id bat_det_dt_match[] = {
	{.compatible = "mediatek,bat_det_1"},
	{},
};

static void bat_det_eint_handler(unsigned long data);
static DECLARE_TASKLET(bat_det_tasklet, bat_det_eint_handler, 0);
static atomic_t send_event_flag = ATOMIC_INIT(0);
static DECLARE_WAIT_QUEUE_HEAD(send_event_wq);
/****************************************************************/
/*******export function defination                             **/
/****************************************************************/
static ssize_t bat_det_status_info_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	BAT_DET_DEBUG("[bat_det_dev] cur_bat_det_status=%d\n", cur_bat_det_status);
	return sprintf(buf, "%u\n", cur_bat_det_status);
}

static ssize_t bat_det_status_info_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
	BAT_DET_DEBUG("[bat_det_dev] %s ON/OFF value = %d:\n ", __func__, cur_bat_det_status);

	if(sscanf(buf, "%u", &cur_bat_det_status) != 1)
	{
		BAT_DET_DEBUG("[bat_det_dev]: Invalid values\n");
		return -EINVAL;
	}
	return size;
}

static DEVICE_ATTR(bat_det_status, 0644, bat_det_status_info_show,  bat_det_status_info_store);

//增加副电池adc  充电检测  start
#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)
static struct iio_channel *batid_channel;
#endif
#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)
static const char *iio_name;
#endif
// 1v == 1000000 uv
//extern int IMM_GetOneChannelValue_Cali(int Channel, int *voltage);
static ssize_t m_bat_adc_status_info_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	int Voltiage_cali =0 ;
	int ret=0;
	int val = 0;
   ret = iio_read_channel_processed(batid_channel, &val);
   if (ret < 0) {
		BAT_DET_DEBUG("[%s] Busy/Timeout, IIO ch read failed %d\n",__func__,ret);
		return ret;
   }
	/*val * 1500 / 4096*/
	//Voltiage_cali = ((val * 1500) >> 12)* 1000; //uv
        Voltiage_cali = val * 1000; //uv
	Voltiage_cali = Voltiage_cali*3;
	  // cw_printk(1,"[%s] info id_volt = %d val =%d\n", __func__,Voltiage_cali,val);
	//ret= IMM_GetOneChannelValue_Cali(4, &Voltiage_cali);
	BAT_DET_DEBUG("[bat_det_dev] val=%d,Voltiage_cali=%d\n", val,Voltiage_cali);
	return sprintf(buf, "%u\n", Voltiage_cali);
}
static ssize_t m_bat_adc_status_info_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
	BAT_DET_DEBUG("[bat_det_dev] %s ON/OFF value = %d:\n ", __func__, cur_bat_det_status);
	//if(sscanf(buf, "%u", &cur_bat_det_status) != 1)
	//{
	//	BAT_DET_DEBUG("[bat_det_dev]: Invalid values\n");
	//	return -EINVAL;
	//}
	return size;
}
static DEVICE_ATTR(m_bat_adc_status, 0644, m_bat_adc_status_info_show,  m_bat_adc_status_info_store);
//增加副电池adc  充电检测  end

/*prize add by liuxuhui for sbat charger ---20221013---start*/
static ssize_t bat_charger_status_info_show(struct device *dev,
                struct device_attribute *attr,
                char *buf)
{
	BAT_DET_DEBUG("[bat_det_dev] is_sbat_dcdc_en=%d\n", is_sbat_dcdc_en);
	return sprintf(buf, "%u\n", is_sbat_dcdc_en);
}
static ssize_t bat_charger_status_info_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
	BAT_DET_DEBUG("[bat_det_dev] is_sbat_dcdc_en=%d\n", is_sbat_dcdc_en);
	return size;
}
static DEVICE_ATTR(bat_charger_status, 0644, bat_charger_status_info_show,  bat_charger_status_info_store);
/*prize add by liuxuhui for sbat charger ---20221013---end*/

static irqreturn_t switch_bat_det_eint_handler(int irq, void *dev_id)
{
    TRACE_FUNC;
    tasklet_schedule(&bat_det_tasklet);
	disable_irq_nosync(bat_det_irq);
	return IRQ_HANDLED;
}
__weak int prize_det_battery_status(char level){
	return 0;
}
#ifdef CONFIG_PRIZE_BAT_DET
extern  int prize_det_battery_status(char level);
#endif
static int sendBatDetEvent(void *unuse)
{
    while(1)
    {
        BAT_DET_DEBUG("[bat_det_dev]:sendBatDetEvent wait\n");
        //wait for signal
        wait_event_interruptible(send_event_wq, (atomic_read(&send_event_flag) != 0));

//prize modified by huarui, update with kernel version 20200408 start
	#if defined(CONFIG_PM_WAKELOCKS)
		__pm_wakeup_event(&bat_det_lock, 2*HZ);
	#else
        wake_lock_timeout(&bat_det_lock, 2*HZ);    //set the wake lock.
	#endif
//prize modified by huarui, update with kernel version 20200408 end
        BAT_DET_DEBUG("[bat_det_dev]:going to send event %d\n", cur_bat_det_status);
#ifdef CONFIG_PRIZE_BAT_DET
        prize_det_battery_status(cur_bat_det_status);
#endif

        //send key event
        if(BAT_DET_OK == cur_bat_det_status)
          {
                BAT_DET_DEBUG("[bat_det_dev]:BAT_DET_OK!\n");

          }
	      else if(BAT_DET_NO == cur_bat_det_status)
          {
                BAT_DET_DEBUG("[bat_det_dev]:BAT_DET_NO!\n");
          }
        atomic_set(&send_event_flag, 0);
    }
    return 0;
}
static ssize_t notify_sendBatDetEvent(int event)
{
    cur_bat_det_status = event;
    atomic_set(&send_event_flag, 1);
    wake_up(&send_event_wq);
    BAT_DET_DEBUG("[bat_det_dev]:notify_sendBatDetEvent !\n");
    return 0;
}
static void bat_det_eint_handler(unsigned long data)
{

    TRACE_FUNC;

   cur_bat_det_status = !cur_bat_det_status;
	BAT_DET_DEBUG("[bat_det_dev]:cur_bat_det_status_%d,gpio_deb_%d \n",cur_bat_det_status,gpio_deb);
   if(cur_bat_det_status)
   {
   	BAT_DET_DEBUG("[bat_det_dev]:BAT_DET_NO \n");
	irq_set_irq_type(bat_det_irq, IRQF_TRIGGER_LOW);
	gpio_set_debounce(gpiopin, gpio_deb);
	notify_sendBatDetEvent(BAT_DET_NO);
   }
   else
   {
   	BAT_DET_DEBUG("[bat_det_dev]:BAT_DET_OK \n");
	irq_set_irq_type(bat_det_irq, IRQF_TRIGGER_HIGH);
	gpio_set_debounce(gpiopin, gpio_deb);
	notify_sendBatDetEvent(BAT_DET_OK);
   }
    /* for detecting the return to old_bat_det_state */
    mdelay(10); 
    enable_irq(bat_det_irq);// mt_eint_unmask(BAT_DET_SWITCH_EINT);
}

static int bat_det_get_dts_fun(struct platform_device *pdev)
{
	int ret = 0;

	bat_det_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(bat_det_pinctrl)) {
		BAT_DET_DEBUG("Cannot find bat_det pinctrl!");
		ret = PTR_ERR(bat_det_pinctrl);
	}
	
	//bat_det eint pin initialization 
	bat_det_eint_default= pinctrl_lookup_state(bat_det_pinctrl, "default");
	if (IS_ERR(bat_det_eint_default)) {
		ret = PTR_ERR(bat_det_eint_default);
		BAT_DET_DEBUG("%s : init err, bat_det_eint_default\n", __func__);
	}

	bat_det_eint_as_int = pinctrl_lookup_state(bat_det_pinctrl, "bat_det_eint");
	if (IS_ERR(bat_det_eint_as_int)) {
		ret = PTR_ERR(bat_det_eint_as_int);
		BAT_DET_DEBUG("%s : init err, bat_det_eint\n", __func__);
	}
	else
		pinctrl_select_state(bat_det_pinctrl, bat_det_eint_as_int);
	
	return ret;
}

static int bat_det_irq_registration(void)
{
	struct device_node *node = NULL;
	u32 ints[2] = { 0, 0 };
	u32 ints1[2] = { 0, 0 };
	#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)	
	int ret = 0;
	#endif
	node = of_find_matching_node(node, bat_det_dt_match);
	if (node) {
#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)		
	      ret = of_property_read_string(node, "io-channel-names",&iio_name);
	      if (ret < 0)
	         printk("%s no iio_name(%d)\n", __func__, ret);
	      else
	   	     printk("%s iio_name (%s)\n", __func__,iio_name);
#endif
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		of_property_read_u32_array(node, "interrupts", ints1, ARRAY_SIZE(ints1));
		gpiopin = of_get_named_gpio(node, "deb-gpios", 0);
		
/*prize add by liuxuhui for sbat charger ---20221013---start*/
		gpio_sbat_dcdc_en = of_get_named_gpio(node, "sbat-dcdc-en-pins", 0);
		gpio_sbat_chg_en = of_get_named_gpio(node, "sbat-chg-en-pins", 0);
		gpio_sbat_cs_eint = of_get_named_gpio(node, "sbat-cs-eint-pins", 0);
                printk("%s gpio_sbat_dcdc_en:%d,gpio_sbat_chg_en:%d,gpio_sbat_cs_eint:%d\n", __func__,gpio_sbat_dcdc_en,gpio_sbat_chg_en,gpio_sbat_cs_eint);
                gpio_request(gpio_sbat_dcdc_en,"gpio_sbat_dcdc_en");
                gpio_request(gpio_sbat_chg_en,"gpio_sbat_chg_en");
                gpio_request(gpio_sbat_cs_eint,"gpio_sbat_cs_eint");
                gpio_direction_output(gpio_sbat_dcdc_en,0);
                gpio_direction_output(gpio_sbat_chg_en,1);
                gpio_direction_input(gpio_sbat_cs_eint);
/*prize add by liuxuhui for sbat charger ---20221013---end*/

		bat_det_eint_type = ints1[1];
		
		gpio_deb = ints[1];
		//gpio_request(ints[0], "bat_det_1");
		gpio_set_debounce(gpiopin, ints[1]);
		pinctrl_select_state(bat_det_pinctrl, bat_det_eint_as_int);
		printk("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);
	
		bat_det_irq = irq_of_parse_and_map(node, 0);
		printk("bat_det_irq = %d\n", bat_det_irq);
		if (!bat_det_irq) {
			printk("bat_det irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		//IRQF_TRIGGER_LOW
		if (request_irq(bat_det_irq, switch_bat_det_eint_handler, IRQF_TRIGGER_HIGH, "bat_det-eint", NULL)) {
			printk("BAT_DET IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
/* begin, prize-lifenfen-20181220, add for bat_det irq wakeup func */
		else
			enable_irq_wake(bat_det_irq);
/* end, prize-lifenfen-20181220, add for bat_det irq wakeup func */
		//enable_irq(bat_det_irq);
	} else {
		printk("null bat_det irq node!!\n");
		return -EINVAL;
	}

	
	return 0;
}
struct class *bat_det_class;
/*prize add by liuxuhui for sbat charger ---20221013---start*/
#if defined(CONFIG_PRIZE_CHARGE_CTRL_POLICY)
extern int g_charge_is_screen_on ;
#endif
#define SBAT_CHARGE_ADC_VALTIAGE 1473000    //4.0v
#define SBAT_CHARGE_UISOC_VAL 40            //40%
static void sbat_chg_work_func(struct work_struct *work)
{
#if defined(CONFIG_PRIZE_CHARGE_CTRL_POLICY)
        int Voltiage_cali =0 ;
        int ret=0;
        int val=0;
        static int gpio_sbat_dcdc_en_val = 0;
        static int gpio_sbat_chg_en_val = 1;
        static int gpio_sbat_cs_eint_val = 1;
        int charger_type = 0;
        int uisoc_val = 0;
        struct power_supply *psy;
        union power_supply_propval status;

        //is charger full
        gpio_sbat_cs_eint_val = gpio_get_value(gpio_sbat_cs_eint);
        psy = power_supply_get_by_name("battery");
        if (psy != NULL)
        {
            power_supply_get_property(psy,POWER_SUPPLY_PROP_CAPACITY,&status);
            uisoc_val = status.intval;
            power_supply_get_property(psy,POWER_SUPPLY_PROP_STATUS,&status);
            charger_type = status.intval;
        }
        else
        {
            printk("%s can not get battery info!!!\n",__func__);
            goto out;
        }
        if (is_sbat_dcdc_en == 0)
        {
            ret = iio_read_channel_processed(batid_channel, &val);
            if (ret < 0) {
                BAT_DET_DEBUG("[%s] Busy/Timeout, IIO ch read failed %d\n",__func__,ret);
                goto out;
            }
            /*val * 1500 / 4096*/
            Voltiage_cali = ((val * 1500) >> 12)* 1000; //uv
            Voltiage_cali = Voltiage_cali*3;
            // cw_printk(1,"[%s] info id_volt = %d val =%d\n", __func__,Voltiage_cali,val);
            //ret= IMM_GetOneChannelValue_Cali(4, &Voltiage_cali);
   //         BAT_DET_DEBUG("[bat_det_dev] val=%d,Voltiage_cali=%d\n", val,Voltiage_cali);
            if (SBAT_CHARGE_ADC_VALTIAGE > Voltiage_cali)
            {
   //             printk("%s current voltige below SBAT_CHARGE_ADC_VALTIAGE\n",__func__);
                if ( g_charge_is_screen_on==1 && charger_type==2 && uisoc_val>SBAT_CHARGE_UISOC_VAL)
                {
                    is_sbat_dcdc_en = 1;
                    printk("%s start charger!!!\n",__func__);
                    gpio_sbat_dcdc_en_val = 1;
                    gpio_sbat_chg_en_val = 0;
                    gpio_set_value(gpio_sbat_dcdc_en,gpio_sbat_dcdc_en_val);
                    gpio_set_value(gpio_sbat_chg_en,gpio_sbat_chg_en_val);
                }
                else
                {
 //                   printk("%s can not charger,wait...\n",__func__);
                }

            }
            else
            {
//                printk("%s current voltige over SBAT_CHARGE_ADC_VALTIAGE\n",__func__);
            }
        }
        else
        {
            if ( g_charge_is_screen_on!=1 || charger_type!=2 || uisoc_val<=SBAT_CHARGE_UISOC_VAL || gpio_sbat_cs_eint_val==0)
            {
                is_sbat_dcdc_en = 0;
                printk("%s stop charger!!!\n",__func__);
                gpio_sbat_dcdc_en_val = 0;
                gpio_sbat_chg_en_val = 1;
                gpio_set_value(gpio_sbat_dcdc_en,gpio_sbat_dcdc_en_val);
                gpio_set_value(gpio_sbat_chg_en,gpio_sbat_chg_en_val);
            }
        }

out:
        printk("%s Voltiage_cali:(%d) gpio_sbat_cs_eint:%d,gpio_sbat_dcdc_en:%d gpio_sbat_chg_en:%d g_charge_is_screen_on:%d\n", __func__, Voltiage_cali,gpio_sbat_cs_eint_val,gpio_sbat_dcdc_en_val,gpio_sbat_chg_en_val,g_charge_is_screen_on);
        printk("%s charger_type:%d,uisoc_val:%d\n",__func__,charger_type,uisoc_val);
        if (is_sbat_dcdc_en == 1)
            schedule_delayed_work(&sbat_chg_work,msecs_to_jiffies(1000));  //1s for 8804 test faster
        else
            schedule_delayed_work(&sbat_chg_work,msecs_to_jiffies(1000));   //1s

#endif
}
/*prize add by liuxuhui for sbat charger ---20221013---end*/

static int bat_det_probe(struct platform_device *pdev)
{
    int ret = 0;
	struct device *bat_det_dev;
    struct task_struct *BatDetEvent_thread = NULL;
    TRACE_FUNC;

//prize modified by huarui, update with kernel version 20200408 start
#if defined(CONFIG_PM_WAKELOCKS)
	wakeup_source_init(&bat_det_lock, "bat_det wakelock");
#else
	wake_lock_init(&bat_det_lock, WAKE_LOCK_SUSPEND, "bat_det wakelock");
#endif
//prize modified by huarui, update with kernel version 20200408 end
   init_waitqueue_head(&send_event_wq);
   //start send key event thread
   BatDetEvent_thread = kthread_run(sendBatDetEvent, 0, "BatDetEvent_send");
   if (IS_ERR(BatDetEvent_thread)) 
   { 
      ret = PTR_ERR(BatDetEvent_thread);
      BAT_DET_DEBUG("[bat_det_dev]:failed to create kernel thread: %d\n", ret);
   }

	   bat_det_class = class_create(THIS_MODULE, "bat_det_state");
	
	if (IS_ERR(bat_det_class)) {
		BAT_DET_DEBUG("Failed to create class(bat_det_class)!");
		return PTR_ERR(bat_det_class);
	}

	bat_det_dev = device_create(bat_det_class, NULL, 0, NULL, "bat_det_data");
	if (IS_ERR(bat_det_dev))
		BAT_DET_DEBUG("Failed to create bat_det_dev device");
	
	if (device_create_file(bat_det_dev, &dev_attr_bat_det_status) < 0)
		BAT_DET_DEBUG("Failed to create device file(%s)!",dev_attr_bat_det_status.attr.name);	
	
	if (device_create_file(bat_det_dev, &dev_attr_m_bat_adc_status) < 0)
		BAT_DET_DEBUG("Failed to create device file(%s)!",dev_attr_m_bat_adc_status.attr.name);	
	
/*prize add by liuxuhui for sbat charger ---20221013---start*/
	if (device_create_file(bat_det_dev, &dev_attr_bat_charger_status) < 0)
		BAT_DET_DEBUG("Failed to create device file(%s)!",dev_attr_bat_charger_status.attr.name);
/*prize add by liuxuhui for sbat charger ---20221013---end*/

		//wyq 20160301 get eint gpio from dts and register
	bat_det_get_dts_fun(pdev);
	bat_det_irq_registration();
#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)		
	    batid_channel = devm_kzalloc(&pdev->dev, sizeof(*batid_channel),GFP_KERNEL);
	      if (!batid_channel)
		      return -ENOMEM;
		batid_channel = iio_channel_get(&pdev->dev, iio_name);
#endif
/*prize add by liuxuhui for sbat charger ---20221013---start*/
        sbat_chg_workqueue = create_workqueue("sbat charger work");        
        INIT_DELAYED_WORK(&sbat_chg_work,sbat_chg_work_func);
        queue_delayed_work(sbat_chg_workqueue,&sbat_chg_work,msecs_to_jiffies(20000));
/*prize add by liuxuhui for sbat charger ---20221013---end*/

    return 0;
}

/*prize add by liuxuhui for sbat charger ---20221013---start*/
static int bat_det_suspend(struct platform_device *dev,pm_message_t state)
{
        printk("%s !!!\n",__func__);
        cancel_delayed_work_sync(&sbat_chg_work);
        printk("%s stop charger & delay work!!!\n",__func__);
        gpio_set_value(gpio_sbat_dcdc_en,0);
        gpio_set_value(gpio_sbat_chg_en,1);
	return 0;
}
static int bat_det_resume(struct platform_device *dev)
{

        printk("%s !!!\n",__func__);
        printk("%s start delay work!!!\n",__func__);
        schedule_delayed_work(&sbat_chg_work,msecs_to_jiffies(10000));  //10s
	return 0;
}
/*prize add by liuxuhui for sbat charger ---20221013---end*/

static int bat_det_remove(struct platform_device *dev)	
{
	BAT_DET_DEBUG("[bat_det_dev]:bat_det_remove begin!\n");
	class_destroy(bat_det_class);
/*prize add by liuxuhui for sbat charger ---20221013---start*/
        destroy_workqueue(sbat_chg_workqueue);
/*prize add by liuxuhui for sbat charger ---20221013---end*/
	BAT_DET_DEBUG("[bat_det_dev]:bat_det_remove Done!\n");
    
	return 0;
}

static struct platform_driver bat_det_driver = {
	.probe	= bat_det_probe,
	.remove  = bat_det_remove,
	.driver    = {
		.name       = "Bat_Det_Driver",
		.of_match_table = of_match_ptr(bat_det_dt_match),

	},
/*prize add by liuxuhui for sbat charger ---20221013---start*/
        .resume = bat_det_resume,
        .suspend = bat_det_suspend,
/*prize add by liuxuhui for sbat charger ---20221013---end*/
};

static int __init bat_det_init(void)
{

    int retval = 0;
    TRACE_FUNC;
    printk("[%s]: bat_det_device, retval=%d \n!", __func__, retval);
	  if (retval != 0) {
		  return retval;
	  }

    platform_driver_register(&bat_det_driver);

    return 0;
}

static void __exit bat_det_exit(void)
{
    TRACE_FUNC;
    platform_driver_unregister(&bat_det_driver);
}
late_initcall(bat_det_init);
//module_init(bat_det_init);
module_exit(bat_det_exit);
MODULE_DESCRIPTION("BAT_DET DEVICE driver");
MODULE_AUTHOR("dingjun <dingj@boruizhiheng.com>");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("bat_detdevice");

