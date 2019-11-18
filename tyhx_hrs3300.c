/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/* drivers/hwmon/mt6516/amit/hrs3300.c - hrs3300 ALS/PS driver
 * 
 * Author: 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#if 0//def CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/sysfs.h>
#include <linux/device.h> 

#include <linux/wakelock.h>
#include <linux/sched.h>
#include <asm/io.h>
//#include <cust_alsps.h>
//#include <alsps.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
//#include "mt_boot_common.h"

#include "hrs3300.h"

/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define Hrs3300_DEV_NAME     "hrs3300"
#define Hrs3300_I2C_ID 0x44 

unsigned int hr_date =0;
#ifdef BP_CUSTDOWN_ALG_LIB					
unsigned int up_date = 0;
unsigned int down_date = 0;
#endif

//static int ty_heartrate_en = 0;
//static int ty_blood_en = 0 ;
unsigned int ty_heartrate_eint_open_status = 1;

static struct hrtimer hrs3300_heartrate_ctimer ; 
static struct hrtimer hrs3300_blood_ctimer ;
static struct work_struct hrs3300_heartrate_work; 
static struct work_struct hrs3300_blood_work;
struct input_dev *hrs3300_input_dev = NULL;

/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_EMERG APS_TAG"%s %d \n", __FUNCTION__ , __LINE__)
#define APS_ERR(fmt, args...)    printk(KERN_EMERG APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_EMERG APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_EMERG APS_TAG fmt, ##args)                 
/******************************************************************************
 * extern functions
*******************************************************************************/

#define LTR_IOCTL_MAGIC         0x1C
#define LTR_IOCTL_GET_PFLAG     _IOR(LTR_IOCTL_MAGIC, 1, int)
#define LTR_IOCTL_GET_LFLAG     _IOR(LTR_IOCTL_MAGIC, 2, int)
#define LTR_IOCTL_SET_PFLAG     _IOW(LTR_IOCTL_MAGIC, 3, int)
#define LTR_IOCTL_SET_LFLAG     _IOW(LTR_IOCTL_MAGIC, 4, int)
#define LTR_IOCTL_GET_DATA      _IOW(LTR_IOCTL_MAGIC, 5, unsigned char)
#define LTR_IOCTL_GET_CHIPINFO  _IOR(LTR_IOCTL_MAGIC, 6, char)


struct mt_gpio_vbase {
	void __iomem *gpio_regs;
};
static volatile u32 g_addr_value;
static volatile u32 g_temp_value;

extern struct mt_gpio_vbase gpio_vbase;

#define GPIO_BASE (gpio_vbase.gpio_regs)

#define READ_REGISTER_UINT32(addr) \
	(*(volatile unsigned int * const)(addr))
	
#define WRITE_REGISTER_UINT32(addr, val) \
	((*(volatile unsigned int * const)(addr)) = (val))

#define SPI_GPIO_MODE_OUT_GPIO80   ((volatile unsigned int *)(0x120+GPIO_BASE))//16 offset
#define SPI_GPIO_MODE_GPIO80   ((volatile unsigned int *)(0x380+GPIO_BASE))//0 offset
#define SPI_GPIO_DIR_GPIO80   ((volatile unsigned int *)(0x020+GPIO_BASE))//16 offset

#define SPI_GPIO80_MODE_MASK   (7)

#define SPI_SET_REG32_GPIO80_OUT(offset, value) \
do {	\
        if(value == 0){ \
            g_temp_value = (~(1<<(offset)));\
            g_addr_value = (READ_REGISTER_UINT32(SPI_GPIO_MODE_OUT_GPIO80) & (unsigned int)(g_temp_value));\
        }else{\
            g_temp_value = (1 << (offset));\
            g_addr_value = (READ_REGISTER_UINT32(SPI_GPIO_MODE_OUT_GPIO80) | (unsigned int)(g_temp_value));\
        }\
	WRITE_REGISTER_UINT32(SPI_GPIO_MODE_OUT_GPIO80, g_addr_value); \
} while (0)

#define SPI_SET_REG32_GPIO80_DIR(offset, value) \
        do {    \
                if(value == 0){ \
                    g_temp_value = (~(1<<(offset)));\
                    g_addr_value = (READ_REGISTER_UINT32(SPI_GPIO_DIR_GPIO80) & (unsigned int)(g_temp_value));\
                }else{\
                    g_temp_value = (1 << (offset));\
                    g_addr_value = (READ_REGISTER_UINT32(SPI_GPIO_DIR_GPIO80) | (unsigned int)(g_temp_value));\
                }\
            WRITE_REGISTER_UINT32(SPI_GPIO_DIR_GPIO80, g_addr_value); \
        } while (0)

#define SPI_SET_GPIO_MODE_REG32_3BIT_GPIO80(offset, value) \
do {	\
        g_temp_value = (value<<(offset));\
        g_addr_value = ((READ_REGISTER_UINT32(SPI_GPIO_MODE_GPIO80)) & (unsigned int)(~(SPI_GPIO80_MODE_MASK << (offset))));\
        g_addr_value = g_addr_value | (unsigned int)(g_temp_value);\
	WRITE_REGISTER_UINT32(SPI_GPIO_MODE_GPIO80, g_addr_value); \
} while (0)

/*----------------------------------------------------------------------------*/
static struct i2c_client *hrs3300_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id hrs3300_i2c_id[] = {{Hrs3300_DEV_NAME,0},{}};
/*----------------------------------------------------------------------------*/

static int hrs3300_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int hrs3300_i2c_remove(struct i2c_client *client);
/*----------------------------------------------------------------------------*/
static int hrs3300_pm_suspend(struct device *dev);
static int hrs3300_pm_resume(struct device *dev);
int hrs3300_setup_eint(struct i2c_client *client);

struct platform_device *alspsPltFmDev;
//static int check_hand_flag =1;

static int	hrs3300_init_flag = -1;	// 0<==>OK -1 <==> fail
//static int  hrs3300_local_init(void);
//static int  hrs3300_local_uninit(void);
extern int em70xx_bpm_dynamic(int RECEIVED_BYTE, int g_sensor_x, int g_sensor_y, int g_sensor_z);
extern int em70xx_reset(int data);

//static int hrs3300_read(struct i2c_client *client,u8 reg);
//static int hrs3300_write(struct i2c_client *client,u8 reg,u8 data);
//static int hrs3300_init_client_for_resume(struct i2c_client *client);

//static int dev_attr_config(u8 reg,int config,int dev_s,int data);
static DEFINE_MUTEX(sensor_lock);
/*static struct alsps_init_info hrs3300_init_info = {
		.name = "hrs3300",
		.init = hrs3300_local_init,
		.uninit = hrs3300_local_uninit,
	
};*/
/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;
/*----------------------------------------------------------------------------*/
struct hrs3300_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};
/*----------------------------------------------------------------------------*/
struct hrs3300_priv {
    //struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;

    /*i2c address group*/
    struct hrs3300_i2c_addr  addr;
    
    /*misc*/
    atomic_t    i2c_retry;


    /*data*/
    u16          hrs;
    u8          _align;

    ulong       enable;         /*enable mask*/

	struct hrtimer hrs_timer;	
	struct workqueue_struct *em_hrs_wq;
	struct work_struct em_hrs_work;
	ktime_t hrs_delay;
	
#if 0//defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif    
	struct device_node *irq_node;
	int irq;
};


//static int ps_enabled=0;

static struct hrs3300_priv *hrs3300_obj = NULL;



/*=================================[begin] i2c ========================================*/
static int hrs3300_i2c_read(u8 reg, u8 *data)
{
	u8 buf[8] = {0}; 
	int rc;
	buf[0] = reg;
	rc = i2c_master_send(hrs3300_i2c_client, buf, 1); //If everything went ok (i.e. 1 msg transmitted), return tbytes B 
	if (rc != 1) {
		printk ("%s (%d)	: FAILED: writing to address 0x%x\n", __func__, __LINE__, reg);
		return -1;
	}

	rc = i2c_master_recv(hrs3300_i2c_client, buf, 1);	// returns negative errno, or else the number of bytes read
	if (rc != 1) {
		printk ("%s (%d)	: FAILED: reading data\n", __func__, __LINE__);
		return -1;
	}
	*data = buf[0];
	return 0;

}

static int hrs3300_i2c_write(u8 reg, u8 *data, int len)
{
	u8 buf[20] = {0}; 
	int rc = 0;
	int ret = 0;
	int i;
	buf[0] = reg;
	if (len >= 20) {
		printk("%s (%d)	: FAILED: buffer size is limitted(20) %d\n", __func__, __LINE__, len);
		return -1;
	}

	for( i=0 ; i<len; i++ ) {
		buf[i+1] = data[i];
	}	
	
	rc = i2c_master_send(hrs3300_i2c_client, buf, len+1);	// Returns negative errno, or else the number of bytes written.
	if (rc != len+1) {
		printk("%s (%d)	: FAILED: writing to reg 0x%x\n", __func__, __LINE__, reg);
		ret = -1;
	}
	return ret;
}

unsigned char Hrs3300_write_reg(unsigned char addr, unsigned char data)
{
	return hrs3300_i2c_write(addr, &data, 1);
}

unsigned char Hrs3300_read_reg(unsigned char addr)
{
	unsigned char data_buf = 0;
	hrs3300_i2c_read(addr, &data_buf);
	return data_buf;
}

extern void heart_rate_meas_timeout_handler(void); 
extern void blood_presure_meas_timeout_handler(void);
static void hrs3300_init_heartrate (int enable);
void hrs3300_input_heartrate(void);

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int hrs3300_open(struct inode *inode, struct file *file)
{
	file->private_data = hrs3300_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int hrs3300_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/

static long hrs3300_ioctl( struct file *file, unsigned int cmd,unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct hrs3300_priv *obj = i2c_get_clientdata(client);  
	int err = 0;
	void __user *ptr = (void __user*) arg;
	uint32_t enable;

	printk("%s (%d)	: ==huangwenjun== cmd = 0x%x\n", __func__, __LINE__, cmd);  // add by hwj
	printk("%s (%d)	: ==huangwenjun== enable = 0x%x\n", __func__, __LINE__, enable);  // add by hwj

	switch (cmd)
	{
		case LTR_IOCTL_SET_LFLAG:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				printk("%s (%d)	: ==huangwenjun==LTR_IOCTL_SET_LFLAG \n", __func__, __LINE__);  // add by hwj
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				hrs3300_init_heartrate(1);
				printk("%s (%d)	: ==huangwenjun==LTR_IOCTL_SET_LFLAG enable enable = 0x%x\n", __func__, __LINE__, enable);  // add by hwj
				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
				hrs3300_init_heartrate(0);
				printk("%s (%d)	: ==huangwenjun==LTR_IOCTL_SET_LFLAG else enable = 0x%x\n", __func__, __LINE__, enable);  // add by hwj
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case LTR_IOCTL_GET_LFLAG:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			printk("%s (%d)	: ==huangwenjun==LTR_IOCTL_GET_LFLAG enable =  0x%x\n", __func__, __LINE__, enable);  // add by hwj
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				printk("%s (%d)	: ==huangwenjun==LTR_IOCTL_GET_LFLAG \n", __func__, __LINE__);  // add by hwj
				err = -EFAULT;
				goto err_out;
			}
			break;

		case LTR_IOCTL_GET_DATA:
			printk("%s (%d)	: ==huangwenjun==LTR_IOCTL_GET_DATA \n", __func__, __LINE__);  // add by hwj
			break;

		//case ALSPS_GET_PS_RAW_DATA:

			//break;              

		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}
	printk("%s (%d)	: ==huangwenjun==\n", __func__, __LINE__);  // add by hwj
	err_out:
	return err;    
}
/*----------------------------------------------------------------------------*/
static struct file_operations hrs3300_fops = {
	.owner = THIS_MODULE,
	.open = hrs3300_open,
	.release = hrs3300_release,
	.unlocked_ioctl = hrs3300_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice hrs3300_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "hrs3300",
	.fops = &hrs3300_fops,
};
/*----------------------------------------------------------------------------*/

static int hrs3300_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk("%s\n",__func__);
	//hrs3300_init_heartrate(0);

	return 0;
}

static int hrs3300_resume(struct i2c_client *client)
{
	printk("%s\n",__func__);
	//hrs3300_init_heartrate(1);
	return 0;
}




static int hrs3300_pm_suspend(struct device *dev) 
{	
	struct i2c_client *client = to_i2c_client(dev);
	pm_message_t mesg = {0};

	hrs3300_suspend(client, mesg);
	printk("%s (%d)	: ==huangwenjun==hrs3300_pm_suspend== \n", __func__, __LINE__);  // add by hwj
	return 0;
}
/*----------------------------------------------------------------------------*/
static int hrs3300_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	hrs3300_resume(client);
	printk("%s (%d)	: ==huangwenjun==hrs3300_pm_resume== \n", __func__, __LINE__);  // add by hwj
	return 0;
}
/*----------------------------------------------------------------------------*/

static void hrs3300_init_heartrate (int enable) // heartrate init
{
	if (enable == 0 ){
		printk ("[%s]	:	[begin] disable chip IC \n",	__func__);
		// hwj
		hrtimer_cancel(&hrs3300_heartrate_ctimer);
		mdelay(2);
		Hrs3300_chip_disable();
		printk ("[%s]	:	[end] disable chip IC \n",__func__	);
		printk("%s (%d)	: ==huangwenjun==hrs3300_init_heartrate== enable = 0x%x\n", __func__, __LINE__, enable);  // add by hwj
	} else if (enable == 1) { // \B3\F5ʼо\C6
		mdelay(5);
		Hrs3300_chip_init();
		Hrs3300_chip_enable();
		Hrs3300_alg_open();
		hrtimer_start(&hrs3300_heartrate_ctimer,ktime_set(0, 40*1000000), HRTIMER_MODE_REL);  // delete by hwj
		printk ("[%s]	:	[end] enable chip IC \n",__func__);
		printk("%s (%d)	: ==huangwenjun==hrs3300_init_heartrate== enable = 0x%x\n", __func__, __LINE__, enable);  // add by hwj
	} else {
		printk ("[%s]	: enable is error \n",__func__);
		printk("%s (%d)	: ==huangwenjun==hrs3300_init_heartrate== enable is error !!!\n", __func__, __LINE__);  // add by hwj
	}
}


static enum hrtimer_restart hrs3300_heartrate_timer_func(struct hrtimer  *timer)
{
	schedule_work (&hrs3300_heartrate_work);   //  delete by hwj
	hrtimer_start(&hrs3300_heartrate_ctimer,ktime_set(0, 40*1000000), HRTIMER_MODE_REL);	// 35ms
	
	return HRTIMER_NORESTART ;
}

static enum hrtimer_restart hrs3300_blood_timer_func(struct hrtimer *timer)
{
	schedule_work(&hrs3300_blood_work);
	//hrtimer_start(&hrs3300_blood_ctimer,ktime_set(0, 20*1000000), HRTIMER_MODE_REL);	// 18ms hwj
	return HRTIMER_NORESTART ;
}

static void hrs3300_heartrate_work_func(struct work_struct *work)
{
	printk ("enter	>>>>%s	\n", 	__func__);
	heart_rate_meas_timeout_handler();
	hrs3300_input_heartrate();
	//hrtimer_start(&hrs3300_heartrate_ctimer,ktime_set(0, 40*1000000), HRTIMER_MODE_REL);	// 35ms hwj
	printk ("end >>>> %s \n", __func__);
}

static void hrs3300_blood_work_func(struct work_struct *work)
{
	printk ("enter	>>>>	%s	\n", 	__func__);
	blood_presure_meas_timeout_handler ();
	//hrtimer_start(&hrs3300_blood_ctimer,ktime_set(0, 18*1000000), HRTIMER_MODE_REL);	// 18ms
	printk ("end >>>> %s \n", __func__);
}

void hrs3300_input_heartrate(void)
{
	printk("enter >>> %s hr_date=%d \n", __func__,hr_date); 
	input_report_abs(hrs3300_input_dev,ABS_MISC, hr_date);
	input_sync(hrs3300_input_dev);
	printk("end >>> %s \n", __func__);
}

static int hrs3300_init_input_data(void)
{
	int ret = 0;
	printk("%s (%d) : initialize data\n",__func__ , __LINE__);
	hrs3300_input_dev = input_allocate_device();
	if (!hrs3300_input_dev){
		printk("%s %d) : could not allocate mouse input device\n", __func__	, __LINE__);
		return -ENOMEM;
	}
	
	hrs3300_input_dev->name = Hrs3300_DEV_NAME;
	hrs3300_input_dev->phys = Hrs3300_DEV_NAME;
	hrs3300_input_dev->id.bustype = BUS_I2C;
	hrs3300_input_dev->dev.parent = &hrs3300_i2c_client->dev;
	hrs3300_input_dev->id.vendor = 0x0001;
	hrs3300_input_dev->id.product = 0x0001;

	__set_bit(EV_ABS, hrs3300_input_dev->evbit);
	//input_set_abs_params(hrs3300_input_dev, ABS_DISTANCE, 0, 1, 0, -1);
	input_set_abs_params(hrs3300_input_dev, ABS_MISC, 0, 100001, 0, -1);
	ret = input_register_device(hrs3300_input_dev);
	printk("%s (%d)	: ==huangwenjun==hrs3300_init_input_data== ret = 0x%x\n", __func__, __LINE__, ret);  // add by hwj
	if (ret < 0) {
		printk("input_register_device failed!\n");
		input_free_device(hrs3300_input_dev);
		hrs3300_input_dev = NULL;
		return ret;
	}
	return 0;
}

static int hrs3300_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct hrs3300_priv *obj;
	int err = 0;
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));

	hrs3300_obj = obj;
	
	APS_LOG("hrs3300_init_client() in!\n");
	
	client->addr = Hrs3300_I2C_ID;
	printk(KERN_EMERG "hrs3300 probe %s addr = 0x%x\n",__func__,client->addr);
	printk("%s (%d)	: ==huangwenjun==hrs3300_i2c_probe== client->addr\n", __func__, __LINE__);  // add by hwj

	obj->client = client;
	i2c_set_clientdata(client, obj);	
	obj->enable = 0;
		
	//set_bit(CMC_BIT_PS, &obj->enable);  // hwj will open
	
	hrs3300_i2c_client = client;
	
	err = Hrs3300_chip_init();  // enable !!!
	if(err ==0)
	{
		goto exit_init_failed;
	}

	APS_LOG("hrs3300_init_client() OK!\n");
	printk("%s (%d)	: ==huangwenjun==hrs3300_i2c_probe== hrs3300_init_client() OK!\n", __func__, __LINE__);  // add by hwj
	

	err = misc_register(&hrs3300_device);

	if(err)
	{
		printk("%s (%d)	: ==huangwenjun==hrs3300_i2c_probe== hrs3300_device register failed\n", __func__, __LINE__);  // add by hwj
		APS_ERR("hrs3300_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	/*
	err = hrs3300_create_attr(&(hrs3300_init_info.platform_diver_addr->driver));
	if(err)
	{
		goto exit_create_attr_failed;
	}
	*/
	if(hrs3300_init_input_data()< 0){ 
		goto exit_create_attr_failed;
	}
	hrs3300_init_heartrate(0);    	// hwj
	obj->em_hrs_wq = create_singlethread_workqueue("em_hrs_wq");
	
	hrtimer_init(&hrs3300_heartrate_ctimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hrs3300_heartrate_ctimer.function = hrs3300_heartrate_timer_func;
	hrtimer_init(&hrs3300_blood_ctimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hrs3300_blood_ctimer.function = hrs3300_blood_timer_func;
	INIT_WORK(&hrs3300_heartrate_work, hrs3300_heartrate_work_func);
	INIT_WORK(&hrs3300_blood_work, hrs3300_blood_work_func);

	hrs3300_init_flag = 0;
	printk("hrs3300 probe %s OK\n",__func__);
	printk("%s (%d)	: ==huangwenjun==hrs3300_i2c_probe== hrs3300 probe is OK\n", __func__, __LINE__);  // add by hwj
	APS_LOG("%s: OK\n", __func__);
	hrs3300_init_heartrate(0);    	// hwj

	return 0;

	exit_create_attr_failed:
	misc_deregister(&hrs3300_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	exit:
	hrs3300_i2c_client = NULL;    
	hrs3300_init_flag = -1;
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int hrs3300_i2c_remove(struct i2c_client *client)
{
	misc_deregister(&hrs3300_device);
	
	hrs3300_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	printk("%s (%d)	: ==huangwenjun==hrs3300_i2c_remove== END\n", __func__, __LINE__);  // add by hwj
	return 0;
}

static const struct dev_pm_ops hrs3300_pm_ops = {
	.suspend = hrs3300_pm_suspend,
	.resume = hrs3300_pm_resume,
};

static const struct i2c_device_id hrs3300_id[] = {
	{Hrs3300_DEV_NAME, 0},
	{}
};

static const struct of_device_id hrs3300_of_match[] = {
	{.compatible = "hrs,hrs3300"},
	{},
};


static struct i2c_driver hrs3300_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name = Hrs3300_DEV_NAME,
		.of_match_table = hrs3300_of_match,
		.pm = &hrs3300_pm_ops
	},
	.probe = hrs3300_i2c_probe,
	.remove = hrs3300_i2c_remove,
	.id_table = hrs3300_id,
};

static int __init hrs3300_init(void)
{
	int ret = -1;
	printk("%s\n",__func__);
	ret = i2c_add_driver(&hrs3300_i2c_driver);
	printk("%s (%d)	: ==huangwenjun==hrs3300_iniT== ret = 0X%x\n", __func__, __LINE__, ret);  // add by hwj
	if (ret) {
		printk("i2c_add_driver failed!\n");
		return ret;
	}
	return ret;


	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit hrs3300_exit(void)
{
	APS_FUN();
	i2c_del_driver(&hrs3300_i2c_driver);
}
/*----------------------------------------------------------------------------*/
module_init(hrs3300_init);     
module_exit(hrs3300_exit);   
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("binghua chen@epticore.com");
MODULE_DESCRIPTION("hrs3300 driver");
MODULE_LICENSE("GPL");
