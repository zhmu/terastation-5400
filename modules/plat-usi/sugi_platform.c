/*	Sugi Platform Driver - V1.2
 *
 * 	Author: Gary Chen
 *	Date  : 2012/07/11
 */


#include <asm/io.h>
#include <asm/msr.h>
#include <asm/errno.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/pci.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include "sugi_platform.h"


/* ---------------------------------------------------------------------------------------------- *
 * BLOCK 1
 * Description: Global variabls and strctures
 * ---------------------------------------------------------------------------------------------- */

/* Define for procfs directory and file */
#if defined(CONFIG_BUFFALO_PLATFORM)
#include <buffalo/kernevnt.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
#define HAVE_PROC_OPS 1
#else
#define HAVE_PROC_OPS 0
#endif



/* **************** Button Driver **************** */

/* declare for file and directory of procfs */
static struct proc_dir_entry *buffalo_dir;
static struct proc_dir_entry *gpio_dir;
static struct proc_dir_entry *switch_dir;
static struct proc_dir_entry *switch_file[MAX_SUPPORTED_BUTTONS];

/* GPIO set3 base addr */
static unsigned int it87_gpioset3_base_addr;

/* declare timer function */
static struct timer_list btn_timer;
static int btn_timer_pin = -1;

/* button information strcture */
static struct button_info_st button_info[] = {
	{"func", IT87_DEB0},
	{"display", IT87_DEB1}
};

/* ************** Fan Control Driver ************* */

/* class and device nodes */
struct class *sugi_cls;
struct device *fan_dev;
dev_t sugit_fan = MKDEV(sugi_fan_major, sugi_fan_minor);

/* IT8721 EC base address */
unsigned int EC_BASE_ADDR;
unsigned int IT87_EC_ADDR_REG;
unsigned int IT87_EC_DATA_REG;

/* core temp used */
unsigned int tjmax[2] = {100,100};
unsigned int eax;
unsigned int edx;

/* fan, temp information strcture */
struct fan_info_st fan_info[] =
{
	{IT87_EC_FAN_TACO1, IT87_EC_FAN_TACO1_EXT, IT87_EC_FAN_SMART_PWM1, IT87_EC_FAN_PWM1, 0x1},
	{IT87_EC_FAN_TACO2, IT87_EC_FAN_TACO2_EXT, IT87_EC_FAN_SMART_PWM2, IT87_EC_FAN_PWM2, 0x2},
};

/* 4 steps of speed */
static unsigned char fan_stop_speed = 0;
static unsigned char fan_slow_speed = 150;
static unsigned char fan_fast_speed = 200;
static unsigned char fan_full_speed = 255;

/* threshold & flag */
static unsigned int fan_threshold = 100;
static unsigned int flag_show_speed_by_string = 1;

/* ************** HDD Control Driver ************* */

/* pci device */
struct pci_dev *pdev = NULL;

/* declare for file and directory of procfs */
static struct proc_dir_entry *hotplug_dir;
static struct proc_dir_entry *power_control_dir;
static struct proc_dir_entry *hotplug_sata_file[MAX_SUPPORTED_DISKS];
static struct proc_dir_entry *power_control_hdd_file[MAX_SUPPORTED_DISKS];

/* GPIO base address */
unsigned int ich10r_gpio_base_addr;

/* polling timer */
struct timer_list sata_hotplug_polling_timer;

/* hotplug status information strcture */
struct sata_hotplug_data_st sata_hotplug_data[MAX_SUPPORTED_DISKS];

/* HDD information strcture */
static struct sugi_hdd_info_st sugi_hdd_info[] = {
	{GP_LVL,	HDD0_PRESENT_BIT,	GP_LVL,		HDD0_POWER_BIT,	HDD0_POWER_MASK},
	{GP_LVL,	HDD1_PRESENT_BIT,	GP_LVL,		HDD1_POWER_BIT,	HDD1_POWER_MASK},
	{GP_LVL,	HDD2_PRESENT_BIT,	GP_LVL,		HDD2_POWER_BIT,	HDD2_POWER_MASK},
	{GP_LVL,	HDD3_PRESENT_BIT,	GP_LVL,		HDD3_POWER_BIT,	HDD3_POWER_MASK},
	{GP_LVL,	HDD4_PRESENT_BIT,	GP_LVL2,	HDD4_POWER_BIT,	HDD4_POWER_MASK},
	{GP_LVL,	HDD5_PRESENT_BIT,	GP_LVL2,	HDD5_POWER_BIT,	HDD5_POWER_MASK},
	{GP_LVL,	HDD6_PRESENT_BIT,	GP_LVL2,	HDD6_POWER_BIT,	HDD6_POWER_MASK},
	{GP_LVL,	HDD7_PRESENT_BIT,	GP_LVL2,	HDD7_POWER_BIT,	HDD7_POWER_MASK},
};

/* ***************** CMOS Driver ***************** */
/* device nodes */
struct device *cmos_dev;
dev_t sugit_cmos = MKDEV(sugi_cmos_major, sugi_cmos_minor);

unsigned int ich10r_pm_base_addr;
unsigned short afterg3_en;

/* Device ID offset */
unsigned char device_id_offset[32] = {0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d,
																0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55,
																0x56, 0x57, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e,
																0x5f, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c};

/* checksum offset */
unsigned char checksum_offset[4] = {0x79, 0x7a, 0x7b, 0x7c};

/* ASCII string to integer */																
struct strtoint_st strtoint_info[] = {
	{0x30, 0x0}, {0x31, 0x1}, {0x32, 0x2}, {0x33, 0x3},
	{0x34, 0x4}, {0x35, 0x5}, {0x36, 0x6}, {0x37, 0x7},
	{0x38, 0x8}, {0x39, 0x9}, {0x61, 0xa}, {0x62, 0xb},
	{0x63, 0xc}, {0x64, 0xd}, {0x65, 0xe}, {0x66, 0xf},
	{0x41, 0xa}, {0x42, 0xb}, {0x43, 0xc}, {0x44, 0xd},
	{0x45, 0xe}, {0x46, 0xf},
};

static unsigned int bios_ver;

/* *************** Boaed ID Driver *************** */

struct device *boardid_dev;
dev_t sugit_boardid = MKDEV(sugi_boardid_major, sugi_boardid_minor);
unsigned int boardid_num;

/* ************* protocol for IT8721 ************* */
void EnterMBPnP(void)
{
	outb(0x87, CONFIG_ADDR_PORT);
	outb(0x01, CONFIG_ADDR_PORT);
	outb(0x55, CONFIG_ADDR_PORT);
	outb(0x55, CONFIG_ADDR_PORT);
}
void ExitMBPnP(void)
{
	outb(0x02, CONFIG_ADDR_PORT);
	outb(0x02, CONFIG_DATA_PORT);
}


/* ---------------------------------------------------------------------------------------------- *
 * BLOCK 2-1
 * Description: Implement functions of button driver
 * ---------------------------------------------------------------------------------------------- */

/* ************** implement function ************* */

/* ----------- get GPIO status ----------- */
static int it87_gpio_getsts(int pin)
{
	int	off;
	if(pin == BTN_DISP){
		off = IT87_GP35_BIT5;	//pin number of DISP button
	}else{
		off = IT87_GP34_BIT4;	//pin number of FUNC button
	}		

	return inb(it87_gpioset3_base_addr) & off;
}

/* ------------- button timer ------------ */	
static void timer_function(struct timer_list* __unused)
{
	int	pin_sts;

	pin_sts=it87_gpio_getsts(btn_timer_pin);
	
	if(pin_sts){
	//Clear de-bounce ping SMI status
		EnterMBPnP();
		outb(IT87_REG_SMI_STS2, CONFIG_ADDR_PORT);
		outb(IT87_DEB0DEB1 | IT87_SMI_STS2_PBD, CONFIG_DATA_PORT);		
		ExitMBPnP();
	//Delete timer
		del_timer(&btn_timer);
	}
	else
    mod_timer(&btn_timer, jiffies + TIMER_DELAY);
}

/* ---------- get button status ---------- */
static int show_button_status(struct seq_file *m, void *v)
{
	struct button_info_st *button_info;
	unsigned char smi_sts;

	button_info = (struct button_info_st *)m->private;
	
	EnterMBPnP();

	//Get SMI status of two de-bounce pin
	outb(IT87_REG_SMI_STS2, CONFIG_ADDR_PORT);
	smi_sts = inb(CONFIG_DATA_PORT);
	smi_sts &= IT87_SMI_DEB_MASK;

	ExitMBPnP();

	//Check which button was pressed
	if ((smi_sts & button_info->reg_stat))
		seq_printf(m, "on\n");
	else
		seq_printf(m, "off\n");

	return 0;
}

static int open_button_status(struct inode *inode, struct file *file)
{
	return single_open(file, show_button_status, PDE_DATA(file_inode(file)));
}

#if HAVE_PROC_OPS
static struct proc_ops button_status_fops = {
    .proc_open		= open_button_status,
	.proc_read		= seq_read,
	.proc_lseek	    = seq_lseek,
	.proc_release	= single_release
};
#else
static struct file_operations button_status_fops = {
	.owner		= THIS_MODULE,
        .open		= open_button_status,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release
};
#endif

/* ----------- interrupt handler ---------- */
static irqreturn_t sugi_interrupt(int irq, void *dev_id)
{
	unsigned char smi_sts;
	
	#ifdef CONFIG_BUFFALO_PLATFORM
		char msg[32];
		memset(msg, 0, sizeof(msg));
	#endif
	
	/* get SMI status of two de-bounce pin */
	EnterMBPnP();	
	
	outb(IT87_REG_SMI_STS2, CONFIG_ADDR_PORT);
	smi_sts = inb(CONFIG_DATA_PORT);
	
	ExitMBPnP();
	
	/* check if the interrupt source is PBD's IRQ */
	if(smi_sts & IT87_SMI_STS2_PBD)
	{
		smi_sts &= IT87_SMI_DEB_MASK;
		
		/* check which button was pressed */
		if(smi_sts == IT87_DEB0){
			#ifdef CONFIG_BUFFALO_PLATFORM
				sprintf(msg, "micon_interrupts");
			#else
				printk("EVENT IS OCCURRED\n");
			#endif
			btn_timer_pin = BTN_FUNC;
		}
		else if(smi_sts == IT87_DEB1){
			#ifdef CONFIG_BUFFALO_PLATFORM
				sprintf(msg, "micon_interrupts");
			#else
				printk("EVENT IS OCCURRED\n");
			#endif
			btn_timer_pin = BTN_DISP;
		}
		
		/* setup timer for checking button GPIO status */
		timer_setup(&btn_timer, timer_function, 0);
		mod_timer(&btn_timer, jiffies + TIMER_DELAY);
		
		#ifdef CONFIG_BUFFALO_PLATFORM
			if(msg[0] != '\0')
				buffalo_kernevnt_queuein(msg);
		#endif
	}
	return IRQ_HANDLED;
}

/* ---------------------------------------------------------------------------------------------- *
 * BLOCK 2-2
 * Description: Implement functions of fan control driver
 * ---------------------------------------------------------------------------------------------- */

/* ************* implement functions ************* */

/* ----------- get temperature ---------- */
unsigned int get_temp(unsigned char temp_num)
{
	unsigned int	ret = 0;
	
	if(temp_num==2){
		outb(IT87_EC_TMPIN3, IT87_EC_ADDR_REG);
		ret = inb(IT87_EC_DATA_REG);
		return ret;
	}
	
	rdmsr_on_cpu(temp_num, MSR_IA32_THERM_STATUS, &eax, &edx);
	ret = tjmax[temp_num] - ((eax >> 16)&0x7f);
	
	return ret;
}

/* ---------- get rotation speed --------- */
unsigned int get_fan_rpm(unsigned char fan_num)
{
	unsigned int	read_value;
	unsigned int	read_value_ext;
	unsigned int	ret = 0;
	
	outb(fan_info[fan_num].reg, IT87_EC_ADDR_REG);
	read_value = inb(IT87_EC_DATA_REG);
	outb(fan_info[fan_num].reg_ext, IT87_EC_ADDR_REG);
	read_value_ext = inb(IT87_EC_DATA_REG);
	ret = 1350000/(read_value_ext*256+read_value)/2;

	if(ret < fan_threshold)
		ret = 0;
	
	return ret;
}

/* -------------- get speed -------------- */
unsigned int get_fan_speed(unsigned char fan_num)
{
	unsigned int	read_value;

		
	outb(IT87_EC_FAN_MAIN_CTL, IT87_EC_ADDR_REG);
	read_value = inb(IT87_EC_DATA_REG);
	read_value |= fan_info[fan_num].offset;
	outb(read_value, IT87_EC_DATA_REG);

	outb(fan_info[fan_num].reg_smart_pwm, IT87_EC_ADDR_REG);
	read_value = inb(IT87_EC_DATA_REG);

	return read_value;
}

/* -------------- set speed -------------- */
unsigned int set_fan_speed(unsigned char fan_num, unsigned char set_val)
{
	unsigned int	write_data;
	unsigned char	read_value;

	/*if(fan_info[fan_num].reg_smart_pwm == 0 ||
		fan_info[fan_num].reg_pwm == 0)
	{
		return -EINVAL;
	}*/

	outb(IT87_EC_FAN_MAIN_CTL, IT87_EC_ADDR_REG);
	read_value = inb(IT87_EC_DATA_REG);
	read_value |= fan_info[fan_num].offset;
	outb(read_value, IT87_EC_DATA_REG);

	write_data = set_val;
	outb(fan_info[fan_num].reg_smart_pwm, IT87_EC_ADDR_REG);
	outb(write_data, IT87_EC_DATA_REG);

	outb(fan_info[fan_num].reg_pwm, IT87_EC_ADDR_REG);
	read_value = inb(IT87_EC_DATA_REG);
	read_value &= 0x7F;
	outb(read_value, IT87_EC_DATA_REG);

	return 0;
}

/* --------- convert string to ul -------- */
static unsigned long fan_speed_str_to_ul(char *buf)
{
	int len = 0;
	
	while(buf[len])
		len++;
	
	if((len == 5) && (strncmp(buf, "stop", strlen("stop")) == 0))
		return fan_stop_speed;
	else if((len == 5) && (strncmp(buf, "slow", strlen("slow")) == 0))
		return fan_slow_speed;
	else if((len == 5) && (strncmp(buf, "fast", strlen("fast")) == 0))
		return fan_fast_speed;
	else if((len == 5) && (strncmp(buf, "full", strlen("full")) == 0))
		return fan_full_speed;

	return simple_strtoul(buf, NULL, 10);
}

/* --------- convert ul to string -------- */
static unsigned long fan_speed_ul_to_str(char *buf, size_t len, unsigned long speed)
{
	if(speed <= fan_stop_speed)
		snprintf(buf, len, "stop");
	else if(speed > fan_stop_speed && speed <= fan_slow_speed)
		snprintf(buf, len, "slow");
	else if(speed > fan_slow_speed && speed <= fan_fast_speed)
		snprintf(buf, len, "fast");
	else if(speed > fan_fast_speed)
		snprintf(buf, len, "full");
	else
		snprintf(buf, len, "Unknown");
		
	return 0;
}

/* ************** device attributes ************** */

/* -------- temperature attributes ------- */
static ssize_t show_temp1(struct device *dev, struct device_attribute *attr, char *buf)

{
	return sprintf(buf, "%d\n", get_temp(0));
}

static ssize_t show_temp2(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", get_temp(1));
}

static ssize_t show_temp3(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", get_temp(2));
}

DEVICE_ATTR(temp1, 0444, show_temp1, NULL);
DEVICE_ATTR(temp2, 0444, show_temp2, NULL);
DEVICE_ATTR(temp3, 0444, show_temp3, NULL);

/* ------ rotation speed attributes ------ */
static ssize_t show_fan1_rpm(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", get_fan_rpm(0));
}

static ssize_t show_fan2_rpm(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", get_fan_rpm(1));
}

DEVICE_ATTR(fan1_rpm, 0444, show_fan1_rpm, NULL);
DEVICE_ATTR(fan2_rpm, 0444, show_fan2_rpm, NULL);

/* fan3_rpm and fan4_rpm nodes are added to fit Buffalo's requirement. */
DEVICE_ATTR(fan3_rpm, 0444, show_fan1_rpm, NULL);
DEVICE_ATTR(fan4_rpm, 0444, show_fan2_rpm, NULL);

/* ----------- speed attributes ---------- */
static ssize_t show_fan1_speed(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(flag_show_speed_by_string == 1)
	{
		char szSpeed[32];
		fan_speed_ul_to_str(szSpeed, sizeof(szSpeed), get_fan_speed(0));
		return sprintf(buf, "%s\n", szSpeed);
	}
	else
		return sprintf(buf, "%d\n", get_fan_speed(0));
}
static ssize_t store_fan1_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long write_val = fan_speed_str_to_ul((char*) buf );
	if(write_val > 0xff)
		return -EINVAL;

	set_fan_speed(0, (unsigned char)(write_val & 0xff));

	return count;
}

static ssize_t show_fan2_speed(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(flag_show_speed_by_string == 1)
	{
		char szSpeed[32];
		fan_speed_ul_to_str(szSpeed, sizeof(szSpeed), get_fan_speed(1));
		return sprintf(buf, "%s\n", szSpeed);
	}
	else
		return sprintf(buf, "%d\n", get_fan_speed(1));
}
static ssize_t store_fan2_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long write_val = fan_speed_str_to_ul((char*) buf );
	if(write_val > 0xff)
		return -EINVAL;

	set_fan_speed(1, (unsigned char)(write_val & 0xff));
	return count;
}

static ssize_t show_fan3_speed(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned int temp;
	if(flag_show_speed_by_string == 1)
	{
		char szSpeed[32];
		fan_speed_ul_to_str(szSpeed, sizeof(szSpeed), get_fan_speed(0));
		/* get_fan_speed(1) is used to set fan4 as software control mode */
		temp = get_fan_speed(1);
		return sprintf(buf, "%s\n", szSpeed);
	}
	else{
		/* get_fan_speed(1) is used to set fan4 as software control mode */
		temp = get_fan_speed(1);
		return sprintf(buf, "%d\n", get_fan_speed(0));
	}
}
static ssize_t store_fan3_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long write_val = fan_speed_str_to_ul((char*) buf );
	if(write_val > 0xff)
		return -EINVAL;
	
	set_fan_speed(0, (unsigned char)(write_val & 0xff));
	set_fan_speed(1, (unsigned char)(write_val & 0xff));
	return count;
}

DEVICE_ATTR(fan1_speed, 0644, show_fan1_speed, store_fan1_speed);
DEVICE_ATTR(fan2_speed, 0644, show_fan2_speed, store_fan2_speed);

/* fan3_speed and fan4_speed are added to fit Buffalo's requirement,
   and their behavior are same to Matsu. */
DEVICE_ATTR(fan3_speed, 0644, show_fan3_speed, store_fan3_speed);
DEVICE_ATTR(fan4_speed, 0644, show_fan2_speed, store_fan2_speed);

/* ---------- 4 steps attributes --------- */
static ssize_t show_fan_stop_speed(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", fan_stop_speed);
}
static ssize_t store_fan_stop_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long tmp_speed = simple_strtoul(buf, NULL, 10);
	if(tmp_speed > 0xFF)
		return -EINVAL;

	fan_stop_speed = tmp_speed;
	return count;
}

static ssize_t show_fan_slow_speed(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", fan_slow_speed);
}
static ssize_t store_fan_slow_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long tmp_speed = simple_strtoul(buf, NULL, 10);
	if(tmp_speed > 0xFF)
		return -EINVAL;

	fan_slow_speed = tmp_speed;
	return count;
}

static ssize_t show_fan_fast_speed(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", fan_fast_speed);
}
static ssize_t store_fan_fast_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long tmp_speed = simple_strtoul(buf, NULL, 10);
	if(tmp_speed > 0xFF)
		return -EINVAL;

	fan_fast_speed = tmp_speed;
	return count;
}

static ssize_t show_fan_full_speed(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", fan_full_speed);
}
static ssize_t store_fan_full_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long tmp_speed = simple_strtoul(buf, NULL, 10);
	if(tmp_speed > 0xFF)
		return -EINVAL;

	fan_full_speed = tmp_speed;
	return count;
}

DEVICE_ATTR(fan_stop_speed, 0644, show_fan_stop_speed, store_fan_stop_speed);
DEVICE_ATTR(fan_slow_speed, 0644, show_fan_slow_speed, store_fan_slow_speed);
DEVICE_ATTR(fan_fast_speed, 0644, show_fan_fast_speed, store_fan_fast_speed);
DEVICE_ATTR(fan_full_speed, 0644, show_fan_full_speed, store_fan_full_speed);

/* --------- read mode attribute --------- */
static ssize_t show_show_speed_by_string(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", flag_show_speed_by_string);
}
static ssize_t store_show_speed_by_string(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);
	
	if(value != 0 && value != 1)
		return -EINVAL;

	flag_show_speed_by_string = value;

	return count;
}
DEVICE_ATTR(show_speed_by_string, 0644, show_show_speed_by_string, store_show_speed_by_string);

/* --------- threshold attribute --------- */
static ssize_t show_fan_threshold(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", fan_threshold);
}
static ssize_t store_fan_threshold(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long tmp_threshold = simple_strtoul(buf, NULL, 10);
	fan_threshold = tmp_threshold;
	return count;
}
DEVICE_ATTR(fan_threshold, 0644, show_fan_threshold, store_fan_threshold);

/* ------- device number attribute ------- */
static ssize_t show_sugi_fan_dev(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sugi_fan_major);
}
DEVICE_ATTR(sugi_fan_dev, 0444, show_sugi_fan_dev, NULL);


/* ---------------------------------------------------------------------------------------------- *
 * BLOCK 2-3
 * Description: Implement functions of HDD control driver
 * ---------------------------------------------------------------------------------------------- */

/* -------- get HDD hotplug status ------- */
static int show_hotplug_status(struct seq_file *m, void *v)
{
	struct sugi_hdd_info_st *hdd_info;

	hdd_info = (struct sugi_hdd_info_st *)m->private;

	if (inl(ich10r_gpio_base_addr + hdd_info->HDD_GPIO_PRESENT_REG) & hdd_info->HDD_PRESENT_BIT)
		seq_printf(m, "unplugged\n");
	else
		seq_printf(m, "plugged\n");

	return 0;
}

static int open_hotplug_status(struct inode *inode, struct file *file)
{
	return single_open(file, show_hotplug_status, PDE_DATA(file_inode(file)));
}

#if HAVE_PROC_OPS
static struct proc_ops hotplug_status_fops = {
	.proc_open		= open_hotplug_status,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release
};
#else
static struct file_operations hotplug_status_fops = {
	.owner		= THIS_MODULE,
	.open		= open_hotplug_status,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release
};
#endif

/* ----- get HDD power control status ---- */
static int show_power_control_status(struct seq_file *m, void *v)
{
	struct sugi_hdd_info_st *hdd_info;

	hdd_info = (struct sugi_hdd_info_st *)m->private;

	if (inl(ich10r_gpio_base_addr + hdd_info->HDD_GPIO_POWER_REG) & hdd_info->HDD_POWER_BIT)
		seq_printf(m, "on\n");
	else
		seq_printf(m, "off\n");

	return 0;
}

static int open_power_control_status(struct inode *inode, struct file *file)
{
	return single_open(file, show_power_control_status, PDE_DATA(file_inode(file)));
}

/* ----- set HDD power control status ---- */
static ssize_t store_power_control_status(struct file *file, const char *buffer, size_t count, loff_t *offset)
{
	int tmpgpioval;
	struct sugi_hdd_info_st *hdd_info = (struct sugi_hdd_info_st *)PDE_DATA(file_inode(file));

	tmpgpioval = inl(ich10r_gpio_base_addr + hdd_info->HDD_GPIO_POWER_REG);

	if (strncmp(buffer, "off", strlen("off")) == 0) {
		if (tmpgpioval & hdd_info->HDD_POWER_BIT) {
			tmpgpioval &= hdd_info->HDD_POWER_MASK;
			outl(tmpgpioval, ich10r_gpio_base_addr + hdd_info->HDD_GPIO_POWER_REG);
			return count;
		}
	}
	
	if(strncmp(buffer, "on", strlen("on")) == 0){
		if (!(tmpgpioval & hdd_info->HDD_POWER_BIT)){
			tmpgpioval |= hdd_info->HDD_POWER_BIT;
			outl(tmpgpioval, ich10r_gpio_base_addr + hdd_info->HDD_GPIO_POWER_REG);
			return count;
		}
	}
	
		return -EINVAL;
}

#if HAVE_PROC_OPS
static struct proc_ops power_control_status_fops = {
    .proc_open		= open_power_control_status,
    .proc_write		= store_power_control_status,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release
};
#else
static struct file_operations power_control_status_fops = {
	.owner		= THIS_MODULE,
        .open		= open_power_control_status,
        .write		= store_power_control_status,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release
};
#endif

/* ----------- polling function ---------- */

static void SataHotplugPollingUpdatePinstat(void)
{
	unsigned int i = 0;
	
	for(i = 0; i < MAX_SUPPORTED_DISKS; i++)
	{
		sata_hotplug_data[i].presentpinstat = (inl(ich10r_gpio_base_addr + sugi_hdd_info[i].HDD_GPIO_PRESENT_REG) & sugi_hdd_info[i].HDD_PRESENT_BIT)? SATA_STAT_UNPLUGGED:SATA_STAT_PLUGGED;
	}
}

static void SataHotplugPolling(struct timer_list* __unused)
{
	unsigned int i = 0;
	char buf[32];

	SataHotplugPollingUpdatePinstat();
	for(i = 0; i < MAX_SUPPORTED_DISKS; i++)
	{
		if(sata_hotplug_data[i].prevplugstat == sata_hotplug_data[i].presentpinstat)
		{
			sata_hotplug_data[i].loops = SATA_POL_LOOPS;
			continue;
		}
		--sata_hotplug_data[i].loops;

		if(sata_hotplug_data[i].loops == 0)
		{
			if(sata_hotplug_data[i].presentpinstat == SATA_STAT_PLUGGED)
				sprintf(buf, PLUGGED_EVENT_MSG, i);
			else
				sprintf(buf, UNPLUGGED_EVENT_MSG, i);

			#ifdef CONFIG_BUFFALO_PLATFORM
				buffalo_kernevnt_queuein(buf);
			#else
				printk("EVENT IS OCCURRED\n");
			#endif
			
			sata_hotplug_data[i].loops = SATA_POL_LOOPS;
			sata_hotplug_data[i].prevplugstat = sata_hotplug_data[i].presentpinstat;
		}
	}
	sata_hotplug_polling_timer.expires = jiffies + SATA_POL_INTERVAL;
	add_timer(&sata_hotplug_polling_timer);
}

static unsigned long SataHotplugPollingStart(void)
{
	unsigned int i = 0;
	for(i = 0; i < MAX_SUPPORTED_DISKS; i++)
	{
		sata_hotplug_data[i].prevplugstat = SATA_STAT_UNKNOWN;
		sata_hotplug_data[i].loops = SATA_POL_LOOPS;
	}

		timer_setup(&sata_hotplug_polling_timer, SataHotplugPolling, 0);
		mod_timer(&sata_hotplug_polling_timer, jiffies + SATA_POL_INTERVAL);

        return 0;
}

/* ---------------------------------------------------------------------------------------------- *
 * BLOCK 2-4
 * Description: Implement functions of CMOS driver
 * ---------------------------------------------------------------------------------------------- */
static unsigned char checksum_caculation(unsigned char wol, unsigned char acp, unsigned char lan)
{
	return (CMOS_CHANGE_CHECKSUM - wol - (acp << 1) - (lan * 6));
}
static int smi_write(void)
{
	int smi_count = 0;
	int read_count = 0;
	
	for(smi_count = 0; smi_count < 3; smi_count++){
		outb(SW_SMI_WRITE, SW_SMI_OFFSET);
		for(read_count = 0; read_count < 10; read_count++){
			outb(CMOS_CHECKSUM_OFFSET, CMOS_INDEX_PORT);
			/* return error code of write "pass" or "fail" */
			if(inb(CMOS_DATA_PORT) == 0x40 || inb(CMOS_DATA_PORT) == 0xEF || inb(CMOS_DATA_PORT) == 0xFF)
				return inb(CMOS_DATA_PORT);
			mdelay(100);
		}
	}
	/* return 0 after sending SMI 3 times and waiting for 0.5s delay 10 times  */
	return 0;
}

static int smi_read(void)
{
	int smi_count = 0;
	int read_count = 0;
	
	/* clear 73h to 0 */
	outb(CMOS_CHECKSUM_OFFSET, CMOS_INDEX_PORT);
	while(inb(CMOS_DATA_PORT))
		outb(0x00, CMOS_DATA_PORT);
	
	for(smi_count = 0; smi_count < 3; smi_count++){
		outb(SW_SMI_READ, SW_SMI_OFFSET);
		for(read_count = 0; read_count < 10; read_count++){
			outb(CMOS_CHECKSUM_OFFSET, CMOS_INDEX_PORT);
			/* return error code of read "pass" or "fail" */
			if(inb(CMOS_DATA_PORT) == 0x40 || inb(CMOS_DATA_PORT) == 0xEF)
				return inb(CMOS_DATA_PORT);
			mdelay(100);
		}
	}
	/* return 0 after sending SMI 3 times and waiting for 0.5s delay 10 times  */
	return 0;
}

/* -------- wake on LAN attribute -------- */
static ssize_t show_wol_setting(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	
	if(bios_ver >= 209){
		ret = smi_read();
		
		switch(ret){
			case 0: 
				printk("BIOS_NO_RESPONSE\n");
				return sprintf(buf, "UNKNOWN\n");
			case 0xEF:
				printk("NVRAM_READ_FAIL\n");
				return sprintf(buf, "UNKNOWN\n");
			case 0x40:
				break;
			default:
				printk("UNKNOWN FAIL\n");
				return sprintf(buf, "UNKNOWN\n");
		}
	}
	
	outb(WOL_OFFSET, CMOS_INDEX_PORT);
	ret = inb(CMOS_DATA_PORT);
	
	if(ret == 0){
		return sprintf(buf, "off\n");
	}
	else if(ret == 1){
		return sprintf(buf, "on\n");
	}
	return sprintf(buf, "UNKNOWN\n");
}
static ssize_t store_wol_setting(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int len = 0;
	unsigned char wol_setting_value = 0;
	unsigned char power_setting_value = 0;
	unsigned char lanboot_setting_value = 0;
	unsigned char checksum_caculated = 0;
	
	while(buf[len])
		len++;
	
	if((len != 3) && (len != 4)){
		printk("WRONG_STRING\n");
		return count;
	}
	
	if(bios_ver >= 209){
		ret = smi_read();

		switch(ret){
			case 0: 
				printk("BIOS_NO_RESPONSE\n");
				return count;
			case 0xEF:
				printk("NVRAM_READ_FAIL\n");
				return count;
			case 0x40:
				outb(LANBOOT_OFFSET, CMOS_INDEX_PORT);
				lanboot_setting_value = inb(CMOS_DATA_PORT);
				break;
			default:
				printk("UNKNOWN FAIL\n");
				return count;
		}
	}
	
	outb(AC_POWER_LOSS_OFFSET, CMOS_INDEX_PORT);
	power_setting_value = inb(CMOS_DATA_PORT);
		
	if((len == 3) && (strncmp(buf, "on", strlen("on")) == 0)){
		outb(WOL_OFFSET, CMOS_INDEX_PORT);
		outb(WOL_ENABLE, CMOS_DATA_PORT);
		
		wol_setting_value = inb(CMOS_DATA_PORT);
		if(wol_setting_value == WOL_ENABLE){
			checksum_caculated = checksum_caculation(wol_setting_value, power_setting_value, lanboot_setting_value);
			outb(CMOS_CHECKSUM_OFFSET, CMOS_INDEX_PORT);
			outb(checksum_caculated, CMOS_DATA_PORT);
		}
		else
			printk("CMOS_VALUE_FAIL\n");
	}
	
	if((len == 4) && (strncmp(buf, "off", strlen("off")) == 0)){
		outb(WOL_OFFSET, CMOS_INDEX_PORT);
		outb(WOL_DISABLE, CMOS_DATA_PORT);
		
		wol_setting_value = inb(CMOS_DATA_PORT);
		if(wol_setting_value == WOL_DISABLE){
			checksum_caculated = checksum_caculation(wol_setting_value, power_setting_value, lanboot_setting_value);
			outb(CMOS_CHECKSUM_OFFSET, CMOS_INDEX_PORT);
			outb(checksum_caculated, CMOS_DATA_PORT);
		}
		else
			printk("CMOS_VALUE_FAIL\n");
	}

	if(bios_ver >=209){
		ret = smi_write();
		
		switch(ret){
			case 0: 
				printk("BIOS_NO_RESPONSE\n");
				break;
			case 0xEF:
				printk("NVRAM_CMOS_COMPARE_FAIL\n");
				break;
			case 0xFF:
				printk("NVRAM_WRITE_FAIL\n");
				break;
			case 0x40:
				break;
			default:
				printk("UNKNOWN FAIL\n");
				break;
		}
	}

	return count;
}

DEVICE_ATTR(wol_setting, 0644, show_wol_setting, store_wol_setting);

/* ------- AC power loss attribute ------- */
static ssize_t show_power_setting(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	
	if(bios_ver >= 209){
		ret = smi_read();
		
		switch(ret){
			case 0: 
				printk("BIOS_NO_RESPONSE\n");
				return sprintf(buf, "UNKNOWN\n");
			case 0xEF:
				printk("NVRAM_READ_FAIL\n");
				return sprintf(buf, "UNKNOWN\n");
			case 0x40:
				break;
			default:
				printk("UNKNOWN FAIL\n");
				return sprintf(buf, "UNKNOWN\n");
		}
	}
	
	outb(AC_POWER_LOSS_OFFSET, CMOS_INDEX_PORT);
	ret = inb(CMOS_DATA_PORT);
	
	if(ret==0){
		return sprintf(buf, "off\n");
	}
	else if(ret==1){
		return sprintf(buf, "on\n");
	}
	else if(ret==2){
		return sprintf(buf, "last state\n");
	}
	return sprintf(buf, "UNKNOWN\n");
}
static ssize_t store_power_setting(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int len = 0;
	unsigned char wol_setting_value = 0;
	unsigned char power_setting_value = 0;
	unsigned char lanboot_setting_value = 0;
	unsigned char checksum_caculated = 0;
	
	while(buf[len])
		len++;
		
//	if((len != 3) || (len != 4) || (len != 5)){
//		printk("WRONG_STRING [%d]\n", len);
//		return count;
//	}
		
	if(bios_ver >= 209){
		ret = smi_read();
		
		switch(ret){
			case 0: 
				printk("BIOS_NO_RESPONSE\n");
				return count;
			case 0xEF:
				printk("NVRAM_READ_FAIL\n");
				return count;
			case 0x40:
				outb(LANBOOT_OFFSET, CMOS_INDEX_PORT);
				lanboot_setting_value = inb(CMOS_DATA_PORT);
				break;
			default:
				printk("UNKNOWN FAIL\n");
				return count;
		}
	}
	
	outb(WOL_OFFSET, CMOS_INDEX_PORT);
	wol_setting_value = inb(CMOS_DATA_PORT);
	
//	if((len == 3) && (strncmp(buf, "on", strlen("on")) == 0)){
	if(strncmp(buf, "on", strlen("on")) == 0){
		outb(AC_POWER_LOSS_OFFSET, CMOS_INDEX_PORT);
		outb(AC_POWER_LOSS_POWER_ON, CMOS_DATA_PORT);
		
		if(bios_ver < 209){
			ret = pci_read_config_word(pdev, GEN_PMCON_3, &afterg3_en);
		afterg3_en &=(~0x0001);
			ret = pci_write_config_word(pdev, GEN_PMCON_3, afterg3_en);
		}
		
		power_setting_value = inb(CMOS_DATA_PORT);
		if(power_setting_value == AC_POWER_LOSS_POWER_ON){
			checksum_caculated = checksum_caculation(wol_setting_value, power_setting_value, lanboot_setting_value);
			outb(CMOS_CHECKSUM_OFFSET, CMOS_INDEX_PORT);
			outb(checksum_caculated, CMOS_DATA_PORT);
		}
		else
			printk("CMOS_VALUE_FAIL\n");
	}
	
//	if((len == 4) && (strncmp(buf, "off", strlen("off")) == 0)){
	if(strncmp(buf, "off", strlen("off")) == 0){
		outb(AC_POWER_LOSS_OFFSET, CMOS_INDEX_PORT);
		outb(AC_POWER_LOSS_POWER_OFF, CMOS_DATA_PORT);
		
		if(bios_ver < 209){
			ret = pci_read_config_word(pdev, GEN_PMCON_3, &afterg3_en);
		afterg3_en |= 0x0001;
			ret = pci_write_config_word(pdev, GEN_PMCON_3, afterg3_en);
		}
		
		power_setting_value = inb(CMOS_DATA_PORT);
		if(power_setting_value == AC_POWER_LOSS_POWER_OFF){
			checksum_caculated = checksum_caculation(wol_setting_value, power_setting_value, lanboot_setting_value);
			outb(CMOS_CHECKSUM_OFFSET, CMOS_INDEX_PORT);
			outb(checksum_caculated, CMOS_DATA_PORT);
		}
		else
			printk("CMOS_VALUE_FAIL\n");
	}

//	if((len == 5) && (strncmp(buf, "last", strlen("last")) == 0)){
	if(strncmp(buf, "last", strlen("last")) == 0){
		outb(AC_POWER_LOSS_OFFSET, CMOS_INDEX_PORT);
		outb(AC_POWER_LOSS_LAST_STATE, CMOS_DATA_PORT);
		
		if(bios_ver){
			ret = pci_read_config_word(pdev, GEN_PMCON_3, &afterg3_en);
		afterg3_en &= 0xFFFE;
			ret = pci_write_config_word(pdev, GEN_PMCON_3, afterg3_en);
		}
		
		power_setting_value = inb(CMOS_DATA_PORT);
		if(power_setting_value == AC_POWER_LOSS_LAST_STATE){
			checksum_caculated = checksum_caculation(wol_setting_value, power_setting_value, lanboot_setting_value);
			outb(CMOS_CHECKSUM_OFFSET, CMOS_INDEX_PORT);
			outb(checksum_caculated, CMOS_DATA_PORT);
		}
		else
			printk("CMOS_VALUE_FAIL\n");
	}

	if(bios_ver >=209){
		ret = smi_write();
		
		switch(ret){
			case 0: 
				printk("BIOS_NO_RESPONSE\n");
				break;
			case 0xEF:
				printk("NVRAM_CMOS_COMPARE_FAIL\n");
				break;
			case 0xFF:
				printk("NVRAM_WRITE_FAIL\n");
				break;
			case 0x40:
				break;
			default:
				printk("UNKNOWN FAIL\n");
				break;
		}
	}

	return count;
}

DEVICE_ATTR(power_setting, 0644, show_power_setting, store_power_setting);

/* -------- Gbe LAN boot attribute ------- */
static ssize_t show_lanboot_setting(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	
	if(bios_ver >= 209){
		ret = smi_read();
		
		switch(ret){
			case 0: 
				printk("BIOS_NO_RESPONSE\n");
				return sprintf(buf, "UNKNOWN\n");
			case 0xEF:
				printk("NVRAM_READ_FAIL\n");
				return sprintf(buf, "UNKNOWN\n");
			case 0x40:
				break;
			default:
				printk("UNKNOWN FAIL\n");
				return sprintf(buf, "UNKNOWN\n");
		}
	}
	
	outb(LANBOOT_OFFSET, CMOS_INDEX_PORT);
	ret = inb(CMOS_DATA_PORT);
	
	if(ret==0){
		return sprintf(buf, "off\n");
	}
	else if(ret==1){
		return sprintf(buf, "on\n");
	}

	return sprintf(buf, "UNKNOWN\n");
}
static ssize_t store_lanboot_setting(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int len = 0;
	unsigned char wol_setting_value = 0;
	unsigned char power_setting_value = 0;
	unsigned char lanboot_setting_value = 0;
	unsigned char checksum_caculated = 0;
	
	while(buf[len])
		len++;
	
	if((len != 3) && (len != 4)){
		printk("WRONG_STRING\n");
		return count;
	}
	
	ret = smi_read();
		
	switch(ret){
		case 0: 
			printk("BIOS_NO_RESPONSE\n");
			return count;
		case 0xEF:
			printk("NVRAM_READ_FAIL\n");
			return count;
		case 0x40:
			break;
		default:
			printk("UNKNOWN FAIL\n");
			return count;
	}
	
	outb(WOL_OFFSET, CMOS_INDEX_PORT);
	wol_setting_value = inb(CMOS_DATA_PORT);
	outb(AC_POWER_LOSS_OFFSET, CMOS_INDEX_PORT);
	power_setting_value = inb(CMOS_DATA_PORT);
		
	if((len == 3) && (strncmp(buf, "on", strlen("on")) == 0)){
		outb(LANBOOT_OFFSET, CMOS_INDEX_PORT);
		outb(LANBOOT_ENABLE, CMOS_DATA_PORT);
		
		lanboot_setting_value = inb(CMOS_DATA_PORT);
		if(lanboot_setting_value == LANBOOT_ENABLE){
			checksum_caculated = checksum_caculation(wol_setting_value, power_setting_value, lanboot_setting_value);
			outb(CMOS_CHECKSUM_OFFSET, CMOS_INDEX_PORT);
			outb(checksum_caculated, CMOS_DATA_PORT);
		}
		else
			printk("CMOS_VALUE_FAIL\n");
	}
	
	if((len == 4) && (strncmp(buf, "off", strlen("off")) == 0)){
		outb(LANBOOT_OFFSET, CMOS_INDEX_PORT);
		outb(LANBOOT_DISABLE, CMOS_DATA_PORT);
		
		lanboot_setting_value = inb(CMOS_DATA_PORT);
		if(lanboot_setting_value == LANBOOT_DISABLE){
			checksum_caculated = checksum_caculation(wol_setting_value, power_setting_value, lanboot_setting_value);
			outb(CMOS_CHECKSUM_OFFSET, CMOS_INDEX_PORT);
			outb(checksum_caculated, CMOS_DATA_PORT);
		}
		else
			printk("CMOS_VALUE_FAIL\n");
	}

	ret = smi_write();
		
	switch(ret){
		case 0: 
			printk("BIOS_NO_RESPONSE\n");
			break;
		case 0xEF:
			printk("NVRAM_CMOS_COMPARE_FAIL\n");
			break;
		case 0xFF:
			printk("NVRAM_WRITE_FAIL\n");
			break;
		case 0x40:
			break;
		default:
			printk("UNKNOWN FAIL\n");
			break;
	}

	return count;
}

DEVICE_ATTR(lanboot_setting, 0644, show_lanboot_setting, store_lanboot_setting);

/* ------- device number attribute ------- */
static ssize_t show_sugi_cmos_dev(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sugi_cmos_major);
}
DEVICE_ATTR(sugi_cmos_dev, 0444, show_sugi_cmos_dev, NULL);

/* ----------- PID attribute ------------ */
static ssize_t show_device_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i=0;
	char rv[32]; 
	
	for(i=0; i<=31; i++){
		outb(device_id_offset[i], CMOS_INDEX_PORT);
		rv[i] = inb(CMOS_DATA_PORT);
	}
	
	return sprintf(buf, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n",
	rv[0], rv[1], rv[2], rv[3], rv[4], rv[5], rv[6], rv[7], 
	rv[8], rv[9], rv[10], rv[11], rv[12], rv[13], rv[14], rv[15],
	rv[16], rv[17], rv[18], rv[19], rv[20], rv[21], rv[22], rv[23], 
	rv[24], rv[25], rv[26], rv[27], rv[28], rv[29], rv[30], rv[31]);
}

static ssize_t store_device_id(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int i;
	int len = 0;
	
	while(buf[len])
		len++;
	
	if(len != 33){
		printk("WRONG_STRING\n");
		return count;
	}
	
	if(len == 33){
		for(i = 0; i <= 31; i++){
			if((buf[i] < 32) || (buf[i] > 126)){
				printk("WRONG_STRING\n");
				return count;
			}
		}
	
		for(i = 0; i <= 31; i++){
			outb(device_id_offset[i], CMOS_INDEX_PORT);
			outb(buf[i], CMOS_DATA_PORT);
		}
	}
	
	return count;
}

DEVICE_ATTR(device_id, 0644, show_device_id, store_device_id);

/* --------- checksum attribute ---------- */
static ssize_t show_checksum(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i;
	unsigned int rv = 0;
	
	for(i=0; i<=3; i++) {
		outb(checksum_offset[i], CMOS_INDEX_PORT);
		rv |= inb(CMOS_DATA_PORT) << 8*(3-i);
	}
	
	return sprintf(buf, "%08x\n", rv);
}

static ssize_t store_checksum(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int i, j, tmp, len=0;
	unsigned char checksum_orig[8];
	unsigned char success_flag = 0;
	
	while(buf[len])
		len++;
	
	if(len != 9){
		printk("WRONG_STRING\n");
		return count;
	}
	
	else if(len == 9){
		for(i=0; i<=7; i++) {
			for(j=0; j<=21; j++) {
				if(buf[i] == strtoint_info[j].strg){
					checksum_orig[i] = strtoint_info[j].inte;
					success_flag = 1;
				}
				else if (j == 21 && success_flag == 0){
					printk("WRONG_STRING\n");
					return count;
				}
			}
			success_flag = 0;
		}
	
		for(i=0; i<=3; i++) {
			tmp = (checksum_orig[2*i] << 4) + checksum_orig[2*i+1];
			outb(checksum_offset[i], CMOS_INDEX_PORT);
			outb(tmp, CMOS_DATA_PORT);
		}
	}
	
	return count;
}

DEVICE_ATTR(checksum, 0644, show_checksum, store_checksum);

/* ---------------------------------------------------------------------------------------------- *
 * BLOCK 2-5
 * Description: Implement functions of board ID driver
 * ---------------------------------------------------------------------------------------------- */

/* ---------- board ID attribute --------- */
static ssize_t show_board_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(boardid_num==0)
		return sprintf(buf,"2HDD\n");
	else if(boardid_num==1)
		return sprintf(buf,"4HDD\n");
	else if(boardid_num==2)
		return sprintf(buf,"6HDD\n");
	else if(boardid_num==3)
		return sprintf(buf,"8HDD\n");
	else if(boardid_num==4)
		return sprintf(buf,"1U\n");
	
	return sprintf(buf, "Unknown\n");
}

DEVICE_ATTR(board_id, 0444, show_board_id, NULL);

/* ------- device number attribute ------- */
static ssize_t show_sugi_boardid_dev(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sugi_boardid_major);
}

DEVICE_ATTR(sugi_boardid_dev, 0444, show_sugi_boardid_dev, NULL);

/* ---------------------------------------------------------------------------------------------- *
 * BLOCK 3
 * Description: Chipsets and file system sheeting
 * ---------------------------------------------------------------------------------------------- */

/* ------- proc file system setting ------ */
static int init_proc_files(void)
{
	int i = 0;
	char buf[32];
	
	/* create /proc/buffalo */
#if ! defined(CONFIG_BUFFALO_PLATFORM)
	buffalo_dir = proc_mkdir(BUFFALO_DIR, NULL);
#endif
	
	/* create /proc/buffalo/gpio */
	gpio_dir = proc_mkdir(GPIO_DIR, NULL);
	
	
	/* button driver */
	/* create /proc/buffalo/gpio/switch */
	switch_dir = proc_mkdir(SWITCH_DIR, NULL);
	
	/* create /proc/buffalo/gpio/switch/func(display) */
	for(i = 0; i < MAX_SUPPORTED_BUTTONS; i++){
		switch_file[i] = proc_create_data(button_info[i].name, 0444, switch_dir, &button_status_fops, &button_info[i]);
}


	/* HDD control driver */
	/* create /proc/buffalo/gpio/hotplug */
	hotplug_dir = proc_mkdir(HOTPLUG_DIR, NULL);
	
	/* create /proc/buffalo/gpio/hotplug/sata# */
	for(i = 0; i < MAX_SUPPORTED_DISKS; i++){
		sprintf(buf, "sata%d", i);
		hotplug_sata_file[i] = proc_create_data(buf, 0444, hotplug_dir, &hotplug_status_fops, &sugi_hdd_info[i]);
}

	/* create /proc/buffalo/gpio/power_control */
	power_control_dir = proc_mkdir(POWER_CONTROL_DIR, NULL);

	/* create /proc/buffalo/gpio/power_control/hdd# */
	for(i = 0; i < MAX_SUPPORTED_DISKS; i++){
		sprintf(buf, "hdd%d", i);
		power_control_hdd_file[i] = proc_create_data(buf, 0644, power_control_dir, &power_control_status_fops, &sugi_hdd_info[i]);
	}	

	return 0;
}


/* ---------- sysfs file system ---------- */
static int init_sysfs_files(void)
{
	int rv;
	
	if(sugi_cls)
		return -EINVAL;

	/* create /sys/class/sugi */
	sugi_cls = class_create(THIS_MODULE, "sugi");


	/* fan control driver */
	/* create /sys/class/sugi/fan */
	fan_dev = device_create(sugi_cls, NULL, sugit_fan, NULL, "fan");
	
	/* create /sys/class/sugi/fan/temp# */
	rv = device_create_file(fan_dev, &dev_attr_temp1);
	rv = device_create_file(fan_dev, &dev_attr_temp2);
	rv = device_create_file(fan_dev, &dev_attr_temp3);
	
	/* create /sys/class/sugi/fan/fan#_rpm */
	rv = device_create_file(fan_dev, &dev_attr_fan1_rpm);
	rv = device_create_file(fan_dev, &dev_attr_fan2_rpm);
	rv = device_create_file(fan_dev, &dev_attr_fan3_rpm);
	rv = device_create_file(fan_dev, &dev_attr_fan4_rpm);
	
	/* create /sys/class/sugi/fan/fan#_speed */
	rv = device_create_file(fan_dev, &dev_attr_fan1_speed);
	rv = device_create_file(fan_dev, &dev_attr_fan2_speed);
	rv = device_create_file(fan_dev, &dev_attr_fan3_speed);
	rv = device_create_file(fan_dev, &dev_attr_fan4_speed);
    
	/* create /sys/class/sugi/fan/fan_pattern_speed */
	rv = device_create_file(fan_dev, &dev_attr_fan_stop_speed);
	rv = device_create_file(fan_dev, &dev_attr_fan_slow_speed);
	rv = device_create_file(fan_dev, &dev_attr_fan_fast_speed);
	rv = device_create_file(fan_dev, &dev_attr_fan_full_speed);
    
	/* create /sys/class/sugi/fan/show_speed_by string */
	rv = device_create_file(fan_dev, &dev_attr_show_speed_by_string);
    
	/* create /sys/class/sugi/fan/fan_threshold */
	rv = device_create_file(fan_dev, &dev_attr_fan_threshold);
    
	/* create /sys/class/sugi/fan/sugi_fan_dev */
	rv = device_create_file(fan_dev, &dev_attr_sugi_fan_dev);
	
	
	/* CMOS driver */
	/* create /sys/class/sugi/cmos */
	cmos_dev = device_create(sugi_cls, NULL, sugit_cmos, NULL, "cmos");
	
	/* create /sys/class/sugi/cmos/wol_setting */
	rv = device_create_file(cmos_dev, &dev_attr_wol_setting);
		
	/* create /sys/class/sugi/cmos/power_setting */
	rv = device_create_file(cmos_dev, &dev_attr_power_setting);
		
	/* create /sys/class/sugi/cmos/sugi_cmos_dev */
	rv = device_create_file(cmos_dev, &dev_attr_sugi_cmos_dev);
	
		/* create /sys/class/sugi/cmos/pid */
	rv = device_create_file(cmos_dev, &dev_attr_device_id);
	
	/* create /sys/class/sugi/cmos/checksum */
	rv = device_create_file(cmos_dev, &dev_attr_checksum);
  
  /* create /sys/class/sugi/cmos/lanboot_setting */
  /* this node and the related funciton will be 
     created if the BIOS version is newer than V209*/
  if(bios_ver >= 209)
  	rv = device_create_file(cmos_dev, &dev_attr_lanboot_setting);
  
	
	/* board ID driver */
	/* create /sys/class/sugi/board */
	boardid_dev = device_create(sugi_cls, NULL, sugit_boardid, NULL, "board");
	
	/* create /sys/class/sugi/board/board_id */
	rv = device_create_file(boardid_dev, &dev_attr_board_id);
		
	/* create /sys/class/sugi/board/sugi_boardid_dev */
	rv = device_create_file(boardid_dev, &dev_attr_sugi_boardid_dev);
	
	return 0;
}

/* ---------- BIOS version check --------- */
static int bios_version_check(void)
{
	int i;
	unsigned char* smbios_sm = NULL;
	unsigned char* smbios_dmi = NULL;
	unsigned char* smbios_eps = NULL;
	unsigned char* smbios_base = NULL;
	unsigned char* smbios_ver = NULL;
	unsigned int smbios_base_addr = 0;

	for(i = 0; i <= 0xFFF0; i += 0x01){
		smbios_sm = ioremap(0x000F0000 + i, 4);
		if(strncmp(smbios_sm, "_SM_", strlen("_SM_")) == 0){
			smbios_dmi = ioremap(0x000F0000 + i + 0x10, 4);
			if(strncmp(smbios_dmi, "_DMI_", strlen("_DMI_"))== 0){
				smbios_eps = ioremap(0x000F0000 + i + 0x18, 4);
				smbios_base_addr = ioread32(smbios_eps);
				break;
			}
		}
	}
	
	for(i = 0; i <= 0xFF; i += 0x01){
		smbios_base = ioremap(smbios_base_addr + i, 1);
		if(ioread8(smbios_base) == 0x33){
			if(strncmp(smbios_base, "31SUG", strlen("31SUG")) == 0){
				smbios_ver = ioremap(smbios_base_addr + i + 0x05, 1);
				bios_ver = simple_strtoul(smbios_ver, NULL, 10);
				iounmap(smbios_sm);
				iounmap(smbios_dmi);
				iounmap(smbios_eps);
				iounmap(smbios_base);
				iounmap(smbios_ver);
				printk("BIOS VERSION : V%d\n",bios_ver);
				return 0;
			}
		}
	}

	return 1;
}
/* ------------ IT8721 setting ----------- */
	
	/* button driver */
void SetBTNMBPnP(void)
{
	unsigned int base_msb;
	unsigned int base_lsb;
	
	/* select LDN 07h */	
	outb(IT87_LDN_ADDR, CONFIG_ADDR_PORT);
	outb(IT87_GPIO_LDN, CONFIG_DATA_PORT);
	
	/* find GPIO set3 base addr */
	outb(IT87_SIMPLE_IO_BASE_MSB, CONFIG_ADDR_PORT);
	base_msb = inb(CONFIG_DATA_PORT);
	outb(IT87_SIMPLE_IO_BASE_LSB, CONFIG_ADDR_PORT);
	base_lsb = inb(CONFIG_DATA_PORT);
	it87_gpioset3_base_addr = (base_msb*256 + base_lsb) + GPIO_SET3_OFFSET;
	
	/* enable to generate SMI of PBD's IRQ */
	outb(IT87_REG_SMI_CTL2, CONFIG_ADDR_PORT);
	outb(inb(CONFIG_DATA_PORT)|0x01, CONFIG_DATA_PORT);
	
	/* set GPIO34/35 as de-bounce GPIO */
	outb(IT87_REG_PNL_DBOUNCE0, CONFIG_ADDR_PORT);
	outb(IT87_GP34_LOC | IT87_IRQEN, CONFIG_DATA_PORT);
	
	outb(IT87_REG_PNL_DBOUNCE1, CONFIG_ADDR_PORT);
	outb(IT87_GP35_LOC, CONFIG_DATA_PORT);
	
	outb(IT87_REG_PNL_IRQ_SEL, CONFIG_ADDR_PORT);
	outb(IT87_PNL_IRQ3, CONFIG_DATA_PORT);
	}
	
/* fan control driver */
void SetFANMBPnP(void)
{
	unsigned short BASE_ADDR_EXT;
	unsigned short BASE_ADDR;

	outb(IT87_LDN_ADDR, CONFIG_ADDR_PORT);
	outb(IT87_EC_LDN, CONFIG_DATA_PORT);
	outb(0x60,CONFIG_ADDR_PORT);
	BASE_ADDR_EXT = inb(CONFIG_DATA_PORT)*256;
	outb(0x61,CONFIG_ADDR_PORT);
	BASE_ADDR = inb(CONFIG_DATA_PORT);
  EC_BASE_ADDR = BASE_ADDR_EXT+BASE_ADDR;
  IT87_EC_ADDR_REG = EC_BASE_ADDR+0x05;
  IT87_EC_DATA_REG = EC_BASE_ADDR+0x06;
}

/* ------------ ich10r setting ----------- */
static int ich10r_setting(void)
{
	int ret;
	
	pdev = pci_get_device(INTEL_ICH10R_VID, INTEL_ICH10R_DID, NULL);
	if(!pdev)
		return -ENODEV;
	
	ret = pci_read_config_dword(pdev, GPIOBASE, &ich10r_gpio_base_addr);
	if(ret)
		return -EINVAL;
		
	ret = pci_read_config_dword(pdev, PMBASE, &ich10r_pm_base_addr);
	if(ret)
		return -EINVAL;
		
	ich10r_gpio_base_addr &= 0x0000FF80;
	
	ich10r_pm_base_addr &= 0x0000FF80;
	
	/* Board ID driver */
	/* set GPIO pin 29, 30 ,31 as GPIO function */
	outl((inl(ich10r_gpio_base_addr+GP_USE_SEL)|0xE0000000), ich10r_gpio_base_addr+GP_USE_SEL);
	
	/* set GPIO pin 29, 30 ,31 as input */
	outl((inl(ich10r_gpio_base_addr+GP_IO_SEL)|0xE0000000), ich10r_gpio_base_addr+GP_IO_SEL);
	
	/* get board ID relative GPIO pins status */
	boardid_num = (inl(ich10r_gpio_base_addr+GP_LVL)&BOARD_ID_MASK) >> BOARD_ID_OFFSET;
	
	/* HDD control driver */
	/* set relative GPIO pins as GPIO function */
	outl((inl(ich10r_gpio_base_addr+GP_USE_SEL)|0x0002A6FD), ich10r_gpio_base_addr+GP_USE_SEL);
	outl((inl(ich10r_gpio_base_addr+GP_USE_SEL2)|0x01000007), ich10r_gpio_base_addr+GP_USE_SEL2);
	
	/* set hotplug relative GPIO pins as input */
	outl((inl(ich10r_gpio_base_addr+GP_IO_SEL)|0x0000A63C), ich10r_gpio_base_addr+GP_IO_SEL);
	
	/* set power control relative GPIO pins as outout */
	outl((inl(ich10r_gpio_base_addr+GP_IO_SEL)&0xFFFDFF3D), ich10r_gpio_base_addr+GP_IO_SEL);
	outl((inl(ich10r_gpio_base_addr+GP_IO_SEL2)&0xFEFFFFF8), ich10r_gpio_base_addr+GP_IO_SEL2);
	
	return 0;
}

/* ------------ initialization ----------- */
static int __init sugi_platform_init(void)
{  	
	int ret;
	
	/* request IRQ3 */
	ret = request_irq(3, sugi_interrupt, IRQF_SHARED, DRVNAME, sugi_interrupt);
  if(ret){
    printk("REQUEST IRQ3 FAIL\n");
		free_irq(3, 0);
    return -EBUSY;
  }
	
	/* chipsets initilization */
	/* ICH10R */
	ret = ich10r_setting();
	if(ret)
		return -ENODEV;
	/* IT8721 */	
	EnterMBPnP();
	SetFANMBPnP();
	//ExitMBPnP();
	
	//EnterMBPnP();
	SetBTNMBPnP();
	ExitMBPnP();
	
	/* register device to system */	
	ret = register_chrdev_region(sugit_fan, sugi_dev_count, DRVNAME);
	ret = register_chrdev_region(sugit_cmos, sugi_dev_count, DRVNAME);
	ret = register_chrdev_region(sugit_boardid, sugi_dev_count, DRVNAME);
	
	/* check BIOS verion */
	ret = bios_version_check();
	if(ret)
		return -EINVAL;
	
	/* create sysfs file system */
	ret = init_sysfs_files();
	if(ret)
		return -ENOMEM;
		
	/* create proc file system */
	ret = init_proc_files();
	if(ret)
		return -ENOMEM;
	
	SataHotplugPollingStart();
	
	printk("SUGI_PLATFORM MODULE HAS BEEN INITIALIZED\n");
	
	return 0;
}

/* ----------------- exit ---------------- */
static void __exit sugi_platform_exit(void)
{
	int i = 0;
	char buf[32];
	
	/* delete hotplug polling timer */
	del_timer(&sata_hotplug_polling_timer);
	
	/* disable and release IRQ3 */
	disable_irq(3);
	free_irq(3, sugi_interrupt);
	
	/* remove proc file system nodes */
	for(i = 0; i < MAX_SUPPORTED_BUTTONS; i++){
		remove_proc_entry(button_info[i].name, switch_dir);
	}
	
	for (i = 0; i < MAX_SUPPORTED_DISKS; i++){
		sprintf(buf, "hdd%d", i);
		remove_proc_entry(buf, power_control_dir);
	}
	
	for (i = 0; i < MAX_SUPPORTED_DISKS; i++){
		sprintf(buf, "sata%d", i);
		remove_proc_entry(buf, hotplug_dir);
	}
	
	remove_proc_entry(SWITCH_DIR, NULL);
	remove_proc_entry(HOTPLUG_DIR, NULL);
	remove_proc_entry(POWER_CONTROL_DIR, NULL);
	remove_proc_entry(GPIO_DIR, NULL);
	remove_proc_entry(BUFFALO_DIR, NULL);
	
	/* remove sysfs file system */
	device_remove_file(fan_dev, &dev_attr_temp1);
	device_remove_file(fan_dev, &dev_attr_temp2);
	device_remove_file(fan_dev, &dev_attr_temp3);
	device_remove_file(fan_dev, &dev_attr_fan1_rpm);
	device_remove_file(fan_dev, &dev_attr_fan2_rpm);
	device_remove_file(fan_dev, &dev_attr_fan1_speed);
	device_remove_file(fan_dev, &dev_attr_fan2_speed);
	device_remove_file(fan_dev, &dev_attr_fan_stop_speed);
	device_remove_file(fan_dev, &dev_attr_fan_slow_speed);
	device_remove_file(fan_dev, &dev_attr_fan_fast_speed);
	device_remove_file(fan_dev, &dev_attr_fan_full_speed);
	device_remove_file(fan_dev, &dev_attr_sugi_fan_dev);
	device_remove_file(fan_dev, &dev_attr_fan_threshold);
	device_remove_file(fan_dev, &dev_attr_show_speed_by_string);
	
	device_remove_file(cmos_dev, &dev_attr_wol_setting);
	device_remove_file(cmos_dev, &dev_attr_power_setting);
	device_remove_file(cmos_dev, &dev_attr_sugi_cmos_dev);
	device_remove_file(cmos_dev, &dev_attr_device_id);
	device_remove_file(cmos_dev, &dev_attr_checksum);
	
	if(bios_ver >= 209)
		device_remove_file(cmos_dev, &dev_attr_lanboot_setting);
	
	device_remove_file(boardid_dev, &dev_attr_sugi_boardid_dev);
	device_remove_file(boardid_dev, &dev_attr_board_id);
	
	device_destroy(sugi_cls, sugit_fan);
	device_destroy(sugi_cls, sugit_cmos);
	device_destroy(sugi_cls, sugit_boardid);
	
	class_destroy(sugi_cls);
	
	unregister_chrdev_region(sugit_fan, sugi_dev_count);
	unregister_chrdev_region(sugit_cmos, sugi_dev_count);
	unregister_chrdev_region(sugit_boardid, sugi_dev_count);
  
	printk("SUGI_PLATFORM MODULE HAS BEEN REMOVED\n");
}

module_init(sugi_platform_init);
module_exit(sugi_platform_exit);
MODULE_LICENSE("GPL");
