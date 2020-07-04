/*
 *  Driver routines for BUFFALO Platform
 *
 *  Copyright (C)  BUFFALO INC.
 *
 *  This software may be used and distributed according to the terms of
 *  the GNU General Public License (GPL), incorporated herein by reference.
 *  Drivers based on or derived from this code fall under the GPL and must
 *  retain the authorship, copyright and license notice.  This file is not
 *  a complete program and may only be used when the entire operating
 *  system is licensed under the GPL.
 *
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <asm/uaccess.h>

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

//#include "kernevntProc.h"
#include "buffalo/kernevnt.h"

/* Globals */
// same as Buffalo Kernel Ver.
#define BUFCORE_VERSION "0.16"

/* Module parameters */
MODULE_AUTHOR("BUFFALO");
MODULE_DESCRIPTION("Buffalo Platform Linux Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(BUFCORE_VERSION);

/* Definitions */
//#define DEBUG

#define USE_PROC_BUFFALO

// ----------------------------------------------------

/* Definitions for DEBUG */
#ifdef DEBUG
 #define FUNCTRACE(x)  x

#else
 #define FUNCTRACE(x)

#endif

/* Function prototypes */

/* variables */
int bfMagicKey = MagicKeyHwPoff;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
extern char saved_command_line[];
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21) */

static char bootver[32];
static char buffalo_series_name[32];
static char buffalo_product_name[32];
static char buffalo_product_id[32];

//----------------------------------------------------------------------
static int kernelfw_proc_show(struct seq_file *m, void *v)
{
	char *p, *pe;

	if (!(buffalo_series_name[0] >= 'a' && buffalo_series_name[0] <= 'z') &&
		!(buffalo_series_name[0] >= 'A' && buffalo_series_name[0] <= 'Z'))
	{
		memset(bootver, 0, sizeof(bootver));
		memset(buffalo_series_name, 0, sizeof(buffalo_series_name));
		memset(buffalo_product_name, 0, sizeof(buffalo_product_name));
		memset(buffalo_product_id, 0, sizeof(buffalo_product_id));

		p = strstr(saved_command_line,"BOOTVER=");
		if (p)
		{
			pe = strstr(p, " ");

			if(pe && (pe - p) < sizeof(bootver))
				strncpy(bootver, p, pe - p);
			else
				strncpy(bootver, p, sizeof(bootver));
		}
		else
			sprintf(bootver, "%s", "BOOTVER=Unknown");

		p = strstr(saved_command_line, "SERIES=");
		if (p)
		{
			p += strlen("SERIES=");
			pe = strstr(p, " ");

			if (pe && (pe - p) < sizeof(buffalo_series_name))
				strncpy(buffalo_series_name, p, pe - p);
			else
				strncpy(buffalo_series_name, p, sizeof(buffalo_series_name));
		}
		else
			sprintf(buffalo_series_name, "TeraStation");

		p = strstr(saved_command_line, "PRODUCTNAME=");
		if (p)
		{
			p += strlen("PRODUCTNAME=");
			pe = strstr(p, " ");

			if (pe && (pe - p) < sizeof(buffalo_product_name))
				strncpy(buffalo_product_name, p, pe - p);
			else
				strncpy(buffalo_product_name, p, sizeof(buffalo_product_name));
		}
		else
		{
#if defined(CONFIG_BUFFALO_MATSU_PLATFORM)
			sprintf(buffalo_product_name, "TS-QVHL/R5(SAIMEI)");
#elif defined(CONFIG_BUFFALO_KIRI_PLATFORM)
			sprintf(buffalo_product_name, "TS-2RZ/R6(KOHNINN)");
#elif defined(CONFIG_BUFFALO_USI_PLATFORM)
			p = strstr(saved_command_line, "ts4400=yes");
			if (p)
			{
				sprintf(buffalo_product_name, "TS4400(UDA)");
			}
			else
			{
				sprintf(buffalo_product_name, "TS5000(GENSHO)");
			}
#else
	#error
#endif
		}

		p = strstr(saved_command_line, "PRODUCTID=");
		if (p)
		{
			p += strlen("PRODUCTID=");
			pe = strstr(p, " ");

			if (pe && (pe - p) < sizeof(buffalo_product_id))
				strncpy(buffalo_product_id, p, pe - p);
			else
				strncpy(buffalo_product_id, p, sizeof(buffalo_product_id));
		}
		else
#if defined(CONFIG_BUFFALO_MATSU_PLATFORM)
			sprintf(buffalo_product_id, "0x00002013");
#elif defined(CONFIG_BUFFALO_KIRI_PLATFORM)
			sprintf(buffalo_product_id, "0x00002017");
#elif defined(CONFIG_BUFFALO_USI_PLATFORM)
			sprintf(buffalo_product_id, "0x00002019");
#else
	#error
#endif
	}

        seq_printf(m, "SERIES=%s\n", buffalo_series_name);
        seq_printf(m, "PRODUCTNAME=%s\n", buffalo_product_name);

        seq_printf(m, "VERSION=%s\n",BUFCORE_VERSION);
        seq_printf(m, "SUBVERSION=FLASH 0.00\n");

        seq_printf(m, "PRODUCTID=%s\n", buffalo_product_id);

	seq_printf(m, "BUILDDATE=2015/02/25 17:59:58\n");
        seq_printf(m, "%s\n",bootver);

        return 0;
}

static int kernelfw_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, kernelfw_proc_show, NULL);
}

static const struct file_operations buffalo_kernelfw_proc_fops = {
	.owner		= THIS_MODULE,
	.open           = kernelfw_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release
};

#if defined(CONFIG_BUFFALO_USE_MICON) || defined(CONFIG_BUFFALO_MATSU_PLATFORM)
//----------------------------------------------------------------------
static int micon_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "on\n");
	return 0;
}

static int micon_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, micon_proc_show, NULL);
}

static const struct file_operations buffalo_micon_proc_fops = {
	.owner          = THIS_MODULE,
	.open           = micon_proc_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int miconint_proc_show(struct seq_file *m, void *v)
{
	return 0;
}

static int miconint_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, miconint_proc_show, NULL);
}

static const struct file_operations  buffalo_miconint_proc_fops = {
	.owner          = THIS_MODULE,
	.open           = miconint_proc_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
}
#endif

//----------------------------------------------------------------------
// begin booting
int buffalo_booting = 1;

static int booting_read_proc(struct file *fp, char *data, size_t count, loff_t *off)
{
	int len = 0;

    if(!*off) {
        char buffer[32];
	    len = sprintf(buffer, "%d\n", buffalo_booting);

        len=count-*off>=len?len:count-*off;
        if(len>0) {
            memcpy(data,buffer,len);
        }
        *off=len;
    }
	return len;
}

static int booting_write_proc(struct file *fp, const char *data, size_t count, loff_t *off)
{
    if(count>=2) {
	    if (strncmp(data, "0\n", count) == 0)
	    {
		    buffalo_booting = 0;
	    }
	    else if (strncmp(data, "1\n", count) == 0)
	    {
		    buffalo_booting = 1;
	    }
    }
    else {
		printk(">%s: short buffer.", __FUNCTION__);
    }
	return count;
}

static const struct file_operations booting_entry_fops =
{
	.owner = THIS_MODULE,
	.read = &booting_read_proc,
	.write = &booting_write_proc,
};
// end booting

static int
bfGetMagicKey(void)
{
	return bfMagicKey;
}

static void
bfSetMagicKey(int key)
{ 
	bfMagicKey = key;
}

static int
BuffaloCpuStatusReadProc(struct file *fp, char *data, size_t count, loff_t *off)
{
	int len = 0;

    if(!*off) {
        char page[32];
	    unsigned int CpuStatus = bfGetMagicKey();

	switch(CpuStatus){
	case MagicKeyReboot:
		len = sprintf(page, "reboot\n");
		break;
	case MagicKeyRebootUbootPassed:
		len = sprintf(page, "reboot_uboot_passed\n");
		break;
	case MagicKeyNormalState:
		len = sprintf(page, "normal_state\n");
		break;
	case MagicKeyHwPoff:
		len = sprintf(page, "hwpoff\n");
		break;
	case MagicKeySwPoff:
		len = sprintf(page, "swpoff\n");
		break;
	case MagicKeySWPoffUbootPassed:
		len = sprintf(page, "swpoff_uboot_passed\n");
		break;
	case MagicKeyFWUpdating:
		len = sprintf(page, "fwup\n");
		break;
	case MagicKeyUpsShutdown:
		len = sprintf(page, "ups_shutdown\n");
		break;
	case MagicKeyWOLReadyState:
		len = sprintf(page, "WOLReady\n");
		break;
	default:
		len = sprintf(page, "Unknown(CpuStatus=%d)\n", CpuStatus);
		break;
	}

            len=count-*off>=len?len:count-*off;
            if(len>0) {
                memcpy(data+*off,page,len);
            }
            *off=len;
    }
    return(len);
}

static int BuffaloCpuStatusWriteProc(struct file *fp, const char *data, size_t count, loff_t *off)
{
	char status[256];
	if (count > 256)
		return count;

	memcpy(status, data, count);

	if (status[count - 1] == '\n')
		status[count - 1] = '\0';
	else
		status[count] = '\0';

	if(strcmp(status, "reboot") == 0){
		bfSetMagicKey(MagicKeyReboot);
	}else if(strcmp(status, "reboot_uboot_passed") == 0){
		bfSetMagicKey(MagicKeyRebootUbootPassed);
	}else if(strcmp(status, "normal_state") == 0){
		bfSetMagicKey(MagicKeyNormalState);
	}else if(strcmp(status, "hwpoff") == 0){
		bfSetMagicKey(MagicKeyHwPoff);
	}else if(strcmp(status, "swpoff") == 0){
		bfSetMagicKey(MagicKeySwPoff);
	}else if(strcmp(status, "swpoff_uboot_passed") == 0){
		bfSetMagicKey(MagicKeySWPoffUbootPassed);
	}else if(strcmp(status, "fwup") == 0){
		bfSetMagicKey(MagicKeyFWUpdating);
	}else if(strcmp(status, "ups_shutdown") == 0){
		bfSetMagicKey(MagicKeyUpsShutdown);
	}else if(strcmp(status, "WOLReady") == 0){
		bfSetMagicKey(MagicKeyWOLReadyState);
	}else{

	}

	return count;
}

static const struct file_operations cpu_status_entry_fops =
{		
	.owner = THIS_MODULE,
	.read = &BuffaloCpuStatusReadProc,
	.write = &BuffaloCpuStatusWriteProc,
};


struct proc_dir_entry *
get_proc_buffalo(void)
{
	static struct proc_dir_entry *buffalo = NULL;

	if (buffalo == NULL)
		buffalo = proc_mkdir("buffalo", NULL);

	return buffalo;
}

//----------------------------------------------------------------------
int __init buffaloDriver_init (void)
{
        struct proc_dir_entry *generic, *buffalo, *booting;
        FUNCTRACE(printk(">%s\n",__FUNCTION__));

	buffalo = get_proc_buffalo();
	proc_create("firmware", 0, buffalo, &buffalo_kernelfw_proc_fops);

#if defined(CONFIG_BUFFALO_USE_MICON) || defined(CONFIG_BUFFALO_MATSU_PLATFORM)
        proc_create("micon", 0, buffalo, &buffalo_micon_proc_fops);
	proc_create("miconint_en", 0, buffalo, &buffalo_miconint_proc_fops);
#endif

	proc_create("cpu_status", 0, buffalo, &cpu_status_entry_fops);

	proc_create("booting", 0, buffalo, &booting_entry_fops);

	//BuffaloKernevnt_init();

	return 0;
}

//----------------------------------------------------------------------
void buffaloDriver_exit(void)
{
	FUNCTRACE(printk(">%s\n",__FUNCTION__));

	//BuffaloKernevnt_exit();
	remove_proc_entry("buffalo/booting", 0);
	remove_proc_entry("buffalo/cpu_status", 0);
#if defined(CONFIG_BUFFALO_USE_MICON) || defined(CONFIG_BUFFALO_MATSU_PLATFORM)
	remove_proc_entry("buffalo/miconint_en", 0);
	remove_proc_entry("buffalo/micon", 0);
#endif
	remove_proc_entry("buffalo/firmware", 0);
	remove_proc_entry ("buffalo", 0);
}

module_init(buffaloDriver_init);
module_exit(buffaloDriver_exit);
