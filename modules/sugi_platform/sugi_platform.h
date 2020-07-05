#if !defined __SUGI_PLATFORM_H__
#define __SUGI_PLATFORM_H__

/* common section */
#define sugi_fan_major						125
#define sugi_fan_minor						0
#define sugi_cmos_major						124
#define sugi_cmos_minor						0
#define sugi_boardid_major				123
#define sugi_boardid_minor				0
#define sugi_dev_count        		1
#define DRVNAME								"sugi_platform"


/* ---------------------------------------------------------------------------------------------- *
 * BLOCK 1
 * Description: Definition of IT8721 registers
 * ---------------------------------------------------------------------------------------------- */
 
/* ----------- Common Sections ----------- */
 
/* IT8721 config section */
#define CONFIG_ADDR_PORT	       	0x2E
#define CONFIG_DATA_PORT	       	0x2F
#define IT87_LDN_ADDR		       		0x07
 
/* ------------ Button Driver ------------ */

/* IT8721 logic device number */
#define	IT87_GPIO_LDN							0x07
 
/* panel button de-bounce mapping */
#define IT87_REG_PNL_DBOUNCE0			0xE0
#define IT87_REG_PNL_DBOUNCE1			0xE1
#define IT87_GP34_LOC							0x1C
#define IT87_GP35_LOC							0x1D
#define	IT87_IRQEN								0x40

/* panel button de-bounce interrupt level select */
#define IT87_REG_PNL_IRQ_SEL			0x70
#define IT87_PNL_IRQ3							0x03	

/* SMI control register, bit0 is used to enable PBD's IRQ */
#define IT87_REG_SMI_CTL2 				0xF1
#define SMI_CTL2_EN_PBD						0x01

/* SMI status register2, bit7-6 is used to detect panel button de-bounce */
#define IT87_REG_SMI_STS2					0xF3
#define IT87_SMI_DEB_MASK					0xC0

/* SMI status register2, bit0 is PBD's IRQ */
#define IT87_SMI_STS2_PBD					0x01

/* Simple I/O base addr */
#define IT87_SIMPLE_IO_BASE_MSB		0x62
#define IT87_SIMPLE_IO_BASE_LSB		0x63
#define GPIO_SET3_OFFSET					0x2


/* ---------- Fan Control Driver --------- */

/* IT8721 logic device number */
#define	IT87_EC_LDN								0x04

/* IT8721 main fan control registers */
#define IT87_EC_FAN_MAIN_CTL	   	0x13
#define IT87_EC_FAN_CTL		       	0x14

/* IT8721 fan tachometer registers */
#define IT87_EC_FAN_TACO1	       	0x0D // FAN1
#define IT87_EC_FAN_TACO2	       	0x0E // FAN2

#define IT87_EC_FAN_TACO1_EXT	   	0x18
#define IT87_EC_FAN_TACO2_EXT	   	0x19

/* select PWM control mode  */
#define IT87_EC_FAN_PWM1					0x15
#define IT87_EC_FAN_PWM2					0x16

/* set the PWM value */
#define IT87_EC_FAN_SMART_PWM1		0x63
#define IT87_EC_FAN_SMART_PWM2		0x6B

/* IT8721 temprature registers */
#define IT87_EC_TMPIN3						0x2B // SYSTIN(TMPIN3@schematic)


/* ---------------------------------------------------------------------------------------------- *
 * BLOCK 2
 * Description: Definition of ICH10R registers
 * ---------------------------------------------------------------------------------------------- */ 
 
/* ----------- Common Sections ----------- */

/* vendor and device ID for Intel ICH10R */
#define INTEL_ICH10R_VID					0x8086
#define INTEL_ICH10R_DID					0x3A16
#define GPIOBASE									0x48
#define PMBASE										0x40

/* GPIO register*/
#define GP_USE_SEL								0x00
#define GP_USE_SEL2								0x30
#define GP_IO_SEL									0x04
#define GP_IO_SEL2								0x34
#define GP_LVL										0x0C
#define GP_LVL2										0x38

/* Power management register */
#define PMBASE										0x40
#define PM1_STS										0x00
#define PM1_EN										0x02
#define GEN_PMCON_3								0xA4


/* ---------------------------------------------------------------------------------------------- *
 * BLOCK 3
 * Description: Path of proc file system
 * ---------------------------------------------------------------------------------------------- */
 
/* ----------- Common Sections ----------- */
#define BUFFALO_DIR								"buffalo"
#define GPIO_DIR									"buffalo/gpio" 
 
/* ------------ Button Driver ------------ */
#define SWITCH_DIR								"buffalo/gpio/switch"
#define MAX_SUPPORTED_BUTTONS 		2

/* ---------- HDD Control Driver --------- */
#define HOTPLUG_DIR								"buffalo/gpio/hotplug"
#define POWER_CONTROL_DIR					"buffalo/gpio/power_control"
#define MAX_SUPPORTED_DISKS				8

/* ---------------------------------------------------------------------------------------------- *
 * BLOCK 4
 * Description: Definitions of individual drivers
 * ---------------------------------------------------------------------------------------------- */
 
/* ------------ Button Driver ------------ */

/* period of timer */      
#define TIMER_DELAY								(HZ/500)

/* Buttons section */
#define BTN_FUNC	    	   				0
#define BTN_DISP	 	  		 				1

/* select SMI status bit7-6 */
#define IT87_DEB0									0x40	
#define IT87_DEB1									0x80	
#define IT87_DEB0DEB1							0xC0

/* bit5-4 in GPIO set3 */
#define IT87_GP34_BIT4						0x10
#define IT87_GP35_BIT5						0x20

/* button information structure */
struct button_info_st
{
	char name[32];
	unsigned char reg_stat;
};

/* ---------- Fan Control Driver --------- */

/* fan control information */
struct fan_info_st{
	unsigned char reg;
	unsigned char reg_ext;
	unsigned char reg_smart_pwm;
	unsigned char reg_pwm;
	unsigned char offset;
};

/* ---------- HDD Control Driver --------- */

/*
    hotplug     |	power control
----------------------------------
sata0 -> GPIO2	|	hdd0 -> GPIO17
sata1 -> GPIO3	|	hdd1 -> GPIO1
sata2 -> GPIO4	|	hdd2 -> GPIO6
sata3 -> GPIO5	|	hdd3 -> GPIO7
sata4 -> GPIO15	|	hdd4 -> GPIO56
sata5 -> GPIO9	|	hdd5 -> GPIO32
sata6 -> GPIO13	|	hdd6 -> GPIO33
sata7 -> GPIO10	|	hdd7 -> GPIO34

*/

/* GPIO pin definition */
#define HDD0_PRESENT_BIT					0x00000004	/* GPIO2 */
#define HDD1_PRESENT_BIT					0x00000008	/* GPIO3 */
#define HDD2_PRESENT_BIT					0x00000010	/* GPIO4 */
#define HDD3_PRESENT_BIT					0x00000020	/* GPIO5 */
#define HDD4_PRESENT_BIT					0x00008000	/* GPIO15*/
#define HDD5_PRESENT_BIT					0x00000200	/* GPIO9 */
#define HDD6_PRESENT_BIT					0x00002000	/* GPIO13*/
#define HDD7_PRESENT_BIT					0x00000400	/* GPIO10*/

#define HDD0_POWER_BIT						0x00020000	/*GPIO17*/
#define HDD1_POWER_BIT						0x00000002	/*GPIO1*/
#define HDD2_POWER_BIT						0x00000040	/*GPIO6*/
#define HDD3_POWER_BIT						0x00000080	/*GPIO7*/
#define HDD4_POWER_BIT						0x01000000	/*GPIO56*/
#define HDD5_POWER_BIT						0x00000001	/*GPIO32*/
#define HDD6_POWER_BIT						0x00000002	/*GPIO33*/
#define HDD7_POWER_BIT						0x00000004	/*GPIO34*/

#define HDD0_POWER_MASK						0xFFFDFFFF
#define HDD1_POWER_MASK						0xFFFFFFFD
#define HDD2_POWER_MASK						0xFFFFFFBF
#define HDD3_POWER_MASK						0xFFFFFF7F
#define HDD4_POWER_MASK						0xFEFFFFFF
#define HDD5_POWER_MASK						0xFFFFFFFE
#define HDD6_POWER_MASK						0xFFFFFFFD
#define HDD7_POWER_MASK						0xFFFFFFFB


/* HDD information structure */
struct sugi_hdd_info_st {
	uint32_t HDD_GPIO_PRESENT_REG;
	uint32_t HDD_PRESENT_BIT;
	uint32_t HDD_GPIO_POWER_REG;
	uint32_t HDD_POWER_BIT;
	uint32_t HDD_POWER_MASK;
};

/* definitions of status */
#define PLUGGED_EVENT_MSG       	"SATA %d plugged"
#define UNPLUGGED_EVENT_MSG     	"SATA %d unplugged"

/* polling timer */
#define SATA_POL_INTERVAL       	HZ/100
#define SATA_POL_LOOPS          	10

/* hotplug ststus strcture */
typedef enum _sata_plug_state {
	SATA_STAT_UNKNOWN,
	SATA_STAT_PLUGGED,
	SATA_STAT_UNPLUGGED,
}SATA_PLUG_STATE;

/* hotplug information strcture */
struct sata_hotplug_data_st {
	SATA_PLUG_STATE presentpinstat;
	SATA_PLUG_STATE prevplugstat;
	unsigned int loops;
};

/* ------------- CMOS Driver ------------- */

/* IO index */
#define	CMOS_INDEX_PORT						0x70
#define	CMOS_DATA_PORT						0x71

/* wake on LAN setting */
#define WOL_OFFSET								0x70
#define WOL_DISABLE								0x00
#define WOL_ENABLE								0x01

/* AC power loss setting */
#define	AC_POWER_LOSS_OFFSET			0x71
#define AC_POWER_LOSS_POWER_OFF 	0x00
#define AC_POWER_LOSS_POWER_ON  	0x01
#define AC_POWER_LOSS_LAST_STATE	0x02

/* Gbe LAN boot setting */
#define LANBOOT_OFFSET						0x75
#define LANBOOT_DISABLE						0x00
#define LANBOOT_ENABLE						0x01

/* CMOS setting */
#define CMOS_CHECKSUM_OFFSET			0x73
#define CMOS_CHANGE_CHECKSUM			0xBF


/* ASCII string strcture */
struct strtoint_st {
	unsigned char strg;
	unsigned char inte;
};

/* SW SMI setting and return code */
#define SW_SMI_OFFSET							0xB2
#define SW_SMI_READ								0x76
#define SW_SMI_WRITE							0x77 

/* ----------- Board ID Driver ----------- */
 
/*
     SKU   | GPIO31   GPIO30   GPIO29 
 ----------|-------- -------- --------
  2HDD box |   0        0        0    
  4HDD box |   0        0        1    
  6HDD box |   0        1        0    
  8HDD box |   0        1        1    
  1U       |   1        0        0    
*/

#define GPIO29										0x20000000
#define GPIO30										0x40000000
#define GPIO31										0x80000000
#define BOARD_ID_MASK							(GPIO29 | GPIO30 | GPIO31)
#define BOARD_ID_OFFSET						29


#endif
