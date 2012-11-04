/*
 *
 * Copyright (C) 2008 Texas Instruments Inc.
 * Vikram Pandita <vikram.pandita@ti.com>
 *
 * Modified from mach-omap2/board-ldp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/zforce.h>

#ifdef CONFIG_INPUT_KXTF9
#include <linux/kxtf9.h>
#define KXTF9_DEVICE_ID			"kxtf9"
#define KXTF9_I2C_SLAVE_ADDRESS		0x0F
#define KXTF9_GPIO_FOR_PWR		34
#define	KXTF9_GPIO_FOR_IRQ		113
#endif /* CONFIG_INPUT_KXTF9 */

#ifdef CONFIG_BATTERY_MAX17042
#include <linux/max17042.h>
#endif

#ifdef CONFIG_MAX9635
#include <linux/max9635.h>
#define MAX9635_I2C_SLAVE_ADDRESS   MAX9635_I2C_SLAVE_ADDRESS1
#define MAX9635_GPIO_FOR_IRQ        0
#define MAX9635_DEFAULT_POLL_INTERVAL 1000
#endif

#if defined(CONFIG_TOUCHSCREEN_ZFORCE) || defined(CONFIG_TOUCHSCREEN_ZFORCE_MODULE)
#include <linux/zforce.h>
#define ZFORCE_I2C_SLAVE_ADDRESS   0x50
#define ZFORCE_GPIO_FOR_IRQ        113
#endif

#ifdef CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C
#include <linux/i2c/pmic-tps65185-i2c.h>
#endif

#define GOSSAMER_CHARGE_ENAB_GPIO 44
#define GOSSAMER_CHARGE_ILM1_GPIO 45
#define GOSSAMER_CHARGE_ILM2_GPIO 61

#define GOSSAMER_PRE1C_CHARGE_ENAB_GPIO 110
#define GOSSAMER_PRE1C_CHARGE_ILM1_GPIO 102
#define GOSSAMER_PRE1C_CHARGE_ILM2_GPIO 61


#define GOSSAMER_DEBUG_LED_GPIO   90

#include <linux/spi/spi.h>
#include <linux/i2c/twl4030.h>
#include <linux/interrupt.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/switch.h>
#include <linux/dma-mapping.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/board-boxer.h>
#include <mach/mcspi.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/common.h>
#include <mach/gpmc.h>
#if 0
#include <mach/hsmmc.h>
#endif
#include <mach/usb.h>
#include <mach/mux.h>

#include <asm/system.h> // For system_serial_high & system_serial_low
#include <asm/io.h>
#include <asm/delay.h>
#include <mach/control.h>
#include <mach/sram.h>

#include <mach/display.h>
#include <linux/i2c/twl4030.h>

#include <linux/usb/android.h>

#include "mmc-twl4030.h"
#include "omap3-opp.h"
#include "prcm-common.h"
#include "prm.h"

#include "sdram-hynix-h5ms2g22mfr.h"

#include <media/v4l2-int-device.h>

#ifdef CONFIG_PM
#include <../drivers/media/video/omap/omap_voutdef.h>
#endif

#ifndef CONFIG_TWL4030_CORE
#error "no power companion board defined!"
#endif

#ifdef CONFIG_WL127X_RFKILL
#include <linux/wl127x-rfkill.h>
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#include <linux/bootmem.h>
#endif

#ifdef CONFIG_LEDS_AS3676
#include <linux/leds-as3676.h>
#endif
#define DEFAULT_BACKLIGHT_BRIGHTNESS 105

#ifdef CONFIG_TOUCHSCREEN_CYTTSP_I2C
/* tma340 i2c address */
#define CYTTSP_I2C_SLAVEADDRESS	34
#define OMAP_CYTTSP_GPIO	99
#define OMAP_CYTTSP_RESET_GPIO 	46
#endif

#define CONFIG_DISABLE_HFCLK 1
#define ENABLE_VAUX1_DEDICATED	0x03
#define ENABLE_VAUX1_DEV_GRP	0x20

#define ENABLE_VAUX3_DEDICATED  0x03
#define ENABLE_VAUX3_DEV_GRP	0x20
#define TWL4030_MSECURE_GPIO	22

#define WL127X_BTEN_GPIO	60

#define GOSSAMER_EXT_QUART_PHYS	0x48000000
#define GOSSAMER_EXT_QUART_VIRT	0xfa000000
#define GOSSAMER_EXT_QUART_SIZE	SZ_256

/* Define for Gossamer keyboard: */
static int gossamer_twl4030_keymap[] = {
#ifdef CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C
	KEY(0, 0, KEY_NEXT),
	KEY(0, 1, KEY_BACK),
	KEY(0, 2, KEY_PREVIOUS),
	KEY(0, 3, KEY_MENU),
///external build key mapping - KEY(0, 0, KEY_MENU),KEY(0, 1, KEY_BACK),KEY(0, 2, KEY_VOLUMEUP),KEY(0, 3, KEY_VOLUMEDOWN),
#else
	KEY(0, 0, KEY_HOME),
	KEY(1, 0, KEY_MENU),
	KEY(0, 1, KEY_BACK),
	KEY(2, 2, KEY_VOLUMEUP),
	KEY(3, 2, KEY_VOLUMEDOWN),
#endif /* CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C */
	0
};

#ifdef CONFIG_LEDS_AS3676
static struct as3676_platform_data as3676_pdata = {
	.pwronkey_shutdown_msec = 6500, /* long-press to shutdown in 6.5s */
	.step_up_frequ = 1,	/* 1 = 500 kHz */
	.step_up_vtuning = 0x0E,       /* 0x0E = 14uA on DCDC_FB */
	.leds[0] = {
		.name = "lcd-backlight",
		.on_charge_pump = 0,
		.max_current_uA = 10000,
	},
	.leds[1] = {
		.name = "lcd-backlight2",
		.on_charge_pump = 0,
		.max_current_uA = 10000,
	}
};
#endif

static struct twl4030_keypad_data gossamer_kp_twl4030_data = {
	.rows		= 8,
	.cols		= 8,
	.keymap		= gossamer_twl4030_keymap,
	.keymapsize	= ARRAY_SIZE(gossamer_twl4030_keymap),
	.rep		= 0, /* Need to detect long key presses */
};

#ifdef CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C
static struct gpio_keys_button gossamer_gpio_buttons[] = {
	{
		.code			= KEY_POWER,	
		.gpio			= 14,
		.desc			= "POWER",
		.active_low		= 0,
		.wakeup			= 1,
	},
	{
		.code			= KEY_HOME,
		.gpio			= 48,
		.desc			= "key-home",
		.active_low		= 1,
		.wakeup			= 1,
	},
};

static struct gpio_led debug_leds[] = {
	{ /* LED90 */
		.name			= "led90",
		.gpio			= GOSSAMER_DEBUG_LED_GPIO,
		.active_low		= 0,
		.default_trigger	= "none",
	}
};

static struct gpio_keys_platform_data gossamer_gpio_key_info = {
	.buttons	= gossamer_gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gossamer_gpio_buttons),
//	.rep		= 1,		/* auto-repeat */
};

static struct platform_device gossamer_keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gossamer_gpio_key_info,
	},
};
#endif /* CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C */

#define t2_out(c, r, v) twl4030_i2c_write_u8(c, r, v)

#ifdef CONFIG_FB_OMAP2
static struct resource gossamer_vout_resource[3 - CONFIG_FB_OMAP2_NUM_FBS] = {
};
#else
static struct resource gossamer_vout_resource[2] = {
};
#endif

#ifdef CONFIG_PM
struct vout_platform_data gossamer_vout_data = {
	.set_min_bus_tput = omap_pm_set_min_bus_tput,
	.set_max_mpu_wakeup_lat =  omap_pm_set_max_mpu_wakeup_lat,
	.set_vdd1_opp = omap_pm_set_min_mpu_freq,
	.set_cpu_freq = omap_pm_cpu_set_freq,
};
#endif

static struct platform_device gossamer_vout_device = {
	.name		= "omap_vout",
	.num_resources	= ARRAY_SIZE(gossamer_vout_resource),
	.resource	= &gossamer_vout_resource[0],
	.id		= -1,
#ifdef CONFIG_PM
	.dev		= {
		.platform_data = &gossamer_vout_data,
	}
#else
	.dev		= {
		.platform_data = NULL,
	}
#endif
};

/* This is just code left here to leverage on this for Maxim battery charger*/
#if defined(CONFIG_REGULATOR_BQ24073) || defined(CONFIG_REGULATOR_BQ24073_MODULE)
static struct bq24073_mach_info bq24073_init_dev_data = {
	.gpio_nce = GOSSAMER_CHARGE_ENAB_GPIO,
	.gpio_en1 = GOSSAMER_CHARGE_ILM1_GPIO,
	.gpio_en2 = GOSSAMER_CHARGE_ILM2_GPIO,
	.gpio_nce_state = 1,
	.gpio_en1_state = 0,
	.gpio_en2_state = 0,
};

static struct regulator_consumer_supply bq24073_vcharge_supply = {
       .supply         = "bq24073",
};

static struct regulator_init_data bq24073_init  = {

       .constraints = {
               .min_uV                 = 0,
               .max_uV                 = 5000000,
               .min_uA                 = 0,
               .max_uA                 = 1500000,
               .valid_modes_mask       = REGULATOR_MODE_NORMAL
                                       | REGULATOR_MODE_STANDBY,
               .valid_ops_mask         = REGULATOR_CHANGE_CURRENT
                                       | REGULATOR_CHANGE_MODE
                                       | REGULATOR_CHANGE_STATUS,
               .boot_on                = 0,
               .always_on              = 0,
	       .state_mem = {
		       .enabled = 1,
	       },
       },
       .num_consumer_supplies  = 1,
       .consumer_supplies      = &bq24073_vcharge_supply,

       .driver_data = &bq24073_init_dev_data,
};

/* GPIOS need to be in order of BQ24073 */
static struct platform_device gossamer_curr_regulator_device = {
	.name           = "bq24073", /* named after init manager for ST */
	.id             = -1,
	.dev 		= {
		.platform_data = &bq24073_init,
	},
};
#endif /* CONFIG_REGULATOR_BQ24073 */

#ifdef CONFIG_CHARGER_MAX8903
static struct platform_device max8903_charger_device = {
	.name		= "max8903_charger",
	.id		= -1,
};
#endif

/* Use address that is most likely unused and untouched by u-boot */
#define GOSSAMER_RAM_CONSOLE_START 0x8e000000
#define GOSSAMER_RAM_CONSOLE_SIZE (0x20000)

static struct resource gossamer_ram_console_resource[] = {
    {
        .start  = GOSSAMER_RAM_CONSOLE_START,
        .end    = GOSSAMER_RAM_CONSOLE_START + GOSSAMER_RAM_CONSOLE_SIZE - 1,
        .flags  = IORESOURCE_MEM,
    }
};

static struct platform_device gossamer_ram_console_device = {
    .name           = "ram_console",
    .id             = 0,
    .num_resources  = ARRAY_SIZE(gossamer_ram_console_resource),
    .resource       = gossamer_ram_console_resource,
};

static struct platform_device *gossamer_devices[] __initdata = {
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	&gossamer_ram_console_device,
#endif
#ifdef CONFIG_WL127X_RFKILL
//	&gossamer_wl127x_device,
#endif
#ifdef CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C
	&gossamer_keys_gpio,
#endif /* CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C */
	&gossamer_vout_device,
#if defined(CONFIG_REGULATOR_BQ24073) || defined(CONFIG_REGULATOR_BQ24073_MODULE)
	&gossamer_curr_regulator_device,
#endif /* CONFIG_REGULATOR_BQ24073 */
#ifdef CONFIG_CHARGER_MAX8903
//	&max8903_charger_device,
#endif
};

static void __init omap_gossamer_init_irq(void)
{
	omap_init_irq();
	omap2_init_common_hw(h5ms2g22mfr_sdrc_params,
			     omap3621_mpu_rate_table,
			     omap3621_dsp_rate_table,
			     omap3621_l3_rate_table);
}

static struct regulator_consumer_supply gossamer_vmmc1_supply = {
	.supply		= "vmmc",
};

static struct regulator_consumer_supply gossamer_vsim_supply = {
	.supply		= "vmmc_aux",
};

static struct regulator_consumer_supply gossamer_vmmc2_supply = {
	.supply		= "vmmc",
};

/* VMMC1 for OMAP VDD_MMC1 (i/o) and MMC1 card */
static struct regulator_init_data gossamer_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &gossamer_vmmc1_supply,
};

/* VMMC2 for MMC2 card */
static struct regulator_init_data gossamer_vmmc2 = {
	.constraints = {
		.min_uV			= 1850000,
		/* This is a virtual regulator on TPS65921, so we can
		 * write anything that will please regulator FW.
		 * Apparently ROM code sometimes leaves VMMC2=3.15V,
		 * yet we restrict it to 1.8V. This causes the regulator
		 * framework to reject all updates, and more importantly,
		 * to return errors for all voltage orations.
		 */
		.max_uV			= 3150000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &gossamer_vmmc2_supply,
};

/* VSIM for OMAP VDD_MMC1A (i/o for DAT4..DAT7) */
static struct regulator_init_data gossamer_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &gossamer_vsim_supply,
};

static struct twl4030_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.wires		= 4,
		.gpio_wp	= -EINVAL,
	},
	{
		.mmc		= 2,
		.wires		= 8,
		.gpio_wp	= -EINVAL,
	},
	{
		.mmc		= 3,
		.wires		= 4,
		.gpio_wp	= -EINVAL,
	},
	{}      /* Terminator */
};

static int __ref gossamer_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ),
	 * gpio + 1 is "mmc1_cd" (input/IRQ)
	 */
	mmc[0].gpio_cd = gpio + 0;
	mmc[1].gpio_cd = gpio + 1;
	twl4030_mmc_init(mmc);

	/* link regulators to MMC adapters ... we "know" the
	 * regulators will be set up only *after* we return.
	*/
	gossamer_vmmc1_supply.dev = mmc[0].dev;
	gossamer_vsim_supply.dev = mmc[0].dev;
	gossamer_vmmc2_supply.dev = mmc[1].dev;

	return 0;
}

static struct omap_uart_config gossamer_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3)),
};

static struct omap_board_config_kernel gossamer_config[] __initdata = {
	{ OMAP_TAG_UART,	&gossamer_uart_config },
};

static struct twl4030_usb_data gossamer_usb_data = {
      .usb_mode	= T2_USB_MODE_ULPI,
#if defined(CONFIG_REGULATOR_BQ24073) || defined(CONFIG_REGULATOR_BQ24073_MODULE)
      .bci_supply     = &bq24073_vcharge_supply,
#endif /* CONFIG_REGULATOR_BQ24073 */
};
static struct twl4030_gpio_platform_data gossamer_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.setup		= gossamer_twl_gpio_setup,
};

static struct twl4030_madc_platform_data gossamer_madc_data = {
	.irq_line	= 1,
};

/*
 * Sequence to control the TRITON Power resources,
 * when the system goes into sleep.
 * Executed upon P1_P2/P3 transition for sleep.
 */
static struct twl4030_ins __initdata sleep_on_seq[] = {
	/* Broadcast message to put res to sleep */
	{MSG_BROADCAST(DEV_GRP_ALL, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R1,
							RES_STATE_SLEEP), 2},
	{MSG_BROADCAST(DEV_GRP_ALL, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R2,
							RES_STATE_SLEEP), 2},
};

static struct twl4030_script sleep_on_script __initdata = {
	.script	= sleep_on_seq,
	.size	= ARRAY_SIZE(sleep_on_seq),
	.flags	= TRITON_SLEEP_SCRIPT,
};

/*
 * Sequence to control the TRITON Power resources,
 * when the system wakeup from sleep.
 * Executed upon P1_P2 transition for wakeup.
 */
static struct twl4030_ins wakeup_p12_seq[] __initdata = {
	/* Broadcast message to put res to active */
	{MSG_BROADCAST(DEV_GRP_ALL, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R1,
							RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_p12_script __initdata = {
	.script	= wakeup_p12_seq,
	.size	= ARRAY_SIZE(wakeup_p12_seq),
	.flags	= TRITON_WAKEUP12_SCRIPT,
};

/*
 * Sequence to control the TRITON Power resources,
 * when the system wakeup from sleep.
 * Executed upon P3 transition for wakeup.
 */
static struct twl4030_ins wakeup_p3_seq[] __initdata = {
	/* Broadcast message to put res to active */
	{MSG_BROADCAST(DEV_GRP_ALL, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R2,
							RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_p3_script __initdata = {
	.script = wakeup_p3_seq,
	.size   = ARRAY_SIZE(wakeup_p3_seq),
	.flags  = TRITON_WAKEUP3_SCRIPT,
};

/*
 * Sequence to reset the TRITON Power resources,
 * when the system gets warm reset.
 * Executed upon warm reset signal.
 */
static struct twl4030_ins wrst_seq[] __initdata = {
/*
 * Reset twl4030.
 * Reset Main_Ref.
 * Reset All type2_group2.
 * Reset VUSB_3v1.
 * Reset All type2_group1.
 * Reset RC.
 * Reenable twl4030.
 */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_Main_Ref, RES_STATE_WRST), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R2,
							RES_STATE_WRST), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_VUSB_3V1, RES_STATE_WRST), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R1,
							RES_STATE_WRST), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_RC, RES_TYPE_ALL, RES_TYPE2_R0,
							RES_STATE_WRST), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wrst_script __initdata = {
	.script = wrst_seq,
	.size   = ARRAY_SIZE(wrst_seq),
	.flags  = TRITON_WRST_SCRIPT,
};

/* TRITON script for sleep, wakeup & warm_reset */
static struct twl4030_script *twl4030_scripts[] __initdata = {
	&wakeup_p12_script,
	&wakeup_p3_script,
	&sleep_on_script,
	&wrst_script,
};

static struct twl4030_resconfig twl4030_rconfig[] __initdata = {
	{ .resource = RES_VPLL1, .devgroup = DEV_GRP_P1, .type = 3,
		.type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_VINTANA1, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VINTANA2, .devgroup = DEV_GRP_ALL, .type = 0,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VINTDIG, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VIO, .devgroup = DEV_GRP_ALL, .type = 2,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VDD1, .devgroup = DEV_GRP_P1,
		.type = 4, .type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_VDD2, .devgroup = DEV_GRP_P1,
		.type = 3, .type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_REGEN, .devgroup = DEV_GRP_ALL, .type = 2,
		.type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_NRES_PWRON, .devgroup = DEV_GRP_ALL, .type = 0,
		.type2 = 1, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_CLKEN, .devgroup = DEV_GRP_ALL, .type = 3,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_SYSEN, .devgroup = DEV_GRP_ALL, .type = 6,
		.type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_HFCLKOUT, .devgroup = DEV_GRP_P3,
		.type = 0, .type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ 0, 0},
};

static struct twl4030_power_data gossamer_t2scripts_data __initdata = {
	.scripts	= twl4030_scripts,
	.size		= ARRAY_SIZE(twl4030_scripts),
	.resource_config = twl4030_rconfig,
};

static struct twl4030_platform_data __refdata gossamer_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.madc		= &gossamer_madc_data,
	.usb		= &gossamer_usb_data,
	.gpio		= &gossamer_gpio_data,
	.keypad		= &gossamer_kp_twl4030_data,
	.power		= &gossamer_t2scripts_data, // Only valid during init
	.vmmc1          = &gossamer_vmmc1,
	.vmmc2          = &gossamer_vmmc2,
	.vsim           = &gossamer_vsim,
//	.vdac		= &gossamer_vdac,
//	.vpll2		= &gossamer_vdsi,
};

#ifdef CONFIG_INPUT_KXTF9
/* KIONIX KXTF9 Digital Tri-axis Accelerometer */

static void kxtf9_dev_init(void)
{
	printk("board-3621_gossamera.c: kxtf9_dev_init ...\n");

//	if (gpio_request(KXTF9_GPIO_FOR_PWR, "kxtf9_pwr") < 0) {
//		printk(KERN_ERR "+++++++++++++ Can't get GPIO for kxtf9 power\n");
//		return;
//	}
        // Roberto's comment: G-sensor is powered by VIO and does not need to be powered enabled
	//gpio_direction_output(KXTF9_GPIO_FOR_PWR, 1);
	
	if (gpio_request(KXTF9_GPIO_FOR_IRQ, "kxtf9_irq") < 0) {
		printk(KERN_ERR "Can't get GPIO for kxtf9 IRQ\n");
		return;
	}

	printk("board-3621_gossamera.c: kxtf9_dev_init > Init kxtf9 irq pin %d !\n", KXTF9_GPIO_FOR_IRQ);
	gpio_direction_input(KXTF9_GPIO_FOR_IRQ);
	omap_set_gpio_debounce(KXTF9_GPIO_FOR_IRQ, 0);
}

struct kxtf9_platform_data kxtf9_platform_data_here = {
        .min_interval   = 1,
        .poll_interval  = 1000,

        .g_range        = KXTF9_G_8G,
        .shift_adj      = SHIFT_ADJ_2G,

		// Map the axes from the sensor to the device.
		
		//. SETTINGS FOR THE EVT1A TEST RIG:
        .axis_map_x     = 1,
        .axis_map_y     = 0,
        .axis_map_z     = 2,
        .negate_x       = 1,
        .negate_y       = 0,
        .negate_z       = 0,
		
		//. SETTINGS FOR THE ENCORE PRODUCT:
        //. .axis_map_x     = 1,
        //. .axis_map_y     = 0,
        //. .axis_map_z     = 2,
        //. .negate_x       = 1,
        //. .negate_y       = 0,
        //. .negate_z       = 0,

        .data_odr_init          = ODR12_5F,
        .ctrl_reg1_init         = KXTF9_G_8G | RES_12BIT | TDTE | WUFE | TPE,
        .int_ctrl_init          = KXTF9_IEN | KXTF9_IEA | KXTF9_IEL,
        .int_ctrl_init          = KXTF9_IEN,
        .tilt_timer_init        = 0x03,
        .engine_odr_init        = OTP12_5 | OWUF50 | OTDT400,
        .wuf_timer_init         = 0x16,
        .wuf_thresh_init        = 0x28,
        .tdt_timer_init         = 0x78,
        .tdt_h_thresh_init      = 0xFF,
        .tdt_l_thresh_init      = 0x14,
        .tdt_tap_timer_init     = 0x53,
        .tdt_total_timer_init   = 0x24,
        .tdt_latency_timer_init = 0x10,
        .tdt_window_timer_init  = 0xA0,

        .gpio = KXTF9_GPIO_FOR_IRQ,
};
#endif	/* CONFIG_INPUT_KXTF9 */

#ifdef CONFIG_BATTERY_MAX17042
static void max17042_dev_init(void)
{
        printk("board-3621_gossamera.c: max17042_dev_init ...\n");

        if (gpio_request(MAX17042_GPIO_FOR_IRQ, "max17042_irq") < 0) {
                printk(KERN_ERR "Can't get GPIO for max17042 IRQ\n");
                return;
        }

        printk("board-3621_gossamera.c: max17042_dev_init > Init max17042 irq pin %d !\n", MAX17042_GPIO_FOR_IRQ);
        gpio_direction_input(MAX17042_GPIO_FOR_IRQ);
        omap_set_gpio_debounce(MAX17042_GPIO_FOR_IRQ, 0);        
        printk("max17042 GPIO pin read %d\n", gpio_get_value(MAX17042_GPIO_FOR_IRQ));
}
#endif

#ifdef CONFIG_BATTERY_MAX17042
struct max17042_platform_data max17042_platform_data_here = {

	//fill in device specific data here
	//load stored parameters from Rom Tokens? 
	//.val_FullCAP =
	//.val_Cycles =
	//.val_FullCAPNom =
	//.val_SOCempty =
	//.val_Iavg_empty =
	//.val_RCOMP0 =
	//.val_TempCo=
	//.val_k_empty0 =
	//.val_dQacc =
	//.val_dPacc =
	
        .gpio = MAX17042_GPIO_FOR_IRQ,
};
#endif

#ifdef CONFIG_MAX9635
static int max9635_device_resource(int allocate)
{
	if (allocate) {
		if (gpio_request(MAX9635_GPIO_FOR_IRQ, "max9635_irq") < 0) {
			printk(KERN_ERR "Failed to get GPIO for max9635\n");
			return -1;
		}
	
		gpio_direction_input(MAX9635_GPIO_FOR_IRQ);
		omap_set_gpio_debounce(MAX9635_GPIO_FOR_IRQ, 0);
	}
	else
	{
		gpio_free(MAX9635_GPIO_FOR_IRQ);
	}
	return 0;
}

static struct max9635_pdata __initdata max9635_platform_data = {
    .gpio           = MAX9635_GPIO_FOR_IRQ,
    .poll_interval  = MAX9635_DEFAULT_POLL_INTERVAL,
	.device_resource = max9635_device_resource,
};
#endif /* CONFIG_MAX9635 */

#if defined(CONFIG_TOUCHSCREEN_ZFORCE) || defined(CONFIG_TOUCHSCREEN_ZFORCE_MODULE)
static struct zforce_platform_data zforce_platform = {
    .width = 600,
    .height = 800,
	.irqflags = IRQF_TRIGGER_FALLING,
};
#endif

static struct i2c_board_info __initdata gossamer_i2c_bus1_info[] = {
#ifdef CONFIG_BATTERY_MAX17042
	{
		I2C_BOARD_INFO(MAX17042_DEVICE_ID, MAX17042_I2C_SLAVE_ADDRESS),
		.platform_data = &max17042_platform_data_here,
		.irq = OMAP_GPIO_IRQ(MAX17042_GPIO_FOR_IRQ),
	},
#endif	/*CONFIG_BATTERY_MAX17042*/	
	{
		I2C_BOARD_INFO("tps65921", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &gossamer_twldata,
	},
#ifdef CONFIG_INPUT_KXTF9
	{
		I2C_BOARD_INFO(KXTF9_DEVICE_ID, KXTF9_I2C_SLAVE_ADDRESS),
		.platform_data = &kxtf9_platform_data_here,
		.irq = OMAP_GPIO_IRQ(KXTF9_GPIO_FOR_IRQ),
	},
#endif /* CONFIG_INPUT_KXTF9 */
#ifdef CONFIG_MAX9635
    {
        I2C_BOARD_INFO(MAX9635_NAME, MAX9635_I2C_SLAVE_ADDRESS),
        .platform_data = &max9635_platform_data,
        .irq = OMAP_GPIO_IRQ(MAX9635_GPIO_FOR_IRQ),
    },
#endif /* CONFIG_MAX9635 */

#if defined(CONFIG_BATTERY_BQ27510) || defined(CONFIG_BATTERY_BQ27510_MODULE)
	{
		I2C_BOARD_INFO("bq27510",  0x55),
	},
#endif /* CONFIG_BATTERY_BQ27510 */
};

#if defined(CONFIG_SND_SOC_DAC3100) || defined(CONFIG_SND_SOC_DAC3100_MODULE) || defined (CONFIG_SND_OMAP_SOC_OMAP3_EDP)
#define AUDIO_CODEC_POWER_ENABLE_GPIO   103
#define AUDIO_CODEC_RESET_GPIO          37
#define AUDIO_CODEC_IRQ_GPIO            59
#define AIC3100_NAME 			"tlv320dac3100"
#define AIC3100_I2CSLAVEADDRESS 	0x18

static void audio_dac_3100_dev_init(void)
{
        printk("board-3621_gossamera.c: audio_dac_3100_dev_init ...\n");
        if (gpio_request(AUDIO_CODEC_RESET_GPIO, "AUDIO_CODEC_RESET_GPIO") < 0) {
                printk(KERN_ERR "can't get AUDIO_CODEC_RESET_GPIO \n");
                return;
        }

        printk("board-3621_gossamera.c: audio_dac_3100_dev_init > set AUDIO_CODEC_RESET_GPIO to output Low!\n");
        gpio_direction_output(AUDIO_CODEC_RESET_GPIO, 0);
	gpio_set_value(AUDIO_CODEC_RESET_GPIO, 0);

        printk("board-3621_gossamera.c: audio_dac_3100_dev_init ...\n");
        if (gpio_request(AUDIO_CODEC_POWER_ENABLE_GPIO, "AUDIO DAC3100 POWER ENABLE") < 0) {
                printk(KERN_ERR "can't get AUDIO_CODEC_POWER_ENABLE_GPIO \n");
                return;
        }

        printk("board-3621_gossamera.c: audio_dac_3100_dev_init > set AUDIO_CODEC_POWER_ENABLE_GPIO to output and value high!\n");
        gpio_direction_output(AUDIO_CODEC_POWER_ENABLE_GPIO, 0);
	gpio_set_value(AUDIO_CODEC_POWER_ENABLE_GPIO, 1);

	/* 1 msec delay needed after PLL power-up */
        mdelay (1);

        printk("board-3621_gossamera.c: audio_dac_3100_dev_init > set AUDIO_CODEC_RESET_GPIO to output and value high!\n");
	gpio_set_value(AUDIO_CODEC_RESET_GPIO, 1);

}
#endif

static struct i2c_board_info __initdata gossamer_i2c_bus2_info[] = {
#if defined(CONFIG_SND_SOC_DAC3100) || defined(CONFIG_SND_SOC_DAC3100_MODULE) || defined(CONFIG_SND_OMAP_SOC_OMAP3_EDP)
	{
		I2C_BOARD_INFO(AIC3100_NAME,  AIC3100_I2CSLAVEADDRESS),
                .irq = OMAP_GPIO_IRQ(AUDIO_CODEC_IRQ_GPIO),
	},
#endif
#ifdef CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C
#if defined(CONFIG_BATTERY_BQ27510) || defined(CONFIG_BATTERY_BQ27510_MODULE)
	{
		I2C_BOARD_INFO("bq27510",  0x55),
	},
#endif /* CONFIG_BATTERY_BQ27510 */
#if defined(CONFIG_TOUCHSCREEN_ZFORCE) || defined(CONFIG_TOUCHSCREEN_ZFORCE_MODULE)
	{
		I2C_BOARD_INFO(ZFORCE_NAME, ZFORCE_I2C_SLAVE_ADDRESS),
		.platform_data = &zforce_platform,
		.irq = OMAP_GPIO_IRQ(ZFORCE_GPIO_FOR_IRQ),
	},
#endif /* CONFIG_TOUCHSCREEN_ZFORCE */
#endif /* CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C */

#if defined(CONFIG_LEDS_AS3676)
	{
		I2C_BOARD_INFO("as3676", 0x40 ),  
		.flags = I2C_CLIENT_WAKE,
		.irq = 0,
		.platform_data = &as3676_pdata,
	}
#endif /* CONFIG_LEDS_AS3676 */
};


#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_ANDROID_MODULE)
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.vendor = "B&N     ",
	.product = "NOOK SimpleTouch",
	.release = 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name = "usb_mass_storage",
	.id = -1,
	.dev = {
		.platform_data = &mass_storage_pdata,
		},
};

// Reserved for serial number passed in from the bootloader.
static char adb_serial_number[32] = "";

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= BN_USB_VENDOR_ID,
	.product_id	= BN_USB_PRODUCT_ID_GOSSAMER,
	.adb_product_id	= BN_USB_PRODUCT_ID_GOSSAMER,
	.version	= 0x0100,
	.product_name	= "NOOK SimpleTouch",
	.manufacturer_name = "B&N",
	.serial_number	= "11223344556677",
	.nluns = 2,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif


static int __init omap_i2c_init(void)
{

    int i2c1_devices;
    int i2c2_devices;

/* Disable OMAP 3630 internal pull-ups for I2Ci */
	if (cpu_is_omap3630()) {

		u32 prog_io;

		prog_io = omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO1);
		/* Program (bit 19)=1 to disable internal pull-up on I2C1 */
		prog_io |= OMAP3630_PRG_I2C1_PULLUPRESX;
		/* Program (bit 0)=1 to disable internal pull-up on I2C2 */
		prog_io |= OMAP3630_PRG_I2C2_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP343X_CONTROL_PROG_IO1);

		prog_io = omap_ctrl_readl(OMAP36XX_CONTROL_PROG_IO_WKUP1);
		/* Program (bit 5)=1 to disable internal pull-up on I2C4(SR) */
		prog_io |= OMAP3630_PRG_SR_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP36XX_CONTROL_PROG_IO_WKUP1);
	}

	i2c1_devices = ARRAY_SIZE(gossamer_i2c_bus1_info);
	i2c2_devices = ARRAY_SIZE(gossamer_i2c_bus2_info);

    if (is_gossamer_board_evt1a()) {
#if defined(CONFIG_BATTERY_BQ27510) || defined(CONFIG_BATTERY_BQ27510_MODULE)
printk("attempting to remove i2c bus info for bq27510 %d\n", i2c2_devices);
        i2c_remove_board_info("bq27510",  0x55,gossamer_i2c_bus2_info, &i2c2_devices);
#endif
#if defined(CONFIG_TOUCHSCREEN_ZFORCE) || defined(CONFIG_TOUCHSCREEN_ZFORCE_MODULE)
printk("attempting to remove i2c bus info for zforce %d\n",i2c2_devices);
        i2c_remove_board_info(ZFORCE_NAME, ZFORCE_I2C_SLAVE_ADDRESS,gossamer_i2c_bus2_info, &i2c2_devices);        
#endif
    }
    else {
#if defined(CONFIG_BATTERY_BQ27510) || defined(CONFIG_BATTERY_BQ27510_MODULE)
        i2c_remove_board_info("bq27510",  0x55,gossamer_i2c_bus1_info, &i2c1_devices);
#endif
    }

//	if (is_encore_board_evt1b()) {
		// The light sensor is not present on gossamerb
//#ifdef CONFIG_MAX9635
//		--i2c1_devices;
//#endif
//	}

//	if (is_encore_board_evt2()) {
//		gossamer_twldata.keypad->keymap = gossamer_twl4030_keymap_evt2;
//		gossamer_twldata.keypad->keymapsize = ARRAY_SIZE(gossamer_twl4030_keymap_evt2);
//#ifdef CONFIG_MAX9635
		// right now evt2 is not stuffed with the max9635 light sensor due to i2c conflict 
		// tbd if it will be reworked on specific units
//		--i2c1_devices;
//#endif
//	}

	omap_register_i2c_bus(1, 100, gossamer_i2c_bus1_info,
			i2c1_devices);
	omap_register_i2c_bus(2, 400, gossamer_i2c_bus2_info,
			i2c2_devices);
	return 0;
}

#if 0
static int __init wl127x_vio_leakage_fix(void)
{
	int ret = 0;

	ret = gpio_request(WL127X_BTEN_GPIO, "wl127x_bten");
	if (ret < 0) {
		printk(KERN_ERR "wl127x_bten gpio_%d request fail",
						WL127X_BTEN_GPIO);
		goto fail;
	}

	gpio_direction_output(WL127X_BTEN_GPIO, 1);
	mdelay(10);
	gpio_direction_output(WL127X_BTEN_GPIO, 0);
	udelay(64);

	gpio_free(WL127X_BTEN_GPIO);
fail:
	return ret;
}
#endif

static void  dump_board_revision(void)
{
	switch (system_rev) {
    case BOARD_GOSSAMER_REV_EVT1A:
        printk("gossamer board revision BOARD_GOSSAMER_REV_EVT1A\n");
        break;
	case BOARD_GOSSAMER_REV_EVTPRE1C:
		printk("gossamer board revision BOARD_GOSSAMER_REV_EVT_PRE_1C\n");
		break;
	case BOARD_GOSSAMER_REV_EVT1C:
		printk("gossamer board revision BOARD_GOSSAMER_REV_EVT1C\n");
		break;
	default:
		printk("gossamer unkown board revision: 0x%0x\n",system_rev);
		break;
	}
}

static struct gpio_led_platform_data led_data;

static struct platform_device gpio_leds_device = {
	.name			= "leds-gpio",
	.id			= -1,
	.dev.platform_data	= &led_data,
};

void __init gpio_leds(struct gpio_led *leds, int nr)
{
	if (!nr)
		return;

	led_data.leds = leds;
	led_data.num_leds = nr;
	platform_device_register(&gpio_leds_device);
}



static void __init omap_gossamer_init(void)
{
	/*we need to have this enable function here to lit up the BL*/
	dump_board_revision();
#if defined(CONFIG_TOUCHSCREEN_ZFORCE) || defined(CONFIG_TOUCHSCREEN_ZFORCE_MODULE)
#ifdef CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C
	// Note: the Papyrus i2c address is now automatically detected
	//       by the driver
	if ( is_gossamer_board_evt_pre1c() )
	{
		zforce_platform.irqflags = IRQF_TRIGGER_RISING;
    }
#endif
#endif /* CONFIG_TOUCHSCREEN_ZFORCE */


	omap_i2c_init();
	/* Fix to prevent VIO leakage on wl127x */
//	wl127x_vio_leakage_fix();
	platform_add_devices(gossamer_devices, ARRAY_SIZE(gossamer_devices));

	omap_board_config = gossamer_config;
	omap_board_config_size = ARRAY_SIZE(gossamer_config);

#ifdef CONFIG_INPUT_KXTF9
//	kxtf9_dev_init();
#endif /* CONFIG_INPUT_KXTF9 */

#ifdef CONFIG_BATTERY_MAX17042
	max17042_dev_init();
#endif

#if defined(CONFIG_SND_SOC_DAC3100) || defined(CONFIG_SND_SOC_DAC3100_MODULE) || defined(CONFIG_SND_OMAP_SOC_OMAP3_EDP)
        audio_dac_3100_dev_init();
#endif
//	msecure_init();
	omap_serial_init();
	usb_musb_init();
#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_ANDROID_MODULE)
	platform_device_register(&usb_mass_storage_device);
	// Set the device serial number passed in from the bootloader.
	if (system_serial_high != 0 || system_serial_low != 0) {
		snprintf(adb_serial_number, sizeof(adb_serial_number), "%08x%08x", system_serial_high, system_serial_low);
		adb_serial_number[16] = '\0';
		android_usb_pdata.serial_number = adb_serial_number;
	}
	platform_device_register(&android_usb_device);
#endif

#ifdef CONFIG_MACH_OMAP3621_GOSSAMER
	if  ( is_gossamer_board_evt_pre1c() )
	{
		gossamer_twl4030_keymap[2] = KEY(1, 2, KEY_VOLUMEUP);
		gossamer_twl4030_keymap[3] = KEY(1, 3, KEY_VOLUMEDOWN);

		bq24073_init_dev_data.gpio_nce = GOSSAMER_PRE1C_CHARGE_ENAB_GPIO;
		bq24073_init_dev_data.gpio_en1 = GOSSAMER_PRE1C_CHARGE_ILM1_GPIO;
	    bq24073_init_dev_data.gpio_en2 = GOSSAMER_PRE1C_CHARGE_ILM2_GPIO;

	}
#endif /* CONFIG_MACH_OMAP3621_GOSSAMER */

#ifdef CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C
	gpio_leds(debug_leds, ARRAY_SIZE(debug_leds));
#endif /* CONFIG_MACH_OMAP3621_GOSSAMER_EVT1C */

    BUG_ON(!cpu_is_omap3630());
}

// This code is yanked from arch/arm/mach-omap2/prcm.c
void machine_emergency_restart(void)
{
	s16 prcm_offs;
	u32 l;

	mdelay(1600);

	prcm_offs = OMAP3430_GR_MOD;
	l = ('B' << 24) | ('M' << 16) | 'h';
	/* Reserve the first word in scratchpad for communicating
	* with the boot ROM. A pointer to a data structure
	* describing the boot process can be stored there,
	* cf. OMAP34xx TRM, Initialization / Software Booting
	* Configuration. */
	omap_writel(l, OMAP343X_SCRATCHPAD + 4);
	omap3_configure_core_dpll_warmreset();
}

static void __init omap_gossamer_map_io(void)
{
#ifdef CONFIG_ANDROID_RAM_CONSOLE
    reserve_bootmem(GOSSAMER_RAM_CONSOLE_START, GOSSAMER_RAM_CONSOLE_SIZE, 0);
#endif /* CONFIG_ANDROID_RAM_CONSOLE */
	omap2_set_globals_343x();
//	iotable_init(gossamer_io_desc, ARRAY_SIZE(gossamer_io_desc));
	omap2_map_common_io();
}
 
MACHINE_START(OMAP3621_GOSSAMER, "OMAP3621 GOSSAMER board")
	/* phys_io is only used for DEBUG_LL early printing.  The Gossamer's
	 * console is on an external quad UART sitting at address 0x10000000
	 */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_gossamer_map_io,
	.init_irq	= omap_gossamer_init_irq,
	.init_machine	= omap_gossamer_init,
	.timer		= &omap_timer,
MACHINE_END
