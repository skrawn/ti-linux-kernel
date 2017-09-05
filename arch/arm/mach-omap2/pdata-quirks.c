/*
 * Legacy platform_data quirks
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/davinci_emac.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/of_platform.h>
#include <linux/ti_wilink_st.h>
#include <linux/wl12xx.h>
#include <linux/spectro.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/spi/spi.h>
#include <linux/usb/musb.h>
#include <linux/usb/usb_phy_generic.h>

#include <linux/platform_data/ad7298.h>
#include <linux/platform_data/gpio-omap.h>
#include <linux/platform_data/iommu-omap.h>
#include <linux/platform_data/pca953x.h>
#include <linux/platform_data/pinctrl-single.h>
#include <linux/platform_data/spi-omap2-mcspi.h>
#include <linux/platform_data/wkup_m3.h>

#include <mach/irqs.h>

#include "common.h"
#include "common-board-devices.h"
#include "dss-common.h"
#include "control.h"
#include "omap_device.h"
#include "omap-secure.h"
#include "soc.h"
#include "hsmmc.h"
#include "mux.h"

#define AM35XX_IPSS_USBOTGSS_BASE      0x5C040000

struct pdata_init {
    const char *compatible;
    void (*fn)(void);
};

static struct of_dev_auxdata omap_auxdata_lookup[];
static struct twl4030_gpio_platform_data twl_gpio_auxdata;

#ifdef CONFIG_MACH_NOKIA_N8X0
static void __init omap2420_n8x0_legacy_init(void)
{
    omap_auxdata_lookup[0].platform_data = n8x0_legacy_init();
}
#else
#define omap2420_n8x0_legacy_init	NULL
#endif

#ifdef CONFIG_ARCH_OMAP3
/*
 * Configures GPIOs 126, 127 and 129 to 1.8V mode instead of 3.0V
 * mode for MMC1 in case bootloader did not configure things.
 * Note that if the pins are used for MMC1, pbias-regulator
 * manages the IO voltage.
 */
static void __init omap3_gpio126_127_129(void)
{
    u32 reg;

    reg = omap_ctrl_readl(OMAP343X_CONTROL_PBIAS_LITE);
    reg &= ~OMAP343X_PBIASLITEVMODE1;
    reg |= OMAP343X_PBIASLITEPWRDNZ1;
    omap_ctrl_writel(reg, OMAP343X_CONTROL_PBIAS_LITE);
    if (cpu_is_omap3630()) {
        reg = omap_ctrl_readl(OMAP34XX_CONTROL_WKUP_CTRL);
        reg |= OMAP36XX_GPIO_IO_PWRDNZ;
        omap_ctrl_writel(reg, OMAP34XX_CONTROL_WKUP_CTRL);
    }
}

static void __init hsmmc2_internal_input_clk(void)
{
    u32 reg;

    reg = omap_ctrl_readl(OMAP343X_CONTROL_DEVCONF1);
    reg |= OMAP2_MMCSDIO2ADPCLKISEL;
    omap_ctrl_writel(reg, OMAP343X_CONTROL_DEVCONF1);
}

static struct iommu_platform_data omap3_iommu_pdata = {
    .reset_name = "mmu",
    .assert_reset = omap_device_assert_hardreset,
    .deassert_reset = omap_device_deassert_hardreset,
};

static int omap3_sbc_t3730_twl_callback(struct device *dev,
                       unsigned gpio,
                       unsigned ngpio)
{
    int res;

    res = gpio_request_one(gpio + 2, GPIOF_OUT_INIT_HIGH,
                   "wlan pwr");
    if (res)
        return res;

    gpio_export(gpio, 0);

    return 0;
}

static void __init omap3_sbc_t3x_usb_hub_init(int gpio, char *hub_name)
{
    int err = gpio_request_one(gpio, GPIOF_OUT_INIT_LOW, hub_name);

    if (err) {
        pr_err("SBC-T3x: %s reset gpio request failed: %d\n",
            hub_name, err);
        return;
    }

    gpio_export(gpio, 0);

    udelay(10);
    gpio_set_value(gpio, 1);
    msleep(1);
}

static void __init omap3_sbc_t3730_twl_init(void)
{
    twl_gpio_auxdata.setup = omap3_sbc_t3730_twl_callback;
}

static void __init omap3_sbc_t3730_legacy_init(void)
{
    omap3_sbc_t3x_usb_hub_init(167, "sb-t35 usb hub");
}

static void __init omap3_sbc_t3530_legacy_init(void)
{
    omap3_sbc_t3x_usb_hub_init(167, "sb-t35 usb hub");
}

static struct ti_st_plat_data wilink_pdata = {
    .nshutdown_gpio = 137,
    .dev_name = "/dev/ttyO1",
    .flow_cntrl = 1,
    .baud_rate = 300000,
};

static struct platform_device wl18xx_device = {
    .name	= "kim",
    .id	= -1,
    .dev	= {
        .platform_data = &wilink_pdata,
    }
};

static struct platform_device btwilink_device = {
    .name	= "btwilink",
    .id	= -1,
};

static void __init omap3_igep0020_rev_f_legacy_init(void)
{
    platform_device_register(&wl18xx_device);
    platform_device_register(&btwilink_device);
}

static void __init omap3_igep0030_rev_g_legacy_init(void)
{
    platform_device_register(&wl18xx_device);
    platform_device_register(&btwilink_device);
}

static void __init omap3_evm_legacy_init(void)
{
    hsmmc2_internal_input_clk();
}

static void am35xx_enable_emac_int(void)
{
    u32 v;

    v = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
    v |= (AM35XX_CPGMAC_C0_RX_PULSE_CLR | AM35XX_CPGMAC_C0_TX_PULSE_CLR |
          AM35XX_CPGMAC_C0_MISC_PULSE_CLR | AM35XX_CPGMAC_C0_RX_THRESH_CLR);
    omap_ctrl_writel(v, AM35XX_CONTROL_LVL_INTR_CLEAR);
    omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR); /* OCP barrier */
}

static void am35xx_disable_emac_int(void)
{
    u32 v;

    v = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
    v |= (AM35XX_CPGMAC_C0_RX_PULSE_CLR | AM35XX_CPGMAC_C0_TX_PULSE_CLR);
    omap_ctrl_writel(v, AM35XX_CONTROL_LVL_INTR_CLEAR);
    omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR); /* OCP barrier */
}

static struct emac_platform_data am35xx_emac_pdata = {
    .interrupt_enable	= am35xx_enable_emac_int,
    .interrupt_disable	= am35xx_disable_emac_int,
};

static void __init am35xx_emac_reset(void)
{
    u32 v;

    v = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
    v &= ~AM35XX_CPGMACSS_SW_RST;
    omap_ctrl_writel(v, AM35XX_CONTROL_IP_SW_RESET);
    omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET); /* OCP barrier */
}

static struct gpio cm_t3517_wlan_gpios[] __initdata = {
    { 56,	GPIOF_OUT_INIT_HIGH,	"wlan pwr" },
    { 4,	GPIOF_OUT_INIT_HIGH,	"xcvr noe" },
};

static void __init omap3_sbc_t3517_wifi_init(void)
{
    int err = gpio_request_array(cm_t3517_wlan_gpios,
                ARRAY_SIZE(cm_t3517_wlan_gpios));
    if (err) {
        pr_err("SBC-T3517: wl12xx gpios request failed: %d\n", err);
        return;
    }

    gpio_export(cm_t3517_wlan_gpios[0].gpio, 0);
    gpio_export(cm_t3517_wlan_gpios[1].gpio, 0);

    msleep(100);
    gpio_set_value(cm_t3517_wlan_gpios[1].gpio, 0);
}

static void __init omap3_sbc_t3517_legacy_init(void)
{
    omap3_sbc_t3x_usb_hub_init(152, "cm-t3517 usb hub");
    omap3_sbc_t3x_usb_hub_init(98, "sb-t35 usb hub");
    am35xx_emac_reset();
    hsmmc2_internal_input_clk();
    omap3_sbc_t3517_wifi_init();
}

static struct omap_musb_board_data usb_otg_brd_data = {
    .interface_type = MUSB_INTERFACE_ULPI,
    .mode = MUSB_OTG,
    .power = 100,
    .set_phy_power = am35x_musb_phy_power,
    .clear_irq = am35x_musb_clear_irq,
    .set_mode = am35x_set_mode,
    .reset = am35x_musb_reset,
};

static struct ehci_hcd_omap_platform_data ehci_data = {
    .port_mode[0]			= OMAP_EHCI_PORT_MODE_PHY,
    .port_mode[1]			= OMAP_USBHS_PORT_MODE_UNUSED,
    .port_mode[2]			= OMAP_USBHS_PORT_MODE_UNUSED,

    .reset_gpio_port[0] 	= 57,
    .reset_gpio_port[1]		= -EINVAL,
    .reset_gpio_port[2]		= -EINVAL,

    .phy_reset 				= 1,
};

static struct ohci_hcd_omap_platform_data ohci_data = {
    .port_mode[0]			= OMAP_USBHS_PORT_MODE_UNUSED,
    .port_mode[1]			= OMAP_USBHS_PORT_MODE_UNUSED,
    .port_mode[2]			= OMAP_USBHS_PORT_MODE_UNUSED,
};

static struct usbhs_omap_platform_data usbhs_pdata = {
    .nports 				= OMAP3_HS_USB_PORTS,

    .port_mode[0] 			= OMAP_EHCI_PORT_MODE_PHY,
    .port_mode[1] 			= OMAP_USBHS_PORT_MODE_UNUSED,
    .port_mode[2]			= OMAP_USBHS_PORT_MODE_UNUSED,

    .reset_gpio_port[0] 	= 57,
    .reset_gpio_port[1]		= -EINVAL,
    .reset_gpio_port[2]		= -EINVAL,

    .ehci_data				= &ehci_data,
    .ohci_data				= &ohci_data,

    .phy_reset 				= 1,

};

#define HALO_ANALOG_BOARD_EXP_BASE	OMAP_MAX_GPIO_LINES
#define HALO_ANALOG_PWR_EN			HALO_ANALOG_BOARD_EXP_BASE
#define HALO_LED_EN					HALO_ANALOG_BOARD_EXP_BASE + 1
#define HALO_TEC1_EN				HALO_ANALOG_BOARD_EXP_BASE + 2
#define HALO_TEC2_EN				HALO_ANALOG_BOARD_EXP_BASE + 3

static int halo_analog_board_io_expander_setup(struct i2c_client *client, unsigned gpio,
                            unsigned ngpio, void *context)
{
    int ret;

    ret = gpio_request(HALO_ANALOG_PWR_EN, "analog_pwr_en");
    if (ret) {
        printk(KERN_ERR "Failed to request analog_pwr_en\n");
    }

    ret = gpio_request(HALO_LED_EN, "led_en");
    if (ret) {
        printk(KERN_ERR "Failed to request led_en\n");
    }

    ret = gpio_request(HALO_TEC1_EN, "tec1_en");
    if (ret) {
        printk(KERN_ERR "Failed to request tec1_en\n");
    }

    ret = gpio_request(HALO_TEC2_EN, "tec2_en");
    if (ret) {
        printk(KERN_ERR "Failed to request tec2_en\n");
    }

    gpio_direction_output(HALO_ANALOG_PWR_EN, 1);
    gpio_direction_output(HALO_LED_EN, 1);
    gpio_direction_output(HALO_TEC1_EN, 1);
    gpio_direction_output(HALO_TEC2_EN, 1);

    return 0;
}

static int halo_analog_board_io_expander_teardown(struct i2c_client *client, unsigned gpio,
                            unsigned ngpio, void *context)
{
    gpio_set_value(HALO_ANALOG_PWR_EN, 0);
    gpio_set_value(HALO_LED_EN, 0);
    gpio_set_value(HALO_TEC1_EN, 0);
    gpio_set_value(HALO_TEC2_EN, 0);

    gpio_free(HALO_ANALOG_PWR_EN);
    gpio_free(HALO_LED_EN);
    gpio_free(HALO_TEC1_EN);
    gpio_free(HALO_TEC2_EN);

    return 0;
}

static struct pca953x_platform_data halo_analog_board_expander_info = {
    .gpio_base	= OMAP_MAX_GPIO_LINES,
    .invert = 0,
    .setup = halo_analog_board_io_expander_setup,
    .teardown = halo_analog_board_io_expander_teardown,
};

#define HALO_POWER_BOARD_EXP_BASE 	OMAP_MAX_GPIO_LINES + 16
#define HALO_LOW_POWERn				HALO_POWER_BOARD_EXP_BASE
#define HALO_LOW_BATn				HALO_POWER_BOARD_EXP_BASE + 1
#define HALO_LAMP_FAULTn		 	HALO_POWER_BOARD_EXP_BASE + 2
#define HALO_REF_COLLECTOR			HALO_POWER_BOARD_EXP_BASE + 3
#define HALO_STEPPER_ENABLEn		HALO_POWER_BOARD_EXP_BASE + 4
#define HALO_STEPPER_DIR			HALO_POWER_BOARD_EXP_BASE + 5
#define HALO_SWIR_LED_CATH			HALO_POWER_BOARD_EXP_BASE + 6
#define HALO_STEPS_MOTOR			HALO_POWER_BOARD_EXP_BASE + 7
#define HALO_STEPPER_SWITCH			HALO_POWER_BOARD_EXP_BASE + 8
#define HALO_BLDC_SWITCH 			HALO_POWER_BOARD_EXP_BASE + 9
#define HALO_LAMP_SWITCH			HALO_POWER_BOARD_EXP_BASE + 10
#define HALO_FAN_SWITCH				HALO_POWER_BOARD_EXP_BASE + 11
#define HALO_EXP_NC_0				HALO_POWER_BOARD_EXP_BASE + 12
#define HALO_STEPPER_SLEEPn			HALO_POWER_BOARD_EXP_BASE + 13
#define HALO_STEPPER_FAULTn			HALO_POWER_BOARD_EXP_BASE + 14
#define HALO_EXP_NC_1				HALO_POWER_BOARD_EXP_BASE + 15


int halo_power_board_io_expander_setup(struct i2c_client *client, unsigned gpio,
                            unsigned ngpio, void *context)
{
    int ret;

    ret = gpio_request(HALO_LOW_POWERn, "low_powern");
    if (ret) {
        printk(KERN_ERR "Failed to request low_powern\n");
    }

    ret = gpio_request(HALO_LOW_BATn, "low_batn");
    if (ret) {
        printk(KERN_ERR "Failed to request low_batn\n");
    }

    ret = gpio_request(HALO_LAMP_FAULTn, "lamp_faultn");
    if (ret) {
        printk(KERN_ERR "Failed to request lamp_faultn\n");
    }

    ret = gpio_request(HALO_REF_COLLECTOR, "ref_collect");
    if (ret) {
        printk(KERN_ERR "Failed to request ref_collect\n");
    }

    /* Stepper related GPIOs are not used */

    ret = gpio_request(HALO_SWIR_LED_CATH, "swir_led_cath");
    if (ret) {
        printk(KERN_ERR "Failed to request swir_led_cath\n");
    }

    ret = gpio_request(HALO_BLDC_SWITCH, "bldc_switch");
    if (ret) {
        printk(KERN_ERR "Failed to request bldc_switch\n");
    }

    ret = gpio_request(HALO_LAMP_SWITCH, "lamp_switch");
    if (ret) {
        printk(KERN_ERR "Failed to request lamp_switch\n");
    }

    ret = gpio_request(HALO_FAN_SWITCH, "fan_switch");
    if (ret) {
        printk(KERN_ERR "Failed to request fan_switch\n");
    }

    gpio_direction_input(HALO_LOW_POWERn);
    gpio_direction_input(HALO_LOW_BATn);
    gpio_direction_input(HALO_LAMP_FAULTn);
    gpio_direction_input(HALO_REF_COLLECTOR);

    gpio_direction_output(HALO_SWIR_LED_CATH, 0);
    gpio_direction_output(HALO_BLDC_SWITCH, 1);
    gpio_direction_output(HALO_LAMP_SWITCH, 1);
    gpio_direction_output(HALO_FAN_SWITCH, 1);

    return 0;
}

int halo_power_board_io_expander_teardown(struct i2c_client *client, unsigned gpio,
                            unsigned ngpio, void *context)
{
    gpio_set_value(HALO_SWIR_LED_CATH, 0);
    gpio_set_value(HALO_BLDC_SWITCH, 0);
    gpio_set_value(HALO_LAMP_SWITCH, 0);
    gpio_set_value(HALO_FAN_SWITCH, 0);

    gpio_free(HALO_LOW_POWERn);
    gpio_free(HALO_LOW_BATn);
    gpio_free(HALO_LAMP_FAULTn);
    gpio_free(HALO_REF_COLLECTOR);
    gpio_free(HALO_SWIR_LED_CATH);
    gpio_free(HALO_BLDC_SWITCH);
    gpio_free(HALO_LAMP_SWITCH);
    gpio_free(HALO_FAN_SWITCH);

    return 0;
}

static struct pca953x_platform_data halo_power_board_expander_info = {
    .gpio_base	= OMAP_MAX_GPIO_LINES + 16,
    .invert = 0,
    .setup = halo_power_board_io_expander_setup,
    .teardown = halo_power_board_io_expander_teardown,
};

static struct i2c_board_info __initdata halo_am3517_i2c3_boardinfo[] = {
    {
        I2C_BOARD_INFO("tca6416", 0x20),
        .platform_data = &halo_analog_board_expander_info,		
    },
    {
        I2C_BOARD_INFO("tca6416", 0x21),
        .platform_data = &halo_power_board_expander_info,		
    },
};

int halo_ui_board_io_expander_setup(struct i2c_client *client, unsigned gpio,
    unsigned ngpio, void *context)
{
    return 0;
}

int halo_ui_board_io_expander_teardown(struct i2c_client *client, unsigned gpio,
    unsigned ngpio, void *context)
{
    return 0;
}

static struct pca953x_platform_data halo_ui_board_expander_info = {
    .gpio_base = OMAP_MAX_GPIO_LINES + 32,
    .invert = 0,	
    .setup = halo_ui_board_io_expander_setup,
    .teardown = halo_ui_board_io_expander_teardown,
};

static struct i2c_board_info __initdata halo_am3517_i2c2_boardinfo[] = {
    {
        I2C_BOARD_INFO("tca6416", 0x20),
        .platform_data = &halo_ui_board_expander_info,		
    }	
};

static struct omap_board_mux board_mux[] __initdata = {	
    OMAP3_MUX(MCBSP2_CLKX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),	/* mcbsp2_clkx.gpio_117 - GPIO_INT1# */
    OMAP3_MUX(MCBSP2_FSX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),	/* mcbsp2_fsx.gpio_116 - GPIO_INT2# */
    OMAP3_MUX(MCBSP2_DX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),	/* mcbsp2_dx.gpio_119 - GPIO_INT3# */
    { .reg_offset = OMAP_MUX_TERMINATOR },
};

struct usbhs_phy_data usbhs_phy = {
    .port = 1,
    .reset_gpio = 57,
};

struct omap2_mcspi_device_config adc_controller_data __initdata = {
    .turbo_mode = 0,
    .cs_per_word = 0,
};

static struct spi_board_info fs5_dev_adc_board_info[] __initdata = {
    {
        .modalias = "ad7298",
        .max_speed_hz = 20000000,
        .bus_num = 1,
        .chip_select = 2,
        .mode = SPI_MODE_1,
        .controller_data = &adc_controller_data,
    }
};

static struct spi_board_info fs5_adc_board_info[] __initdata = {
    {
        .modalias = "ad7298",
        .max_speed_hz = 20000000,
        .bus_num = 1,
        .chip_select = 0,
        .mode = SPI_MODE_1,
        .controller_data = &adc_controller_data,
    },
    {
        .modalias = "ad7298",
        .max_speed_hz = 20000000,
        .bus_num = 1,
        .chip_select = 1,
        .mode = SPI_MODE_1,
        .controller_data = &adc_controller_data,
    },
    {
        .modalias = "ad7298",
        .max_speed_hz = 20000000,
        .bus_num = 1,
        .chip_select = 2,
        .mode = SPI_MODE_1,
        .controller_data = &adc_controller_data,
    }
};

static void __init am3517_evm_legacy_init(void)
{
    omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);

    am35xx_emac_reset();
    hsmmc2_internal_input_clk();

    usb_musb_init(&usb_otg_brd_data);

    usbhs_init_phys(&usbhs_phy, 1);
    usbhs_init(&usbhs_pdata);

    /*Drive the output enable for WL1271 module low*/
    gpio_export(129, 0);
        udelay(10);
    gpio_direction_output(129, 0);
        msleep(1);
}

static void __init halo_am3517_legacy_init(void)
{
    omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);

    hsmmc2_internal_input_clk();

    usb_musb_init(&usb_otg_brd_data);

    usbhs_init_phys(&usbhs_phy, 1);
    usbhs_init(&usbhs_pdata);	
    
    /*Drive the output enable for WL1271 module low*/
    gpio_export(129, 0);
        udelay(10);
    gpio_direction_output(129, 0);
        msleep(1);

    // IRQ numbers are determined at runtime, so need to update the board info here	to 
    // associate these IO expanders with their GPIO interrupt lines.
    halo_am3517_i2c2_boardinfo[0].irq = gpio_to_irq(116);
    halo_am3517_i2c3_boardinfo[0].irq = gpio_to_irq(119);
    halo_am3517_i2c3_boardinfo[1].irq = gpio_to_irq(117);

    omap_register_i2c_bus(2, 100, halo_am3517_i2c2_boardinfo,
        ARRAY_SIZE(halo_am3517_i2c2_boardinfo));

    omap_register_i2c_bus(3, 400, halo_am3517_i2c3_boardinfo,
        ARRAY_SIZE(halo_am3517_i2c3_boardinfo));
        
    spi_register_board_info(fs5_dev_adc_board_info, ARRAY_SIZE(fs5_dev_adc_board_info));		
}

static void __init fs5_am3517_dev_legacy_init(void)
{
    omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
    
    am35xx_emac_reset();
    hsmmc2_internal_input_clk();

    usb_musb_init(&usb_otg_brd_data);

    usbhs_init_phys(&usbhs_phy, 1);
    usbhs_init(&usbhs_pdata);	
    
    /*Drive the output enable for WL1271 module low*/
    gpio_export(129, 0);
        udelay(10);
    gpio_direction_output(129, 0);
        msleep(1);

    // IRQ numbers are determined at runtime, so need to update the board info here	to 
    // associate these IO expanders with their GPIO interrupt lines.
    halo_am3517_i2c2_boardinfo[0].irq = gpio_to_irq(116);
    halo_am3517_i2c3_boardinfo[0].irq = gpio_to_irq(119);
    halo_am3517_i2c3_boardinfo[1].irq = gpio_to_irq(117);

    omap_register_i2c_bus(2, 100, halo_am3517_i2c2_boardinfo,
        ARRAY_SIZE(halo_am3517_i2c2_boardinfo));

    omap_register_i2c_bus(3, 400, halo_am3517_i2c3_boardinfo,
        ARRAY_SIZE(halo_am3517_i2c3_boardinfo));

    spi_register_board_info(fs5_dev_adc_board_info, ARRAY_SIZE(fs5_dev_adc_board_info));
}

#define FS5_ANALOG_EXPANDER_BASE	OMAP_MAX_GPIO_LINES
#define FS5_ANALOG_PWR_EN			FS5_ANALOG_EXPANDER_BASE
#define FS5_VNIR_LED_ENn			FS5_ANALOG_EXPANDER_BASE + 1
#define FS5_TEC1_EN					FS5_ANALOG_EXPANDER_BASE + 2
#define FS5_TEC2_EN					FS5_ANALOG_EXPANDER_BASE + 3
#define FS5_TEC1_STATUS				FS5_ANALOG_EXPANDER_BASE + 4
#define FS5_TEC2_STATUS				FS5_ANALOG_EXPANDER_BASE + 5
#define FS5_GPIO1_6					FS5_ANALOG_EXPANDER_BASE + 6
#define FS5_GPIO1_7					FS5_ANALOG_EXPANDER_BASE + 7
#define FS5_USB0_OCn				FS5_ANALOG_EXPANDER_BASE + 8
#define FS5_USB1_OCn				FS5_ANALOG_EXPANDER_BASE + 9
#define FS5_ETH_RSTn				FS5_ANALOG_EXPANDER_BASE + 10
#define FS5_STS_LED_EN				FS5_ANALOG_EXPANDER_BASE + 11
#define FS5_GPIO1_14				FS5_ANALOG_EXPANDER_BASE + 12
#define FS5_GPIO1_15				FS5_ANALOG_EXPANDER_BASE + 13
#define FS5_GPIO1_16				FS5_ANALOG_EXPANDER_BASE + 14
#define FS5_GPIO1_17				FS5_ANALOG_EXPANDER_BASE + 15

int fs5_analog_io_expander_setup(struct i2c_client *client, unsigned gpio,
    unsigned ngpio, void *context)
{
    int ret;
    
    ret = gpio_request(FS5_ANALOG_PWR_EN, "analog_pwr_en");
    if (ret) {
        printk(KERN_ERR "Failed to request analog_pwr_en\n");
    }

    ret = gpio_request(FS5_VNIR_LED_ENn, "vnir_led_enn");
    if (ret) {
        printk(KERN_ERR "Failed to request vnir_led_en\n");
    }

    ret = gpio_request(FS5_TEC1_EN, "tec1_en");
    if (ret) {
        printk(KERN_ERR "Failed to request tec1_en\n");
    }

    ret = gpio_request(FS5_TEC2_EN, "tec2_en");
    if (ret) {
        printk(KERN_ERR "Failed to request tec2_en\n");
    }	

    ret = gpio_request(FS5_TEC1_STATUS, "tec1_status");
    if (ret) {
        printk(KERN_ERR "Failed to request tec1_status\n");
    }

    ret = gpio_request(FS5_TEC2_STATUS, "tec2_status");
    if (ret) {
        printk(KERN_ERR "Failed to request tec2_status\n");
    }

    ret = gpio_request(FS5_USB0_OCn, "usb0_ocn");
    if (ret) {
        printk(KERN_ERR "Failed to request usb0_ocn\n");
    }

    ret = gpio_request(FS5_USB1_OCn, "usb1_ocn");
    if (ret) {
        printk(KERN_ERR "Failed to request usb1_ocn\n");
    }

    ret = gpio_request(FS5_ETH_RSTn, "eth_rstn");
    if (ret) {
        printk(KERN_ERR "Failed to request eth_rstn\n");
    }

    ret = gpio_request(FS5_STS_LED_EN, "sts_led_en");
    if (ret) {
        printk(KERN_ERR "Failed to request sts_led_en\n");
    }

    gpio_direction_input(FS5_TEC1_STATUS);	
    gpio_direction_input(FS5_TEC2_STATUS);
    gpio_direction_input(FS5_USB0_OCn);
    gpio_direction_input(FS5_USB1_OCn);

    gpio_direction_output(FS5_ANALOG_PWR_EN, 1);
    gpio_direction_output(FS5_VNIR_LED_ENn, 1);
    gpio_direction_output(FS5_TEC1_EN, 1);
    gpio_direction_output(FS5_TEC2_EN, 1);
    gpio_direction_output(FS5_ETH_RSTn, 1);
    gpio_direction_output(FS5_STS_LED_EN, 0);

    return 0;
}

int fs5_analog_io_expander_teardown(struct i2c_client *client, unsigned gpio,
    unsigned ngpio, void *context)
{
    gpio_set_value(FS5_ANALOG_PWR_EN, 0);
    gpio_set_value(FS5_VNIR_LED_ENn, 1);
    gpio_set_value(FS5_TEC1_EN, 0);
    gpio_set_value(FS5_TEC2_EN, 0);
    gpio_set_value(FS5_ETH_RSTn, 1);
    gpio_set_value(FS5_STS_LED_EN, 0);

    gpio_free(FS5_TEC1_STATUS);	
    gpio_free(FS5_TEC2_STATUS);
    gpio_free(FS5_USB0_OCn);
    gpio_free(FS5_USB1_OCn);
    gpio_free(FS5_ANALOG_PWR_EN);
    gpio_free(FS5_VNIR_LED_ENn);
    gpio_free(FS5_TEC1_EN);
    gpio_free(FS5_TEC2_EN);
    gpio_free(FS5_ETH_RSTn);
    gpio_free(FS5_STS_LED_EN);
    
    return 0;
}

static struct pca953x_platform_data fs5_analog_expander_info = {
    .gpio_base = FS5_ANALOG_EXPANDER_BASE,
    .invert = 0,	
    .setup = fs5_analog_io_expander_setup,
    .teardown = fs5_analog_io_expander_teardown,
};

#define FS5_DIGITAL_EXPANDER_BASE	FS5_GPIO1_17 + 1
#define FS5_GPIO2_0					FS5_DIGITAL_EXPANDER_BASE
#define FS5_GPIO2_1					FS5_DIGITAL_EXPANDER_BASE + 1
#define FS5_ACCY_FLTn				FS5_DIGITAL_EXPANDER_BASE + 2
#define FS5_SWIR2_LED_ENn			FS5_DIGITAL_EXPANDER_BASE + 3
#define FS5_GPIO2_4					FS5_DIGITAL_EXPANDER_BASE + 4
#define FS5_GPIO2_5					FS5_DIGITAL_EXPANDER_BASE + 5
#define FS5_SWIR1_LED_ENn			FS5_DIGITAL_EXPANDER_BASE + 6
#define FS5_TRIG_LED_EN				FS5_DIGITAL_EXPANDER_BASE + 7
#define FS5_ACCY_ENn				FS5_DIGITAL_EXPANDER_BASE + 8
#define FS5_MOTOR_EN				FS5_DIGITAL_EXPANDER_BASE + 9
#define FS5_LAMP_EN					FS5_DIGITAL_EXPANDER_BASE + 10
#define FS5_FAN_EN					FS5_DIGITAL_EXPANDER_BASE + 11
#define FS5_GPIO2_14				FS5_DIGITAL_EXPANDER_BASE + 12
#define FS5_MOT_DIR_GPIO			FS5_DIGITAL_EXPANDER_BASE + 13
#define FS5_GPIO2_16				FS5_DIGITAL_EXPANDER_BASE + 14
#define FS5_GPIO2_17				FS5_DIGITAL_EXPANDER_BASE + 15

int fs5_digital_io_expander_setup(struct i2c_client *client, unsigned gpio,
    unsigned ngpio, void *context)
{
    int ret;
    
    ret = gpio_request(FS5_ACCY_FLTn, "accy_fltn");
    if (ret) {
        printk(KERN_ERR "Failed to request accy_fltn\n");
    }

    ret = gpio_request(FS5_SWIR2_LED_ENn, "swir2_led_enn");
    if (ret) {
        printk(KERN_ERR "Failed to request swir2_led_enn\n");
    }

    ret = gpio_request(FS5_SWIR1_LED_ENn, "swir1_led_enn");
    if (ret) {
        printk(KERN_ERR "Failed to request swir1_led_enn\n");
    }

    ret = gpio_request(FS5_TRIG_LED_EN, "trig_led_en");
    if (ret) {
        printk(KERN_ERR "Failed to request trig_led_en\n");
    }	

    ret = gpio_request(FS5_ACCY_ENn, "fs5_accy_enn");
    if (ret) {
        printk(KERN_ERR "Failed to request fs5_accy_enn\n");
    }

    ret = gpio_request(FS5_MOTOR_EN, "fs5_motor_en");
    if (ret) {
        printk(KERN_ERR "Failed to request fs5_motor_en\n");
    }

    ret = gpio_request(FS5_LAMP_EN, "lamp_en");
    if (ret) {
        printk(KERN_ERR "Failed to request lamp_en\n");
    }

    ret = gpio_request(FS5_FAN_EN, "fan_en");
    if (ret) {
        printk(KERN_ERR "Failed to request fan_en\n");
    }

    ret = gpio_request(FS5_MOT_DIR_GPIO, "motor_dir_gpio");
    if (ret) {
        printk(KERN_ERR "Failed to request motor_dir_gpio\n");
    }

    gpio_direction_input(FS5_ACCY_FLTn);	

    gpio_direction_output(FS5_SWIR2_LED_ENn, 1);
    gpio_direction_output(FS5_SWIR1_LED_ENn, 1);
    gpio_direction_output(FS5_TRIG_LED_EN, 0);
    gpio_direction_output(FS5_ACCY_ENn, 1);
    gpio_direction_output(FS5_MOTOR_EN, 1);
    gpio_direction_output(FS5_LAMP_EN, 0);
    gpio_direction_output(FS5_FAN_EN, 1);
    gpio_direction_output(FS5_MOT_DIR_GPIO, 0);

    return 0;
}

int fs5_digital_io_expander_teardown(struct i2c_client *client, unsigned gpio,
    unsigned ngpio, void *context)
{
    gpio_set_value(FS5_SWIR2_LED_ENn, 1);
    gpio_set_value(FS5_SWIR1_LED_ENn, 1);
    gpio_set_value(FS5_TRIG_LED_EN, 0);
    gpio_set_value(FS5_ACCY_ENn, 1);
    gpio_set_value(FS5_MOTOR_EN, 0);
    gpio_set_value(FS5_LAMP_EN, 0);
    gpio_set_value(FS5_FAN_EN, 0);
    gpio_set_value(FS5_MOT_DIR_GPIO, 0);

    gpio_free(FS5_ACCY_FLTn);
    gpio_free(FS5_SWIR2_LED_ENn);
    gpio_free(FS5_SWIR1_LED_ENn);
    gpio_free(FS5_TRIG_LED_EN);
    gpio_free(FS5_ACCY_ENn);
    gpio_free(FS5_MOTOR_EN);
    gpio_free(FS5_LAMP_EN);
    gpio_free(FS5_FAN_EN);
    gpio_free(FS5_MOT_DIR_GPIO);

    return 0;
}

static struct pca953x_platform_data fs5_digital_expander_info = {
    .gpio_base = FS5_DIGITAL_EXPANDER_BASE,
    .invert = 0,	
    .setup = fs5_digital_io_expander_setup,
    .teardown = fs5_digital_io_expander_teardown,
};

static struct i2c_board_info __initdata fs5_i2c2_boardinfo[] = {
    {
        I2C_BOARD_INFO("tca6416", 0x20),
        .platform_data = &fs5_analog_expander_info,		
    },
    {
        I2C_BOARD_INFO("tca6416", 0x21),
        .platform_data = &fs5_digital_expander_info,
    }
};

static void __init fs5_am3517_legacy_init(void)
{
    omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);

    am35xx_emac_reset();
    hsmmc2_internal_input_clk();

    usb_musb_init(&usb_otg_brd_data);

    usbhs_init_phys(&usbhs_phy, 1);
    usbhs_init(&usbhs_pdata);	
    
    /*Drive the output enable for WL1271 module low*/
    gpio_export(129, 0);
        udelay(10);
    gpio_direction_output(129, 0);
        msleep(1);

    // IRQ numbers are determined at runtime, so need to update the board info here	to 
    // associate these IO expanders with their GPIO interrupt lines.
    fs5_i2c2_boardinfo[0].irq = gpio_to_irq(116);
    fs5_i2c2_boardinfo[1].irq = gpio_to_irq(117);

    omap_register_i2c_bus(2, 100, fs5_i2c2_boardinfo,
        ARRAY_SIZE(fs5_i2c2_boardinfo));	

    spi_register_board_info(fs5_adc_board_info, ARRAY_SIZE(fs5_adc_board_info));
}

static struct platform_device omap3_rom_rng_device = {
    .name		= "omap3-rom-rng",
    .id		= -1,
    .dev	= {
        .platform_data	= rx51_secure_rng_call,
    },
};

static void __init nokia_n900_legacy_init(void)
{
    hsmmc2_internal_input_clk();

    if (omap_type() == OMAP2_DEVICE_TYPE_SEC) {
        if (IS_ENABLED(CONFIG_ARM_ERRATA_430973)) {
            pr_info("RX-51: Enabling ARM errata 430973 workaround\n");
            /* set IBE to 1 */
            rx51_secure_update_aux_cr(BIT(6), 0);
        } else {
            pr_warn("RX-51: Not enabling ARM errata 430973 workaround\n");
            pr_warn("Thumb binaries may crash randomly without this workaround\n");
        }

        pr_info("RX-51: Registring OMAP3 HWRNG device\n");
        platform_device_register(&omap3_rom_rng_device);

    }
}

static void __init omap3_tao3530_legacy_init(void)
{
    hsmmc2_internal_input_clk();
}

/* omap3pandora legacy devices */
#define PANDORA_WIFI_IRQ_GPIO		21
#define PANDORA_WIFI_NRESET_GPIO	23

static struct platform_device pandora_backlight = {
    .name	= "pandora-backlight",
    .id	= -1,
};

static struct regulator_consumer_supply pandora_vmmc3_supply[] = {
    REGULATOR_SUPPLY("vmmc", "omap_hsmmc.2"),
};

static struct regulator_init_data pandora_vmmc3 = {
    .constraints = {
        .valid_ops_mask		= REGULATOR_CHANGE_STATUS,
    },
    .num_consumer_supplies	= ARRAY_SIZE(pandora_vmmc3_supply),
    .consumer_supplies	= pandora_vmmc3_supply,
};

static struct fixed_voltage_config pandora_vwlan = {
    .supply_name		= "vwlan",
    .microvolts		= 1800000, /* 1.8V */
    .gpio			= PANDORA_WIFI_NRESET_GPIO,
    .startup_delay		= 50000, /* 50ms */
    .enable_high		= 1,
    .init_data		= &pandora_vmmc3,
};

static struct platform_device pandora_vwlan_device = {
    .name		= "reg-fixed-voltage",
    .id		= 1,
    .dev = {
        .platform_data = &pandora_vwlan,
    },
};

static void pandora_wl1251_init_card(struct mmc_card *card)
{
    /*
     * We have TI wl1251 attached to MMC3. Pass this information to
     * SDIO core because it can't be probed by normal methods.
     */
    if (card->type == MMC_TYPE_SDIO || card->type == MMC_TYPE_SD_COMBO) {
        card->quirks |= MMC_QUIRK_NONSTD_SDIO;
        card->cccr.wide_bus = 1;
        card->cis.vendor = 0x104c;
        card->cis.device = 0x9066;
        card->cis.blksize = 512;
        card->cis.max_dtr = 24000000;
        card->ocr = 0x80;
    }
}

static struct omap2_hsmmc_info pandora_mmc3[] = {
    {
        .mmc		= 3,
        .caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
        .gpio_cd	= -EINVAL,
        .gpio_wp	= -EINVAL,
        .init_card	= pandora_wl1251_init_card,
    },
    {}	/* Terminator */
};

static void __init pandora_wl1251_init(void)
{
    struct wl1251_platform_data pandora_wl1251_pdata;
    int ret;

    memset(&pandora_wl1251_pdata, 0, sizeof(pandora_wl1251_pdata));

    pandora_wl1251_pdata.power_gpio = -1;

    ret = gpio_request_one(PANDORA_WIFI_IRQ_GPIO, GPIOF_IN, "wl1251 irq");
    if (ret < 0)
        goto fail;

    pandora_wl1251_pdata.irq = gpio_to_irq(PANDORA_WIFI_IRQ_GPIO);
    if (pandora_wl1251_pdata.irq < 0)
        goto fail_irq;

    pandora_wl1251_pdata.use_eeprom = true;
    ret = wl1251_set_platform_data(&pandora_wl1251_pdata);
    if (ret < 0)
        goto fail_irq;

    return;

fail_irq:
    gpio_free(PANDORA_WIFI_IRQ_GPIO);
fail:
    pr_err("wl1251 board initialisation failed\n");
}

static void __init omap3_pandora_legacy_init(void)
{
    platform_device_register(&pandora_backlight);
    platform_device_register(&pandora_vwlan_device);
    omap_hsmmc_init(pandora_mmc3);
    omap_hsmmc_late_init(pandora_mmc3);
    pandora_wl1251_init();
}
#endif /* CONFIG_ARCH_OMAP3 */

#if defined(CONFIG_ARCH_OMAP4) || defined(CONFIG_SOC_OMAP5)
static struct iommu_platform_data omap4_iommu_pdata = {
    .reset_name = "mmu_cache",
    .assert_reset = omap_device_assert_hardreset,
    .deassert_reset = omap_device_deassert_hardreset,
};
#endif

#if defined(CONFIG_SOC_AM33XX) || defined(CONFIG_SOC_AM43XX)
static struct wkup_m3_platform_data wkup_m3_data = {
    .reset_name = "wkup_m3",
    .assert_reset = omap_device_assert_hardreset,
    .deassert_reset = omap_device_deassert_hardreset,
};
#endif

#ifdef CONFIG_SOC_OMAP5
static void __init omap5_uevm_legacy_init(void)
{
}
#endif

static struct pcs_pdata pcs_pdata;

void omap_pcs_legacy_init(int irq, void (*rearm)(void))
{
    pcs_pdata.irq = irq;
    pcs_pdata.rearm = rearm;
}

/*
 * GPIOs for TWL are initialized by the I2C bus and need custom
 * handing until DSS has device tree bindings.
 */
void omap_auxdata_legacy_init(struct device *dev)
{
    if (dev->platform_data)
        return;

    if (strcmp("twl4030-gpio", dev_name(dev)))
        return;

    dev->platform_data = &twl_gpio_auxdata;
}

/*
 * Few boards still need auxdata populated before we populate
 * the dev entries in of_platform_populate().
 */
static struct pdata_init auxdata_quirks[] __initdata = {
#ifdef CONFIG_SOC_OMAP2420
    { "nokia,n800", omap2420_n8x0_legacy_init, },
    { "nokia,n810", omap2420_n8x0_legacy_init, },
    { "nokia,n810-wimax", omap2420_n8x0_legacy_init, },
#endif
#ifdef CONFIG_ARCH_OMAP3
    { "compulab,omap3-sbc-t3730", omap3_sbc_t3730_twl_init, },
#endif
    { /* sentinel */ },
};

static struct of_dev_auxdata omap_auxdata_lookup[] __initdata = {
#ifdef CONFIG_MACH_NOKIA_N8X0
    OF_DEV_AUXDATA("ti,omap2420-mmc", 0x4809c000, "mmci-omap.0", NULL),
    OF_DEV_AUXDATA("menelaus", 0x72, "1-0072", &n8x0_menelaus_platform_data),
    OF_DEV_AUXDATA("tlv320aic3x", 0x18, "2-0018", &n810_aic33_data),
#endif
#ifdef CONFIG_ARCH_OMAP3
    OF_DEV_AUXDATA("ti,omap3-padconf", 0x48002030, "48002030.pinmux", &pcs_pdata),
    OF_DEV_AUXDATA("ti,omap3-padconf", 0x480025a0, "480025a0.pinmux", &pcs_pdata),
    OF_DEV_AUXDATA("ti,omap3-padconf", 0x48002a00, "48002a00.pinmux", &pcs_pdata),
    OF_DEV_AUXDATA("ti,omap2-iommu", 0x5d000000, "5d000000.mmu",
               &omap3_iommu_pdata),
    /* Only on am3517 */
    OF_DEV_AUXDATA("ti,davinci_mdio", 0x5c030000, "davinci_mdio.0", NULL),
    OF_DEV_AUXDATA("ti,am3517-emac", 0x5c000000, "davinci_emac.0",
               &am35xx_emac_pdata),
#endif
#ifdef CONFIG_SOC_AM33XX
    OF_DEV_AUXDATA("ti,am3352-wkup-m3", 0x44d00000, "44d00000.wkup_m3",
               &wkup_m3_data),
#endif
#ifdef CONFIG_ARCH_OMAP4
    OF_DEV_AUXDATA("ti,omap4-padconf", 0x4a100040, "4a100040.pinmux", &pcs_pdata),
    OF_DEV_AUXDATA("ti,omap4-padconf", 0x4a31e040, "4a31e040.pinmux", &pcs_pdata),
#endif
#ifdef CONFIG_SOC_OMAP5
    OF_DEV_AUXDATA("ti,omap5-padconf", 0x4a002840, "4a002840.pinmux", &pcs_pdata),
    OF_DEV_AUXDATA("ti,omap5-padconf", 0x4ae0c840, "4ae0c840.pinmux", &pcs_pdata),
#endif
#ifdef CONFIG_SOC_DRA7XX
    OF_DEV_AUXDATA("ti,dra7-padconf", 0x4a003400, "4a003400.pinmux", &pcs_pdata),
#endif
#ifdef CONFIG_SOC_AM43XX
    OF_DEV_AUXDATA("ti,am437-padconf", 0x44e10800, "44e10800.pinmux", &pcs_pdata),
    OF_DEV_AUXDATA("ti,am4372-wkup-m3", 0x44d00000, "44d00000.wkup_m3",
               &wkup_m3_data),
#endif
#if defined(CONFIG_ARCH_OMAP4) || defined(CONFIG_SOC_OMAP5)
    OF_DEV_AUXDATA("ti,omap4-iommu", 0x4a066000, "4a066000.mmu",
               &omap4_iommu_pdata),
    OF_DEV_AUXDATA("ti,omap4-iommu", 0x55082000, "55082000.mmu",
               &omap4_iommu_pdata),
#endif
    { /* sentinel */ },
};

/*
 * Few boards still need to initialize some legacy devices with
 * platform data until the drivers support device tree.
 */
static struct pdata_init pdata_quirks[] __initdata = {
#ifdef CONFIG_ARCH_OMAP3
    { "compulab,omap3-sbc-t3517", omap3_sbc_t3517_legacy_init, },
    { "compulab,omap3-sbc-t3530", omap3_sbc_t3530_legacy_init, },
    { "compulab,omap3-sbc-t3730", omap3_sbc_t3730_legacy_init, },
    { "nokia,omap3-n900", nokia_n900_legacy_init, },
    { "nokia,omap3-n9", hsmmc2_internal_input_clk, },
    { "nokia,omap3-n950", hsmmc2_internal_input_clk, },
    { "isee,omap3-igep0020-rev-f", omap3_igep0020_rev_f_legacy_init, },
    { "isee,omap3-igep0030-rev-g", omap3_igep0030_rev_g_legacy_init, },
    { "logicpd,dm3730-torpedo-devkit", omap3_gpio126_127_129, },
    { "ti,omap3-evm-37xx", omap3_evm_legacy_init, },
    { "ti,am3517-evm", am3517_evm_legacy_init, },
    { "asd,halo-am3517", halo_am3517_legacy_init, },
    { "asd,fs5-am3517", fs5_am3517_legacy_init, },
    { "asd,fs5-am3517-dev", fs5_am3517_dev_legacy_init },
    { "technexion,omap3-tao3530", omap3_tao3530_legacy_init, },
    { "openpandora,omap3-pandora-600mhz", omap3_pandora_legacy_init, },
    { "openpandora,omap3-pandora-1ghz", omap3_pandora_legacy_init, },
#endif
#ifdef CONFIG_SOC_OMAP5
    { "ti,omap5-uevm", omap5_uevm_legacy_init, },
#endif
    { /* sentinel */ },
};

static void pdata_quirks_check(struct pdata_init *quirks)
{
    while (quirks->compatible) {
        if (of_machine_is_compatible(quirks->compatible)) {
            if (quirks->fn)
                quirks->fn();
            break;
        }
        quirks++;
    }
}

void __init pdata_quirks_init(const struct of_device_id *omap_dt_match_table)
{
    /*
     * We still need this for omap2420 and omap3 PM to work, others are
     * using drivers/misc/sram.c already.
     */
    if (of_machine_is_compatible("ti,omap2420") ||
        of_machine_is_compatible("ti,omap3"))
        omap_sdrc_init(NULL, NULL);

    pdata_quirks_check(auxdata_quirks);
    of_platform_populate(NULL, omap_dt_match_table,
                 omap_auxdata_lookup, NULL);
    pdata_quirks_check(pdata_quirks);
}
