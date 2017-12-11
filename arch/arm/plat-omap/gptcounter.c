/*
 *  Copyright (C) 2017 ASD, Inc.
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <plat/dmtimer.h>

#define DRIVER_NAME     "ti-gptcounter"

#define COUNTER_CLASS   "ti-gptcounter"

#define FILTER_SIZE             5

struct ti_gptcounter {
    struct omap_dm_timer *gptimer;
    struct device *dev;
    u8 pulse_per_rev;
    u8 pulse_counter;
    u32 speed_filter[FILTER_SIZE];
    u8 filter_index;
    u32 motor_rpm;
    u32 rev_start_time;
    u32 clk_hz;
    u32 sum_count;
    bool enabled;
};

static irqreturn_t counter_irq_handler(int irq, void *dev_id)
{
    struct ti_gptcounter *counter = (struct ti_gptcounter *) dev_id;
    unsigned long rev_end_time;
    unsigned long time_diff;
    unsigned int retval;
    unsigned int i;
    unsigned int sum = 0;

    retval = omap_dm_timer_read_status(counter->gptimer);
    if (!retval) {
        if (counter->gptimer)
            omap_dm_timer_set_int_disable(counter->gptimer, OMAP_TIMER_INT_CAPTURE);
        return IRQ_NONE;
    }

    // Clear the interrupt flags
    omap_dm_timer_write_status(counter->gptimer, 
        OMAP_TIMER_INT_MATCH	|
        OMAP_TIMER_INT_OVERFLOW	|
        OMAP_TIMER_INT_CAPTURE);

    if (counter->pulse_per_rev % 2)
        counter->pulse_counter++;
    else
        counter->pulse_counter += 2;

    if (counter->pulse_counter >= counter->pulse_per_rev) {
        if (counter->pulse_per_rev % 2)
            rev_end_time = omap_dm_timer_read_capture(counter->gptimer, 1);
        else
            rev_end_time = omap_dm_timer_read_capture(counter->gptimer, 2);

        // Handle rollover
        if (rev_end_time < counter->rev_start_time) {
            time_diff = (UINT_MAX - counter->rev_start_time) + rev_end_time;
        } else {
            time_diff = rev_end_time - counter->rev_start_time;
        }        
        counter->rev_start_time = rev_end_time;

        if (time_diff > 0) {
            counter->speed_filter[counter->filter_index] = (u32) (((counter->clk_hz * 1000) / time_diff) * 60 / 1000);
            counter->filter_index = (counter->filter_index + 1) % FILTER_SIZE;
            if (counter->sum_count < FILTER_SIZE) {
                counter->sum_count++;
            }

            for (i = 0; i < counter->sum_count; i++) {
                sum += counter->speed_filter[i];
            }

            counter->motor_rpm = sum / counter->sum_count;
        }

        counter->pulse_counter = 0;
    }

    return IRQ_HANDLED;
}

static ssize_t ti_gptcounter_get_rpm(struct device *dev, struct device_attribute *attr,
    char *buf) 
{
    struct ti_gptcounter *counter = (struct ti_gptcounter *) dev_get_drvdata(dev);
    return sprintf(buf, "%d\n", counter->motor_rpm);
}

static ssize_t ti_gptcounter_show_name(struct device *dev, struct device_attribute *attr,
    char *buf) 
{
    return sprintf(buf, "%s\n", DRIVER_NAME);
}

static ssize_t ti_gptcounter_enable(struct device *dev, struct device_attribute *attr, 
    const char *buf, size_t count) 
{
    unsigned long val;
    int ret;
    struct ti_gptcounter *counter;

    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    counter = (struct ti_gptcounter *) dev_get_drvdata(dev);
    if (val) {
        counter->filter_index = 0;
        memset(counter->speed_filter, 0, sizeof(counter->speed_filter));
        counter->sum_count = 0;
        counter->pulse_counter = 0;
        omap_dm_timer_set_int_enable(counter->gptimer, OMAP_TIMER_INT_CAPTURE);
        omap_dm_timer_start(counter->gptimer);
        omap_dm_timer_write_counter(counter->gptimer, 0);
        counter->enabled = true;
    } else {
        omap_dm_timer_set_int_disable(counter->gptimer, OMAP_TIMER_INT_CAPTURE);
        omap_dm_timer_stop(counter->gptimer);
        counter->enabled = false;
    }    

    return count;
}

static ssize_t ti_gptcounter_show_enable(struct device *dev, struct device_attribute *attr,
    char *buf)
{
    struct ti_gptcounter *counter = (struct ti_gptcounter *) dev_get_drvdata(dev);
    return sprintf(buf, "%d\n", counter->enabled);
}

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO, ti_gptcounter_show_enable, ti_gptcounter_enable);
static DEVICE_ATTR(rpm, S_IRUGO, ti_gptcounter_get_rpm, NULL);
static DEVICE_ATTR(name, S_IRUGO, ti_gptcounter_show_name, NULL);

static struct attribute *ti_gptcounter_attributes[] = {    
    &dev_attr_enable.attr,
    &dev_attr_rpm.attr,
    &dev_attr_name.attr,
    NULL
};

static const struct attribute_group ti_gptcounter_group = {
    .attrs = ti_gptcounter_attributes,
};

static const struct of_device_id ti_gptcounter_of_match[] = {
    { .compatible = "ti-gptcounter", },
    { },
};
MODULE_DEVICE_TABLE(of, ti_gptcounter_of_match);

static int ti_gptcounter_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    int err;
    const struct of_device_id *of_id = of_match_device(ti_gptcounter_of_match, dev);
    struct device_node *np = dev->of_node;
    struct device_node *gptnp;
    const __be32 *parp;
    struct ti_gptcounter *counter;
    int irq;
    int pulse_per_rev;    
    int capture_pulses;
    struct clk *clk_fclk;

    if (!of_id) {
        dev_err(dev, "missing device tree entry\n");
        return -EINVAL;
    }

    parp = of_get_property(np, "gptimer", NULL);
    if (!parp) {
        dev_err(dev, "missing gptimer tree entry\n");
        return -EINVAL;
    }
    gptnp = of_find_node_by_phandle(be32_to_cpup(parp));
    if (!gptnp) {
        dev_err(dev, "could not find gptimer\n");
        return -EINVAL;
    }

    err = of_property_read_u32(np, "pp-rev", &pulse_per_rev);
    if (err) {
        dev_err(dev, "missing pp-rev property\n");
        return -EINVAL;
    }

    counter = kzalloc(sizeof(struct ti_gptcounter), GFP_KERNEL);    
    if (!counter) {        
        dev_err(dev, "failed to reserve memory for counter\n");
        err = -ENOMEM;
        goto exit_free_mem;
    }

    counter->pulse_per_rev = pulse_per_rev;
    counter->pulse_counter = 0;

    counter->gptimer = omap_dm_timer_request_by_node(gptnp);
    if (!counter->gptimer) {
        dev_err(dev, "failed to request gptimer\n");        
        err = -EINVAL;
        goto exit_free_mem;
    }

    // Set clock source to system clock
    omap_dm_timer_set_source(counter->gptimer, OMAP_TIMER_SRC_SYS_CLK);

    // Use 1:256 prescalar
    omap_dm_timer_set_prescaler(counter->gptimer, 7);

    clk_fclk = omap_dm_timer_get_fclk(counter->gptimer);
    counter->clk_hz = clk_get_rate(clk_fclk) / 256;

    irq = omap_dm_timer_get_irq(counter->gptimer);

    err = request_irq(irq, counter_irq_handler, 0, 
        "gptcounter", counter);
    if (err) {
        dev_err(dev, "request_irq failed (on irq %d)\n", irq);
        goto exit_free_mem;
    }    

    if (counter->pulse_per_rev % 2) 
        capture_pulses = OMAP_TIMER_CAPTURE_FIRST;
    else
        capture_pulses = OMAP_TIMER_CAPTURE_SECOND;

    omap_dm_timer_set_capture(counter->gptimer, capture_pulses, 
        OMAP_TIMER_CAPTURE_FALLING);

    // Clear the interrupt flags
    omap_dm_timer_write_status(counter->gptimer, 
        OMAP_TIMER_INT_MATCH	|
        OMAP_TIMER_INT_OVERFLOW	|
        OMAP_TIMER_INT_CAPTURE);

    // Enable capture interrupt
    omap_dm_timer_set_int_enable(counter->gptimer, OMAP_TIMER_INT_CAPTURE);

    err = sysfs_create_group(&pdev->dev.kobj, &ti_gptcounter_group);

    platform_set_drvdata(pdev, counter);

    return 0;

exit_free_mem:
    if (counter->gptimer)
        omap_dm_timer_free(counter->gptimer);

    kfree(counter);

    return err;
}

static int ti_gptcounter_remove(struct platform_device *pdev)
{
    struct ti_gptcounter *counter = platform_get_drvdata(pdev);

    omap_dm_timer_stop(counter->gptimer);

    omap_dm_timer_free(counter->gptimer);
    sysfs_remove_group(&pdev->dev.kobj, &ti_gptcounter_group);

    kfree(counter);

    return 0;
}

static struct platform_driver ti_gptcounter_driver = {
    .probe = ti_gptcounter_probe,
    .remove = ti_gptcounter_remove,
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = of_match_ptr(ti_gptcounter_of_match),
    }
};
module_platform_driver(ti_gptcounter_driver);

MODULE_ALIAS("platform: " DRIVER_NAME);
MODULE_DESCRIPTION("Counter using the TI General Purpose Timer input");
MODULE_AUTHOR("Sean Donohue <sean.donohue@panalytical.com");
MODULE_LICENSE("GPL v2");