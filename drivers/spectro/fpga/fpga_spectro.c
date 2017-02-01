/* halo_fpga.c
 *
 * Loadable kernel module for the Halo FPGA
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string.h>

#include <asm/uaccess.h>
#include "../../gpio/gpiolib.h"

static u32 write_strobe_delay = 0;
module_param(write_strobe_delay, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(write_strobe_delay, "Write strobe delay time");

static u32 trans_strobe_delay = 0;
module_param(trans_strobe_delay, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(trans_strobe_delay, "End of write transaction strobe delay time");

#define DEVICE_NAME				"fpga_spectro"
#define CHAR_DEVICE_NAME        "halo_fpga"
#define CLASS_NAME              "asd"
#define DISABLE_SPIN_LOCKS      (0)

static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);

static struct file_operations fops =
{
   .open = dev_open,
   .read = dev_read,
   .write = dev_write,
   .release = dev_release,
};

struct halo_fpga_platform_data {
	struct gpio_descs *bidi_descs;
	struct gpio_desc *nData_desc;
	struct gpio_desc *nWrite_desc;
	struct gpio_desc *nAddr_desc;
	struct gpio_desc *nWait_desc;
	struct gpio_desc *interrupt_desc;

	struct platform_device *pdev;
	struct device *char_dev;
	struct class *char_class;
	int majorNumber;

	u32 port_data_mask;
	u32 port_data_shift;

	void __iomem *port_data_set;
	void __iomem *port_data_clr;
	void __iomem *port_data_oe;
	void __iomem *port_data_in;
	void __iomem *port_data_out;

	u8 port_dir;
};

static struct halo_fpga_platform_data fpga_pdata;

static const struct of_device_id halo_fpga_id_table[] = {
    { .compatible = "asd,halo" },
    { .compatible = "asd,fs5" },
    { },
};

#define OMAP24XX_GPIO_OE			0x0034
#define OMAP24XX_GPIO_DATAIN		0x0038
#define OMAP24XX_GPIO_DATAOUT		0x003c
#define OMAP24XX_GPIO_CLEARDATAOUT	0x0090
#define OMAP24XX_GPIO_SETDATAOUT	0x0094

#define FPGA_PORT_WIDTH      8 //bits

#define FPGA_OK                   0
#define FPGA_WAIT_TIMEOUT         -3

#define PORT_DIR_OUT              0
#define PORT_DIR_IN               1

#define CMD_WRITE_ADDR            0x01
#define CMD_READ_ADDR             0x02
#define CMD_WRITE_DATA            0x03
#define CMD_READ_DATA             0x04

#if !DISABLE_SPIN_LOCKS
static DEFINE_SPINLOCK(fpga_lock);
#endif

// gpiolib functions have too much over-head. Must use register direct writing
// for optimal timing.
static inline void set_bidi_pins(u8 port_val)
{
    u32 val_set;
    u32 val_clr;

    val_set = (((u32) (port_val)) << fpga_pdata.port_data_shift) & fpga_pdata.port_data_mask;
    val_clr = (((u32) (~port_val)) << fpga_pdata.port_data_shift) & fpga_pdata.port_data_mask;
  	writel(val_set, fpga_pdata.port_data_set);
  	writel(val_clr, fpga_pdata.port_data_clr);
}

static inline u8 get_bidi_pins(void)
{
    return (u8) ((readl(fpga_pdata.port_data_in) & fpga_pdata.port_data_mask) >> fpga_pdata.port_data_shift);
}

static inline int halo_fpga_wait_busy(void)
{
    unsigned long timeout = jiffies + usecs_to_jiffies(1);

    while (!gpiod_get_value(fpga_pdata.nWait_desc)) {
		if (time_after(jiffies, timeout)) {
			return FPGA_WAIT_TIMEOUT;
		}
	}
	return FPGA_OK;
}

static inline int halo_fpga_wait_done(void)
{
    unsigned long timeout = jiffies + usecs_to_jiffies(1);

    while (gpiod_get_value(fpga_pdata.nWait_desc)) {
		if (time_after(jiffies, timeout)) {
			return FPGA_WAIT_TIMEOUT;
		}
	}
	return FPGA_OK;
}

static inline void halo_fpga_set_port_out(void)
{
    // Set the data pins as outputs
    u32 val;

    val = readl(fpga_pdata.port_data_oe);
    val &= ~(fpga_pdata.port_data_mask);
    writel(val, fpga_pdata.port_data_oe);

    fpga_pdata.port_dir = PORT_DIR_OUT;
}

static inline void halo_fpga_set_port_in(void)
{
    // Set the data pins as inputs
    u32 val;

    val = readl(fpga_pdata.port_data_oe);
    val |= fpga_pdata.port_data_mask;
    writel(val, fpga_pdata.port_data_oe);

    fpga_pdata.port_dir = PORT_DIR_IN;
}

static inline size_t halo_fpga_write_data(const void *buf, size_t len)
{
    size_t retval;

    // Check that our port direction is out
    if (fpga_pdata.port_dir != PORT_DIR_OUT)
        halo_fpga_set_port_out();

    // Bring Write Strobe Low, wait 1 us
    gpiod_set_value(fpga_pdata.nWrite_desc, 1);

    // Put the data on the port and bring the data strobe low
    set_bidi_pins(*((u8 *) buf));
    gpiod_set_value(fpga_pdata.nData_desc, 1);

    // Wait for WAIT to go high
    if (halo_fpga_wait_busy() != FPGA_OK) {
        dev_err(&fpga_pdata.pdev->dev, "Data write bus wait timeout\n");
        retval = FPGA_WAIT_TIMEOUT;
    }
    else
        retval = len;

    // Bring data strobe high
    gpiod_set_value(fpga_pdata.nData_desc, 0);

    // Bring write strobe high and set the data back to 0
    gpiod_set_value(fpga_pdata.nWrite_desc, 0);
    set_bidi_pins(0);

    // Wait for bus to clear
    if (halo_fpga_wait_done() != FPGA_OK) {
        dev_err(&fpga_pdata.pdev->dev, "Data write bus clear timeout\n");
        retval = FPGA_WAIT_TIMEOUT;
    }

    return retval;
}

static inline size_t halo_fpga_read_data(void *buf, size_t len)
{
    u8 *data_ptr = (u8 *) buf;
    size_t retval = len;
    u8 data;

    // Check that our port direction is in
    if (fpga_pdata.port_dir == PORT_DIR_OUT)
        halo_fpga_set_port_in();

    while (len-- > 0) {
	   // Bring Data Strobe Low
    	gpiod_set_value(fpga_pdata.nData_desc, 1);

	    // Wait for WAIT to go high
	    if (halo_fpga_wait_busy() != FPGA_OK) {
    		dev_err(&fpga_pdata.pdev->dev, "Data read bus wait timeout\n");
    		retval = FPGA_WAIT_TIMEOUT;

    		gpiod_set_value(fpga_pdata.nData_desc, 0);
    		goto out;
        }

        // Read the value on the port
        data = get_bidi_pins();

        // Bring data strobe high
        gpiod_set_value(fpga_pdata.nData_desc, 0);

        // Wait for bus to clear
        if (halo_fpga_wait_done() != FPGA_OK) {
            dev_err(&fpga_pdata.pdev->dev, "Data read bus clear timeout\n");
            retval = FPGA_WAIT_TIMEOUT;
        }

        // Mask and shift the data
        *(data_ptr++) = data;
    }

out:
    return retval;
}

static inline size_t halo_fpga_write_addr(const void *buf, size_t len)
{
    size_t retval;

    // Check that our port direction is out
    if (fpga_pdata.port_dir != PORT_DIR_OUT)
        halo_fpga_set_port_out();

    // Bring Write Strobe Low
    gpiod_set_value(fpga_pdata.nWrite_desc, 1);

    // Put the address on the port and bring the address strobe low
    set_bidi_pins(*((u8 *) buf));
    gpiod_set_value(fpga_pdata.nAddr_desc, 1);

    // Wait for WAIT to go high
    if (halo_fpga_wait_busy() != FPGA_OK) {
        dev_err(&fpga_pdata.pdev->dev, "Address write bus wait timeout\n");
        retval = FPGA_WAIT_TIMEOUT;
    }
    else
        retval = len;

    // Bring address strobe high
    gpiod_set_value(fpga_pdata.nAddr_desc, 0);

    // Bring write strobe high and set the data back to 0
    gpiod_set_value(fpga_pdata.nWrite_desc, 0);
    set_bidi_pins(0);

    // Wait for bus to clear
    if (halo_fpga_wait_done() != FPGA_OK) {
        dev_err(&fpga_pdata.pdev->dev, "Address write bus clear timeout\n");
        retval = FPGA_WAIT_TIMEOUT;
    }

    return retval;
}

static inline size_t halo_fpga_read_addr(void *buf, size_t len)
{
    u8 *addr_ptr = (u8 *) buf;
    size_t retval;
    u8 data;

    // Check that our port direction is in
    if (fpga_pdata.port_dir == PORT_DIR_OUT)
        halo_fpga_set_port_in();

    // Bring Address Strobe Low
    gpiod_set_value(fpga_pdata.nAddr_desc, 1);

    // Wait for WAIT to go high
    if (halo_fpga_wait_busy() != FPGA_OK) {
        dev_err(&fpga_pdata.pdev->dev, "Address read bus wait timeout\n");
        retval = FPGA_WAIT_TIMEOUT;
    }
    else
        retval = len;

    // Read the value on the port
    data = get_bidi_pins();

    // Bring address strobe high
    gpiod_set_value(fpga_pdata.nAddr_desc, 0);

    // Wait for bus to clear
    if (halo_fpga_wait_done() != FPGA_OK) {
        dev_err(&fpga_pdata.pdev->dev, "Address read bus clear timeout\n");
        retval = FPGA_WAIT_TIMEOUT;
    }

    // Return the data
    *addr_ptr = data;

    return retval;
}

/* Initialize the LKM */
int __init halo_fpga_init(void)
{
    int ret;
    u8 status;

    dev_info(&fpga_pdata.pdev->dev, "Initializing character device\n");

    // Try to dynamically allocate a major number for the device -- more difficult but worth it
	fpga_pdata.majorNumber = register_chrdev(0, CHAR_DEVICE_NAME, &fops);
    if (fpga_pdata.majorNumber < 0){
        pr_alert("halo_fpga: Failed to register a major number");
        ret = fpga_pdata.majorNumber;
        goto err0;
	}
    dev_dbg(&fpga_pdata.pdev->dev, "Registered correctly with major number %d\n", fpga_pdata.majorNumber);

	// Register the device class
    fpga_pdata.char_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(fpga_pdata.char_class)) {                // Check for error and clean up if there is
		unregister_chrdev(fpga_pdata.majorNumber, CHAR_DEVICE_NAME);
		pr_alert("halo_fpga: Failed to register device class");
		ret = PTR_ERR(fpga_pdata.char_class);          // Correct way to return an error on a pointer
        goto err1;
	}
	dev_dbg(&fpga_pdata.pdev->dev, "Device class registered correctly\n");

    // Register the device driver
	fpga_pdata.char_dev = device_create(fpga_pdata.char_class, NULL, MKDEV(fpga_pdata.majorNumber, 0), NULL, CHAR_DEVICE_NAME);
    if (IS_ERR(fpga_pdata.char_dev)){               // Clean up if there is an error
        class_destroy(fpga_pdata.char_class);           // Repeated code but the alternative is goto statements
        unregister_chrdev(fpga_pdata.majorNumber, CHAR_DEVICE_NAME);

        dev_alert(&fpga_pdata.pdev->dev, "Failed to create the device\n");
        ret = PTR_ERR(fpga_pdata.char_dev);
        goto err2;
    }

    halo_fpga_set_port_out();

    set_bidi_pins(0);

    if (!write_strobe_delay) {
        write_strobe_delay = 0;
    }

    if (!trans_strobe_delay) {
        trans_strobe_delay = 0;
    }

    // Perform a status register read to verify functionality
    status = 0xE;
    if (halo_fpga_write_addr(&status, 1) <= 0) {
        dev_err(&fpga_pdata.pdev->dev, "could not write status register address\n");
        goto err3;
    }
    if (halo_fpga_read_data(&status, 1) <= 0) {
        dev_err(&fpga_pdata.pdev->dev, "could not read status register\n");
        goto err3;
    }

    dev_info(&fpga_pdata.pdev->dev, "device class created correctly\n");
    return 0;

err3:
    device_destroy(fpga_pdata.char_class, MKDEV(fpga_pdata.majorNumber, 0));     // remove the device
err2:
    class_unregister(fpga_pdata.char_class);
    class_destroy(fpga_pdata.char_class);
err1:
    unregister_chrdev(fpga_pdata.majorNumber, CHAR_DEVICE_NAME);
err0:
    return ret;
}
EXPORT_SYMBOL(halo_fpga_init);

/* Cleanup - undo whatever init_module did */
static void __exit halo_fpga_exit(void)
{
    device_destroy(fpga_pdata.char_class, MKDEV(fpga_pdata.majorNumber, 0));     // remove the device
    class_unregister(fpga_pdata.char_class);                          // unregister the device class
    class_destroy(fpga_pdata.char_class);                             // remove the device class
    unregister_chrdev(fpga_pdata.majorNumber, CHAR_DEVICE_NAME);             // unregister the major number
    dev_info(&fpga_pdata.pdev->dev, "unloading character driver\n");
}
EXPORT_SYMBOL(halo_fpga_exit);

/** @brief The device open function that is called each time the device is opened
 *  This will only increment the numberOpens counter in this case.
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_open(struct inode *inodep, struct file *filep)
{
    dev_dbg(&fpga_pdata.pdev->dev, "opened\n");
    return 0;
}

/** @brief This function is called whenever device is being read from user space i.e. data is
 *  being sent from the device to the user. In this case is uses the copy_to_user() function to
 *  send the buffer string to the user and captures any errors.
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 *  @param buffer The pointer to the buffer to which this function writes the data
 *  @param len The length of the b
 *  @param offset The offset if required
 */
static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
    uint8_t *data;
    uint8_t cmd;
    ssize_t retval = len;
    unsigned long flags;

    if (len < 2) {
        dev_warn(&fpga_pdata.pdev->dev, "Invalid command for read 0x%x\n", buffer[0]);
        return -EFAULT;
    }

    data = kzalloc(len - 1, GFP_KERNEL);

    retval = copy_from_user(&cmd, buffer, 1);
    if (retval < 0) {
        dev_err(&fpga_pdata.pdev->dev, "failed to copy read data from user space\n");
        goto out;
    }

#if !DISABLE_SPIN_LOCKS
    spin_lock_irqsave(&fpga_lock, flags);
#endif

    switch(cmd) {
    case CMD_READ_ADDR:
        retval = halo_fpga_read_addr((void *) data, (len - 1));
        break;

    case CMD_READ_DATA:
        retval = halo_fpga_read_data((void *) data, (len - 1));
        break;

    default:
        retval = -EFAULT;
        break;
    }

#if !DISABLE_SPIN_LOCKS
    spin_unlock_irqrestore(&fpga_lock, flags);
#endif

    if (retval > 0) {
        retval = copy_to_user(buffer, data, (len - 1));
        if (retval < 0) {
            dev_warn(&fpga_pdata.pdev->dev, "Failed to copy data to user space\n");
        }
    }

out:
    kfree(data);

    return retval;
}

/** @brief This function is called whenever the device is being written to from user space i.e.
 *  data is sent to the device from the user. The data is copied to the message[] array in this
 *  LKM using the sprintf() function along with the length of the string.
 *  @param filep A pointer to a file object
 *  @param buffer The buffer to that contains the string to write to the device
 *  @param len The length of the array of data that is being passed in the const char buffer
 *  @param offset The offset if required
 */
static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
    size_t retval = len;
    uint8_t *cmd_buf;
    unsigned long flags;

    // Parse the command from the input character buffer
    if (len < 1) {
        dev_warn(&fpga_pdata.pdev->dev, "Invalid write command 0x%x\n", buffer[0]);
        retval = -EFAULT;
        goto out;
    }

    cmd_buf = kzalloc(len, GFP_KERNEL);
    retval = copy_from_user(cmd_buf, buffer, len);

    if (retval < 0) {
        dev_err(&fpga_pdata.pdev->dev, "failed to copy write data from user space\n");
        goto out1;
    }

#if !DISABLE_SPIN_LOCKS
    spin_lock_irqsave(&fpga_lock, flags);
#endif

    switch (cmd_buf[0]) {
    case CMD_WRITE_ADDR:
        retval = halo_fpga_write_addr((const char *) &cmd_buf[1], 1);
        break;

    case CMD_WRITE_DATA:
        retval = halo_fpga_write_data((const char *) &cmd_buf[1], (len - 1));
        break;

    default:
        retval = -EFAULT;
    }

#if !DISABLE_SPIN_LOCKS
    spin_unlock_irqrestore(&fpga_lock, flags);
#endif

out1:
    kfree(cmd_buf);

out:
    return retval;
}

/** @brief The device release function that is called whenever the device is closed/released by
 *  the userspace program
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_release(struct inode *inodep, struct file *filep)
{
    dev_dbg(&fpga_pdata.pdev->dev, "released\n");
    return 0;
}

static int halo_fpga_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;
	void __iomem *port_base_address;
	struct gpio_desc *desc;
	struct gpio_descs *descs;
	int retval;

	of_id = of_match_device(halo_fpga_id_table, &pdev->dev);

	fpga_pdata.pdev = pdev;

	if (!of_id) {
		dev_err(&fpga_pdata.pdev->dev, "could not find device tree entry");
		retval = -ENODEV;
		goto err0;
	}

	descs = devm_gpiod_get_array(&fpga_pdata.pdev->dev, "bidi", GPIOD_IN);
	if (IS_ERR(descs)) {
		dev_err(&fpga_pdata.pdev->dev, "bidi-gpios not defined");
		retval = PTR_ERR(descs);
		goto err0;
	}

	fpga_pdata.bidi_descs = descs;

    // Set up the port base addresses
    port_base_address = (void __iomem *) gpiod_get_bank_base(fpga_pdata.bidi_descs->desc[0]);
    fpga_pdata.port_data_set = port_base_address + OMAP24XX_GPIO_SETDATAOUT;
    fpga_pdata.port_data_clr = port_base_address + OMAP24XX_GPIO_CLEARDATAOUT;
    fpga_pdata.port_data_oe = port_base_address + OMAP24XX_GPIO_OE;
    fpga_pdata.port_data_in = port_base_address + OMAP24XX_GPIO_DATAIN;
    fpga_pdata.port_data_out = port_base_address + OMAP24XX_GPIO_DATAOUT;

    fpga_pdata.port_data_shift = (desc_to_gpio(fpga_pdata.bidi_descs->desc[0]) - 32);
    fpga_pdata.port_data_mask = (0xFFul) << fpga_pdata.port_data_shift;

    // Note that the strobes are marked active low in the device tree, so when setting
    // them to their default state, they need to be set to low so the value on the pin
    // is actually high.
    //desc = devm_gpiod_get(&fpga_pdata.pdev->dev, "ndata", GPIOD_OUT_HIGH);
    desc = devm_gpiod_get(&fpga_pdata.pdev->dev, "ndata", GPIOD_OUT_LOW);
    if (IS_ERR(desc)) {
    	dev_err(&fpga_pdata.pdev->dev, "ndata-gpios not defined");
    	retval = PTR_ERR(desc);
    	goto err1;
    }
    fpga_pdata.nData_desc = desc;

    //desc = devm_gpiod_get(&fpga_pdata.pdev->dev, "naddr", GPIOD_OUT_HIGH);
    desc = devm_gpiod_get(&fpga_pdata.pdev->dev, "naddr", GPIOD_OUT_LOW);
    if (IS_ERR(desc)) {
    	dev_err(&fpga_pdata.pdev->dev, "naddr-gpios not defined");
    	retval = PTR_ERR(desc);
    	goto err2;
    }
    fpga_pdata.nAddr_desc = desc;

    //desc = devm_gpiod_get(&fpga_pdata.pdev->dev, "nwrite", GPIOD_OUT_HIGH);
    desc = devm_gpiod_get(&fpga_pdata.pdev->dev, "nwrite", GPIOD_OUT_LOW);
    if (IS_ERR(desc)) {
    	dev_err(&fpga_pdata.pdev->dev, "nwrite-gpios not defined");
    	retval = PTR_ERR(desc);
    	goto err3;
    }
    fpga_pdata.nWrite_desc = desc;

    desc = devm_gpiod_get(&fpga_pdata.pdev->dev, "nwait", GPIOD_IN);
    if (IS_ERR(desc)) {
    	dev_err(&fpga_pdata.pdev->dev, "nwait-gpios not defined");
    	retval = PTR_ERR(desc);
    	goto err4;
    }
    fpga_pdata.nWait_desc = desc;

    // Initialize the INTERRUPT pin as an input, but no need to store it
    // because it isn't used by this driver. It does need to be an input
    // or the FPGA will be confused if the MPU is driving it.
    desc = devm_gpiod_get(&fpga_pdata.pdev->dev, "nint", GPIOD_IN);
    if (IS_ERR(desc)) {
    	dev_err(&fpga_pdata.pdev->dev, "nint-gpios not defined");
    	retval = PTR_ERR(desc);
		goto err5;
    }

    retval = halo_fpga_init();
    if (retval)
    	goto err5;

    return 0;

err5:
	devm_gpiod_put(&fpga_pdata.pdev->dev, fpga_pdata.nWait_desc);
err4:
	devm_gpiod_put(&fpga_pdata.pdev->dev, fpga_pdata.nWrite_desc);
err3:
	devm_gpiod_put(&fpga_pdata.pdev->dev, fpga_pdata.nAddr_desc);
err2:
	devm_gpiod_put(&fpga_pdata.pdev->dev, fpga_pdata.nData_desc);
err1:
	devm_gpiod_put_array(&fpga_pdata.pdev->dev, fpga_pdata.bidi_descs);
err0:
	return retval;
}

static int halo_fpga_remove(struct platform_device *pdev)
{
	halo_fpga_exit();

	devm_gpiod_put(&fpga_pdata.pdev->dev, fpga_pdata.nWait_desc);
	devm_gpiod_put(&fpga_pdata.pdev->dev, fpga_pdata.nWrite_desc);
	devm_gpiod_put(&fpga_pdata.pdev->dev, fpga_pdata.nAddr_desc);
	devm_gpiod_put(&fpga_pdata.pdev->dev, fpga_pdata.nData_desc);
	devm_gpiod_put_array(&fpga_pdata.pdev->dev, fpga_pdata.bidi_descs);

	return 0;
}

static struct platform_driver fpga_driver = {
	.probe = halo_fpga_probe,
	.remove = halo_fpga_remove,
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = halo_fpga_id_table,
	},
};

module_platform_driver(fpga_driver);
MODULE_AUTHOR("Sean Donohue");
MODULE_DESCRIPTION("Halo FPGA character driver");
MODULE_VERSION("0.0.1");
MODULE_LICENSE("GPL");

//MODULE_DEVICE_TABLE(of, halo_fpga_id_table);
//module_init(halo_fpga_init);
//module_exit(halo_fpga_exit);
