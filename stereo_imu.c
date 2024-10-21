#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>

//
#include "stereo_imu_mag.h"

dev_t dev = 0;
static struct class *dev_class;
static struct cdev etx_cdev;

static int __init etx_driver_init(void);
static void __exit etx_driver_exit(void);

static int etx_open(struct inode *inode, struct file *file);
static int etx_release(struct inode *inode, struct file *file);
static ssize_t etx_read(struct file *filp, char __user *buf, size_t len, loff_t *off);
static ssize_t etx_write(struct file *filp, const char *buf, size_t len, loff_t *off);

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .read = etx_read,
    .write = etx_write,
    .open = etx_open,
    .release = etx_release,
};

static int etx_open(struct inode *inode, struct file *file) {
    pr_info("Device File Opened...!!!\n");
    return 0;
}

static int etx_release(struct inode *inode, struct file *file) {
    pr_info("Device File Closed...!!!\n");
    return 0;
}

static ssize_t etx_read(struct file *filp, char __user *buf, size_t len, loff_t *off) {
    pr_info("Read!!!\n");
    return 0;
}

static ssize_t etx_write(struct file *filp, const char __user *b, size_t n, loff_t *off) {
    // uint8_t cmd;
    // if ((0 < n) && (0 == copy_from_user(&cmd, b, n))) {
    //     if ('I' == cmd) {
    //         uint8_t id;
    //         pr_info("_rd_id(&id)=%d, id=0x%x.\n", _rd_id(&id), id);

    //     } else if ('R' == cmd) {
    //         pr_info("_wr_ic1(1u)=%d.\n", _wr_ic1(1u));

    //     } else if ('S' == cmd) {
    //         uint8_t status;
    //         pr_info("_rd_status(&status)=%d, status=0x%x, OTP=%d, T=%d, M=%d.\n", _rd_status(&status), status,   //
    //                 (0 != (0x10u & status)), (0 != (0x02u & status)), (0 != (0x01u & status)));

    //     } else if ('T' == cmd) {
    //         pr_info("_wr_ic0(0u, 1u, 0u)=%d.\n", _wr_ic0(0u, 1u, 0u));

    //     } else if ('t' == cmd) {
    //         int16_t t = 0;
    //         pr_info("_rd_data_t(&t)=%d, t=%d.\n", _rd_data_t(&t), t);

    //     } else if ('$' == cmd) {   // smart reset
    //         const unsigned int N = 1024;
    //         unsigned int n;

    //         pr_info("_wr_ic1(1u)=%d.\n", _wr_ic1(1u));   // reset

    //         for (n = 0; n < N; n++) {
    //             uint8_t status;
    //             pr_info("_rd_status(&status)=%d, status=0x%x, OTP=%d.\n", _rd_status(&status), status, (0 != (0x10u & status)));

    //             if (0 != (0x10u & status)) {   // check OTP bit
    //                 break;
    //             }
    //         }

    //         if (n < N) {
    //             uint8_t id;

    //             pr_info("OTP=1, #OTP=%d.\n", n);
    //             pr_info("_rd_id(&id)=%d, id=0x%x.\n", _rd_id(&id), id);

    //             if (0x30 == id) {
    //                 pr_info("ID=0x%x.\n", id);
    //             } else {
    //                 pr_err("ID=0x%x.\n", id);
    //             }
    //         } else {
    //             pr_err("OTP=0.\n");
    //         }

    //     } else if ('%' == cmd) {   // Read data (T)
    //         const unsigned int N = 1024;
    //         unsigned int n;

    //         pr_info("_wr_ic0(0u, 1u, 0u)=%d.\n", _wr_ic0(0u, 1u, 0u));   // measure T

    //         for (n = 0; n < N; n++) {
    //             uint8_t status;
    //             pr_info("_rd_status(&status)=%d, status=0x%x, TD=%d.\n", _rd_status(&status), status, (0 != (0x02u & status)));

    //             if (0 != (0x02u & status)) {   // check TD bit
    //                 break;
    //             }
    //         }

    //         if (n < N) {
    //             int16_t t;
    //             pr_info("TD=1, #TD=%d.\n", n);
    //             pr_info("_rd_data_t(&t)=%d, t=%d.\n", _rd_data_t(&t), t);
    //         } else {
    //             pr_err("TD=0.\n");
    //         }

    //     } else if ('&' == cmd) {   // Read data (M)
    //         const unsigned int N = 1024;
    //         unsigned int n;

    //         pr_info("_wr_ic0(0u, 0u, 1u)=%d.\n", _wr_ic0(0u, 0u, 1u));   // measure M

    //         for (n = 0; n < N; n++) {
    //             uint8_t status;
    //             pr_info("_rd_status(&status)=%d, status=0x%x, MD=%d.\n", _rd_status(&status), status, (0 != (0x01u & status)));

    //             if (0 != (0x01u & status)) {   // check MD bit
    //                 break;
    //             }
    //         }

    //         if (n < N) {
    //             uint32_t x, y, z;
    //             uint8_t status;
    //             pr_info("MD=1, #MD=%d.\n", n);
    //             pr_info("_rd_data_m(&x, &y, &z)=%d, x=%5d, y=%5d, z=%5d.\n", _rd_data_m(&x, &y, &z), x, y, z);
    //             pr_info("_rd_status(&status)=%d, status=0x%x, MD=%d.\n", _rd_status(&status), status, (0 != (0x01u & status)));
    //         } else {
    //             pr_err("MD=0.\n");
    //         }

    //     } else if ('#' == cmd) {   // Measure continously (M)
    //         g_cont = 1u - g_cont;

    //     } else {
    //         pr_info("Unknown operation.\n");
    //     }
    // } else {
    //     pr_info("Unspecified operation.\n");
    // }

    return n;
}

static int __init etx_driver_init(void) {
    int result;

    if ((alloc_chrdev_region(&dev, 0, 1, "etx_Dev")) < 0) {
        pr_err("Cannot allocate major number\n");
        goto r_unreg;
    }
    pr_info("Major = %d Minor = %d \n", MAJOR(dev), MINOR(dev));

    cdev_init(&etx_cdev, &fops);

    if ((cdev_add(&etx_cdev, dev, 1)) < 0) {
        pr_err("Cannot add the device to the system\n");
        goto r_del;
    }

    if (IS_ERR(dev_class = class_create(THIS_MODULE, "etx_class"))) {
        pr_err("Cannot create the struct class\n");
        goto r_class;
    }

    if (IS_ERR(device_create(dev_class, NULL, dev, NULL, "etx_device"))) {
        pr_err("Cannot create the Device \n");
        goto r_device;
    }

    result = stereo_imu_mag_init();
    if (0 != result) {
        pr_err("stereo_imu_mag_init()=%d\n", result);
        goto r_device;
    }

    pr_info("Device Driver Insert...Done!!!\n");
    return 0;

r_device:
    device_destroy(dev_class, dev);
r_class:
    class_destroy(dev_class);
r_del:
    cdev_del(&etx_cdev);
r_unreg:
    unregister_chrdev_region(dev, 1);

    return -1;
}

static void __exit etx_driver_exit(void) {
    stereo_imu_mag_deinit();

    device_destroy(dev_class, dev);
    class_destroy(dev_class);
    cdev_del(&etx_cdev);
    unregister_chrdev_region(dev, 1);
    pr_info("Device Driver Remove...Done!!\n");
}

module_init(etx_driver_init);
module_exit(etx_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("EmbeTronicX <embetronicx@gmail.com>");
MODULE_DESCRIPTION("A simple device driver - GPIO Driver (GPIO Interrupt) ");
MODULE_VERSION("1.33");
