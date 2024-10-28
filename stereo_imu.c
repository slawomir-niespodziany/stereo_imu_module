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
#include "stereo_imu_mem.h"

dev_t dev = 0;
static struct class *dev_class;
static struct cdev etx_cdev;

static int _open(struct inode *pInode, struct file *pFile) {
    stereo_imu_mag_enable();
    return 0;
}

static int _release(struct inode *pInode, struct file *pFile) {
    stereo_imu_mag_disable();
    return 0;
}

static ssize_t _read(struct file *pFile, char __user *pBuffer, size_t size, loff_t *pOffset) {
    size_t n;
    unsigned int full;
    int result;
    const entry_t *pEntry;
    char buffer[128];

    n = 0;
    full = stereo_imu_mem_full();
    while ((0 < full--) && ((n + (sizeof(buffer) / sizeof(buffer[0]))) <= size)) {
        result = 0;
        pEntry = stereo_imu_mem_readAt(0u);
        switch (pEntry->type) {
            case ENTRY_TYPE_CALIB_MAG_SET:
                result = snprintf(&buffer[0], sizeof(buffer), "ts=%lu, type=CALIB_MAG_SET, x=%d, y=%d, z=%d\n",   //
                                  pEntry->timestamp, pEntry->data_mag.x, pEntry->data_mag.y, pEntry->data_mag.z);
                break;

            case ENTRY_TYPE_CALIB_MAG_RESET:
                result = snprintf(&buffer[0], sizeof(buffer), "ts=%lu, type=CALIB_MAG_RESET, x=%d, y=%d, z=%d\n",   //
                                  pEntry->timestamp, pEntry->data_mag.x, pEntry->data_mag.y, pEntry->data_mag.z);
                break;

            case ENTRY_TYPE_DATA_MAG:
                result = snprintf(&buffer[0], sizeof(buffer), "ts=%lu, type=DATA_MAG, x=%d, y=%d, z=%d\n",   //
                                  pEntry->timestamp, pEntry->data_mag.x, pEntry->data_mag.y, pEntry->data_mag.z);
                break;

            case ENTRY_TYPE_DATA_GYRO:
                result = snprintf(&buffer[0], sizeof(buffer), "ts=%lu, type=DATA_GYRO, x=%d, y=%d, z=%d\n",   //
                                  pEntry->timestamp, pEntry->data_gyro.x, pEntry->data_gyro.y, pEntry->data_gyro.z);
                break;

            case ENTRY_TYPE_DATA_ACC:
                result = snprintf(&buffer[0], sizeof(buffer), "ts=%lu, type=DATA_ACC, x=%d, y=%d, z=%d\n",   //
                                  pEntry->timestamp, pEntry->data_acc.x, pEntry->data_acc.y, pEntry->data_acc.z);
                break;

            default:
                pr_err("Unknown entry type: ts=%lu, type=%d\n", pEntry->timestamp, pEntry->type);
                break;
        }
        stereo_imu_mem_read(1u);

        if ((0 < result) && (result <= (sizeof(buffer) / sizeof(buffer[0])))) {
            if (0ul == copy_to_user(pBuffer + n, &buffer[0], result)) {
                n += result;

            } else {
                pr_err("Copying serialized entry failed: ts=%lu, type=%d", pEntry->timestamp, pEntry->type);
            }
        } else {
            pr_err("Serializing entry failed: ts=%lu, type=%d", pEntry->timestamp, pEntry->type);
        }
    }

    return n;
}

static ssize_t _write(struct file *filp, const char __user *b, size_t n, loff_t *off) {
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

static struct file_operations mg_fileOperations = {
    .owner = THIS_MODULE,
    .read = _read,
    .write = _write,
    .open = _open,
    .release = _release,
};

static int __init stereo_imu_module_init(void) {
    int result;

    if ((alloc_chrdev_region(&dev, 0, 1, "etx_Dev")) < 0) {
        pr_err("Cannot allocate major number\n");
        goto r_unreg;
    }
    pr_info("Major = %d Minor = %d \n", MAJOR(dev), MINOR(dev));

    cdev_init(&etx_cdev, &mg_fileOperations);

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

    result = stereo_imu_mem_init();
    if (0 != result) {
        pr_err("stereo_imu_mag_init()=%d\n", result);
        goto r_device;
    }

    result = stereo_imu_mag_init();
    if (0 != result) {
        pr_err("stereo_imu_mag_init()=%d\n", result);
        stereo_imu_mem_deinit();
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

static void __exit stereo_imu_module_exit(void) {
    stereo_imu_mag_deinit();
    stereo_imu_mem_deinit();

    device_destroy(dev_class, dev);
    class_destroy(dev_class);
    cdev_del(&etx_cdev);
    unregister_chrdev_region(dev, 1);
    pr_info("Device Driver Remove...Done!!\n");
}

module_init(stereo_imu_module_init);
module_exit(stereo_imu_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Slawomir Niespodziany (sniespod@gmail.com)");
MODULE_DESCRIPTION("Driver module for stereo_imu PCB.");
MODULE_VERSION("0.1");
