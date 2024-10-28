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
#include "stereo_imu_imm.h"
#include "stereo_imu_mag.h"
#include "stereo_imu_mem.h"

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

static ssize_t _write(struct file *pFile, const char __user *pBuffer, size_t size, loff_t *pOffset) { return size; }

static int _open(struct inode *pInode, struct file *pFile) {
    stereo_imu_mag_enable();
    stereo_imu_imm_enable();
    return 0;
}

static int _release(struct inode *pInode, struct file *pFile) {
    stereo_imu_imm_disable();
    stereo_imu_mag_disable();
    return 0;
}

static dev_t mg_dev;
static struct cdev mg_cdev;
static struct file_operations mg_fileOperations = {
    .owner = THIS_MODULE,
    .read = _read,
    .write = _write,
    .open = _open,
    .release = _release,
};
static struct class *mg_pClass;

static int __init stereo_imu_module_init(void) {
    int result;

    result = alloc_chrdev_region(&mg_dev, 0, 1, "stereo_imu");
    if (0 != result) {
        pr_err("alloc_chrdev_region()=%d\n", result);
        return -1;
    }

    cdev_init(&mg_cdev, &mg_fileOperations);

    result = cdev_add(&mg_cdev, mg_dev, 1);
    if (0 > result) {
        pr_err("cdev_add()=%d\n", result);
        goto _l_unregister_chrdev_region;
    }

    mg_pClass = class_create(THIS_MODULE, "stereo_imu");
    if (IS_ERR(mg_pClass)) {
        pr_err("class_create()=ERR_PTR\n");
        goto _l_cdev_del;
    }

    if (IS_ERR(device_create(mg_pClass, NULL, mg_dev, NULL, "stereo_imu"))) {
        pr_err("device_create()=ERR_PTR\n");
        goto _l_class_destroy;
    }

    result = stereo_imu_mem_init();
    if (0 != result) {
        pr_err("stereo_imu_mem_init()=%d\n", result);
        goto _l_device_destroy;
    }

    result = stereo_imu_mag_init();
    if (0 != result) {
        pr_err("stereo_imu_mag_init()=%d\n", result);
        goto _l_stereo_imu_mem_deinit;
    }

    result = stereo_imu_imm_init();
    if (0 != result) {
        pr_err("stereo_imu_imm_init()=%d\n", result);
        goto _l_stereo_imu_mag_deinit;
    }

    pr_info("Module stereo_imu initialized.\n");

    return 0;

_l_stereo_imu_mag_deinit:
    stereo_imu_mag_deinit();

_l_stereo_imu_mem_deinit:
    stereo_imu_mem_deinit();

_l_device_destroy:
    device_destroy(mg_pClass, mg_dev);

_l_class_destroy:
    class_destroy(mg_pClass);

_l_cdev_del:
    cdev_del(&mg_cdev);

_l_unregister_chrdev_region:
    unregister_chrdev_region(mg_dev, 1);

    return -1;
}

static void __exit stereo_imu_module_exit(void) {
    stereo_imu_imm_deinit();
    stereo_imu_mag_deinit();
    stereo_imu_mem_deinit();

    device_destroy(mg_pClass, mg_dev);
    class_destroy(mg_pClass);
    cdev_del(&mg_cdev);
    unregister_chrdev_region(mg_dev, 1);

    pr_info("Module stereo_imu deinitialized.\n");
}

module_init(stereo_imu_module_init);
module_exit(stereo_imu_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Slawomir Niespodziany (sniespod@gmail.com)");
MODULE_DESCRIPTION("Driver module for stereo_imu PCB.");
MODULE_VERSION("0.1");
