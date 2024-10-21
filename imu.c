#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>

static struct spi_device *mg_pSpiDevice = 0;

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
    // pr_info("Device File Opened...!!!\n");
    return 0;
}

static int etx_release(struct inode *inode, struct file *file) {
    // pr_info("Device File Closed...!!!\n");
    return 0;
}

static ssize_t etx_read(struct file *filp, char __user *buf, size_t len, loff_t *off) {
    unsigned char txBuf[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    unsigned char rxBuf[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    struct spi_transfer spiTransfer = {.tx_buf = &txBuf[0],   //
                                       .rx_buf = &rxBuf[0],   //
                                       .len = 16u};

    pr_info("Read function - xfer %d: %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x.\n",
            spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1), rxBuf[0], rxBuf[1], rxBuf[2], rxBuf[3], rxBuf[4], rxBuf[5], rxBuf[6],
            rxBuf[7], rxBuf[8], rxBuf[9], rxBuf[10], rxBuf[11], rxBuf[12], rxBuf[13], rxBuf[14], rxBuf[15]);
    // pr_info("Read function - xfer %ld.\n", spi_w8r8(mg_pSpiDevice, 0x85));

    return 0;
}

// --------------------------------------------------------------------------------------------------

static int _rd_data_m(uint32_t *pX, uint32_t *pY, uint32_t *pZ) {
    uint8_t buf[] = {0x80u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    int result;

    result = spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);
    if (0 == result) {
        if (NULL != pX) {
            *pX = (((uint32_t) buf[1]) << 10u) | (((uint32_t) buf[2]) << 2u) | ((uint32_t)((0xC0 & buf[7]) >> 6u));
        }
        if (NULL != pY) {
            *pY = (((uint32_t) buf[3]) << 10u) | (((uint32_t) buf[4]) << 2u) | ((uint32_t)((0x30 & buf[7]) >> 4u));
        }
        if (NULL != pZ) {
            *pZ = (((uint32_t) buf[5]) << 10u) | (((uint32_t) buf[6]) << 2u) | ((uint32_t)((0x0C & buf[7]) >> 2u));
        }
    }

    return result;
}

static int _rd_data_t(int16_t *pT) {
    uint8_t buf[] = {0x87u, 0u};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    int result;

    result = spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);
    if ((0 == result) && (NULL != pT)) {
        *pT = ((((int16_t) buf[1]) << 3u) / 10) - 75;
    }

    return result;
}

static int _rd_data_mt(uint32_t *pX, uint32_t *pY, uint32_t *pZ, int16_t *pT) {
    uint8_t buf[] = {0x80u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    int result;

    result = spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);
    if (0 == result) {
        if (NULL != pX) {
            *pX = (((uint32_t) buf[1]) << 10u) | (((uint32_t) buf[2]) << 2u) | ((uint32_t)((0xC0 & buf[7]) >> 6u));
        }
        if (NULL != pY) {
            *pY = (((uint32_t) buf[3]) << 10u) | (((uint32_t) buf[4]) << 2u) | ((uint32_t)((0x30 & buf[7]) >> 4u));
        }
        if (NULL != pZ) {
            *pZ = (((uint32_t) buf[5]) << 10u) | (((uint32_t) buf[6]) << 2u) | ((uint32_t)((0x0C & buf[7]) >> 2u));
        }
        if (NULL != pT) {
            *pT = ((((int16_t) buf[1]) << 3u) / 10) - 75;
        }
    }

    return result;
}

static int _rd_status(uint8_t *pStatus) {
    uint8_t buf[] = {0x88u, 0u};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    int result = spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);

    if ((0 == result) && (NULL != pStatus)) {
        *pStatus = buf[1];
    }

    return result;
}

static int _wr_status(uint8_t t, uint8_t m) {
    uint8_t buf[] = {0x08u, ((0 != t) ? (0x02u) : (0x00u)) | ((0 != m) ? (0x01u) : (0x00u))};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    return spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);
}

static int _wr_ic0(uint8_t irq, uint8_t t, uint8_t m) {
    uint8_t buf[] = {0x09u, ((0 != irq) ? (0x04u) : (0x00u)) | ((0 != t) ? (0x02u) : (0x00u)) | ((0 != m) ? (0x01u) : (0x00u))};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    return spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);
}

static int _wr_ic1(uint8_t rst) {
    uint8_t buf[] = {0x0Au, (0 != rst) ? (0x80u) : (0x00u)};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    return spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);
}

static int _wr_ic2(uint8_t freq) {
    uint8_t buf[] = {0x0Bu, ((0 != freq) ? (0x08u) : (0x00u)) | (0x07u & freq)};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    return spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);
}

static int _rd_id(uint8_t *pId) {
    uint8_t buf[] = {0xAFu, 0u};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    int result = spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);

    if ((0 == result) && (NULL != pId)) {
        *pId = buf[1];
    }

    return result;
}

// -------------------------------------------------------------------------------

#define GPIO_PIN (194)
static struct gpio_chip *g_gpioChip = NULL;
static int mg_irqNumber = 0;
static struct task_struct *etx_thread;

static int match_chip(struct gpio_chip *chip, void *data) {
    pr_info("Label: \"%s\" vs \"%s\", %d", chip->label, (char *) data, !strcmp(chip->label, data));
    pr_info("Name: %s", chip->parent->init_name);
    pr_info("OF Node Full Name: %s", chip->of_node->full_name);

    return !strcmp(chip->label, data);
}

static uint8_t irqPending = 0;
static unsigned long g_j = 0;

static irqreturn_t _irq_handler(int irq, void *dev_id) {
    unsigned long j = jiffies;
    unsigned long dms = (j - g_j) * 4ul;

    pr_err("Irq handled diff=%10lu, %s.\n", (0ul != g_j) ? dms : 1000, ((0ul != g_j) && (1300 < dms)) ? ("LOST") : (""));

    g_j = j;
    irqPending = 1;

    return IRQ_HANDLED;
}

uint8_t g_cont = 0u;
uint8_t g_measuring = 0u;

static int thread_function(void *pv) {
    static int i = 0;

    while (!kthread_should_stop()) {
        if (0u != g_cont) {
            if (0u == g_measuring) {
                g_measuring = 1u;
                pr_info("_wr_ic0(1u, 0u, 0u)=%d.\n", _wr_ic0(1u, 0u, 0u));   // irq on
                pr_info("_wr_ic2(1u)=%d.\n", _wr_ic2(1u));                   // measure continously
            }
        } else {
            if (0u != g_measuring) {
                g_measuring = 0u;
                pr_info("_wr_ic2(1u)=%d.\n", _wr_ic2(0u));   // measure continously
            }
        }

        if (0 != irqPending) {
            uint32_t x, y, z;
            uint8_t status;

            irqPending = 0;

            pr_info("In EmbeTronicX Thread Function %d\n", i++);
            pr_info("_rd_status(&status)=%d, status=0x%x, MD=%d.\n", _rd_status(&status), status, (0 != (0x01u & status)));
            pr_info("_rd_data_m(&x, &y, &z)=%d, x=%5d, y=%5d, z=%5d.\n", _rd_data_m(&x, &y, &z), x, y, z);
            pr_info("_rd_status(&status)=%d, status=0x%x, MD=%d.\n", _rd_status(&status), status, (0 != (0x01u & status)));
            pr_info("_wr_status(0u, 1u)=%d.\n", _wr_status(0u, 1u));
            pr_info("_rd_status(&status)=%d, status=0x%x, MD=%d.\n", _rd_status(&status), status, (0 != (0x01u & status)));
        }

        msleep(1);
    }

    return 0;
}

// -------------------------------------------------------------------------------

static ssize_t etx_write(struct file *filp, const char __user *b, size_t n, loff_t *off) {
    uint8_t cmd;
    if ((0 < n) && (0 == copy_from_user(&cmd, b, n))) {
        if ('I' == cmd) {
            uint8_t id;
            pr_info("_rd_id(&id)=%d, id=0x%x.\n", _rd_id(&id), id);

        } else if ('R' == cmd) {
            pr_info("_wr_ic1(1u)=%d.\n", _wr_ic1(1u));

        } else if ('S' == cmd) {
            uint8_t status;
            pr_info("_rd_status(&status)=%d, status=0x%x, OTP=%d, T=%d, M=%d.\n", _rd_status(&status), status,   //
                    (0 != (0x10u & status)), (0 != (0x02u & status)), (0 != (0x01u & status)));

        } else if ('T' == cmd) {
            pr_info("_wr_ic0(0u, 1u, 0u)=%d.\n", _wr_ic0(0u, 1u, 0u));

        } else if ('t' == cmd) {
            int16_t t = 0;
            pr_info("_rd_data_t(&t)=%d, t=%d.\n", _rd_data_t(&t), t);

        } else if ('$' == cmd) {   // smart reset
            const unsigned int N = 1024;
            unsigned int n;

            pr_info("_wr_ic1(1u)=%d.\n", _wr_ic1(1u));   // reset

            for (n = 0; n < N; n++) {
                uint8_t status;
                pr_info("_rd_status(&status)=%d, status=0x%x, OTP=%d.\n", _rd_status(&status), status, (0 != (0x10u & status)));

                if (0 != (0x10u & status)) {   // check OTP bit
                    break;
                }
            }

            if (n < N) {
                uint8_t id;

                pr_info("OTP=1, #OTP=%d.\n", n);
                pr_info("_rd_id(&id)=%d, id=0x%x.\n", _rd_id(&id), id);

                if (0x30 == id) {
                    pr_info("ID=0x%x.\n", id);
                } else {
                    pr_err("ID=0x%x.\n", id);
                }
            } else {
                pr_err("OTP=0.\n");
            }

        } else if ('%' == cmd) {   // Read data (T)
            const unsigned int N = 1024;
            unsigned int n;

            pr_info("_wr_ic0(0u, 1u, 0u)=%d.\n", _wr_ic0(0u, 1u, 0u));   // measure T

            for (n = 0; n < N; n++) {
                uint8_t status;
                pr_info("_rd_status(&status)=%d, status=0x%x, TD=%d.\n", _rd_status(&status), status, (0 != (0x02u & status)));

                if (0 != (0x02u & status)) {   // check TD bit
                    break;
                }
            }

            if (n < N) {
                int16_t t;
                pr_info("TD=1, #TD=%d.\n", n);
                pr_info("_rd_data_t(&t)=%d, t=%d.\n", _rd_data_t(&t), t);
            } else {
                pr_err("TD=0.\n");
            }

        } else if ('&' == cmd) {   // Read data (M)
            const unsigned int N = 1024;
            unsigned int n;

            pr_info("_wr_ic0(0u, 0u, 1u)=%d.\n", _wr_ic0(0u, 0u, 1u));   // measure M

            for (n = 0; n < N; n++) {
                uint8_t status;
                pr_info("_rd_status(&status)=%d, status=0x%x, MD=%d.\n", _rd_status(&status), status, (0 != (0x01u & status)));

                if (0 != (0x01u & status)) {   // check MD bit
                    break;
                }
            }

            if (n < N) {
                uint32_t x, y, z;
                uint8_t status;
                pr_info("MD=1, #MD=%d.\n", n);
                pr_info("_rd_data_m(&x, &y, &z)=%d, x=%5d, y=%5d, z=%5d.\n", _rd_data_m(&x, &y, &z), x, y, z);
                pr_info("_rd_status(&status)=%d, status=0x%x, MD=%d.\n", _rd_status(&status), status, (0 != (0x01u & status)));
            } else {
                pr_err("MD=0.\n");
            }

        } else if ('#' == cmd) {   // Measure continously (M)
            g_cont = 1u - g_cont;

        } else {
            pr_info("Unknown operation.\n");
        }
    } else {
        pr_info("Unspecified operation.\n");
    }

    return n;
}

static int setup_spi1(void) {
    const unsigned bus = 1u;
    const unsigned chipSelect = 0u;

    struct spi_master *pSpiMaster;
    char deviceName[64];
    struct device *pDevice;
    int result;

    pSpiMaster = spi_busnum_to_master(bus);
    if (NULL == pSpiMaster) {
        pr_err("spi_busnum_to_master(%d) failed.\n", bus);
        return -1;
    }

    snprintf(&deviceName[0], sizeof(deviceName), "%s.%u", dev_name(&pSpiMaster->dev), chipSelect);
    pDevice = bus_find_device_by_name(&spi_bus_type, NULL, &deviceName[0]);
    if (NULL == pDevice) {
        pr_err("bus_find_device_by_name(\"%s\") failed.\n", &deviceName[0]);
        return -1;
    }

    mg_pSpiDevice = to_spi_device(pDevice);
    if (NULL == mg_pSpiDevice) {
        pr_err("to_spi_device().\n");
        return -1;
    }

    mg_pSpiDevice->max_speed_hz = 250000u;
    mg_pSpiDevice->bits_per_word = 8u;
    mg_pSpiDevice->mode = SPI_MODE_0;
    result = spi_setup(mg_pSpiDevice);
    if (0 != result) {
        pr_err("spi_setup(\"%s\") failed with %d.\n", &deviceName[0], result);
        return -1;
    }

    return 0;
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

    g_gpioChip = gpiochip_find("tegra-gpio", &match_chip);
    if (NULL == g_gpioChip) {
        pr_err("gpiochip_find()\n");
        goto r_device;
    }
    pr_info("Got valid GPIO Chip Total num gpios %d\n", g_gpioChip->ngpio);

    result = g_gpioChip->request(g_gpioChip, GPIO_PIN);
    if (0 > result) {
        pr_err("g_gpioChip->request()\n");
        goto r_gpio_free;
    }
    pr_info("g_gpioChip->request() = %d\n", result);

    result = g_gpioChip->direction_input(g_gpioChip, GPIO_PIN);
    printk(KERN_DEBUG "g_gpioChip->direction_input() = %d\n", result);

    result = g_gpioChip->to_irq(g_gpioChip, GPIO_PIN);
    printk(KERN_DEBUG "g_gpioChip->to_irq() = %d\n", result);

    mg_irqNumber = result;

    if (request_irq(mg_irqNumber, _irq_handler, IRQ_TYPE_EDGE_RISING, "imuDevice", NULL)) {
        pr_err("my_device: cannot register IRQ 11111");
        goto r_gpio_free;
    }

    etx_thread = kthread_run(thread_function, NULL, "eTx Thread");
    if (etx_thread) {
        pr_info("Kthread Created Successfully...\n");
    } else {
        pr_err("Cannot create kthread\n");
        goto r_gpio_free;
    }

    setup_spi1();

    pr_info("Device Driver Insert...Done!!!\n");
    return 0;

r_gpio_free:
    g_gpioChip->free(g_gpioChip, GPIO_PIN);
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
    kthread_stop(etx_thread);

    free_irq(mg_irqNumber, NULL);
    g_gpioChip->free(g_gpioChip, GPIO_PIN);

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
