#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>

//
#include "stereo_imu_mag.h"

/**
 * Magnetometer SPI interface.
 */

static struct spi_device *mg_pSpiDevice = NULL;

static int spi_init(void) {
    const uint8_t bus = 1u;
    const uint8_t chipSelect = 0u;

    struct spi_master *pSpiMaster;
    char deviceName[64];
    struct device *pDevice;
    struct spi_device *pSpiDevice;
    int result;

    pSpiMaster = spi_busnum_to_master(bus);
    if (NULL == pSpiMaster) {
        pr_err("spi_busnum_to_master(%d)=NULL\n", bus);
        return -1;
    }

    snprintf(&deviceName[0], sizeof(deviceName) / sizeof(deviceName[0]), "%s.%u", dev_name(&pSpiMaster->dev), chipSelect);
    pDevice = bus_find_device_by_name(&spi_bus_type, NULL, &deviceName[0]);
    if (NULL == pDevice) {
        pr_err("bus_find_device_by_name(\"%s\")=NULL\n", &deviceName[0]);
        return -1;
    }

    pSpiDevice = to_spi_device(pDevice);
    if (NULL == pSpiDevice) {
        pr_err("to_spi_device().\n");
        return -1;
    }

    pSpiDevice->max_speed_hz = 250000u;
    pSpiDevice->bits_per_word = 8u;
    pSpiDevice->mode = SPI_MODE_0;
    result = spi_setup(pSpiDevice);
    if (0 != result) {
        pr_err("spi_setup(\"%s\")=%d\n", &deviceName[0], result);
        return -1;
    }

    mg_pSpiDevice = pSpiDevice;
    pr_info("spi_init() ok, deviceName=\"%s\", bitrate=%d, mode=%d\n", &deviceName[0], pSpiDevice->max_speed_hz, pSpiDevice->mode);

    return 0;
}

static void spi_deinit(void) {}

static int spi_rd_data_m(uint32_t *pX, uint32_t *pY, uint32_t *pZ) {
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

static int spi_rd_status(uint8_t *pStatus) {
    uint8_t buf[] = {0x88u, 0u};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    int result = spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);

    if ((0 == result) && (NULL != pStatus)) {
        *pStatus = buf[1];
    }

    return result;
}

static int spi_wr_status(uint8_t t, uint8_t m) {
    uint8_t buf[] = {0x08u, ((0 != t) ? (0x02u) : (0x00u)) | ((0 != m) ? (0x01u) : (0x00u))};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    return spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);
}

static int spi_wr_ic0(uint8_t irq, uint8_t t, uint8_t m) {
    uint8_t buf[] = {0x09u, ((0 != irq) ? (0x04u) : (0x00u)) | ((0 != t) ? (0x02u) : (0x00u)) | ((0 != m) ? (0x01u) : (0x00u))};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    return spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);
}

static int spi_wr_ic1(uint8_t rst) {
    uint8_t buf[] = {0x0Au, (0 != rst) ? (0x80u) : (0x00u)};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    return spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);
}

static int spi_wr_ic2(uint8_t freq) {
    uint8_t buf[] = {0x0Bu, ((0 != freq) ? (0x08u) : (0x00u)) | (0x07u & freq)};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    return spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);
}

static int spi_rd_id(uint8_t *pId) {
    uint8_t buf[] = {0xAFu, 0u};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    int result = spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);

    if ((0 == result) && (NULL != pId)) {
        *pId = buf[1];
    }

    return result;
}

/**
 * Magnetometer interrupt.
 */
static const uint8_t mg_irqPin = 194u;

static struct gpio_chip *mg_pGpioChip = NULL;
static unsigned int mg_irqLine;

static unsigned long mg_initTimestamp;
static unsigned long mg_irqTimestamp;
static unsigned long mg_prevIrqTimestamp;

static int gpiochip_match(struct gpio_chip *chip, void *data) {
    const char *pLabel = (const char *) data;
    int result;

    result = strcmp(chip->label, pLabel);
    if (0 != result) {
        pr_info("Not matched: gpio_chip->label=\"%s\" vs label=\"%s\"", chip->label, pLabel);
    }

    return (0 == result);
}

static irqreturn_t irq_handler_top(int irq, void *dev_id) {
    mg_irqTimestamp = jiffies - mg_initTimestamp;
    pr_info("mg_irqTimestamp=%lu\n", mg_irqTimestamp);
    return IRQ_WAKE_THREAD;
}

static irqreturn_t irq_handler_bottom(int irq, void *dev_id) {
    uint32_t x, y, z;
    uint8_t status;

    pr_info("irqDiff=%lu\n", mg_irqTimestamp - mg_prevIrqTimestamp);
    mg_prevIrqTimestamp = mg_irqTimestamp;

    // if (0u != g_cont) {
    //     if (0u == g_measuring) {
    //         g_measuring = 1u;
    //         pr_info("_wr_ic0(1u, 0u, 0u)=%d.\n", _wr_ic0(1u, 0u, 0u));   // irq on
    //         pr_info("_wr_ic2(1u)=%d.\n", _wr_ic2(1u));                   // measure continously
    //     }
    // } else {
    //     if (0u != g_measuring) {
    //         g_measuring = 0u;
    //         pr_info("_wr_ic2(1u)=%d.\n", _wr_ic2(0u));   // measure continously
    //     }
    // }

    // pr_info("spi_rd_status(&status)=%d, status=0x%x, MD=%d.\n", spi_rd_status(&status), status, (0 != (0x01u & status)));
    pr_info("spi_rd_data_m(&x, &y, &z)=%d, x=%5d, y=%5d, z=%5d.\n", spi_rd_data_m(&x, &y, &z), x, y, z);
    pr_info("spi_rd_status(&status)=%d, status=0x%x, MD=%d.\n", spi_rd_status(&status), status, (0 != (0x01u & status)));
    pr_info("spi_wr_status(0u, 1u)=%d.\n", spi_wr_status(0u, 1u));
    // pr_info("spi_rd_status(&status)=%d, status=0x%x, MD=%d.\n", spi_rd_status(&status), status, (0 != (0x01u & status)));

    return IRQ_HANDLED;
}

static int irq_init(void) {
    char gpioChipLabel[] = "tegra-gpio";

    struct gpio_chip *pGpioChip;
    int result;
    unsigned int irqLine;

    pGpioChip = gpiochip_find(&gpioChipLabel[0], &gpiochip_match);
    if (NULL == pGpioChip) {
        pr_err("gpiochip_find(\"%s\")=NULL\n", &gpioChipLabel[0]);
        return -1;
    }

    if (mg_irqPin >= pGpioChip->ngpio) {
        pr_err("(mg_irqPin=%d) >= (pGpioChip->ngpio=%d)\n", mg_irqPin, pGpioChip->ngpio);
        return -1;
    }

    result = pGpioChip->request(pGpioChip, mg_irqPin);
    if (0 != result) {
        pr_err("pGpioChip->request(%d)=%d\n", mg_irqPin, result);
        return -1;
    }

    result = pGpioChip->direction_input(pGpioChip, mg_irqPin);
    if (0 != result) {
        pGpioChip->free(pGpioChip, mg_irqPin);   // revert pGpioChip->request() call

        pr_err("pGpioChip->direction_input(%d)=%d\n", mg_irqPin, result);
        return -1;
    }

    result = pGpioChip->to_irq(pGpioChip, mg_irqPin);
    if (0 > result) {
        pGpioChip->free(pGpioChip, mg_irqPin);   // revert pGpioChip->request() call

        pr_err("pGpioChip->to_irq(%d)=%d\n", mg_irqPin, result);
        return -1;
    }
    irqLine = result;

    if (request_threaded_irq(irqLine, &irq_handler_top, &irq_handler_bottom, IRQ_TYPE_EDGE_RISING, "stereo_imu", NULL)) {
        pGpioChip->free(pGpioChip, mg_irqPin);   // revert pGpioChip->request() call

        pr_err("request_threaded_irq(%d)=%d\n", irqLine, result);
        return -1;
    }

    mg_pGpioChip = pGpioChip;
    mg_irqLine = irqLine;
    mg_initTimestamp = jiffies;

    return 0;
}

static void irq_deinit(void) {
    if (NULL != mg_pGpioChip) {
        free_irq(mg_irqLine, NULL);
        mg_pGpioChip->free(mg_pGpioChip, mg_irqPin);
    }
}

/**
 * Magnetometer driver.
 */
static int mag_init(void) {
    unsigned int n;
    const unsigned int N = 128u;
    uint8_t id = 0;

    pr_info("spi_wr_ic1(1u)=%d\n", spi_wr_ic1(1u));   // reset
    for (n = 0; n < N; n++) {
        uint8_t status;
        pr_info("spi_rd_status(&status)=%d, status=0x%x, OTP=%d\n", spi_rd_status(&status), status, (0 != (0x10 & status)));

        if (0 != (0x10 & status)) {
            break;
        }
    }

    pr_info("spi_rd_id(&id)=%d, id=0x%x\n", spi_rd_id(&id), id);   // read id

    // TODO
    pr_info("spi_wr_ic0(1u, 0u, 0u)=%d.\n", spi_wr_ic0(1u, 0u, 0u));   // irq on
    pr_info("spi_wr_ic2(1u)=%d.\n", spi_wr_ic2(1u));                   // measure continously

    return 0;
}

static void mag_deinit(void) {
    pr_info("spi_wr_ic2(1u)=%d.\n", spi_wr_ic2(0u));                   // measure stop
    pr_info("spi_wr_ic0(1u, 0u, 0u)=%d.\n", spi_wr_ic0(0u, 0u, 0u));   // irq off
}

int stereo_imu_mag_init(void) {
    if (0 != spi_init()) {
        return -1;
    }

    if (0 != irq_init()) {
        goto _l_spi_deinit;
    }

    if (0 != mag_init()) {
        goto _l_irq_deinit;
    }

    return 0;

_l_irq_deinit:
    irq_deinit();   // revert irq_init() call

_l_spi_deinit:
    spi_deinit();   // revert spi_init() call

    return -1;
}

void stereo_imu_mag_deinit(void) {
    irq_deinit();
    spi_deinit();
    mag_deinit();
}
