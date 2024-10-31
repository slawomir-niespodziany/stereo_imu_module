#include <linux/delay.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>

//
#include "stereo_imu_mag.h"
#include "stereo_imu_mem.h"

/**
 * SPI interface.
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

static inline int spi_rd_data_m(uint32_t *pX, uint32_t *pY, uint32_t *pZ) {
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

static inline int spi_rd_status(uint8_t *pStatus) {
    uint8_t buf[] = {0x88u, 0u};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    int result = spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);

    if ((0 == result) && (NULL != pStatus)) {
        *pStatus = buf[1];
    }

    return result;
}

static inline int spi_wr_status(uint8_t t, uint8_t m) {
    uint8_t buf[] = {0x08u, ((0 != t) ? (0x02u) : (0x00u)) | ((0 != m) ? (0x01u) : (0x00u))};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    return spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);
}

static inline int spi_wr_ic0(uint8_t r, uint8_t s, uint8_t irq, uint8_t t, uint8_t m) {
    uint8_t buf[] = {0x09u, ((0 != r) ? (0x10u) : (0x00u)) |         //
                                ((0 != s) ? (0x08u) : (0x00u)) |     //
                                ((0 != irq) ? (0x04u) : (0x00u)) |   //
                                ((0 != t) ? (0x02u) : (0x00u)) |     //
                                ((0 != m) ? (0x01u) : (0x00u))};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    return spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);
}

static inline int spi_wr_ic1(uint8_t rst) {
    uint8_t buf[] = {0x0Au, (0 != rst) ? (0x80u) : (0x00u)};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    return spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);
}

static inline int spi_wr_ic2(uint8_t freq) {
    uint8_t buf[] = {0x0Bu, ((0 != freq) ? (0x08u) : (0x00u)) | (0x07u & freq)};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    return spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);
}

static inline int spi_rd_id(uint8_t *pId) {
    uint8_t buf[] = {0xAFu, 0u};
    struct spi_transfer spiTransfer = {.tx_buf = &buf[0], .rx_buf = &buf[0], .len = (sizeof(buf) / sizeof(buf[0]))};
    int result = spi_sync_transfer(mg_pSpiDevice, &spiTransfer, 1);

    if ((0 == result) && (NULL != pId)) {
        *pId = buf[1];
    }

    return result;
}

/**
 * Interrupt.
 */
static const uint8_t mg_irqPin = 194u;

static struct gpio_chip *mg_pGpioChip = NULL;
static unsigned int mg_irqLine;

static unsigned long mg_irqTimestamp;
static uint8_t mg_irqInProgress = 0u;

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
    mg_irqInProgress = 1u;
    mg_irqTimestamp = (1000ul * jiffies) / HZ;
    return IRQ_WAKE_THREAD;
}

static irqreturn_t irq_handler_bottom(int irq, void *dev_id) {
    entry_t entry = {
        .type = ENTRY_TYPE_DATA_MAG,
        .timestamp = mg_irqTimestamp,
    };
    int result;
    uint8_t status;

    result = spi_rd_data_m(&(entry.data_mag.x), &(entry.data_mag.y), &(entry.data_mag.z));
    // pr_info("spi_rd_data_m(&x, &y, &z)=%d, x=%5d, y=%5d, z=%5d.\n", result, entry.data_mag.x, entry.data_mag.y, entry.data_mag.z);

    result = spi_rd_status(&status);
    // pr_info("spi_rd_status(&status)=%d, status=0x%x, MD=%d.\n", result, status, (0 != (0x01u & status)));

    result = spi_wr_status(0u, 1u);
    // pr_info("spi_wr_status(0u, 1u)=%d.\n", result);

    stereo_imu_mem_lock();
    if (0 < stereo_imu_mem_empty()) {
        memcpy(stereo_imu_mem_writeAt(0u), &entry, sizeof(entry_t));
        stereo_imu_mem_write(1u);

    } else {
        pr_err("Magnetometer measurement could not be stored.");
    }
    stereo_imu_mem_unlock();

    mg_irqInProgress = 0u;
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

    disable_irq(irqLine);

    mg_pGpioChip = pGpioChip;
    mg_irqLine = irqLine;

    return 0;
}

static void irq_deinit(void) {
    if (NULL != mg_pGpioChip) {
        free_irq(mg_irqLine, NULL);
        mg_pGpioChip->free(mg_pGpioChip, mg_irqPin);
        mg_pGpioChip = NULL;
    }
}

/**
 * Magnetometer.
 */
static int mag_set_reset(void) {
    unsigned int n;
    const unsigned int N = 64u;
    uint8_t status;
    entry_t entry;

    spi_wr_ic0(0u, 1u, 0u, 0u, 0u);   // perform set
    msleep(10u);

    spi_wr_ic0(0u, 0u, 0u, 0u, 1u);   // measure

    for (n = 0; n < N; n++) {
        uint8_t status;
        spi_rd_status(&status);

        if (0 != (0x01u & status)) {
            break;
        }

        msleep(1u);
    }
    if (n == N) {
        pr_err("Magnetometer measurement not done for set.");
        return -1;
    }

    entry.type = ENTRY_TYPE_CALIB_MAG_SET;
    entry.timestamp = (1000 * jiffies) / HZ;

    spi_rd_data_m(&(entry.data_mag.x), &(entry.data_mag.y), &(entry.data_mag.z));   // read data
    spi_rd_status(&status);
    spi_wr_status(0u, 1u);

    stereo_imu_mem_lock();
    if (0 < stereo_imu_mem_empty()) {
        memcpy(stereo_imu_mem_writeAt(0u), &entry, sizeof(entry_t));
        stereo_imu_mem_write(1u);

    } else {
        pr_err("Magnetometer set measurement could not be stored.");
    }
    stereo_imu_mem_unlock();

    spi_wr_ic0(1u, 0u, 0u, 0u, 0u);   // perform reset
    msleep(10u);

    spi_wr_ic0(0u, 0u, 0u, 0u, 1u);   // measure

    for (n = 0; n < N; n++) {
        uint8_t status;
        spi_rd_status(&status);

        if (0 != (0x01u & status)) {
            break;
        }

        msleep(1u);
    }
    if (n == N) {
        pr_err("Magnetometer measurement not done for reset.");
        return -1;
    }

    entry.type = ENTRY_TYPE_CALIB_MAG_RESET;
    entry.timestamp = (1000 * jiffies) / HZ;

    spi_rd_data_m(&(entry.data_mag.x), &(entry.data_mag.y), &(entry.data_mag.z));   // read data
    spi_rd_status(&status);
    spi_wr_status(0u, 1u);

    stereo_imu_mem_lock();
    if (0 < stereo_imu_mem_empty()) {
        memcpy(stereo_imu_mem_writeAt(0u), &entry, sizeof(entry_t));
        stereo_imu_mem_write(1u);

    } else {
        pr_err("Magnetometer reset measurement could not be stored.");
    }
    stereo_imu_mem_unlock();

    return 0;
}

int stereo_imu_mag_enable(void) {
    unsigned int n;
    const unsigned int N = 128u;
    uint8_t id = 0;

    spi_wr_ic1(1u);   // reset
    msleep(20u);      // device starts in ~10ms

    for (n = 0; n < N; n++) {
        uint8_t status;
        spi_rd_status(&status);

        if (0 != (0x10 & status)) {
            break;
        }

        msleep(1u);
    }
    if (n == N) {
        pr_err("Magnetometer OTP not loaded correctly.");
        return -1;
    }

    spi_rd_id(&id);
    if (0x30 != id) {
        pr_err("Magnetometer id=0x%x invalid. Shall be 0x30.", id);
        return -1;
    }

    stereo_imu_mem_read(stereo_imu_mem_full());   // clear the buffer

    enable_irq(mg_irqLine);

    if (0 != mag_set_reset()) {
        return -1;
    }

    spi_wr_ic0(0u, 0u, 1u, 0u, 0u);   // irq on
    spi_wr_ic2(5u);                   // measure continously

    return 0;
}

void stereo_imu_mag_disable(void) {
    disable_irq(mg_irqLine);   // does it wait for the threaded irq part to finish too? assuming so - if not, then after clearing the entry
                               // buffer we may get a sample there sometimes (not a big deal but need to fix that in such case - below is a
                               // fix/method to check how in fact does it work)
    while (0u != mg_irqInProgress) {
        pr_err(
            "Bottom handler indeed still does some job 'after' disable_irq() returned. Keep the functionality of mg_irqInProgress or "
            "implement something else.\n");
    }   // now the bottom handler has definitely finished handling the last interrupt

    spi_wr_ic2(0u);                   // measure stop
    spi_wr_ic0(0u, 0u, 0u, 0u, 0u);   // irq off
}

/**
 * Module.
 */
int stereo_imu_mag_init(void) {
    if (0 != spi_init()) {
        return -1;
    }

    if (0 != irq_init()) {
        goto _l_spi_deinit;
    }

    return 0;

_l_spi_deinit:
    spi_deinit();   // revert spi_init() call

    return -1;
}

void stereo_imu_mag_deinit(void) {
    stereo_imu_mag_disable();
    irq_deinit();
    spi_deinit();
}
