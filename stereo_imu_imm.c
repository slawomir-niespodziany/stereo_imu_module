#include <linux/delay.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>

//
#include "stereo_imu_imm.h"
#include "stereo_imu_mem.h"

/**
 * SPI interface.
 */
// static struct spi_device *mg_pSpiDevice = NULL;

// static int spi_init(void) {
//     const uint8_t bus = 0u;
//     const uint8_t chipSelect = 0u;

//     struct spi_master *pSpiMaster;
//     char deviceName[64];
//     struct device *pDevice;
//     struct spi_device *pSpiDevice;
//     int result;

//     pSpiMaster = spi_busnum_to_master(bus);
//     if (NULL == pSpiMaster) {
//         pr_err("spi_busnum_to_master(%d)=NULL\n", bus);
//         return -1;
//     }

//     snprintf(&deviceName[0], sizeof(deviceName) / sizeof(deviceName[0]), "%s.%u", dev_name(&pSpiMaster->dev), chipSelect);
//     pDevice = bus_find_device_by_name(&spi_bus_type, NULL, &deviceName[0]);
//     if (NULL == pDevice) {
//         pr_err("bus_find_device_by_name(\"%s\")=NULL\n", &deviceName[0]);
//         return -1;
//     }

//     pSpiDevice = to_spi_device(pDevice);
//     if (NULL == pSpiDevice) {
//         pr_err("to_spi_device().\n");
//         return -1;
//     }

//     pSpiDevice->max_speed_hz = 250000u;
//     pSpiDevice->bits_per_word = 8u;
//     pSpiDevice->mode = SPI_MODE_0;
//     result = spi_setup(pSpiDevice);
//     if (0 != result) {
//         pr_err("spi_setup(\"%s\")=%d\n", &deviceName[0], result);
//         return -1;
//     }

//     mg_pSpiDevice = pSpiDevice;
//     pr_info("spi_init() ok, deviceName=\"%s\", bitrate=%d, mode=%d\n", &deviceName[0], pSpiDevice->max_speed_hz, pSpiDevice->mode);

//     return 0;
// }

// static void spi_deinit(void) {}

/**
 * Interrupt.
 */
// static const uint8_t mg_irqPin = 194u;

// static struct gpio_chip *mg_pGpioChip = NULL;
// static unsigned int mg_irqLine;

// static unsigned long mg_irqTimestamp;
// static uint8_t mg_irqInProgress = 0u;

// static int gpiochip_match(struct gpio_chip *chip, void *data) {
//     const char *pLabel = (const char *) data;
//     int result;

//     result = strcmp(chip->label, pLabel);
//     if (0 != result) {
//         pr_info("Not matched: gpio_chip->label=\"%s\" vs label=\"%s\"", chip->label, pLabel);
//     }

//     return (0 == result);
// }

// static irqreturn_t irq_handler_top(int irq, void *dev_id) {
//     mg_irqInProgress = 1u;
//     mg_irqTimestamp = (1000ul * jiffies) / HZ;
//     return IRQ_WAKE_THREAD;
// }

// static irqreturn_t irq_handler_bottom(int irq, void *dev_id) {
//     entry_t entry = {
//         .type = ENTRY_TYPE_DATA_MAG,
//         .timestamp = mg_irqTimestamp,
//     };
//     int result;
//     uint8_t status;

//     result = spi_rd_data_m(&(entry.data_mag.x), &(entry.data_mag.y), &(entry.data_mag.z));
//     // pr_info("spi_rd_data_m(&x, &y, &z)=%d, x=%5d, y=%5d, z=%5d.\n", result, entry.data_mag.x, entry.data_mag.y, entry.data_mag.z);

//     result = spi_rd_status(&status);
//     // pr_info("spi_rd_status(&status)=%d, status=0x%x, MD=%d.\n", result, status, (0 != (0x01u & status)));

//     result = spi_wr_status(0u, 1u);
//     // pr_info("spi_wr_status(0u, 1u)=%d.\n", result);

//     stereo_imu_mem_lock();
//     if (0 < stereo_imu_mem_empty()) {
//         memcpy(stereo_imu_mem_writeAt(0u), &entry, sizeof(entry_t));
//         stereo_imu_mem_write(1u);

//     } else {
//         pr_err("Magnetometer measurement could not be stored.");
//     }
//     stereo_imu_mem_unlock();

//     mg_irqInProgress = 0u;
//     return IRQ_HANDLED;
// }

// static int irq_init(void) {
//     char gpioChipLabel[] = "tegra-gpio";

//     struct gpio_chip *pGpioChip;
//     int result;
//     unsigned int irqLine;

//     pGpioChip = gpiochip_find(&gpioChipLabel[0], &gpiochip_match);
//     if (NULL == pGpioChip) {
//         pr_err("gpiochip_find(\"%s\")=NULL\n", &gpioChipLabel[0]);
//         return -1;
//     }

//     if (mg_irqPin >= pGpioChip->ngpio) {
//         pr_err("(mg_irqPin=%d) >= (pGpioChip->ngpio=%d)\n", mg_irqPin, pGpioChip->ngpio);
//         return -1;
//     }

//     result = pGpioChip->request(pGpioChip, mg_irqPin);
//     if (0 != result) {
//         pr_err("pGpioChip->request(%d)=%d\n", mg_irqPin, result);
//         return -1;
//     }

//     result = pGpioChip->direction_input(pGpioChip, mg_irqPin);
//     if (0 != result) {
//         pGpioChip->free(pGpioChip, mg_irqPin);   // revert pGpioChip->request() call

//         pr_err("pGpioChip->direction_input(%d)=%d\n", mg_irqPin, result);
//         return -1;
//     }

//     result = pGpioChip->to_irq(pGpioChip, mg_irqPin);
//     if (0 > result) {
//         pGpioChip->free(pGpioChip, mg_irqPin);   // revert pGpioChip->request() call

//         pr_err("pGpioChip->to_irq(%d)=%d\n", mg_irqPin, result);
//         return -1;
//     }
//     irqLine = result;

//     if (request_threaded_irq(irqLine, &irq_handler_top, &irq_handler_bottom, IRQ_TYPE_EDGE_RISING, "stereo_imu", NULL)) {
//         pGpioChip->free(pGpioChip, mg_irqPin);   // revert pGpioChip->request() call

//         pr_err("request_threaded_irq(%d)=%d\n", irqLine, result);
//         return -1;
//     }

//     disable_irq(irqLine);

//     mg_pGpioChip = pGpioChip;
//     mg_irqLine = irqLine;

//     return 0;
// }

// static void irq_deinit(void) {
//     if (NULL != mg_pGpioChip) {
//         free_irq(mg_irqLine, NULL);
//         mg_pGpioChip->free(mg_pGpioChip, mg_irqPin);
//         mg_pGpioChip = NULL;
//     }
// }

/**
 * IMM.
 */
int stereo_imu_imm_enable(void) { return 0; }
void stereo_imu_imm_disable(void) {}

/**
 * Module.
 */
int stereo_imu_imm_init(void) {
    // if (0 != spi_init()) {
    //     return -1;
    // }

    // if (0 != irq_init()) {
    //     goto _l_spi_deinit;
    // }

    return 0;

    // _l_spi_deinit:
    //     spi_deinit();   // revert spi_init() call

    //     return -1;
}

void stereo_imu_imm_deinit(void) {
    // stereo_imu_mag_disable();
    // irq_deinit();
    // spi_deinit();
}
