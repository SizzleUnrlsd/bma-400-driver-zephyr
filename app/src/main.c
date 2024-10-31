/*
 * Copyright (C) 2024 Eve Redero
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

// #include <app/drivers/bma4xx.h>
// #include <drivers/bma400.h>
#include <zephyr/logging/log.h>
// LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);
// #if !DT_NODE_EXISTS(DT_NODELABEL(radio0))
// #error "whoops, node label radio0 not found"
// #endif

// #ifdef CONFIG_NRF24L01_TRIGGER
// #define TRIGGER
// #endif

// #if CONFIG_ROLE == 0
// 	#define ALICE
// #elif CONFIG_ROLE == 1
// 	#define BOB
// #else
// 	#define EVE
// #endif


// int main(void)
// {
// 	static const struct device *nrf24 = DEVICE_DT_GET(DT_NODELABEL(radio0));
// 	uint8_t data_len = 16;
// 	uint8_t buffer[16] = {0};
// #ifndef TRIGGER
// 	int i;
// #endif // TRIGGER

// 	if (!device_is_ready(nrf24)) {
// 		LOG_ERR("Sensor not ready");
// 		return 0;
// 	}
// 	LOG_INF("Device ready");
// #ifdef TRIGGER
// 	LOG_DBG("Trigger mode activated");
// #endif //TRIGGER

// #ifdef ALICE
// 	LOG_WRN("I am Alice!");
// 	while (true) {
// 		strncpy(buffer, "I am Alice, hi!", 16);
// #ifdef TRIGGER
// 		while (nrf24_write(nrf24, buffer, data_len))
// 		{
// 			k_sleep(K_MSEC(10));
// 		}
// #else
// 		for (i=0; i<10; i++)
// 		{
// 			nrf24_write(nrf24, buffer, data_len);
// 			k_sleep(K_MSEC(10));
// 		}
// #endif // TRIGGER
// 		LOG_HEXDUMP_INF(buffer, data_len, "Sent: ");
// 		LOG_DBG("Switch to read");
// #ifdef TRIGGER
// 		while (nrf24_read(nrf24, buffer, data_len));
// #else
// 		nrf24_read(nrf24, buffer, data_len);
// #endif // TRIGGER
// 		LOG_HEXDUMP_INF(buffer, data_len, "Received: ");
// 		k_sleep(K_MSEC(1000));
// 		LOG_DBG("Switch to write");
// 	}
// #endif // ALICE

// #ifdef BOB
// 	LOG_WRN("I am Bob!");
// 	while (true) {
// #ifdef TRIGGER
// 		while (nrf24_read(nrf24, buffer, data_len));
// #else
// 		nrf24_read(nrf24, buffer, data_len);
// #endif // TRIGGER
// 		LOG_HEXDUMP_INF(buffer, data_len, "Received: ");
// 		LOG_DBG("Switch to write");
// 		strncpy(buffer, "Hi Alice Im Bob", 16);
// 		k_sleep(K_MSEC(1000));
// #ifdef TRIGGER
// 		while (nrf24_write(nrf24, buffer, data_len))
// 		{
// 			k_sleep(K_MSEC(10));
// 		}
// #else
// 		for (i=0; i<10; i++)
// 		{
// 			nrf24_write(nrf24, buffer, data_len);
// 			k_sleep(K_MSEC(10));
// 		}
// #endif // TRIGGER
// 		LOG_HEXDUMP_INF(buffer, data_len, "Sent: ");
// 		LOG_DBG("Switch to read");
// 	}
// #endif // BOB

// #ifdef EVE
// 	LOG_WRN("I am Eve!");
// 	while (true) {
// #ifdef TRIGGER
// 		while (nrf24_read(nrf24, buffer, data_len));
// #else
// 		strncpy(buffer, "               ", 16);
// 		nrf24_read(nrf24, buffer, data_len);
// #endif // TRIGGER
// 		LOG_HEXDUMP_INF(buffer, data_len, "I spied: ");
// 	}
// #endif // EVE

// 	return 0;
// }



// // #include <include/bma_wrapper.h>
// // #include <include/bma_wrapper.h>
// // #include <app/drivers/bma4xx.h>
// // #include <drivers/bma4xx.h>
// #include <app/drivers/nrf24.h>
// #include <zephyr/logging/log.h>
// // #include <app/drivers/bma4xx.h>

// #if CONFIG_ROLE == 0
// 	#define POLLING
// #elif CONFIG_ROLE == 1
// 	#define INTERRUPT
// #else
// 	#define ERROR
// #endif

// LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

// int main(void)
// {
// 	#ifdef POLLING
// 		LOG_WRN("Polling mode");
// 	#endif
// 	#ifdef INTERRUPT
// 		LOG_WRN("Interrupt mode");
// 	#endif
// 	k_sleep(K_MSEC(5000));
// 	LOG_WRN("-- started execution --");
// 	printk("-- started execution --\n");
//     while (true) {
//         k_sleep(K_MSEC(100));
//         // printk("ok{%d}\n", test(13));
// 		#ifdef POLLING
// 			LOG_WRN("Polling mode");
// 		#endif
// 		#ifdef INTERRUPT
// 			LOG_WRN("Interrupt mode");
// 		#endif
//         printk("ooook{%d}\n", (13));
//     }
//     return 0;
// }
// // polling
// // interrupt

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>

// Manage Flash device
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash/nrf_qspi_nor.h>


#include <zephyr/sys/printk.h>

#define LED1_NODE DT_ALIAS(ledg)
#define LED2_NODE DT_ALIAS(ledr)
#define LED3_NODE DT_ALIAS(ledb)
#define I2C_INBOARD_NODE DT_ALIAS(in_i2c)
#define SPI_NODE DT_ALIAS(artic_spi)
#define QSPI_FLASH_NODE DT_ALIAS(spi_flash0)
//#define QSPI_FLASH_NODE DT_NODELABEL(is25lp128f)

static const struct gpio_dt_spec ledg = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec ledr = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec ledb = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

#define UNUSED __attribute__((unused))

#if CONFIG_ROLE == 0
	#define POLLING
#elif CONFIG_ROLE == 1
	#define INTERRUPT
#else
	#define ERROR
#endif

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#if !DT_NODE_EXISTS(DT_NODELABEL(bma4xx))
	#error "whoops, node label bma4xx not found"
#endif

void scan_i2c_devices(const struct device *i2c_dev) {
    uint8_t i2c_data[] = {0x00, 0x01};
    int ret;
    
    printk("Scanning I2C bus for devices...\n");
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) { // 0x03 to 0x77 are the usual 7-bit I2C address range
        ret = i2c_write(i2c_dev, i2c_data, sizeof(i2c_data), addr);
        if (ret < 0) {
            //printk("No device found at address 0x%02x\n", addr);
        } else {
            printk("Device found at address 0x%02x\n", addr);
        }
    }
}


// #define QSPI_NODE DT_NODELABEL(qspi)
#define QSPI_TEST_OFFSET 0
#define QSPI_TEST_SIZE 256
void test_qspi_flash(void) {
    const struct device *flash_dev = DEVICE_DT_GET(QSPI_FLASH_NODE);
    uint8_t qspi_write_buf[QSPI_TEST_SIZE];
    uint8_t qspi_read_buf[QSPI_TEST_SIZE];
    int ret;

    if (!device_is_ready(flash_dev)) {
        printk("Error: flash device is not ready\n");
        return;
    }
    

    // Fill write buffer with test pattern
    for (int i = 0; i < QSPI_TEST_SIZE; i++) {
        qspi_write_buf[i] = i & 0xFF;
    }

    // Erase QSPI sector
    printk("Erasing flash: dev=%p, offset=0x%08x, size=0x%08x\n", flash_dev, QSPI_TEST_OFFSET, QSPI_TEST_SIZE);

    ret = flash_erase(flash_dev, QSPI_TEST_OFFSET, 4096);
    if (ret != 0) {
        printk("Error: QSPI erase failed with error code %d\n", ret);
        return;
    }

    // Write to QSPI
    ret = flash_write(flash_dev, QSPI_TEST_OFFSET, qspi_write_buf, QSPI_TEST_SIZE);
    if (ret != 0) {
        printk("Error: QSPI write failed\n");
        return;
    }

    // Read from QSPI
    ret = flash_read(flash_dev, QSPI_TEST_OFFSET, qspi_read_buf, QSPI_TEST_SIZE);
    if (ret != 0) {
        printk("Error: QSPI read failed\n");
        return;
    }

    // Verify data
    bool qspi_test_passed = true;
    for (int i = 0; i < QSPI_TEST_SIZE; i++) {
        if (qspi_write_buf[i] != qspi_read_buf[i]) {
            qspi_test_passed = false;
            break;
        }
    }

    if (qspi_test_passed) {
        printk("QSPI test passed\n");
    } else {
        printk("QSPI test failed: data mismatch\n");
    }
}

static struct gpio_callback reed_cb_data;

void reed_changed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("Reed switch state changed\n");
}

int main(void)
{
    int ret = 0;
	UNUSED static const struct device *bma = DEVICE_DT_GET(DT_NODELABEL(bma4xx));
	bma4xx_i2c_init(bma);

    // Initialize LEDs
    if (!device_is_ready(ledg.port) || !device_is_ready(ledr.port) || !device_is_ready(ledb.port)) {
        printk("Error: LEDs are not ready\n");
        return 1;
    }

    ret = gpio_pin_configure_dt(&ledg, GPIO_OUTPUT_ACTIVE);
    ret |= gpio_pin_configure_dt(&ledr, GPIO_OUTPUT_ACTIVE);
    ret |= gpio_pin_configure_dt(&ledb, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Error: Failed to configure LEDs\n");
        return ret;
    }

    // Initialize Reed switch
    const struct gpio_dt_spec reed_switch = GPIO_DT_SPEC_GET_OR(DT_ALIAS(reed_switch), gpios, {0});
    if (!device_is_ready(reed_switch.port)) {
        printk("Error: Reed switch device is not ready\n");
        return ret;
    }

    ret = gpio_pin_configure_dt(&reed_switch, GPIO_INPUT);
    if (ret != 0) {
        printk("Error %d: failed to configure reed switch pin\n", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&reed_switch, GPIO_INT_EDGE_BOTH);
    if (ret != 0) {
        printk("Error %d: failed to configure reed switch interrupt\n", ret);
        return ret;
    }

    gpio_init_callback(&reed_cb_data, reed_changed, BIT(reed_switch.pin));
    gpio_add_callback(reed_switch.port, &reed_cb_data);

    // Initialize I2C
    const struct device *i2c_dev = DEVICE_DT_GET(I2C_INBOARD_NODE);
    if (!device_is_ready(i2c_dev)) {
        printk("Error: I2C device is not ready\n");
        return ret;
    }

    // // Initialize SPI
    // const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);
    // if (!device_is_ready(spi_dev)) {
    //     printk("Error: SPI device is not ready\n");
    //     return ret;
    // }


    while (1) {
        printk("Hello World from my custom board!\n");

        // Toggle LEDs
        gpio_pin_toggle_dt(&ledg);
        k_msleep(333);
        gpio_pin_toggle_dt(&ledr);
        k_msleep(333);
        gpio_pin_toggle_dt(&ledb);
        k_msleep(333);

        // I2C test (assuming a device at address 0x50)
        scan_i2c_devices(i2c_dev);
        // SPI test
        // uint8_t tx_buffer[] = {0x01, 0x02, 0x03};
        // uint8_t rx_buffer[sizeof(tx_buffer)] = {0};
        // struct spi_buf tx_buf = {.buf = tx_buffer, .len = sizeof(tx_buffer)};
        // struct spi_buf rx_buf = {.buf = rx_buffer, .len = sizeof(rx_buffer)};
        // struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
        // struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

        // ret = spi_transceive(spi_dev, NULL, &tx, &rx);
        // if (ret < 0) {
        //     printk("Error: SPI transaction failed\n");
        // } else {
        //     printk("SPI transaction successful\n");
        // }


        k_msleep(1000);

        // QSPI test
        test_qspi_flash();

        k_msleep(5000);  // Increased delay to 5 seconds
    }
	return ret;
}
