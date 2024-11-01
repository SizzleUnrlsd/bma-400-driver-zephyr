# Zephyr out-of-tree driver for BMA4xx

This repository contains a Zephyr out-of-tree driver.

This repository is versioned together with the [Zephyr main tree][zephyr]. This
means that every time that Zephyr is tagged, this repository is tagged as well
with the same version number, and the [manifest](west.yml) entry for `zephyr`
will point to the corresponding Zephyr tag. For example, the `example-application`
v2.6.0 will point to Zephyr v2.6.0. Note that the `main` branch always
points to the development branch of Zephyr, also `main`.

[bindings]: https://docs.zephyrproject.org/latest/guides/dts/bindings.html
[drivers]: https://docs.zephyrproject.org/latest/reference/drivers/index.html
[zephyr]: https://github.com/zephyrproject-rtos/zephyr
[west_ext]: https://docs.zephyrproject.org/latest/develop/west/extensions.html

## Getting Started

Before getting started, make sure you have a proper Zephyr development
environment. Follow the official
[Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html).

### Initialization

The first step is to initialize the workspace folder (``my-workspace``) where
the application and all Zephyr modules will be cloned. Run the following
command:

```shell
# initialize my-workspace for the example-application (main branch)
west init -m https://github.com/SizzleUnrlsd/bma-400-driver-zephyr --mr main my-workspace
# update Zephyr modules
cd my-workspace
west update
```

This has been tested with Zephyr 3.5.99

### Building and running

To build the application, run the following command:

```shell
west build -b $BOARD -p always app -- -DOVERLAY_CONFIG=prj.conf
```

where `$BOARD` is the target board.
```shell
BOARD="linkit_core_nrf52840"
```

In order to activate debug logs:
```shell
west build -b $BOARD -p always app -- -DOVERLAY_CONFIG=debug.conf
```

Once you have built the application, run the following command to flash it:

```shell
west flash
```

For more detailed information, see the [example app Readme](app/README.md)

### Testing

To execute Twister integration tests, run the following command:

```shell
west twister -T tests --integration
```

This only tests correct compilation under 3 different platforms, it does not run tests on target or emulator.

# API reference

This driver uses a minimalist custom API.

## Read
```
int nrf24_read(const struct device *dev, uint8_t *buffer, uint8_t data_len)
```

This methods reads data\_len bytes from the device dev, and places it in buffer.
In trigger mode, if NRF24L01\_READ\_TIMEOUT is exceeded, the function times out.

In polling mode, it will loop forever.

# Write
```
int nrf24_write(const struct device *dev, uint8_t *buffer, uint8_t data_len)
```

This methods writes data\_len bytes from buffer, and sends it through device dev.
In trigger mode, if NRF24L01\_WRITE\_TIMEOUT is exceeded, the function times out.

In polling mode, it will loop forever.

# Troubleshooting

## No RX received

* Verify your device tree
* Verify CE GPIO is active high and IRQ active low.

## Issue with SPI read/write

Write in a register and read it.

* Verify NRF24 is correctly plugged
* Verify it is power supplied correctly
* Verify the incoming SPI data is correct
