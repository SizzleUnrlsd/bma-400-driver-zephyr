#include <errno.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(BMA_400, CONFIG_BMA_400_LOG_LEVEL);

int test(int ret)
{
    return ret;
}
