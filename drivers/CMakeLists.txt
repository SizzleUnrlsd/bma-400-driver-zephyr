# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

#add_subdirectory_ifdef(CONFIG_NRF24L01 nrf24l01)
# zephyr_include_directories(.)
# zephyr_library()
add_subdirectory(nrf24l01)
add_subdirectory(bma4xx)
message(WARNING "CMAKE DRIVER BMA4XX")
