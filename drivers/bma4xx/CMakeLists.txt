# zephyr_library()
# zephyr_library_sources(bma400.c)

# # This shouldn't be needed if we send this driver upstream.
# zephyr_include_directories(.)
# zephyr_library_sources_ifdef(CONFIG_BLINK_GPIO_LED bma400.c)
message(WARNING "BMA4XX CMAKE DRIVER")
zephyr_library_sources_ifdef(CONFIG_NRF24L01 bma400.c)
