idf_component_register(SRCS "gardensensor_breadboarding.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_uart esp_driver_i2c esp_adc onewire_bus ds18b20)