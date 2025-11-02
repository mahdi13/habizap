idf_component_register(SRCS "main.c" "app.c" "training.c" "motion.c" "vibration.c" "battery.c"
        PRIV_REQUIRES spi_flash esp_driver_gpio esp_driver_i2c esp_adc
        REQUIRES esp_timer
        INCLUDE_DIRS "")
