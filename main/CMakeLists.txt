idf_component_register(SRCS "main.c" "app.c" "training.c" "motion.c" "vibration.c"
        PRIV_REQUIRES spi_flash esp_driver_gpio esp_driver_i2c
        REQUIRES esp_timer
        INCLUDE_DIRS "")
