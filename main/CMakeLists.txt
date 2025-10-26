idf_component_register(SRCS "main.c" "app.c" "training.c" "motion.c"
        PRIV_REQUIRES spi_flash esp_driver_gpio esp_driver_i2c
        REQUIRES esp_timer heartbeat
        INCLUDE_DIRS "")
