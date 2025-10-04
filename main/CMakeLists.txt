idf_component_register(SRCS "main.c" "app.c" "training.c"
        PRIV_REQUIRES spi_flash
        REQUIRES esp_timer
        INCLUDE_DIRS "")
