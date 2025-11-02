idf_component_register(SRCS "main.c" "app.c" "training.c" "motion.c" "vibration.c" "battery.c" "inference.c" "inference_tflm.cpp"
        PRIV_REQUIRES spi_flash esp_driver_gpio esp_driver_i2c esp_adc cxx
        REQUIRES esp_timer
        INCLUDE_DIRS "." "../habizap-cpp-mcu-v1")
