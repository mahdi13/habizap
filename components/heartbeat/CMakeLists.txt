idf_component_register(
    SRCS "heartbeat.c" "algo_maxim.c" "session.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer esp_driver_i2c
)