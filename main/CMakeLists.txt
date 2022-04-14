idf_component_register(
    SRCS "main.c" "ble.c" "utils.c"
    INCLUDE_DIRS "."
    REQUIRES mics6814 dht ssd1306 bt
)
