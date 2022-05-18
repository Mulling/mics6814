idf_component_register(SRCS "main.c" "ble.c" "utils.c"
    INCLUDE_DIRS "."
    REQUIRES mics6814 am2302 ssd1306 bt)
