idf_component_register(
    SRCS "dht.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos log
)
