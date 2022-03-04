idf_component_register(
    SRCS pcal6416a.c
    INCLUDE_DIRS .
    REQUIRES i2cdev log esp_idf_lib_helpers
)
