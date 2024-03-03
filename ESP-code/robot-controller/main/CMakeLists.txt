
idf_component_register(
        SRCS "main.c" "bt_handler.c"
        INCLUDE_DIRS "${CMAKE_CURRENT_BINARY_DIR}")
        REQUIRES esp_adc btstack
