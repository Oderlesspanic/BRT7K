idf_component_register(
    SRCS
        "main.c"
        "ina226_10.c"
        "monitor_node.c"

    INCLUDE_DIRS
        "."

    REQUIRES
        driver
        freertos
        esp_timer
        micro_ros_espidf_component
)