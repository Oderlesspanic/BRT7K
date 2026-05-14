# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS
        "main.c"
        "motor.c"
        "urdf_parser.c"
        "drive_node.c"

    INCLUDE_DIRS
        "."

    REQUIRES
        driver
        esp_timer
        micro_ros_espidf_component
)
