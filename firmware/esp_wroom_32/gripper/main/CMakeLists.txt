# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS
        "main.c"
        "gripper_node.c"
        "gripper_hardware.c"
        "esp32_serial_transport.c"

    INCLUDE_DIRS
        "."

    REQUIRES
        driver
        micro_ros_espidf_component
)
