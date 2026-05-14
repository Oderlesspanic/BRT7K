#include "gripper_node.h"

#include "driver/uart.h"
#include <rmw_microros/rmw_microros.h>

#include "esp32_serial_transport.h"

static size_t uart_port = UART_NUM_0;

void app_main(void)
{
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    rmw_uros_set_custom_transport(
        true,
        (void *)&uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read
    );
#else
#error micro-ROS transports misconfigured
#endif

    gripper_node_start();
}
