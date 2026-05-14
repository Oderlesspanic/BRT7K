#include "monitor_node.h"
#include "esp32_serial_transport.h"

#include "driver/i2c.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <rmw_microros/rmw_microros.h>

static size_t uart_port = UART_NUM_0;

#define I2C_PORT              I2C_NUM_0
#define I2C_SDA_PIN           21
#define I2C_SCL_PIN           22
#define I2C_FREQ_HZ           100000

static void i2c_master_init(void)
{
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
        .clk_flags = 0
    };

    i2c_param_config(I2C_PORT, &config);
    i2c_driver_install(I2C_PORT, config.mode, 0, 0, 0);
}

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

    i2c_master_init();

    monitor_node_start();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
