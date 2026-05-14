#include "gripper_hardware.h"

#include <math.h>

#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PWM_TIMER              LEDC_TIMER_0
#define PWM_MODE               LEDC_LOW_SPEED_MODE
#define PWM_FREQ_HZ            20000
#define PWM_RESOLUTION         LEDC_TIMER_10_BIT
#define PWM_MAX_DUTY           ((1U << 10) - 1U)

// SparkFun HX711 Load Cell Amplifier: DOUT goes low when a 24-bit sample is ready.
#define HX711_TIMEOUT_LOOPS    100000

static gripper_hw_t *isr_hw = NULL;

static uint8_t encoder_state(gpio_num_t a_pin, gpio_num_t b_pin)
{
    const uint8_t a = gpio_get_level(a_pin) ? 1 : 0;
    const uint8_t b = gpio_get_level(b_pin) ? 1 : 0;

    return (uint8_t)((a << 1) | b);
}

static void encoder_isr(void *arg)
{
    const gripper_axis_id_t axis_id = (gripper_axis_id_t)(uintptr_t)arg;

    if (isr_hw == NULL || axis_id >= GRIPPER_AXIS_COUNT) {
        return;
    }

    gripper_axis_t *axis = &isr_hw->axes[axis_id];
    const uint8_t new_state = encoder_state(axis->encoder_a_pin, axis->encoder_b_pin);
    const uint8_t transition = (uint8_t)((axis->last_encoder_state << 2) | new_state);

    switch (transition) {
        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:
            axis->encoder_count++;
            break;

        case 0b0010:
        case 0b1011:
        case 0b1101:
        case 0b0100:
            axis->encoder_count--;
            break;

        default:
            break;
    }

    axis->last_encoder_state = new_state;
}

static void configure_output_pin(gpio_num_t pin)
{
    gpio_config_t config = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&config);
    gpio_set_level(pin, 0);
}

static void configure_input_pin(gpio_num_t pin, gpio_int_type_t interrupt_type)
{
    gpio_config_t config = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = interrupt_type
    };

    gpio_config(&config);
}

static void configure_axis(gripper_axis_t *axis)
{
    configure_output_pin(axis->in1_pin);
    configure_output_pin(axis->in2_pin);
    configure_output_pin(axis->pwm_pin);
    configure_output_pin(axis->hx711_sck_pin);

    configure_input_pin(axis->encoder_a_pin, GPIO_INTR_ANYEDGE);
    configure_input_pin(axis->encoder_b_pin, GPIO_INTR_ANYEDGE);
    configure_input_pin(axis->min_sensor_pin, GPIO_INTR_DISABLE);

    if (axis->max_sensor_pin != GPIO_NUM_NC) {
        configure_input_pin(axis->max_sensor_pin, GPIO_INTR_DISABLE);
    }

    configure_input_pin(axis->hx711_dout_pin, GPIO_INTR_DISABLE);

    ledc_channel_config_t channel_config = {
        .gpio_num = axis->pwm_pin,
        .speed_mode = PWM_MODE,
        .channel = axis->pwm_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };

    ledc_channel_config(&channel_config);

    axis->last_encoder_state =
        encoder_state(axis->encoder_a_pin, axis->encoder_b_pin);
}

static void set_axis_pins(gripper_axis_t *axis, const gripper_arm_pins_t *pins, ledc_channel_t channel)
{
    axis->in1_pin = pins->in1_pin;
    axis->in2_pin = pins->in2_pin;
    axis->pwm_pin = pins->pwm_pin;
    axis->encoder_a_pin = pins->encoder_a_pin;
    axis->encoder_b_pin = pins->encoder_b_pin;
    axis->min_sensor_pin = pins->home_sensor_pin;
    axis->max_sensor_pin = GPIO_NUM_NC;
    axis->hx711_dout_pin = pins->hx711_dout_pin;
    axis->hx711_sck_pin = pins->hx711_sck_pin;
    axis->pwm_channel = channel;
    axis->encoder_count = 0;
    axis->last_encoder_state = 0;
}

static void set_lift_pins(gripper_axis_t *axis, const gripper_lift_pins_t *pins, ledc_channel_t channel)
{
    axis->in1_pin = pins->in1_pin;
    axis->in2_pin = pins->in2_pin;
    axis->pwm_pin = pins->pwm_pin;
    axis->encoder_a_pin = pins->encoder_a_pin;
    axis->encoder_b_pin = pins->encoder_b_pin;
    axis->min_sensor_pin = pins->bottom_sensor_pin;
    axis->max_sensor_pin = pins->top_sensor_pin;
    axis->hx711_dout_pin = pins->hx711_dout_pin;
    axis->hx711_sck_pin = pins->hx711_sck_pin;
    axis->pwm_channel = channel;
    axis->encoder_count = 0;
    axis->last_encoder_state = 0;
}

void gripper_hw_init(gripper_hw_t *hw, const gripper_pins_t *pins)
{
    hw->standby_pin = pins->standby_pin;

    set_axis_pins(&hw->axes[GRIPPER_AXIS_LEFT_ARM], &pins->left_arm, LEDC_CHANNEL_0);
    set_axis_pins(&hw->axes[GRIPPER_AXIS_RIGHT_ARM], &pins->right_arm, LEDC_CHANNEL_1);
    set_lift_pins(&hw->axes[GRIPPER_AXIS_LIFT], &pins->lift, LEDC_CHANNEL_2);

    ledc_timer_config_t timer_config = {
        .speed_mode = PWM_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&timer_config);

    configure_output_pin(hw->standby_pin);

    for (int i = 0; i < GRIPPER_AXIS_COUNT; i++) {
        configure_axis(&hw->axes[i]);
    }

    isr_hw = hw;
    gpio_install_isr_service(0);

    for (int i = 0; i < GRIPPER_AXIS_COUNT; i++) {
        gpio_isr_handler_add(hw->axes[i].encoder_a_pin, encoder_isr, (void *)(uintptr_t)i);
        gpio_isr_handler_add(hw->axes[i].encoder_b_pin, encoder_isr, (void *)(uintptr_t)i);
    }

    gripper_hw_enable(hw, true);
}

void gripper_hw_enable(gripper_hw_t *hw, bool enabled)
{
    gpio_set_level(hw->standby_pin, enabled ? 1 : 0);
}

void gripper_hw_set_axis_speed(gripper_hw_t *hw, gripper_axis_id_t axis_id, float speed)
{
    if (axis_id >= GRIPPER_AXIS_COUNT) {
        return;
    }

    gripper_axis_t *axis = &hw->axes[axis_id];
    const float limited_speed = fminf(fmaxf(speed, -1.0f), 1.0f);

    if (limited_speed > 0.0f) {
        gpio_set_level(axis->in1_pin, 1);
        gpio_set_level(axis->in2_pin, 0);
    } else if (limited_speed < 0.0f) {
        gpio_set_level(axis->in1_pin, 0);
        gpio_set_level(axis->in2_pin, 1);
    } else {
        gpio_set_level(axis->in1_pin, 0);
        gpio_set_level(axis->in2_pin, 0);
    }

    const uint32_t duty = (uint32_t)(fabsf(limited_speed) * PWM_MAX_DUTY);
    ledc_set_duty(PWM_MODE, axis->pwm_channel, duty);
    ledc_update_duty(PWM_MODE, axis->pwm_channel);
}

void gripper_hw_stop_axis(gripper_hw_t *hw, gripper_axis_id_t axis_id)
{
    gripper_hw_set_axis_speed(hw, axis_id, 0.0f);
}

void gripper_hw_stop_all(gripper_hw_t *hw)
{
    for (int i = 0; i < GRIPPER_AXIS_COUNT; i++) {
        gripper_hw_stop_axis(hw, (gripper_axis_id_t)i);
    }
}

bool gripper_hw_min_sensor_active(const gripper_hw_t *hw, gripper_axis_id_t axis_id)
{
    return gpio_get_level(hw->axes[axis_id].min_sensor_pin) == 0;
}

bool gripper_hw_max_sensor_active(const gripper_hw_t *hw, gripper_axis_id_t axis_id)
{
    const gpio_num_t pin = hw->axes[axis_id].max_sensor_pin;

    if (pin == GPIO_NUM_NC) {
        return false;
    }

    return gpio_get_level(pin) == 0;
}

int32_t gripper_hw_encoder_count(const gripper_hw_t *hw, gripper_axis_id_t axis_id)
{
    return hw->axes[axis_id].encoder_count;
}

void gripper_hw_reset_encoder(gripper_hw_t *hw, gripper_axis_id_t axis_id)
{
    hw->axes[axis_id].encoder_count = 0;
}

int32_t gripper_hw_read_load_cell_raw(const gripper_hw_t *hw, gripper_axis_id_t axis_id)
{
    const gripper_axis_t *axis = &hw->axes[axis_id];
    int32_t value = 0;
    int timeout = HX711_TIMEOUT_LOOPS;

    while (gpio_get_level(axis->hx711_dout_pin) != 0 && timeout-- > 0) {
        esp_rom_delay_us(1);
    }

    if (timeout <= 0) {
        return 0;
    }

    for (int i = 0; i < 24; i++) {
        gpio_set_level(axis->hx711_sck_pin, 1);
        esp_rom_delay_us(1);
        value = (value << 1) | (gpio_get_level(axis->hx711_dout_pin) ? 1 : 0);
        gpio_set_level(axis->hx711_sck_pin, 0);
        esp_rom_delay_us(1);
    }

    gpio_set_level(axis->hx711_sck_pin, 1);
    esp_rom_delay_us(1);
    gpio_set_level(axis->hx711_sck_pin, 0);

    if (value & 0x800000) {
        value |= 0xFF000000;
    }

    return value;
}
