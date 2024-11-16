#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#define IN1_GPIO  27    // H-Bridge input 1
#define IN2_GPIO  26    // H-Bridge input 2
#define EN_GPIO   25    // H-Bridge enable pin (PWM control)

#define PWM_CHANNEL 0
#define PWM_FREQUENCY 5000 // PWM frequency in Hz
#define PWM_RESOLUTION LEDC_TIMER_8_BIT // 8-bit resolution (0-255)
#define PWM_MAX_DUTY_CYCLE 255

// Function to initialize the GPIO pins for direction control
void gpio_init()
{
    gpio_set_direction(IN1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN2_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(EN_GPIO, GPIO_MODE_OUTPUT);
}

// Function to initialize PWM on the ESP32
void pwm_init()
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_USE_APB_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = EN_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0, // Start with motor stopped (0% PWM)
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

// Function to set motor direction
void set_motor_direction(int forward)
{
    if (forward) {
        gpio_set_level(IN1_GPIO, 1);
        gpio_set_level(IN2_GPIO, 0);
    } else {
        gpio_set_level(IN1_GPIO, 0);
        gpio_set_level(IN2_GPIO, 1);
    }
}

// Function to control motor speed
void set_motor_speed(uint8_t speed)
{
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL, speed);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL);
}

// Function to stop the motor
void stop_motor()
{
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL);
    gpio_set_level(IN1_GPIO, 0);
    gpio_set_level(IN2_GPIO, 0);
}

void app_main()
{
    //Initialize GPIO pins and PWM
   gpio_init();
    pwm_init();
 while (1){
    // Example 1: Motor Forward at Full Speed
    ESP_LOGI("Motor Control", "Running motor forward at full speed");

    set_motor_direction(1); // Set motor direction to forward
   set_motor_speed(PWM_MAX_DUTY_CYCLE); // Full speed
     
   vTaskDelay(10000 / portTICK_PERIOD_MS); // Run forward for 2 seconds

    // Example 2: Motor Reverse at Half Speed
    ESP_LOGI("Motor Control", "Running motor reverse at half speed");
    set_motor_direction(0); // Set motor direction to reverse
    set_motor_speed(PWM_MAX_DUTY_CYCLE / 2); // Half speed

    vTaskDelay(10000 / portTICK_PERIOD_MS); // Run reverse for 2 seconds

    // Example 3: Stop Motor
    ESP_LOGI("Motor Control", "Stopping motor");
    stop_motor(); // Stop the motor

    vTaskDelay(10000 / portTICK_PERIOD_MS); // Wait before next cycle
 }
}