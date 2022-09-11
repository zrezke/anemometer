#include "driver/ledc.h"
#include <time.h>
#include "driver/gpio.h"

/**
 * @brief: Start PWM at 36kHz on pin 16
 */
void start_ir_led_pwm()
{
  ledc_timer_config_t timer_config;
  timer_config.speed_mode = LEDC_HIGH_SPEED_MODE;
  timer_config.timer_num = LEDC_TIMER_0;
  timer_config.duty_resolution = 1;
  timer_config.freq_hz = 38000;
  timer_config.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&timer_config);

  ledc_channel_config_t channel_config;
  channel_config.channel = LEDC_CHANNEL_0;
  channel_config.speed_mode = LEDC_HIGH_SPEED_MODE;
  channel_config.gpio_num = 16;
  channel_config.intr_type = LEDC_INTR_DISABLE;
  channel_config.timer_sel = LEDC_TIMER_0;
  channel_config.duty = 1;
  ledc_channel_config(&channel_config);
}

float *value_array;
time_t *time_array;
const int ARRAY_SIZE = 10000000000;
void app_main()
{
  time_array = malloc(ARRAY_SIZE * (sizeof(time_t)));
  value_array = malloc(ARRAY_SIZE * (sizeof(float)));
  start_ir_led_pwm();
  gpio_config_t io_conf;
  


  unsigned long i = 0;
  while(1) {
    value_array[i] 
  }
}
