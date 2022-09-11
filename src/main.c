#include "driver/ledc.h"
#include <time.h>
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "anemometer.h"
#include "driver/timer.h"

pcnt_config_t pulse_counter_config = {
    .unit = PCNT_UNIT_0,
    .channel = PCNT_CHANNEL_0,
    .pulse_gpio_num = GPIO_NUM_27,
    .ctrl_gpio_num = GPIO_NUM_26,
    .lctrl_mode = PCNT_CHANNEL_LEVEL_ACTION_KEEP,
    .hctrl_mode = PCNT_CHANNEL_LEVEL_ACTION_HOLD,
    .neg_mode = PCNT_CHANNEL_EDGE_ACTION_INCREASE,
    .pos_mode = PCNT_CHANNEL_EDGE_ACTION_HOLD,
    .counter_l_lim = 0,
    .counter_h_lim = N_PULSES_PER_REV};

void pcnt_h_lim_interrupt_handler()
{
  current_second_rotations++;
  pcnt_counter_pause(pulse_counter_config.unit);
  pcnt_counter_clear(pulse_counter_config.unit);
  pcnt_counter_resume(pulse_counter_config.unit);
}

void start_counter()
{
  // configure pcnt
  pcnt_unit_config(&pulse_counter_config);
  // register max val interrupt
  pcnt_event_enable(pulse_counter_config.unit, PCNT_EVT_H_LIM);
  pcnt_isr_service_install(0);
  pcnt_isr_handler_add(PCNT_UNIT_0, pcnt_h_lim_interrupt_handler, NULL);
}

#define TIMER_DIVIDER (16)
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)

const timer_group_t TIMER_GROUP = TIMER_GROUP_0;
const timer_idx_t TIMER_INDEX = TIMER_0;
const int ONE_SECOND = 1;

void update_rotation_frequency()
{
  rotation_frequency = current_second_rotations;
  current_second_rotations = 0;
}

void timer_isr_callback()
{
  uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP, TIMER_INDEX);
  update_rotation_frequency();
}

timer_config_t timer_config = {
    .divider = TIMER_DIVIDER,
    .counter_dir = TIMER_COUNT_UP,
    .counter_en = TIMER_START,
    .alarm_en = TIMER_ALARM_EN,
    .auto_reload = true};

void start_timer()
{
  timer_init(TIMER_GROUP, TIMER_INDEX, &timer_config);
  timer_set_counter_value(TIMER_GROUP, TIMER_INDEX, 0);
  timer_set_alarm_value(TIMER_GROUP, TIMER_INDEX, ONE_SECOND * TIMER_SCALE);
  timer_enable_intr(TIMER_GROUP, TIMER_INDEX);

  timer_isr_callback_add(TIMER_GROUP, TIMER_INDEX, timer_isr_callback, NULL, 0);
}

void app_main()
{
  printf("HELLO WORLD");
  start_counter();
  start_timer();
  while (true)
  {
    printf("ROTATIONS: %d, WIND SPEED: %f\n", rotation_frequency, Ar * rotation_frequency + B);
  }
}
