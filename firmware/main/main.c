#include <stdio.h>
#include "sdkconfig.h"
#include <time.h>
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "anemometer.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

typedef struct
{
  bool update;
} timer_event_t;
typedef struct
{
  bool update;
} counter_event_t;

static xQueueHandle s_timer_queue;
static xQueueHandle s_counter_queue;

pcnt_config_t pulse_counter_config = {
    .unit = PCNT_UNIT_0,
    .channel = PCNT_CHANNEL_0,
    .pulse_gpio_num = GPIO_NUM_27,
    .ctrl_gpio_num = GPIO_NUM_26,
    .lctrl_mode = PCNT_MODE_KEEP,
    .hctrl_mode = PCNT_MODE_DISABLE,
    .neg_mode = PCNT_COUNT_INC,
    .pos_mode = PCNT_COUNT_DIS,
    .counter_l_lim = 0,
    .counter_h_lim = N_PULSES_PER_REV};

static void IRAM_ATTR pcnt_h_lim_interrupt_handler(void *arg)
{

  // pcnt_counter_pause(pulse_counter_config.unit);
  // pcnt_counter_clear(pulse_counter_config.unit);
  // pcnt_counter_resume(pulse_counter_config.unit);
  counter_event_t evt = {
      .update = true};
  xQueueSendFromISR(s_counter_queue, &evt, NULL);
}

void start_counter()
{
  // configure pcnt
  pcnt_unit_config(&pulse_counter_config);
  // register max val interrupt
  pcnt_event_enable(pulse_counter_config.unit, PCNT_EVT_H_LIM);
  pcnt_isr_service_install(0);
  pcnt_isr_handler_add(PCNT_UNIT_0, pcnt_h_lim_interrupt_handler, NULL);
  pcnt_counter_pause(pulse_counter_config.unit);
  pcnt_counter_clear(pulse_counter_config.unit);
  pcnt_counter_resume(pulse_counter_config.unit);
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

static bool IRAM_ATTR timer_isr_callback(void *arg)
{
  BaseType_t high_task_awoken = pdFALSE;
  uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP, TIMER_INDEX);
  timer_event_t evt = {
      .update = true,
  };
  xQueueSendFromISR(s_timer_queue, &evt, &high_task_awoken);
  return high_task_awoken == pdTRUE;
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
  timer_start(TIMER_GROUP, TIMER_INDEX);
}

void handleCounterQueue() {
  counter_event_t counter_event;
  portBASE_TYPE cnt_res;

  while (true) {
    cnt_res = xQueueReceive(s_counter_queue, &counter_event, 0);
    if (cnt_res == pdFALSE) {
      return;
    }
    current_second_rotations++;
  }
}

void app_main()
{
  s_timer_queue = xQueueCreate(10, sizeof(timer_event_t));
  s_counter_queue = xQueueCreate(10, sizeof(counter_event_t));
  printf("HELLO WORLD\n");
  start_counter();
  start_timer();

  timer_event_t timer_event;
  bool hasElement = false;
  while (true)
  {
    handleCounterQueue();
    hasElement = xQueueReceive(s_timer_queue, &timer_event, 0);
    if (hasElement && timer_event.update)
    {
      update_rotation_frequency();
      //printf("Current second rotations: %d\n", current_second_rotations);
      printf("ROTATIONS: %d, WIND SPEED: %f\n", rotation_frequency, Ar * rotation_frequency + B);
    }
  }
}
