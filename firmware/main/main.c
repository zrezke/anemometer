#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include <time.h>
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "anemometer.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_main.h"

#include "anemometer_bt_profile.h"
#include "btle_gap.h"
#include "btle_gatt.h"

#define GATTS_TABLE_TAG "GATTS TABLE TAG"
/*
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

void handleCounterQueue()
{
  counter_event_t counter_event;
  portBASE_TYPE cnt_res;

  while (true)
  {
    cnt_res = xQueueReceive(s_counter_queue, &counter_event, 0);
    if (cnt_res == pdFALSE)
    {
      return;
    }
    current_second_rotations++;
  }
}

*/

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
/// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t anemometer_gatt_db[WIND_SPEED_NUMBER_OF_CHARACTERISTICS] =
    {
        // Heart Rate Service Declaration
        [WIND_SPEED_CHARACTERISTIC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(true_wind_speed_service_uuid), (uint8_t *)&true_wind_speed_service_uuid}},
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst anemometer_profile_tab[ANEMOMETER_PROFILE_NUM] = {
    [ANEMOMETER_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};
uint16_t wind_speed_handle_table[WIND_SPEED_NUMBER_OF_CHARACTERISTICS];

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
  ESP_LOGE(GATTS_TABLE_TAG, "event = %x\n", event);
  switch (event)
  {
  case ESP_GATTS_REG_EVT:
  {
    ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
    esp_ble_gap_set_device_name(DEVICE_NAME);
    ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
    esp_ble_gap_config_adv_data(&anemometer_adv_config);
    ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
    esp_ble_gatts_create_attr_tab(anemometer_gatt_db, gatts_if, WIND_SPEED_NUMBER_OF_CHARACTERISTICS, SERVICE_INSTANCE_ID);
    ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
    break;
  }
  case ESP_GATTS_CONNECT_EVT:
  {
    /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
    esp_ble_conn_update_params_t conn_params = {
        .latency = 0,
        .max_int = 0x30, // max_int = 0x30*1.25ms = 40ms
        .min_int = 0x10, // min_int = 0x10*1.25ms = 20ms
        .timeout = 400   // timeout = 400*10ms = 4000ms
    };
    memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:, is_conn %d",
             param->connect.conn_id,
             param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
             param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
    // start sent the update connection parameters to the peer device.
    esp_ble_gap_update_conn_params(&conn_params);
    anemometer_profile_tab[ANEMOMETER_PROFILE_APP_IDX].conn_id = param->connect.conn_id;
    break;
  }
  case ESP_GATTS_CREAT_ATTR_TAB_EVT:
  {
    ESP_LOGI(GATTS_TABLE_TAG, "The number handle =%x\n", param->add_attr_tab.num_handle);
    if (param->add_attr_tab.status != ESP_GATT_OK)
    {
      ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
    }
    else if (param->add_attr_tab.num_handle != WIND_SPEED_NUMBER_OF_CHARACTERISTICS)
    {
      ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) \
                    doesn't equal to WIND_SPEED_NUMBER_OF_CHARACTERISTICS(%d)",
               param->add_attr_tab.num_handle, WIND_SPEED_NUMBER_OF_CHARACTERISTICS);
    }
    else
    {
      memcpy(wind_speed_handle_table, param->add_attr_tab.handles, sizeof(wind_speed_handle_table));
      esp_ble_gatts_start_service(wind_speed_handle_table[WIND_SPEED_CHARACTERISTIC]);
      ESP_LOGI(GATTS_TABLE_TAG, "Created handle table successfully.");
    }
    break;
  }
  default:
    break;
  }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
  ESP_LOGE(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

  switch (event)
  {
  case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    esp_ble_gap_start_advertising(&anemometer_adv_params);
    break;
  case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
    // advertising start complete event to indicate advertising start successfully or failed
    if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
    {
      ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed\n");
    }
    break;
  default:
    break;
  }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
  ESP_LOGI(GATTS_TABLE_TAG, "EVT %d, gatts if %d\n", event, gatts_if);

  /* If event is register event, store the gatts_if for each profile */
  if (event == ESP_GATTS_REG_EVT)
  {
    if (param->reg.status == ESP_GATT_OK)
    {
      anemometer_profile_tab[ANEMOMETER_PROFILE_APP_IDX].gatts_if = gatts_if;
    }
    else
    {
      ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n",
               param->reg.app_id,
               param->reg.status);
      return;
    }
  }

  do
  {
    int idx;
    for (idx = 0; idx < ANEMOMETER_PROFILE_NUM; idx++)
    {
      if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
          gatts_if == anemometer_profile_tab[idx].gatts_if)
      {
        if (anemometer_profile_tab[idx].gatts_cb)
        {
          anemometer_profile_tab[idx].gatts_cb(event, gatts_if, param);
        }
      }
    }
  } while (0);
}

void app_main()
{
  esp_err_t ret;

  // Initialize NVS.
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret)
  {
    ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed\n", __func__);
    return;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret)
  {
    ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed\n", __func__);
    return;
  }

  ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth\n", __func__);
  ret = esp_bluedroid_init();
  if (ret)
  {
    ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed\n", __func__);
    return;
  }
  ESP_LOGI(GATTS_TABLE_TAG, "%s enable bluetooth\n", __func__);
  ret = esp_bluedroid_enable();
  if (ret)
  {
    ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed\n", __func__);
    return;
  }

  esp_ble_gatts_register_callback(gatts_event_handler);
  esp_ble_gap_register_callback(gap_event_handler);
  esp_ble_gatts_app_register(ESP_ANEMOMETER_APP_ID);
  /*
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
      // printf("Current second rotations: %d\n", current_second_rotations);
      //printf("ROTATIONS: %d, WIND SPEED: %f\n", rotation_frequency, Ar * rotation_frequency + B);
    }
  }
  */
}
