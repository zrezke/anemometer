enum
{
  WIND_SPEED_SERVICE,
  WIND_SPEED_CHARACTERISTIC,
  WIND_SPEED_CHAR_VAL,
  WIND_SPEED_CFG,
  WIND_SPEED_NUMBER_OF_CHARACTERISTICS,
};

struct gatts_profile_inst
{
  esp_gatts_cb_t gatts_cb;
  uint16_t gatts_if;
  uint16_t app_id;
  uint16_t conn_id;
  uint16_t service_handle;
  esp_gatt_srvc_id_t service_id;
  uint16_t char_handle;
  esp_bt_uuid_t char_uuid;
  esp_gatt_perm_t perm;
  esp_gatt_char_prop_t property;
  uint16_t descr_handle;
  esp_bt_uuid_t descr_uuid;
};
