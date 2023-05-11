
#ifndef INC_VCU_ESP_H_
#define INC_VCU_ESP_H_

#include "esp_vcu_communication.h"
#include "vcu.h"

void vcu_data_time_send(uint8_t hour,uint8_t minute,uint8_t sec,uint8_t day,uint8_t month,uint8_t year);
void vcu_pass_key_send(uint32_t pass_key);
void vcu_bike_name_send(char *bike_name,uint8_t bike_name_len);
void vcu_user_name_send(char *user_name,uint8_t user_name_len);
void vcu_user_dob_send(uint32_t user_bod_year,uint8_t user_bod_month,uint8_t user_bod_day);
void vcu_dash_indicator_send(uint8_t ble_indicator,uint8_t gps_indicator,uint8_t net_indicator);
void vuc_shake_mode_send(uint8_t shake_status);
void vuc_anti_theft_mode_send(uint8_t anti_theft_mode);
void vcu_kill_bypass(uint8_t kill_switch_state);
void vcu_stand_bypass(uint8_t stand_state);
void bike_autoff_timer_send(uint8_t timer_value);
void bike_dash_mode_send(uint8_t dash_mode);
void bike_time_format(uint8_t format);
void dash_brightness_level(uint8_t format);
void Send_BLE_Connection_Status_To_VCU(uint8_t format);
void Send_BLE_Pairing_Status_To_VCU(uint8_t format);

#endif