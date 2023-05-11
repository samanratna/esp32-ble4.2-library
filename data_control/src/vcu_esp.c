
#include "vcu_esp.h"
#include "esp_vcu.h"
#include "esp_rec.h"
#include "esp_log.h"

void vcu_data_time_send(uint8_t hour, uint8_t minute, uint8_t sec, uint8_t day, uint8_t month, uint8_t year)
{
    data_time_save_struct.hour = hour;
    data_time_save_struct.minute = minute;
    data_time_save_struct.sec = sec;
    data_time_save_struct.day = day;
    data_time_save_struct.month = month;
    data_time_save_struct.year = year;
    esp_vcu_addTask(0, vcu_data_time_task);
}
void vcu_pass_key_send(uint32_t pass_key)
{
    pass_key_struct.passkey = pass_key;
    printf("CAN_STATUS :: Sending Passkey to VCU\n");
    esp_vcu_addTask(0, vcu_pass_key_task);
}
void vcu_bike_name_send(char *bike_name, uint8_t bike_name_len)
{
    if (bike_name_len > 8)
    {
        bike_name_len = 8;
        ESP_LOGE("BIKE NAME::", "Bike Name length is greater than 8");
    }
    memset(bike_name_struct.bike_name, '\0', sizeof(bike_name_struct.bike_name));
    bike_name_struct.bike_name_len = bike_name_len;
    for (uint8_t i = 0; i < bike_name_len; i++)
    {
        bike_name_struct.bike_name[i] = bike_name[i];
    }

    printf("BLE_status :::: %s length is %d\n", bike_name, bike_name_len);
    esp_vcu_addTask(0, vcu_bike_name_task);
}

void vcu_user_name_send(char *user_name, uint8_t user_name_len)
{
    if (user_name_len > 8)
    {
        user_name_len = 8;
        ESP_LOGE("USER NAME::", "User Name length is greater than 8");
    }
        memset(user_name_struct.user_name, '\0', sizeof(user_name_struct.user_name));

    user_name_struct.user_name_len = user_name_len;
    for (uint8_t i = 0; i < user_name_len; i++)
    {
        user_name_struct.user_name[i] = user_name[i];
    }
    esp_vcu_addTask(0, vcu_user_name_task);
}

void vcu_user_dob_send(uint32_t user_bod_year, uint8_t user_bod_month, uint8_t user_bod_day)
{
    user_dob_struct.user_bod_year = user_bod_year;
    user_dob_struct.user_bod_month = user_bod_month;
    if (user_bod_month > 12)
    {
        ESP_LOGE("USER DOB::", "Month is greater than 12");
        return;
    }
    user_dob_struct.user_bod_day = user_bod_day;
    if (user_bod_day > 32)
    {
        ESP_LOGE("USER DOB::", "Day is greater than 32");
        return;
    }
    esp_vcu_addTask(0, vcu_user_dob_task);
}

void vcu_dash_indicator_send(uint8_t ble_indicator, uint8_t gps_indicator, uint8_t net_indicator)
{
    dash_indicator_struct.ble_indicator = ble_indicator;
    dash_indicator_struct.gps_indicator = gps_indicator;
    dash_indicator_struct.net_indicator = net_indicator;
    esp_vcu_addTask(0, vcu_dash_indicator_task);
}

void vuc_shake_mode_send(uint8_t shake_status)
{
    uint8_t data[8] = {0};
    data[0] = shake_status;
    uint8_t data_len = 1;
    transmit_can_message(vcu_shake_mode_can_id, data, data_len);
}
void vuc_anti_theft_mode_send(uint8_t anti_theft_mode)
{
    uint8_t data[8] = {0};
    data[0] = anti_theft_mode;
    uint8_t data_len = 1;
    transmit_can_message(vcu_theft_mode_can_id, data, data_len);
}
void vcu_kill_bypass(uint8_t kill_switch_state)
{
    uint8_t data[8] = {0};
    data[0] = kill_switch_state;
    uint8_t data_len = 1;
    transmit_can_message(vcu_kill_switch_can_id, data, data_len);
}
void vcu_stand_bypass(uint8_t stand_state)
{
    uint8_t data[8] = {0};
    data[0] = stand_state;
    uint8_t data_len = 1;
    transmit_can_message(vcu_stand_bypass_can_id, data, data_len);
}
void bike_autoff_timer_send(uint8_t timer_value)
{
    uint8_t data[8] = {0};
    data[0] = timer_value;
    uint8_t data_len = 1;
    transmit_can_message(vcu_autoff_timer_can_id, data, data_len);
}
void bike_dash_mode_send(uint8_t dash_mode)
{
    uint8_t data[8] = {0};
    data[0] = dash_mode;
    uint8_t data_len = 1;
    transmit_can_message(vcu_dash_mode_can_id, data, data_len);
}

void bike_time_format(uint8_t format)
{
    uint8_t data[8] = {0};
    data[0] = format;
    uint8_t data_len = 1;
    transmit_can_message(vcu_time_format_can_id, data, data_len);
}

void dash_brightness_level(uint8_t format)
{
    uint8_t data[8] = {0};
    data[0] = format;
    uint8_t data_len = 1;
    transmit_can_message(vcu_dashBrightnessLevel_can_id, data, data_len);
}

void Send_BLE_Pairing_Status_To_VCU(uint8_t format)
{
    uint8_t data[8] = {0};
    data[0] = format;
    uint8_t data_len = 1;
    transmit_can_message(vcu_BLE_pairing_status_can_id, data, data_len);
}

void Send_BLE_Connection_Status_To_VCU(uint8_t format)
{
    uint8_t data[8] = {0};
    data[0] = format;
    uint8_t data_len = 1;
    transmit_can_message(vcu_BLE_connection_status_can_id, data, data_len);
}

void vcu_mode_button_bypass(uint8_t format)
{
    uint8_t data[8] = {0};
    data[0] = format;
    uint8_t data_len = 1;
    transmit_can_message(vcu_mode_button_bypass_command_can_id, data, data_len);
}

