
#include "vcu.h"

Vcu_DateTimeSave_Handel data_time_save_struct;
Vcu_PassKey_Handel pass_key_struct;
Vcu_BikeNameSave_Handel bike_name_struct;
Vcu_UserNameSave_Handel user_name_struct;
Vcu_UserDobSave_Handel user_dob_struct;
Vcu_BikeSettingSave_Handel user_bike_setting_struct;
Vcu_BikeDashSettingSave_Handel bike_dash_setting_struct;
Vcu_DashIndicator_Handler dash_indicator_struct;
Vcu_UserBikePower_State user_bike_power_status;

ESP_BLE_ConnectAck_Handler end_of_data;
Vcu_BikeSettingSave_Handel user_bike_setting_struct;
Vcu_BikeDashSettingSave_Handel bike_dash_setting_struct;
Dash_OverallDailyOdo_Handel overall_odo_struct;
Dash_SusteDailyOdo_Handel suste_odo_struct;
Dash_ThikkaDailyOdo_Handel thikka_odo_struct;
Dash_BabalDailyOdo_Handel babbal_odo_struct;
ESP_BLE_soc_range soc_and_range;
ESP_BLE_Battery_Handler eta_and_charging_status;
Esp_trip_telemetry_Handel trip_telemetry;

int vcu_data_time_task(void)
{
    return (vcu_can_task(&data_time_save_struct, sizeof(data_time_save_struct), vcu_date_time_can_id));
}

int vcu_pass_key_task(void)
{
    return (vcu_can_task(&pass_key_struct, sizeof(pass_key_struct), vcu_pass_key_can_id));
}

int vcu_bike_name_task(void)
{
    return (vcu_can_task(&bike_name_struct, sizeof(bike_name_struct), vcu_bike_name_can_id));
}

int vcu_user_name_task(void)
{
    return (vcu_can_task(&user_name_struct, sizeof(user_name_struct), vcu_user_name_can_id));
}

int vcu_user_dob_task(void)
{
    return (vcu_can_task(&user_dob_struct, sizeof(user_dob_struct), vcu_user_dob_can_id));
}

int vcu_bike_setting_task(void)
{
    return (vcu_can_task(&user_bike_setting_struct, sizeof(user_bike_setting_struct), vcu_user_bike_setting_can_id));
}

int vcu_dash_setting_task(void)
{
    return (vcu_can_task(&bike_dash_setting_struct, sizeof(bike_dash_setting_struct), vcu_dash_setting_can_id));
}

int vcu_dash_indicator_task(void)
{
    return (vcu_can_task(&dash_indicator_struct, sizeof(dash_indicator_struct), vcu_dash_indicator_can_id));
}

void send_release_notes(void)
{
    esp_vcu_addTask(0, send_release_notes_task);
}
int send_release_notes_task(void)
{
    return (vcu_can_task(&update_detais, sizeof(update_detais), Send_Release_Notes));
}

void vcu_hunk_command(void)
{
    uint8_t data[8] = {0};
    uint8_t data_len = 1;
    transmit_can_message(vcu_hunk_can_id, data, data_len);
}

void vcu_bunk_command(void)
{
    uint8_t data[8] = {0};
    uint8_t data_len = 1;
    transmit_can_message(vcu_bunk_can_id, data, data_len);
}

void vcu_trip_erase_command(void)
{
    uint8_t data[8] = {0};
    uint8_t data_len = 1;
    transmit_can_message(vcu_trip_erase_can_id, data, data_len);
}

void vcu_vehicle_on_command(void)
{
    uint8_t data[8] = {0};
    uint8_t data_len = 1;
    transmit_can_message(vcu_vehicle_on_command_can_id, data, data_len);
}

void vcu_vehicle_off_command(void)
{
    uint8_t data[8] = {0};
    uint8_t data_len = 1;
    transmit_can_message(vcu_vehicle_off_command_can_id, data, data_len);
}

void vcu_mode_button_bypass_command(void)
{
    uint8_t data[8] = {0};
    uint8_t data_len = 1;
    transmit_can_message(vcu_mode_button_bypass_command_can_id, data, data_len);
}

void vcu_mode_button_unbypass_command(void)
{
    uint8_t data[8] = {0};
    uint8_t data_len = 1;
    transmit_can_message(vcu_mode_button_unbypass_command_can_id, data, data_len);
}
