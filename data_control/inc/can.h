#ifndef INC_CAN_H_
#define INC_CAN_H_

//////////////////////////////////////            OTA CAN ID                /////////////////////////////////////////////

/*Send*/
#define Bike_cred_req 0x0001FAFB
#define Send_Release_Notes 0x0010FAFB
// #define bike_cred_receive_ack 0x0002FAFB
#define OTA_POP_REQ 0x0003FAFB

#define OTA_UI_DISPLAY 0x0001FCFB
#define OTA_EST_TIME 0x0002FCFB

#define ask_bike_status 0x0004FAFB
#define OTA_REQ_JUMP 0x0005FAFB
#define ESP_OTA_STATUS 0x0006FAFB
#define ESP_File_Size 0x0007FAFB
#define TOTAL_File_Size 0x0008FAFB
#define CRC_SEND_VCU 0x0009FAFB
#define SEND_File_Size 0x000AFAFB
#define Data_To_VCU 0x000BFAFB
#define ESP_UPDATE_WAIT_ACK 0x000CFAFB
#define CRC_SEND_DISPLAY 0x000DFAFB
#define Version_Length 0x000EFAFB
#define Send_Version 0x000FFAFB

/*Receive*/
#define bike_cred_ack 0x0001FBFA
#define state_check_ota 0x0002FBFA
#define VCU_TO_ESP_OTA 0x0003FBFA
#define VCU_TO_ESP_ISSUE 0x0004FBFA

// #define JUMP_ACK 0x18DF28B0

#define Get_Display_CRC 0x18DF28E1 // TODO

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////            OBD CAN ID                /////////////////////////////////////////////
/*SEND*/
#define ESP_OBD_SEND 0x0401FAFB

/*RECEIVE*/
#define ESP_OBD_RECEIVE 0x0401FBFA
#define OBD_DATA 0x0402FBFA
#define OBD_POST_REQ 0x0403FBFA

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////            SIM CAN ID                /////////////////////////////////////////////

#define POST_SOC 0x0804FBFA
#define POST_TELEMETRY 0x0805FBFA
#define POST_BATTERY_LOG 0x0806FBFA
#define POST_CHARGE_PLUGGED 0x0809FBFA
#define POST_CHARGE_UNPLUGGED 0x0807FBFA
#define POST_CHARGE_COMPLETION 0x080AFBFA
#define POST_THEFT_ALERT 0x080BFBFA
#define POST_CRASH_DETECTION 0x0808FBFA
#define POST_GPS_LOCATION 0x080DFBFA
#define UPDATE_TIME 0x080CFBFA

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////            BLE CAN ID                /////////////////////////////////////////////

#define OVERALL_DATA 0x0854FBFA
#define SUSTE_DATA 0x0857FBFA
#define THIKKA_DATA 0x0856FBFA
#define BABBAL_DATA 0x0855FBFA
#define SOC_AND_RANGE 0x0859FBFA
#define ETA_AND_CHARGING_STATUS 0x085AFBFA
#define END_OF_DATA 0x0858FBFA
#define BIKE_SETTINGS 0x0852FBFA
#define BIKE_DASH_SETTINGS 0x0853FBFA
#define TRIP_TELEMETRY 0x085BFBFA
#define BIKE_POWER_STATUS 0x0851FBFA

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define response_can_id 0x0801FBFA
#define ack_can_id 0x0802FBFA
#define stop_can_id 0x0803FBFA

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define ESP_REC_START_ACK_ID 0x080FFBFA
#define ESP_REC_DATA_ACK_ID 0x080EFBFA
#define ESP_REC_END_ACK_ID 0x0810FBFA

// vehicle data send command id
#define vcu_date_time_can_id 0x0C04FAFB
#define vcu_pass_key_can_id 0x0C05FAFB
#define vcu_bike_name_can_id 0x0C06FAFB
#define vcu_user_name_can_id 0x0C07FAFB
#define vcu_user_dob_can_id 0x0C08FAFB
#define vcu_user_bike_setting_can_id 0x18CE2808
#define vcu_dash_setting_can_id 0x18CE2809
#define vcu_dash_indicator_can_id 0x18CE280A

// vehicle can command id
#define vcu_hunk_can_id 0x0C12FAFB
#define vcu_bunk_can_id 0x0C13FAFB
#define vcu_trip_erase_can_id 0x0C14FAFB
#define vcu_vehicle_on_command_can_id 0x0C15FAFB
#define vcu_vehicle_off_command_can_id 0x0C16FAFB
#define vcu_mode_button_bypass_command_can_id 0x0C17FAFB
#define vcu_mode_button_unbypass_command_can_id 0x0C18FAFB

// vehicle can id for BLE
#define vcu_shake_mode_can_id 0x0C0AFAFB
#define vcu_theft_mode_can_id 0x0C0BFAFB
#define vcu_kill_switch_can_id 0x0C0CFAFB
#define vcu_stand_bypass_can_id 0x0C0DFAFB
#define vcu_autoff_timer_can_id 0x0C0EFAFB
#define vcu_dash_mode_can_id 0x0C0FFAFB
#define vcu_time_format_can_id 0x0C10FAFB
#define vcu_dashBrightnessLevel_can_id 0x0C11FAFB
// #define vcu_BLE_pairing_status_can_id 0x18CE2821
#define vcu_BLE_pairing_status_can_id 0x0C09FAFB
#define vcu_BLE_connection_status_can_id 0x0C19FAFB

// SIM voltage send to VCU

#define esp_sim_voltage_can_id 0x18FF0D28
#define esp_gps_post_start_can_id 0x0C1BFAFB
#define esp_gps_signal_status 0x0C1AFAFB
#define esp_post_status_can_id 0x0C1CFAFB
// #define sim_module_reset_can_id 0x0C1DFAFB

extern uint8_t twai_tx_data[8];
extern const twai_message_t tx_msg;

void twai_task(void *arg);
void twai_init(void);
// void twai_tx(uint32_t add, uint8_t data_len);
int twaiTxData(uint32_t id, const uint8_t *tx_buff, uint8_t data_len);
void twai_alert_read(void);

#endif