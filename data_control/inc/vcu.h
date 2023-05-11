
#ifndef INC_VCU_H_
#define INC_VCU_H_

#include "esp_vcu_communication.h"
#include "stdio.h"
#include "stdint.h"

/* ESP32 to VCU CAN IDs */

typedef struct
{
	uint8_t hour;
	uint8_t minute;
	int8_t sec;
	uint8_t day;
	uint8_t month;
	uint8_t year;
} Vcu_DateTimeSave_Handel;
extern Vcu_DateTimeSave_Handel data_time_save_struct;

typedef struct
{
	uint32_t passkey;
} Vcu_PassKey_Handel;
extern Vcu_PassKey_Handel pass_key_struct;

typedef struct
{
	uint8_t bike_name_len;
	char bike_name[8];
} Vcu_BikeNameSave_Handel;
extern Vcu_BikeNameSave_Handel bike_name_struct;

typedef struct
{
	uint8_t user_name_len;
	char user_name[8];
} Vcu_UserNameSave_Handel;
extern Vcu_UserNameSave_Handel user_name_struct;

typedef struct
{
	uint32_t user_bod_year;
	uint8_t user_bod_month;
	uint8_t user_bod_day;
} Vcu_UserDobSave_Handel;
extern Vcu_UserDobSave_Handel user_dob_struct;

typedef struct
{
	uint8_t bike_power_status;
	uint8_t kill_bypass_status;
	uint8_t mode_bypass_status;
	uint8_t stand_switch_status;
} Vcu_UserBikePower_State;
extern Vcu_UserBikePower_State user_bike_power_status;

typedef struct
{
	uint8_t bike_shake_mode;
	uint8_t bike_anti_theft;
} Vcu_BikeSettingSave_Handel;
extern Vcu_BikeSettingSave_Handel user_bike_setting_struct;

typedef struct
{
	uint8_t bike_autoff_time;
	uint8_t bike_dash_mode;
	uint8_t bike_dash_format;
	uint8_t brightness_level;
} Vcu_BikeDashSettingSave_Handel;
extern Vcu_BikeDashSettingSave_Handel bike_dash_setting_struct;

typedef struct
{
	uint8_t ble_indicator;
	uint8_t gps_indicator;
	uint8_t net_indicator;
} Vcu_DashIndicator_Handler;
extern Vcu_DashIndicator_Handler dash_indicator_struct;

typedef struct
{
	uint8_t overall_odo_day1;
	uint8_t overall_odo_day2;
	uint8_t overall_odo_day3;
	uint8_t overall_odo_day4;
	uint8_t overall_odo_day5;
	uint8_t overall_odo_day6;
	uint8_t overall_odo_day7;
	uint8_t overall_avg_speed;
	uint8_t overall_max_speed;
	uint8_t overall_avg_whpkm;
	uint8_t day;
	float overall_odo;
} Dash_OverallDailyOdo_Handel;
extern Dash_OverallDailyOdo_Handel overall_odo_struct;

typedef struct
{
	uint8_t babal_odo_day1;
	uint8_t babal_odo_day2;
	uint8_t babal_odo_day3;
	uint8_t babal_odo_day4;
	uint8_t babal_odo_day5;
	uint8_t babal_odo_day6;
	uint8_t babal_odo_day7;
	uint8_t babal_avg_speed;
	uint8_t babal_max_speed;
	uint8_t babal_avg_whpkm;
	uint8_t day;
	float babal_odo;
} Dash_BabalDailyOdo_Handel;
extern Dash_BabalDailyOdo_Handel babbal_odo_struct;

typedef struct
{
	uint8_t thikka_odo_day1;
	uint8_t thikka_odo_day2;
	uint8_t thikka_odo_day3;
	uint8_t thikka_odo_day4;
	uint8_t thikka_odo_day5;
	uint8_t thikka_odo_day6;
	uint8_t thikka_odo_day7;
	uint8_t thikka_avg_speed;
	uint8_t thikka_max_speed;
	uint8_t thikka_avg_whpkm;
	uint8_t day;
	float thikka_odo;
} Dash_ThikkaDailyOdo_Handel;
extern Dash_ThikkaDailyOdo_Handel thikka_odo_struct;

typedef struct
{
	uint8_t suste_odo_day1;
	uint8_t suste_odo_day2;
	uint8_t suste_odo_day3;
	uint8_t suste_odo_day4;
	uint8_t suste_odo_day5;
	uint8_t suste_odo_day6;
	uint8_t suste_odo_day7;
	uint8_t suste_avg_speed;
	uint8_t suste_max_speed;
	uint8_t suste_avg_whpkm;
	uint8_t day;
	float suste_odo;
} Dash_SusteDailyOdo_Handel;
extern Dash_SusteDailyOdo_Handel suste_odo_struct;

typedef struct
{
	uint8_t ble_conenct_ack;
} ESP_BLE_ConnectAck_Handler;
extern ESP_BLE_ConnectAck_Handler end_of_data;

typedef struct
{
	uint8_t vehicle_soc;
	uint8_t vehicle_range;
} ESP_BLE_soc_range;
extern ESP_BLE_soc_range soc_and_range;

typedef struct
{
	uint8_t battery_eta;
	uint8_t charging_status;
} ESP_BLE_Battery_Handler;
extern ESP_BLE_Battery_Handler eta_and_charging_status;

typedef struct
{
	uint8_t trip_maxSpeed;
	uint8_t trip_avgSpeed;
	float trip_Distance;
} Esp_trip_telemetry_Handel;
extern Esp_trip_telemetry_Handel trip_telemetry;
/********************************************************/

// Structure send task handler
int vcu_data_time_task(void);
int vcu_pass_key_task(void);
int vcu_bike_name_task(void);
int vcu_user_name_task(void);
int vcu_user_dob_task(void);
int vcu_bike_setting_task(void);
int vcu_dash_setting_task(void);
int vcu_dash_indicator_task(void);

// command send task handler
void vcu_hunk_command(void);
void vcu_bunk_command(void);
void vcu_trip_erase_command(void);
void vcu_vehicle_on_command(void);
void vcu_vehicle_off_command(void);
void vcu_mode_button_bypass_command(void);
void vcu_mode_button_unbypass_command(void);

void send_release_notes(void);
int send_release_notes_task(void);

#endif
