#ifndef INC_BLE_H_
#define INC_BLE_H_

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "main.h"

#define BLE_CONNECTED 1
#define BLE_DISCONNECTED 0

#define BLE_PAIRED 1
#define BLE_UNPAIRED 0

#define GATTS_TABLE_TAG "BLE_STATUS ::"
#define NUMBER_OF_PROFILES 1
#define BLE_PROFILE_APP_IDX 0
#define ESP_BLE_APP_ID 0x55

#define BLE_SVC_INST_ID 0
#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)
#define WDT_TIMEOUT 400
#define HRPS_HT_MEAS_MAX_LEN (13)

#define CONN_SUCCESS_STATUS "ACK,CONN_SUCCESS"
#define ASK_TELEMETRY_TO_APP "REQ,TO_APP_1"
#define ACK_CANCELLATION "CANCEL"

#define FIRST_CONN_INITIATED 1
#define FIRST_CONN_DATA_EXCHANGE_COMPLETE 0

#define SHAKE_MODE_DISABLE "SHAKE,DIS"
#define SHAKE_MODE_ENABLE "SHAKE,EN"

#define ANTI_THEFT_DISABLE "THEFT,DIS"
#define ANTI_THEFT_ENABLE "THEFT,EN"

#define AUTO_TIMER_DISABLE "A_TIMER,OFF"
#define AUTO_TIMER_ENABLE "A_TIMER,10"

#define TIME_FORMAT_12 "T_FORMAT,12"
#define TIME_FORMAT_24 "T_FORMAT,24"

#define LIGHT_MODE "UI_MODE,1"
#define DARK_MODE "UI_MODE,2"
#define AUTO_UI_MODE "UI_MODE,3"

#define BRIGHTNESS_LEVEL "BRIGHTNESS,78"
#define RECEIVE_SUCCESSFUL "ACK,SUCCESS"
#define CONN_PACKET "SRBSSP,2S"

#define IDLE 1
#define BUSY 0
#define complete_data "DATA_COMPLETE"

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

#define UPPER_LIMIT 999999
#define LOWER_LIMIT 100000

#define ALPHA 0.4

// Bike Settings
char shake_mode_setting[15];
char anti_theft_setting[15];

// Bike Power Control Settings
char stand_bypass_setting[15];
char kill_switch_bypass_setting[15];
char mode_button_bypass_setting[15];

char BLE_DEVICE_NAME[20];

// Bike Dash Setting
char auto_timer_setting[15];
char time_format_setting[15];
char ui_mode_setting[15];
char brightness_level_setting[15];

// Bike Dash Indicator
char BLE_status[5];
char GPS_status[5];
char SIM_status[5];

// Odometry
char overall_data[50];
char suste_data[50];
char thikka_data[50];
char babbal_data[50];

// SoC
char SoC_and_range[10];
char ETA_and_chargingStatus[20];
char tripTelemetry[15];

// Bike Power State
char bike_power_state[15];

char BLE_write_value[25];
char bikenames[25];

char _overall_data[5][30];
char _suste_data[5][30];
char _thikka_data[5][30];
char _babal_data[5][30];

// Airlock Setting
char airlock_setting_char[20];
int airlock_setting_int;

uint16_t param_write_handle;
uint16_t temp_param_write_handle;

// SERVICES AND CHARACTERISTICS
static const uint8_t VEHICLE_SERVICE_UUID[16] = {0x20, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t ACKNOWLEDGEMENT_CHARACTERISTIC_UUID[16] = {0x10, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};

static const uint8_t SOC_ESTRANGE_CHARACTERISTIC_UUID[16] = {0x61, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t ETA_CHARGINGSTATUS_CHARACTERISTIC_UUID[16] = {0x62, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t TRIPTELEMETRY_CHARACTERISTIC_UUID[16] = {0x63, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t OVERALLDATA_CHARACTERISTIC_UUID[16] = {0x64, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t SUSTEDATA_CHARACTERISTIC_UUID[16] = {0x65, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t THIKKADATA_CHARACTERISTIC_UUID[16] = {0x66, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t BABBALDATA_CHARACTERISTIC_UUID[16] = {0x67, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};

static const uint8_t SHAKE_MODE_CHARACTERISTIC_UUID[16] = {0x21, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t ANTI_THEFT_CHARACTERISTIC_UUID[16] = {0x22, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t AUTO_TIMER_CHARACTERISTIC_UUID[16] = {0x23, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t TIME_FORMAT_CHARACTERISTIC_UUID[16] = {0x24, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t UI_MODE_CHARACTERISTIC_UUID[16] = {0x25, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t BRIGHTNESS_LEVEL_CHARACTERISTIC_UUID[16] = {0x26, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t KILL_SWITCH_BYPASS_CHARACTERISTIC_UUID[16] = {0x27, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t MODE_BUTTON_BYPASS_CHARACTERISTIC_UUID[16] = {0x28, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};

static const uint8_t USERNAME_CHARACTERISTIC_UUID[16] = {0x41, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t BIKENAME_CHARACTERISTIC_UUID[16] = {0x42, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t BIRTHDAY_CHARACTERISTIC_UUID[16] = {0x43, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t LIGHTS_CHARACTERISTIC_UUID[16] = {0x44, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t LIGHTS_AND_HORN_CHARACTERISTIC_UUID[16] = {0x45, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t TRIP_RESET_CHARACTERISTIC_UUID[16] = {0x46, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t BIKE_ON_OFF_CHARACTERISTIC_UUID[16] = {0x47, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};

static const uint8_t MAINTAIN_BLE_CONNECTION_CHARACTERISTIC_UUID[16] = {0x81, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t RSSI_VALUE_CHARACTERISTIC_UUID[16] = {0x82, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t OTA_CHARACTERISTIC_UUID[16] = {0x83, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
static const uint8_t AIRLOCK_CHARACTERISTIC_UUID[16] = {0x84, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};

static const uint8_t PING_CHARACTERISTIC_UUID[16] = {0x30, 0xd8, 0x3e, 0x12, 0x54, 0x1f, 0x94, 0xad, 0x6c, 0x4b, 0x92, 0x81, 0x22, 0x35, 0xc8, 0x2c};
int number_of_chunks_overall_data;
int number_of_chunks_suste_data;
int number_of_chunks_thikka_data;
int number_of_chunks_babbal_data;

int BLE_STATE; // This state will be BUSY when COMM is sending/receiving BLE data via VCU
int pairing_status;
int BLE_CONNECTION_STATUS;
int ble_get_data;
int ble_task_flag;
int got_all_data_from_VCU_to_be_notified;
int RECEIVED_CHANGE_OF_SETTING;
int ble_conn_maintain;

int timer_already_started;

// RSSI Calculation
int last_rssi_value;
int current_rssi_value;
int accurate_rssi_value;

int Just_Connected_flag;
int _bike_power_state;
int timer_status_for_rssi;
int initial_rssi_value;

enum
{
    CONN_SUCCESS = 1,
    GOT_BIKE_SETTINGS,
    GOT_BIKE_DASH_SETTINGS,
    GOT_OVERALL_DATA,
    GOT_SUSTE_DATA,
    GOT_THIKKA_DATA,
    GOT_BABBAL_DATA,
    GOT_SOC_AND_RANGE,
    GOT_ETA_AND_CHARGING_STATUS,
    GOT_TRIP_TELEMETRY,
    GOT_BIKE_POWER_STATUS
};

enum
{
    SETTING_BIKENAME = 1,
    SETTING_USERNAME,
    SETTING_LIGHTS,
    SETTING_LIGHTS_AND_HORN,
    SETTING_TRIP_RESET,
    SETTING_BIKE_POWER,
    SETTING_BIRTHDAY,
    SETTING_SHAKE_MODE,
    SETTING_UI_MODE,
    SETTING_ANTI_THEFT,
    SETTING_AUTO_TIMER,
    SETTING_TIME_FORMAT,
    SETTING_BRIGHTNESS,
    SETTING_KILL_SWITCH_BYPASS,
    SETTING_MODE_BUTTON_BYPASS,
    SETTING_STAND_BYPASS
};

typedef struct
{
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

// Attributes State Machine
enum
{
    vehicle_service,

    acknowledgement_characteristics_index,
    acknowledgement_characteristics_value,
    acknowledgement_characteristics_descriptor,

    maintain_ble_connection_characteristics_index,
    maintain_ble_connection_characteristics_value,
    maintain_ble_connection_characteristics_descriptor,

    HRS_IDX_NB,
};

// -------------------- DUMMY DATA FOR TEST -------------------- //
#define dummy_SoC_estRange_data "46,58"
#define dummy_SoC_estRange_data1 "90,14"
#define dummy_eta_ChargingStatus_data "57,1"
#define dummy_trip_telemetry_data "22,33,44"
// -------------------- DUMMY DATA FOR TEST -------------------- //

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

void ble_conn_timer_init(void);

// Function Prototypes
void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void BLE_notify_data(char *value, int characterictic_value);

#endif
