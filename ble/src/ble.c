#include "main.h"
#include "ble.h"

/* FUNCTION PROTOTYPES */

uint8_t notify_data[30];
static prepare_type_env_t prepare_write_env;
static uint8_t adv_config_done = 0;
static uint16_t BLE_COMM_DATABASE_TABLE[HRS_IDX_NB];
static uint8_t test_manufacturer[10] = {'E', 'S', 'P', '3', '2', 'A', 'B', 'C', 'D', 'E'};

uint8_t timer_count;
uint8_t timerStoppedFlag;
uint8_t timerStoppedFlag1;

esp_timer_handle_t periodic_timer;

static void periodic_timer_callback(void *arg);

const esp_timer_create_args_t periodic_timer_args = {
    .callback = &periodic_timer_callback,
    /* name is optional, but may help identify the timer when debugging */
    .name = "Periodic timer for BLE"};

void ble_conn_timer_init(void)
{
    // Configure timer
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    printf("BLE_STATUS :: Initializing Timer for BLE\n");
}

// UUID
static uint8_t sec_service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0x18,
    0x0E,
    0x00,
    0x00,
};

// config adv data
static esp_ble_adv_data_t ble_adv_config = {
    .set_scan_rsp = false,
    .include_txpower = true,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, // &test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(sec_service_uuid),
    .p_service_uuid = sec_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// config scan response data
static esp_ble_adv_data_t ble_rsp_config = {
    .set_scan_rsp = true,
    .include_name = true,
    .manufacturer_len = sizeof(test_manufacturer),
    .p_manufacturer_data = test_manufacturer,
};

static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min = 0x100,
    .adv_int_max = 0x100,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_RANDOM,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
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

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst ble_profile_tab[NUMBER_OF_PROFILES] = {
    [BLE_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t ble_measurement_cc[2] = {0x00, 0x00};

// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t ble_gatt_db[HRS_IDX_NB] = {

    [vehicle_service] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(VEHICLE_SERVICE_UUID), (uint8_t *)&VEHICLE_SERVICE_UUID}},

    [acknowledgement_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [acknowledgement_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&ACKNOWLEDGEMENT_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [acknowledgement_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [SoC_estRange_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [SoC_estRange_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&SOC_ESTRANGE_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [SoC_estRange_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [eta_ChargingStatus_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [eta_ChargingStatus_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&ETA_CHARGINGSTATUS_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [eta_ChargingStatus_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [overall_data_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [overall_data_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&OVERALLDATA_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [overall_data_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [suste_data_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [suste_data_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&SUSTEDATA_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [suste_data_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [thikka_data_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [thikka_data_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&THIKKADATA_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [thikka_data_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [babbal_data_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [babbal_data_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&BABBALDATA_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [babbal_data_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [trip_telemetry_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [trip_telemetry_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&TRIPTELEMETRY_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [trip_telemetry_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [shake_mode_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [shake_mode_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&SHAKE_MODE_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [shake_mode_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [anti_theft_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [anti_theft_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&ANTI_THEFT_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [anti_theft_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [auto_timer_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [auto_timer_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&AUTO_TIMER_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [auto_timer_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [time_format_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [time_format_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&TIME_FORMAT_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [time_format_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [ui_mode_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [ui_mode_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&UI_MODE_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [ui_mode_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [brightness_level_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [brightness_level_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&BRIGHTNESS_LEVEL_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [brightness_level_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [username_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [username_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&USERNAME_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [username_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [bikename_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [bikename_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&BIKENAME_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [bikename_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [birthday_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [birthday_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&BIRTHDAY_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [birthday_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [lights_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [lights_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&LIGHTS_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [lights_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [lights_and_horn_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [lights_and_horn_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&LIGHTS_AND_HORN_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [lights_and_horn_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [trip_reset_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [trip_reset_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&TRIP_RESET_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [trip_reset_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [bike_ON_OFF_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [bike_ON_OFF_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&BIKE_ON_OFF_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [bike_ON_OFF_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [kill_switch_bypass_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [kill_switch_bypass_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&KILL_SWITCH_BYPASS_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [kill_switch_bypass_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [mode_button_bypass_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [mode_button_bypass_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&MODE_BUTTON_BYPASS_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [mode_button_bypass_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [maintain_ble_connection_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [maintain_ble_connection_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&MAINTAIN_BLE_CONNECTION_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [maintain_ble_connection_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [rssi_value_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [rssi_value_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&RSSI_VALUE_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [rssi_value_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [ping_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [ping_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&PING_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [ping_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},

    [ota_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [ota_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&OTA_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [ota_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},
    
    [airlock_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [airlock_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&AIRLOCK_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [airlock_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},
};

static char *esp_key_type_to_str(esp_ble_key_type_t key_type)
{
    char *key_str = NULL;
    switch (key_type)
    {
    case ESP_LE_KEY_NONE:
        key_str = "ESP_LE_KEY_NONE";
        break;
    case ESP_LE_KEY_PENC:
        key_str = "ESP_LE_KEY_PENC";
        break;
    case ESP_LE_KEY_PID:
        key_str = "ESP_LE_KEY_PID";
        break;
    case ESP_LE_KEY_PCSRK:
        key_str = "ESP_LE_KEY_PCSRK";
        break;
    case ESP_LE_KEY_PLK:
        key_str = "ESP_LE_KEY_PLK";
        break;
    case ESP_LE_KEY_LLK:
        key_str = "ESP_LE_KEY_LLK";
        break;
    case ESP_LE_KEY_LENC:
        key_str = "ESP_LE_KEY_LENC";
        break;
    case ESP_LE_KEY_LID:
        key_str = "ESP_LE_KEY_LID";
        break;
    case ESP_LE_KEY_LCSRK:
        key_str = "ESP_LE_KEY_LCSRK";
        break;
    default:
        key_str = "INVALID BLE KEY TYPE";
        break;
    }
    return key_str;
}

// Authentication Types
static char *esp_auth_req_to_str(esp_ble_auth_req_t auth_req)
{
    char *auth_str = NULL;
    switch (auth_req)
    {
    case ESP_LE_AUTH_NO_BOND:
        auth_str = "ESP_LE_AUTH_NO_BOND";
        break;
    case ESP_LE_AUTH_BOND:
        auth_str = "ESP_LE_AUTH_BOND";
        break;
    case ESP_LE_AUTH_REQ_MITM:
        auth_str = "ESP_LE_AUTH_REQ_MITM";
        break;
    case ESP_LE_AUTH_REQ_BOND_MITM:
        auth_str = "ESP_LE_AUTH_REQ_BOND_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_ONLY:
        auth_str = "ESP_LE_AUTH_REQ_SC_ONLY";
        break;
    case ESP_LE_AUTH_REQ_SC_BOND:
        auth_str = "ESP_LE_AUTH_REQ_SC_BOND";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM:
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM_BOND:
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM_BOND";
        break;
    default:
        auth_str = "INVALID BLE AUTH REQ";
        break;
    }

    return auth_str;
}

// Remove bonded devices
static void __attribute__((unused)) remove_all_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    for (int i = 0; i < dev_num; i++)
    {
        esp_ble_remove_bond_device(dev_list[i].bd_addr);
    }
    free(dev_list);
}

/**
 * @brief 
 * GAP EVENTS
 * 
 * @param event 
 * @param param 
 */
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGV(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

    switch (event)
    {
    case  ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EVT FOR RSSI");
        printf("\n==============Inisde RSSI value==============\n");
        current_rssi_value = param->read_rssi_cmpl.rssi;

        if (initial_rssi_value == 1){
            last_rssi_value = current_rssi_value;
            initial_rssi_value = 0;
        }

        // EXPONENTIAL FILTER Calculation
        accurate_rssi_value = (1 - ALPHA) * last_rssi_value + ALPHA * current_rssi_value;

        printf("\nlast RSSI Value: %d     current RSSI Value: %d      accurate RSSI Value: %d\n", last_rssi_value, current_rssi_value, accurate_rssi_value);
        last_rssi_value = accurate_rssi_value;

        // Airlock
        if (accurate_rssi_value > (-80)){

            // Turn Bike ON
            printf("\n--------------Bike turning ON in close proximity--------------\n");
            vcu_vehicle_on_command();
            Just_Connected_flag = 0;
            timer_status_for_rssi = 0;
        }

        break;

    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&ble_adv_params);
        }
        break;

    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&ble_adv_params);
        }
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
        // advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed, error status = %x", param->adv_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TABLE_TAG, "advertising start success");
        break;

    case ESP_GAP_BLE_PASSKEY_REQ_EVT:

        /* passkey request event */
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
        /* Call the following function to input the passkey which is displayed on the remote device */
        // esp_ble_passkey_reply(ble_profile_tab[BLE_PROFILE_APP_IDX].remote_bda, true, 0x00);
        break;

    case ESP_GAP_BLE_OOB_REQ_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_OOB_REQ_EVT");
        uint8_t tk[16] = {1}; // If you paired with OOB, both devices need to use the same tk
        esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk, sizeof(tk));
        break;

    case ESP_GAP_BLE_LOCAL_IR_EVT:

        /* BLE local IR event */
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_LOCAL_IR_EVT");
        break;

    case ESP_GAP_BLE_LOCAL_ER_EVT:

        /* BLE local ER event */
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_LOCAL_ER_EVT");
        break;

    case ESP_GAP_BLE_NC_REQ_EVT:

        /* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
        show the passkey number to the user to confirm it with the number displayed by peer device. */
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%d", param->ble_security.key_notif.passkey);
        break;

    case ESP_GAP_BLE_SEC_REQ_EVT:

        /* send the positive(true) security response to the peer device to accept the security request.
        If not accept the security request, should send the security response with negative(false) accept value*/
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:

        // the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
        // show the passkey number to the user to input it in the peer device.
        ESP_LOGI(GATTS_TABLE_TAG, "The passkey Notify number: %d", param->ble_security.key_notif.passkey);

        int _passkey = param->ble_security.key_notif.passkey;
        vcu_pass_key_send(_passkey); // Send passkey to VCU
        break;

    case ESP_GAP_BLE_KEY_EVT:

        // shows the ble key info share with peer device to the user.
        ESP_LOGI(GATTS_TABLE_TAG, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));

        // Pairing Successful EVENT that confirms bonding
        pairing_status = 1;
        Send_BLE_Pairing_Status_To_VCU(BLE_PAIRED);
        printf("BLE_STATUS :: PAIRING SUCCESSFUL\n");
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
    {
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTS_TABLE_TAG, "remote BD_ADDR: %08x%04x", (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3], (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(GATTS_TABLE_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(GATTS_TABLE_TAG, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");

        if (!param->ble_security.auth_cmpl.success)
        {
            ESP_LOGI(GATTS_TABLE_TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);

            // Pairing Unsuccessful
            printf("BLE_STATUS :: PAIRING FAILED : CAN'T SEND CAN REQUEST \n");

            // Disconnect on Connection unsuccessful
            esp_ble_gatts_close(3, 0);

            // Reset Pairing Flag
            pairing_status = 0;

            // Send BLE Connected info to VCU
            Send_BLE_Connection_Status_To_VCU(BLE_DISCONNECTED);

            // Send BLE Unpaired info to VCU
            Send_BLE_Pairing_Status_To_VCU(BLE_UNPAIRED);
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "auth mode = %s", esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));

            // Set Pairing Status to SUCCESSFUL
            pairing_status = 1;

            // Notify pairing success to YatriHub
            vTaskDelay(1500 / portTICK_PERIOD_MS);

            // Send BLE Connected info to VCU
            Send_BLE_Connection_Status_To_VCU(BLE_CONNECTED);

            // Test Airlock
            Just_Connected_flag = 1;
            initial_rssi_value = 1;
        }
        // show_bonded_devices();
        break;
    }

    case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT:
    {
        ESP_LOGD(GATTS_TABLE_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT status = %d", param->remove_bond_dev_cmpl.status);
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV");
        ESP_LOGI(GATTS_TABLE_TAG, "-----ESP_GAP_BLE_REMOVE_BOND_DEV----");
        esp_log_buffer_hex(GATTS_TABLE_TAG, (void *)param->remove_bond_dev_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTS_TABLE_TAG, "------------------------------------");
        break;
    }

    case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:

        if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config local privacy failed, error status = %x", param->local_privacy_cmpl.status);
            break;
        }

        esp_err_t ret = esp_ble_gap_config_adv_data(&ble_adv_config);
        if (ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
        }
        else
        {
            adv_config_done |= ADV_CONFIG_FLAG;
        }

        ret = esp_ble_gap_config_adv_data(&ble_rsp_config);
        if (ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
        }
        else
        {
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
        }

        break;
    default:
        break;
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");

    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf)
    {
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }
    else
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
    }

    if (prepare_write_env->prepare_buf)
    {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

/**
 * @brief 
 * Notify data through BLE
 * 
 * @param value                 : char data to be notified
 * @param characteristic_value  : characteristic UUID
 */
void BLE_notify_data(char *value, int characteristic_value)
{
    printf("BLE_STATUS :: Notifying the following data :: %s\n", value);
    esp_ble_gatts_send_indicate(3, 0, BLE_COMM_DATABASE_TABLE[characteristic_value], strlen(value), (uint8_t *)value, false);
}

/**
 * @brief 
 * GATTS PROFILE EVENT HANDLER
 * 
 * @param event 
 * @param gatts_if 
 * @param param 
 */
void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGV(GATTS_TABLE_TAG, "event = %x\n", event);
    switch (event)
    {

    // EVENT : WHEN BLE DRIVERS ARE ALL CONFIGURED
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_REG_EVT");
        esp_ble_gap_set_device_name(BLE_DEVICE_NAME);
        esp_ble_gap_config_local_privacy(true);
        esp_ble_gatts_create_attr_tab(ble_gatt_db, gatts_if, HRS_IDX_NB, BLE_SVC_INST_ID);
        break;

    case ESP_GATTS_READ_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
        break;

    case ESP_GATTS_WRITE_EVT:
        if (!param->write.is_prep)
        {
            if (pairing_status == 1)
            {

                if (BLE_COMM_DATABASE_TABLE[maintain_ble_connection_characteristics_value] == param->write.handle)
                {
                    printf("BLE_STATUS :: BLE CONNECTION IS STABLE\n");
                    ble_conn_maintain = 0;
                }

                // Airlock EN Logic
                if (airlock_setting_int == 1 || airlock_setting_int == 0){
                    if (timer_status_for_rssi == 1){
                        if (Just_Connected_flag == 1){

                            // Check if Bike is OFF
                            if(_bike_power_state == 2){

                                //for RSSI value write
                                esp_err_t ret = esp_ble_gap_read_rssi(param->write.bda);
                                if(ret == ESP_OK)
                                {
                                    printf("RSSI Initialization Successful!\n");
                                }
                            }
                        }
                    }
                }
#if 0
                ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT, write value:");
                esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);

                Write Operation
                printf("--------------------------- \n");
                printf("WRITE VALUE LENGTH : %d \n", param->write.len);
                printf("WRITE VALUE CONTENT : %s \n", param->write.value);
#endif
                sprintf(BLE_write_value, "%s", param->write.value);
                param_write_handle = param->write.handle;

                if (BLE_COMM_DATABASE_TABLE[ota_characteristics_value] == param->write.handle)
                {
                    char rx_buffer[30];
                    sprintf(rx_buffer, "%s", BLE_write_value);
                    char *data_array[30] = {0};
                    int i = 0;
                    char delimiter[] = ",";
                    char *ptr = strtok((char *)rx_buffer, delimiter);
                    while (ptr != NULL)
                    {
                        data_array[i] = ptr;
                        ptr = strtok(NULL, delimiter);
                        printf(data_array[i]);
                        printf("\n");
                        i++;
                    }

                    if (strstr(data_array[0], "ALLOW_DWLD"))
                    {
                        OTA_ACTIVATE = true;
                        flag_sim_task = false;
                    }
                }

                // Notify SoC and Estimated Range
                if (BLE_COMM_DATABASE_TABLE[acknowledgement_characteristics_value] == param->write.handle)
                {
                    char rx_buffer[30];
                    sprintf(rx_buffer, "%s", BLE_write_value);
                    char *data_array[30] = {0};
                    int i = 0;
                    char delimiter[] = ",";
                    char *ptr = strtok((char *)rx_buffer, delimiter);
                    while (ptr != NULL)
                    {
                        data_array[i] = ptr;
                        ptr = strtok(NULL, delimiter);
                        printf(data_array[i]);
                        printf("\n");
                        i++;
                    }

                    if (strstr(data_array[0], "CONN_COMPLETE"))
                    {
                        BLE_CONNECTION_STATUS = FIRST_CONN_DATA_EXCHANGE_COMPLETE;

                        // Start timer to maintain stable BLE connection
                        ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 2000000)); // 5 sec timer set
                        timer_already_started = 1;
                        timer_status_for_rssi = 1;

                        printf("BLE_STATUS :: Timer has been started\n");
                    }
                    else if (strstr(data_array[0], "REQ") && (strstr(data_array[1], "TO_ESP_1")))
                    {
                        BLE_CONNECTION_STATUS = FIRST_CONN_INITIATED;
                        printf("BLE_STATUS :: FIRST CONNECTION INITIATED :: %s \n", BLE_write_value);
                        BLE_notify_data(bike_power_state, bike_ON_OFF_characteristics_value);
                    }
                }

                if (BLE_CONNECTION_STATUS == FIRST_CONN_INITIATED)
                {
                    // If characteristic UUID has been changed then, only go to the condition checking
                    if (temp_param_write_handle != param_write_handle)
                    {
                        // printf("flag has been set\n");
                        First_Connect_Data_Transfer(param_write_handle);
                        // printf("Inside First_Connect_Data_Transfer\n");
                    }
                }

                else if (BLE_CONNECTION_STATUS == FIRST_CONN_DATA_EXCHANGE_COMPLETE)
                {
                    // Get bikename
                    if (BLE_COMM_DATABASE_TABLE[bikename_characteristics_value] == param->write.handle)
                    {
                        printf("BLE_STATUS :: RECEIVED BIKENAME FROM APP :: %s \n", BLE_write_value);
                        vcu_bike_name_send(BLE_write_value, strlen(BLE_write_value));
                        BLE_STATE = BUSY;
                        RECEIVED_CHANGE_OF_SETTING = SETTING_BIKENAME;
                    }

                    // Get username
                    else if (BLE_COMM_DATABASE_TABLE[username_characteristics_value] == param->write.handle)
                    {

                        printf("BLE_STATUS :: RECEIVED USERNAME FROM APP :: %s \n", BLE_write_value);
                        vcu_user_name_send(BLE_write_value, strlen(BLE_write_value));
                        BLE_STATE = BUSY;
                        RECEIVED_CHANGE_OF_SETTING = SETTING_USERNAME;
                    }

                    // Get lights
                    else if (BLE_COMM_DATABASE_TABLE[lights_characteristics_value] == param->write.handle)
                    {
                        vcu_hunk_command();
                        printf("BLE_STATUS :: RECEIVED HUNK FROM APP :: %s \n", BLE_write_value);
                        BLE_STATE = BUSY;
                        RECEIVED_CHANGE_OF_SETTING = SETTING_LIGHTS;
                    }

                    // Get lights_and_horn
                    else if (BLE_COMM_DATABASE_TABLE[lights_and_horn_characteristics_value] == param->write.handle)
                    {
                        vcu_bunk_command();
                        printf("BLE_STATUS :: RECEIVED BUNK FROM APP :: %s \n", BLE_write_value);
                        BLE_STATE = BUSY;
                        RECEIVED_CHANGE_OF_SETTING = SETTING_LIGHTS_AND_HORN;
                    }

                    // Get trip_reset
                    else if (BLE_COMM_DATABASE_TABLE[trip_reset_characteristics_value] == param->write.handle)
                    {
                        vcu_trip_erase_command();
                        printf("BLE_STATUS :: RECEIVED TRIP_RESET FROM APP :: %s \n", BLE_write_value);
                        BLE_STATE = BUSY;
                        RECEIVED_CHANGE_OF_SETTING = SETTING_TRIP_RESET;
                    }

                    // Get bike_ON_OFF
                    else if (BLE_COMM_DATABASE_TABLE[bike_ON_OFF_characteristics_value] == param->write.handle)
                    {
                        char rx_buffer[30];
                        sprintf(rx_buffer, "%s", BLE_write_value);
                        char *data_array[30] = {0};
                        int i = 0;
                        char delimiter[] = ",";
                        char *ptr = strtok((char *)rx_buffer, delimiter);
                        while (ptr != NULL)
                        {
                            data_array[i] = ptr;
                            ptr = strtok(NULL, delimiter);
                            printf(data_array[i]);
                            printf("\n");
                            i++;
                        }

                        if (strstr(data_array[1], "OFF"))
                        {
                            printf("BLE_STATUS :: RECEIVED VEHICLE OFF CMD FROM APP :: %s \n", BLE_write_value);
                            vcu_vehicle_off_command();
                        }
                        else if (strstr(data_array[1], "ON"))
                        {
                            printf("BLE_STATUS :: RECEIVED VEHICLE ON CMD FROM APP :: %s \n", BLE_write_value);
                            vcu_vehicle_on_command();
                        }

                        BLE_STATE = BUSY;
                        RECEIVED_CHANGE_OF_SETTING = SETTING_BIKE_POWER;
                    }

                    // Get birthday
                    else if (BLE_COMM_DATABASE_TABLE[birthday_characteristics_value] == param->write.handle)
                    {
                        char rx_buffer[30];
                        sprintf(rx_buffer, "%s", BLE_write_value);
                        char *data_array[30] = {0};
                        int i = 0;
                        char delimiter[] = ",";
                        char *ptr = strtok((char *)rx_buffer, delimiter);
                        while (ptr != NULL)
                        {
                            data_array[i] = ptr;
                            ptr = strtok(NULL, delimiter);
                            printf(data_array[i]);
                            printf("\n");
                            i++;
                        }

                        printf("BLE_STATUS :: RECEIVED DATE_OF_BIRTH FROM APP :: %s \n", BLE_write_value);
                        vcu_user_dob_send(atoi(data_array[0]), atoi(data_array[1]), atoi(data_array[2]));
                        BLE_STATE = BUSY;
                        RECEIVED_CHANGE_OF_SETTING = SETTING_BIRTHDAY;
                    }

                    // Shake Mode
                    else if (BLE_COMM_DATABASE_TABLE[shake_mode_characteristics_value] == param->write.handle)
                    {
                        char rx_buffer[30];
                        sprintf(rx_buffer, "%s", BLE_write_value);
                        char *data_array[30] = {0};
                        int i = 0;
                        char delimiter[] = ",";
                        char *ptr = strtok((char *)rx_buffer, delimiter);
                        while (ptr != NULL)
                        {
                            data_array[i] = ptr;
                            ptr = strtok(NULL, delimiter);
                            printf(data_array[i]);
                            printf("\n");
                            i++;
                        }

                        if (strstr(data_array[0], "ACK"))
                        {
                            printf("BLE_STATUS :: RECEIVED SHAKE MODE STATUS FROM APP :: %s \n", BLE_write_value);
                            BLE_notify_data(ui_mode_setting, ui_mode_characteristics_value);
                        }
                        else if (strstr(data_array[0], "SHAKE"))
                        {
                            printf("BLE_STATUS :: RECEIVED SHAKE MODE STATUS FROM APP :: %s \n", BLE_write_value);

                            // Send data to CAN
                            if (strstr(data_array[1], "EN"))
                            {
                                vuc_shake_mode_send(1);
                            }
                            else if (strstr(data_array[1], "DIS"))
                            {
                                vuc_shake_mode_send(0);
                            }

                            // Set BLE state to busy
                            BLE_STATE = BUSY;
                            RECEIVED_CHANGE_OF_SETTING = SETTING_SHAKE_MODE;
                        }
                    }

                    // Notify AutoTimer Data
                    else if (BLE_COMM_DATABASE_TABLE[anti_theft_characteristics_value] == param->write.handle)
                    {
                        char rx_buffer[30];
                        sprintf(rx_buffer, "%s", BLE_write_value);
                        char *data_array[30] = {0};
                        int i = 0;
                        char delimiter[] = ",";
                        char *ptr = strtok((char *)rx_buffer, delimiter);
                        while (ptr != NULL)
                        {
                            data_array[i] = ptr;
                            ptr = strtok(NULL, delimiter);
                            printf(data_array[i]);
                            printf("\n");
                            i++;
                        }

                        if (strstr(data_array[0], "ACK"))
                        {
                            printf("BLE_STATUS :: RECEIVED ANTI_THEFT STATUS FROM APP :: %s \n", BLE_write_value);
                            BLE_notify_data(auto_timer_setting, auto_timer_characteristics_value);
                        }
                        else if (strstr(data_array[0], "THEFT"))
                        {
                            printf("BLE_STATUS :: RECEIVED ANTI_THEFT STATUS FROM APP :: %s \n", BLE_write_value);

                            // Send data to CAN
                            if (strstr(data_array[1], "EN"))
                            {
                                vuc_anti_theft_mode_send(1);
                            }
                            else if (strstr(data_array[1], "DIS"))
                            {
                                vuc_anti_theft_mode_send(0);
                            }

                            // Set BLE state to busy
                            BLE_STATE = BUSY;
                            RECEIVED_CHANGE_OF_SETTING = SETTING_ANTI_THEFT;
                        }
                    }

                    // Notify TimeFormat Data
                    else if (BLE_COMM_DATABASE_TABLE[auto_timer_characteristics_value] == param->write.handle)
                    {

                        char rx_buffer[30];
                        sprintf(rx_buffer, "%s", BLE_write_value);
                        char *data_array[30] = {0};
                        int i = 0;
                        char delimiter[] = ",";
                        char *ptr = strtok((char *)rx_buffer, delimiter);
                        while (ptr != NULL)
                        {
                            data_array[i] = ptr;
                            ptr = strtok(NULL, delimiter);
                            printf(data_array[i]);
                            printf("\n");
                            i++;
                        }

                        if (strstr(data_array[0], "ACK"))
                        {
                            printf("BLE_STATUS :: RECEIVED AUTO_TIMER STATUS FROM APP :: %s \n", BLE_write_value);
                            BLE_notify_data(time_format_setting, time_format_characteristics_value);
                        }
                        else if (strstr(data_array[0], "A_TIMER"))
                        {
                            printf("BLE_STATUS :: RECEIVED AUTO_TIMER STATUS FROM APP :: %s \n", BLE_write_value);

                            // Send data to CAN
                            if (atoi(data_array[1]) == 0)
                            {
                                vcu_stand_bypass(1);
                                // BLE_notify_data(RECEIVE_SUCCESSFUL, auto_timer_characteristics_value);
                                RECEIVED_CHANGE_OF_SETTING = SETTING_STAND_BYPASS;
                            }
                            else
                            {
                                vcu_stand_bypass(0);
                                bike_autoff_timer_send(atoi(data_array[1]));
                                RECEIVED_CHANGE_OF_SETTING = SETTING_AUTO_TIMER;
                            }

                            // Set BLE state to busy
                            BLE_STATE = BUSY;
                        }
                    }

                    // Notify ui_mode Data
                    else if (BLE_COMM_DATABASE_TABLE[time_format_characteristics_value] == param->write.handle)
                    {

                        char rx_buffer[30];
                        sprintf(rx_buffer, "%s", BLE_write_value);
                        char *data_array[30] = {0};
                        int i = 0;
                        char delimiter[] = ",";
                        char *ptr = strtok((char *)rx_buffer, delimiter);
                        while (ptr != NULL)
                        {
                            data_array[i] = ptr;
                            ptr = strtok(NULL, delimiter);
                            printf(data_array[i]);
                            printf("\n");
                            i++;
                        }

                        if (strstr(data_array[0], "ACK"))
                        {
                            printf("BLE_STATUS :: RECEIVED TIME_FORMAT STATUS FROM APP :: %s \n", BLE_write_value);
                            BLE_notify_data(ui_mode_setting, ui_mode_characteristics_value);
                        }
                        else if (strstr(data_array[0], "T_FORMAT"))
                        {
                            printf("BLE_STATUS :: RECEIVED TIME_FORMAT STATUS FROM APP :: %s \n", BLE_write_value);

                            // Send data to CAN
                            bike_time_format(atoi(data_array[1]));

                            // Set BLE state to busy
                            BLE_STATE = BUSY;
                            RECEIVED_CHANGE_OF_SETTING = SETTING_TIME_FORMAT;
                        }
                    }

                    // Notify brightness_level Data
                    else if (BLE_COMM_DATABASE_TABLE[ui_mode_characteristics_value] == param->write.handle)
                    {
                        char rx_buffer[30];
                        sprintf(rx_buffer, "%s", BLE_write_value);
                        char *data_array[30] = {0};
                        int i = 0;
                        char delimiter[] = ",";
                        char *ptr = strtok((char *)rx_buffer, delimiter);
                        while (ptr != NULL)
                        {
                            data_array[i] = ptr;
                            ptr = strtok(NULL, delimiter);
                            printf(data_array[i]);
                            printf("\n");
                            i++;
                        }

                        if (strstr(data_array[0], "ACK"))
                        {
                            printf("BLE_STATUS :: RECEIVED UI_MODE STATUS FROM APP :: %s \n", BLE_write_value);
                            BLE_notify_data(BRIGHTNESS_LEVEL, brightness_level_characteristics_value);
                        }
                        else if (strstr(data_array[0], "UI_MODE"))
                        {
                            printf("BLE_STATUS :: RECEIVED UI_MODE STATUS FROM APP :: %s \n", BLE_write_value);

                            // Send data to CAN
                            bike_dash_mode_send(atoi(data_array[1]));

                            // Set BLE state to busy
                            BLE_STATE = BUSY;
                            RECEIVED_CHANGE_OF_SETTING = SETTING_UI_MODE;
                        }
                    }

                    // Notify brightness_level Data
                    else if (BLE_COMM_DATABASE_TABLE[brightness_level_characteristics_value] == param->write.handle)
                    {
                        char rx_buffer[30];
                        sprintf(rx_buffer, "%s", BLE_write_value);
                        char *data_array[30] = {0};
                        int i = 0;
                        char delimiter[] = ",";
                        char *ptr = strtok((char *)rx_buffer, delimiter);
                        while (ptr != NULL)
                        {
                            data_array[i] = ptr;
                            ptr = strtok(NULL, delimiter);
                            printf(data_array[i]);
                            printf("\n");
                            i++;
                        }

                        if (strstr(data_array[0], "ACK"))
                        {
                            printf("BLE_STATUS :: RECEIVED BRIGHTNESS LEVEL FROM APP :: %s \n", BLE_write_value);

                            // Notify Kill Switch Status
                            BLE_notify_data(kill_switch_bypass_setting, kill_switch_bypass_characteristics_value);
                        }
                        else if (strstr(data_array[0], "BRIGHTNESS"))
                        {
                            printf("BLE_STATUS :: RECEIVED BRIGHTNESS LEVEL FROM APP :: %s \n", BLE_write_value);

                            // Send data to CAN
                            dash_brightness_level(atoi(data_array[1]));

                            // Set BLE state to busy
                            BLE_STATE = BUSY;
                            RECEIVED_CHANGE_OF_SETTING = SETTING_BRIGHTNESS;
                        }
                    }

                    // Notify Kill_Switch_Bypass Status Data
                    else if (BLE_COMM_DATABASE_TABLE[kill_switch_bypass_characteristics_value] == param->write.handle)
                    {
                        char rx_buffer[30];
                        sprintf(rx_buffer, "%s", BLE_write_value);
                        char *data_array[30] = {0};
                        int i = 0;
                        char delimiter[] = ",";
                        char *ptr = strtok((char *)rx_buffer, delimiter);
                        while (ptr != NULL)
                        {
                            data_array[i] = ptr;
                            ptr = strtok(NULL, delimiter);
                            printf(data_array[i]);
                            printf("\n");
                            i++;
                        }

                        if (strstr(data_array[0], "ACK"))
                        {
                            printf("BLE_STATUS :: RECEIVED KILL-SWITCH BYPASS STATUS FROM APP :: %s \n", BLE_write_value);

                            // Notify Kill Switch Status
                            BLE_notify_data(mode_button_bypass_setting, mode_button_bypass_characteristics_value);
                        }
                        else if (strstr(data_array[0], "KILL"))
                        {
                            printf("BLE_STATUS :: RECEIVED KILL-SWITCH BYPASS STATUS FROM APP :: %s \n", BLE_write_value);

                            // Send data to CAN
                            if (strstr(data_array[1], "EN"))
                            {
                                vcu_kill_bypass(1);
                            }
                            else if (strstr(data_array[1], "DIS"))
                            {
                                vcu_kill_bypass(0);
                            }

                            // Set BLE state to busy
                            BLE_STATE = BUSY;
                            RECEIVED_CHANGE_OF_SETTING = SETTING_KILL_SWITCH_BYPASS;
                        }
                    }

                    // Notify Mode_Button_Bypass Status Data
                    else if (BLE_COMM_DATABASE_TABLE[mode_button_bypass_characteristics_value] == param->write.handle)
                    {
                        char rx_buffer[30];
                        sprintf(rx_buffer, "%s", BLE_write_value);
                        char *data_array[30] = {0};
                        int i = 0;
                        char delimiter[] = ",";
                        char *ptr = strtok((char *)rx_buffer, delimiter);
                        while (ptr != NULL)
                        {
                            data_array[i] = ptr;
                            ptr = strtok(NULL, delimiter);
                            printf(data_array[i]);
                            printf("\n");
                            i++;
                        }

                        if (strstr(data_array[0], "DRIVE"))
                        {
                            printf("BLE_STATUS :: RECEIVED MODE-BUTTON BYPASS STATUS FROM APP :: %s \n", BLE_write_value);

                            // Send data to CAN
                            if (strstr(data_array[1], "EN"))
                            {
                                vcu_mode_button_bypass_command();
                            }
                            else if (strstr(data_array[1], "DIS"))
                            {
                                vcu_mode_button_unbypass_command();
                            }

                            // Set BLE state to busy
                            BLE_STATE = BUSY;
                            RECEIVED_CHANGE_OF_SETTING = SETTING_MODE_BUTTON_BYPASS;
                        }
                    }

                    // Receive Airlock Setting
                    else if (BLE_COMM_DATABASE_TABLE[airlock_characteristics_value] == param->write.handle)
                    {
                        char rx_buffer[30];
                        sprintf(rx_buffer, "%s", BLE_write_value);
                        char *data_array[30] = {0};
                        int i = 0;
                        char delimiter[] = ",";
                        char *ptr = strtok((char *)rx_buffer, delimiter);
                        while (ptr != NULL)
                        {
                            data_array[i] = ptr;
                            ptr = strtok(NULL, delimiter);
                            printf(data_array[i]);
                            printf("\n");
                            i++;
                        }

                        if (strstr(data_array[0], "AIRLOCK"))
                        {
                            printf("BLE_STATUS :: RECEIVED AIRLOCK SETTING FROM APP :: %s \n", BLE_write_value);

                            // Send data to CAN
                            if (strstr(data_array[1], "EN"))
                            {
                                // ENABLE AIRLOCK
                                airlock_setting_int = 1;
                                sprintf(airlock_setting_char, "AIRLOCK,EN");
                            }
                            else if (strstr(data_array[1], "DIS"))
                            {
                                // DISABLE AIRLOCK
                                airlock_setting_int = 2;
                                sprintf(airlock_setting_char, "AIRLOCK,DIS");
                            }

                            // Set BLE state to busy
                            BLE_STATE = BUSY;
                            // RECEIVED_CHANGE_OF_SETTING = SETTING_MODE_BUTTON_BYPASS;
                        
                        }
                    }

                }
            }
        }

        if (param->write.need_rsp)
        {
            printf("Reached Write Event...");
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        }
        break;

    case ESP_GATTS_EXEC_WRITE_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
        example_exec_write_event_env(&prepare_write_env, param);
        break;

    case ESP_GATTS_MTU_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT");
        break;

    case ESP_GATTS_CONF_EVT:

        break;

    case ESP_GATTS_UNREG_EVT:

        break;

    case ESP_GATTS_DELETE_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DELETE_EVT");
        break;

    case ESP_GATTS_START_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_START_EVT");
        esp_ble_gap_start_advertising(&ble_adv_params); // advertise once again
        break;

    case ESP_GATTS_STOP_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_STOP_EVT");
        break;

    case ESP_GATTS_CONNECT_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT");
        ESP_LOGI(GATTS_TABLE_TAG, "BLE_STATUS :: Device is just connected!");
        /* start security connect with peer device when receive the connect event sent by the master */
        esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
        esp_ble_gap_start_advertising(&ble_adv_params);
        // Send BLE Connection MSG to VCU
        break;

    case ESP_GATTS_DISCONNECT_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);

        /* start advertising again when missing the connect */
        esp_ble_gap_start_advertising(&ble_adv_params);
        BLE_CONNECTION_STATUS = FIRST_CONN_DATA_EXCHANGE_COMPLETE;

        // Send BLE Connection MSG to VCU
        Send_BLE_Connection_Status_To_VCU(BLE_DISCONNECTED);

        ble_conn_maintain = 0;
        timer_status_for_rssi = 0;

        // Code that stops the timer
        if (timer_already_started == 1)
        {
            ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
            printf("BLE_STATUS :: Periodic Timer Stopped!\n");
            timer_already_started = 0;
        }

        got_all_data_from_VCU_to_be_notified = 0;
        break;

    case ESP_GATTS_OPEN_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT from ESP_GATTS_OPEN_EVT\n");
        break;

    case ESP_GATTS_CANCEL_OPEN_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT from ESP_GATTS_CANCEL_OPEN_EVT\n");
        break;

    case ESP_GATTS_CLOSE_EVT:

        printf("BLE_STATUS :: DEVICE IS DISCONNECTED from ESP_GATTS_CLOSE_EVT\n");
        break;

    case ESP_GATTS_LISTEN_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT from ESP_GATTS_LISTEN_EVT\n");
        break;

    case ESP_GATTS_CONGEST_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT from ESP_GATTS_CONGEST_EVT\n");
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        ESP_LOGI(GATTS_TABLE_TAG, "The number handle = %x", param->add_attr_tab.num_handle);
        if (param->create.status == ESP_GATT_OK)
        {
            if (param->add_attr_tab.num_handle == HRS_IDX_NB)
            {
                memcpy(BLE_COMM_DATABASE_TABLE, param->add_attr_tab.handles, sizeof(BLE_COMM_DATABASE_TABLE));
                esp_ble_gatts_start_service(BLE_COMM_DATABASE_TABLE[vehicle_service]);
                printf("BLE_STATUS :: STARTED SERVICE HERE ::\n");
            }
            else
            {
                ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)",
                         param->add_attr_tab.num_handle, HRS_IDX_NB);
            }
        }
        else
        {
            ESP_LOGE(GATTS_TABLE_TAG, " Create attribute table failed, error code = %x", param->create.status);
        }
        break;
    }
    default:
        break;
    }
}

/**
 * @brief 
 * GATTS EVENT HANDLER
 * 
 * @param event 
 * @param gatts_if 
 * @param param 
 */
void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            ble_profile_tab[BLE_PROFILE_APP_IDX].gatts_if = gatts_if;
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
        for (idx = 0; idx < NUMBER_OF_PROFILES; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == ble_profile_tab[idx].gatts_if)
            {
                if (ble_profile_tab[idx].gatts_cb)
                {
                    ble_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void Load_Bike_Power_Data(void)
{
    sprintf(bike_power_state, "BIKE,%s", user_bike_power_status.bike_power_status == 1 ? "ON" : "OFF");
    printf("CAN_STATUS :: GOT BIKE_POWER_STATUS :: %s\n", bike_power_state);

    if (strstr(bike_power_state, "BIKE,ON")){
        _bike_power_state = 1;
    } else if (strstr(bike_power_state, "BIKE,OFF")) {
        _bike_power_state = 2;
    }
    
    sprintf(kill_switch_bypass_setting, "KILL,%s", user_bike_power_status.kill_bypass_status == 1 ? "EN" : "DIS");
    printf("CAN_STATUS :: GOT BIKE_KILL_BYPASS_STATUS :: %s\n", kill_switch_bypass_setting);

    sprintf(mode_button_bypass_setting, "DRIVE,%s", user_bike_power_status.mode_bypass_status == 1 ? "EN" : "DIS");
    printf("CAN_STATUS :: GOT BIKE_MODE_BYPASS_STATUS :: %s\n", mode_button_bypass_setting);

    sprintf(stand_bypass_setting, "STAND,%s", user_bike_power_status.stand_switch_status == 1 ? "EN" : "DIS");
    printf("CAN_STATUS :: GOT BIKE_STAND_BYPASS_STATUS :: %s\n", stand_bypass_setting);
}

void Load_Bike_Settings_Data(void)
{
    sprintf(shake_mode_setting, "SHAKE,%s", user_bike_setting_struct.bike_shake_mode == 1 ? "EN" : "DIS");
    printf("CAN_STATUS :: GOT SHAKE MODE SETTING :: %s\n", shake_mode_setting);

    sprintf(anti_theft_setting, "THEFT,%s", user_bike_setting_struct.bike_anti_theft == 1 ? "EN" : "DIS");
    printf("CAN_STATUS :: GOT ANTI-THEFT MODE SETTING :: %s\n", anti_theft_setting);
}

void Load_Bike_Dash_Settings_Data(void)
{
    sprintf(auto_timer_setting, "A_TIMER,%d", bike_dash_setting_struct.bike_autoff_time);
    printf("\033[1;31mCAN_STATUS :: GOT AUTO-TIMER SETTING :: %s\033[1;31m\n", auto_timer_setting);

    sprintf(ui_mode_setting, "UI_MODE,%d", bike_dash_setting_struct.bike_dash_mode);
    printf("\033[1;31mCAN_STATUS :: GOT UI-MODE SETTING :: %s\033[1;31m\n", ui_mode_setting);

    sprintf(time_format_setting, "T_FORMAT,%d", bike_dash_setting_struct.bike_dash_format);
    printf("\033[1;31mCAN_STATUS :: GOT TIMER-FORMAT SETTING :: %s\033[1;31m\n", time_format_setting);

    sprintf(brightness_level_setting, "BRIGHTNESS,%d", bike_dash_setting_struct.brightness_level);
    printf("\033[1;31mCAN_STATUS :: GOT BRIGHTNESS LEVEL :: %s\033[1;31m\n", brightness_level_setting);
}

void Load_Bike_Dash_Indicator_Data(void)
{
    sprintf(BLE_status, "%d", dash_indicator_struct.ble_indicator);
    sprintf(GPS_status, "%d", dash_indicator_struct.gps_indicator);
    sprintf(SIM_status, "%d", dash_indicator_struct.net_indicator);
    printf("CAN_STATUS :: GOT UI-MODE SETTING :: %s\n", BLE_status);
    printf("CAN_STATUS :: GOT UI-MODE SETTING :: %s\n", GPS_status);
    printf("CAN_STATUS :: GOT UI-MODE SETTING :: %s\n", SIM_status);
}

void Load_Overall_Data(void)
{
    sprintf(_overall_data[0], "C1,%d,%d,%d,%d",
            overall_odo_struct.overall_odo_day1,
            overall_odo_struct.overall_odo_day2,
            overall_odo_struct.overall_odo_day3,
            overall_odo_struct.overall_odo_day4);
    printf("CAN_STATUS :: GOT OVERALL DATA :: %s\n", _overall_data[0]);

    sprintf(_overall_data[1], "C2,%d,%d,%d,%d",
            overall_odo_struct.overall_odo_day5,
            overall_odo_struct.overall_odo_day6,
            overall_odo_struct.overall_odo_day7,
            overall_odo_struct.overall_avg_speed);
    printf("CAN_STATUS :: GOT OVERALL DATA :: %s\n", _overall_data[1]);

    sprintf(_overall_data[2], "C3,%d,%d,%.2f",
            overall_odo_struct.overall_max_speed,
            overall_odo_struct.overall_avg_whpkm,
            overall_odo_struct.overall_odo);
    printf("CAN_STATUS :: GOT OVERALL DATA :: %s\n", _overall_data[2]);
}

void Load_Suste_Data(void)
{
    sprintf(_suste_data[0], "C1,%d,%d,%d,%d",
            suste_odo_struct.suste_odo_day1,
            suste_odo_struct.suste_odo_day2,
            suste_odo_struct.suste_odo_day3,
            suste_odo_struct.suste_odo_day4);
    printf("CAN_STATUS :: GOT SUSTE DATA :: %s\n", _suste_data[0]);

    sprintf(_suste_data[1], "C2,%d,%d,%d,%d",
            suste_odo_struct.suste_odo_day5,
            suste_odo_struct.suste_odo_day6,
            suste_odo_struct.suste_odo_day7,
            suste_odo_struct.suste_avg_speed);
    printf("CAN_STATUS :: GOT SUSTE DATA :: %s\n", _suste_data[1]);

    sprintf(_suste_data[2], "C3,%d,%d,%.2f",
            suste_odo_struct.suste_max_speed,
            suste_odo_struct.suste_avg_whpkm,
            suste_odo_struct.suste_odo);
    printf("CAN_STATUS :: GOT SUSTE DATA :: %s\n", _suste_data[2]);
}

void Load_Thikka_Data(void)
{
    sprintf(_thikka_data[0], "C1,%d,%d,%d,%d",
            thikka_odo_struct.thikka_odo_day1,
            thikka_odo_struct.thikka_odo_day2,
            thikka_odo_struct.thikka_odo_day3,
            thikka_odo_struct.thikka_odo_day4);
    printf("CAN_STATUS :: GOT THIKKA DATA :: %s\n", _thikka_data[0]);

    sprintf(_thikka_data[1], "C2,%d,%d,%d,%d",
            thikka_odo_struct.thikka_odo_day5,
            thikka_odo_struct.thikka_odo_day6,
            thikka_odo_struct.thikka_odo_day7,
            thikka_odo_struct.thikka_avg_speed);
    printf("CAN_STATUS :: GOT THIKKA DATA :: %s\n", _thikka_data[1]);

    sprintf(_thikka_data[2], "C3,%d,%d,%.2f",
            thikka_odo_struct.thikka_max_speed,
            thikka_odo_struct.thikka_avg_whpkm,
            thikka_odo_struct.thikka_odo);
    printf("CAN_STATUS :: GOT THIKKA DATA :: %s\n", _thikka_data[2]);
}

void Load_Babbal_Data(void)
{
    sprintf(_babal_data[0], "C1,%d,%d,%d,%d",
            babbal_odo_struct.babal_odo_day1,
            babbal_odo_struct.babal_odo_day2,
            babbal_odo_struct.babal_odo_day3,
            babbal_odo_struct.babal_odo_day4);
    printf("CAN_STATUS :: GOT BABBAL DATA :: %s\n", _babal_data[0]);

    sprintf(_babal_data[1], "C2,%d,%d,%d,%d",
            babbal_odo_struct.babal_odo_day5,
            babbal_odo_struct.babal_odo_day6,
            babbal_odo_struct.babal_odo_day7,
            babbal_odo_struct.babal_avg_speed);
    printf("CAN_STATUS :: GOT BABBAL DATA :: %s\n", _babal_data[1]);

    sprintf(_babal_data[2], "C3,%d,%d,%.2f",
            babbal_odo_struct.babal_max_speed,
            babbal_odo_struct.babal_avg_whpkm,
            babbal_odo_struct.babal_odo);
    printf("CAN_STATUS :: GOT BABBAL DATA :: %s\n", _babal_data[2]);
}

void Load_SoC_and_Range_Data(void)
{
    sprintf(SoC_and_range, "%d,%d", soc_and_range.vehicle_soc, soc_and_range.vehicle_range);
    printf("CAN_STATUS :: GOT SOC_AND_RANGE :: %s\n", SoC_and_range);
}

void Load_ETA_and_ChargingStatus_Data(void)
{
    sprintf(ETA_and_chargingStatus, "%d,%d", eta_and_charging_status.battery_eta, eta_and_charging_status.charging_status);
    printf("CAN_STATUS :: GOT ETA_AND_CHARGING_STATUS :: %s\n", ETA_and_chargingStatus);
}

void Load_Trip_Telemetry_Data(void)
{
    sprintf(tripTelemetry, "%d,%d,%.2f", trip_telemetry.trip_maxSpeed, trip_telemetry.trip_avgSpeed, trip_telemetry.trip_Distance);
    printf("CAN_STATUS :: GOT TRIP_TELEMERTY :: %s\n", tripTelemetry);
}

void First_Connect_Data_Transfer(uint16_t param_write_handle)
{
    printf("BLE_STATUS :: First_Connect_Data_Transfer\n");

    // Notify SoC_estRange
    if (BLE_COMM_DATABASE_TABLE[bike_ON_OFF_characteristics_value] == param_write_handle)
    {
        BLE_notify_data(SoC_and_range, SoC_estRange_characteristics_value);
        temp_param_write_handle = param_write_handle;
    }

    // Notify ETA_and_chargingStatus
    else if (BLE_COMM_DATABASE_TABLE[SoC_estRange_characteristics_value] == param_write_handle)
    {
        BLE_notify_data(ETA_and_chargingStatus, eta_ChargingStatus_characteristics_value);
        temp_param_write_handle = param_write_handle;
    }

    // Notify trip_telemetry
    else if (BLE_COMM_DATABASE_TABLE[eta_ChargingStatus_characteristics_value] == param_write_handle)
    {
        BLE_notify_data(tripTelemetry, trip_telemetry_characteristics_value);
        temp_param_write_handle = param_write_handle;
    }

    // Notify Overall data
    else if (BLE_COMM_DATABASE_TABLE[trip_telemetry_characteristics_value] == param_write_handle)
    {
        BLE_notify_data(_overall_data[0], overall_data_characteristics_value);
        temp_param_write_handle = param_write_handle;
    }

    // Notify Overall Data remaining chunks
    else if (BLE_COMM_DATABASE_TABLE[overall_data_characteristics_value] == param_write_handle)
    {
        char rx_buffer[30];
        sprintf(rx_buffer, "%s", BLE_write_value);
        char *data_array[30] = {0};
        int i = 0;
        char delimiter[] = ",";
        char *ptr = strtok((char *)rx_buffer, delimiter);
        while (ptr != NULL)
        {
            data_array[i] = ptr;
            ptr = strtok(NULL, delimiter);
            i++;
        }

        if (strstr(data_array[1], "CHUNK"))
        {
            if (number_of_chunks_overall_data > 1)
            {
                for (int i = 1; i <= number_of_chunks_overall_data; i++)
                {
                    if (atoi(data_array[2]) == i && number_of_chunks_overall_data != atoi(data_array[2]))
                    {
                        BLE_notify_data(_overall_data[i], overall_data_characteristics_value);
                    }
                    else if (atoi(data_array[2]) == i && number_of_chunks_overall_data == atoi(data_array[2]))
                    {
                        BLE_notify_data(complete_data, overall_data_characteristics_value);
                    }
                }
            }
        }
        else if (strstr(data_array[1], "SUCCESS"))
        {
            BLE_notify_data(_suste_data[0], suste_data_characteristics_value);
            temp_param_write_handle = param_write_handle;
        }
        else
        {
            printf("BLE_STATUS :: CHUNK PROBLEM\n");
        }
    }

    // Notify Suste Data
    else if (BLE_COMM_DATABASE_TABLE[suste_data_characteristics_value] == param_write_handle)
    {

        char rx_buffer[30];
        sprintf(rx_buffer, "%s", BLE_write_value);
        char *data_array[30] = {0};
        int i = 0;
        char delimiter[] = ",";
        char *ptr = strtok((char *)rx_buffer, delimiter);
        while (ptr != NULL)
        {
            data_array[i] = ptr;
            ptr = strtok(NULL, delimiter);
            i++;
        }

        if (strstr(data_array[1], "CHUNK"))
        {
            if (number_of_chunks_suste_data > 1)
            {
                for (int i = 1; i <= number_of_chunks_suste_data; i++)
                {
                    if (atoi(data_array[2]) == i && number_of_chunks_suste_data != atoi(data_array[2]))
                    {
                        BLE_notify_data(_suste_data[i], suste_data_characteristics_value);
                    }
                    else if (atoi(data_array[2]) == i && number_of_chunks_suste_data == atoi(data_array[2]))
                    {
                        BLE_notify_data(complete_data, suste_data_characteristics_value);
                    }
                }
            }
        }
        else if (strstr(data_array[1], "SUCCESS"))
        {
            BLE_notify_data(_thikka_data[0], thikka_data_characteristics_value);
            temp_param_write_handle = param_write_handle;
        }
        else
        {
            printf("BLE_STATUS :: CHUNK PROBLEM\n");
        }
    }

    // Notify Thikka Data
    else if (BLE_COMM_DATABASE_TABLE[thikka_data_characteristics_value] == param_write_handle)
    {

        char rx_buffer[30];
        sprintf(rx_buffer, "%s", BLE_write_value);
        char *data_array[30] = {0};
        int i = 0;
        char delimiter[] = ",";
        char *ptr = strtok((char *)rx_buffer, delimiter);
        while (ptr != NULL)
        {
            data_array[i] = ptr;
            ptr = strtok(NULL, delimiter);
            i++;
        }

        if (strstr(data_array[1], "CHUNK"))
        {
            if (number_of_chunks_thikka_data > 1)
            {
                for (int i = 1; i <= number_of_chunks_thikka_data; i++)
                {
                    if (atoi(data_array[2]) == i && number_of_chunks_thikka_data != atoi(data_array[2]))
                    {
                        BLE_notify_data(_thikka_data[i], thikka_data_characteristics_value);
                    }
                    else if (atoi(data_array[2]) == i && number_of_chunks_thikka_data == atoi(data_array[2]))
                    {
                        BLE_notify_data(complete_data, thikka_data_characteristics_value);
                    }
                }
            }
        }
        else if (strstr(data_array[1], "SUCCESS"))
        {
            BLE_notify_data(_babal_data[0], babbal_data_characteristics_value);
            temp_param_write_handle = param_write_handle;
        }
        else
        {
            printf("BLE_STATUS :: CHUNK PROBLEM\n");
        }
    }

    // Notify Babbal Data
    else if (BLE_COMM_DATABASE_TABLE[babbal_data_characteristics_value] == param_write_handle)
    {
        char rx_buffer[30];
        sprintf(rx_buffer, "%s", BLE_write_value);
        char *data_array[30] = {0};
        int i = 0;
        char delimiter[] = ",";
        char *ptr = strtok((char *)rx_buffer, delimiter);
        while (ptr != NULL)
        {
            data_array[i] = ptr;
            ptr = strtok(NULL, delimiter);
            i++;
        }

        if (strstr(data_array[1], "CHUNK"))
        {
            if (number_of_chunks_babbal_data > 1)
            {
                for (int i = 1; i <= number_of_chunks_babbal_data; i++)
                {
                    if (atoi(data_array[2]) == i && number_of_chunks_babbal_data != atoi(data_array[2]))
                    {
                        BLE_notify_data(_babal_data[i], babbal_data_characteristics_value);
                    }
                    else if (atoi(data_array[2]) == i && number_of_chunks_babbal_data == atoi(data_array[2]))
                    {
                        BLE_notify_data(complete_data, babbal_data_characteristics_value);
                    }
                }
            }
        }
        else if (strstr(data_array[1], "SUCCESS"))
        {
            BLE_notify_data(shake_mode_setting, shake_mode_characteristics_value);
            temp_param_write_handle = param_write_handle;
        }
        else
        {
            printf("BLE_STATUS :: CHUNK PROBLEM\n");
        }
    }

    // Notify Anti-theft Setting
    else if (BLE_COMM_DATABASE_TABLE[shake_mode_characteristics_value] == param_write_handle)
    {
        BLE_notify_data(anti_theft_setting, anti_theft_characteristics_value);
        temp_param_write_handle = param_write_handle;
    }

    // Notify Auto-timer Setting
    else if (BLE_COMM_DATABASE_TABLE[anti_theft_characteristics_value] == param_write_handle)
    {
        BLE_notify_data(auto_timer_setting, auto_timer_characteristics_value);
        temp_param_write_handle = param_write_handle;
    }

    // Notify Time-format Setting
    else if (BLE_COMM_DATABASE_TABLE[auto_timer_characteristics_value] == param_write_handle)
    {
        BLE_notify_data(time_format_setting, time_format_characteristics_value);
        temp_param_write_handle = param_write_handle;
    }

    // Notify UI_Mode Setting
    else if (BLE_COMM_DATABASE_TABLE[time_format_characteristics_value] == param_write_handle)
    {
        BLE_notify_data(ui_mode_setting, ui_mode_characteristics_value);
        temp_param_write_handle = param_write_handle;
    }

    // Notify Brightness Level Setting
    else if (BLE_COMM_DATABASE_TABLE[ui_mode_characteristics_value] == param_write_handle)
    {
        BLE_notify_data(BRIGHTNESS_LEVEL, brightness_level_characteristics_value);
        temp_param_write_handle = param_write_handle;
    }

    // Notify Kill Switch Bypass Setting
    else if (BLE_COMM_DATABASE_TABLE[brightness_level_characteristics_value] == param_write_handle)
    {
        BLE_notify_data(kill_switch_bypass_setting, kill_switch_bypass_characteristics_value);
        temp_param_write_handle = param_write_handle;
    }

    // Notify Mode Button Bypass Setting
    else if (BLE_COMM_DATABASE_TABLE[kill_switch_bypass_characteristics_value] == param_write_handle)
    {
        BLE_notify_data(mode_button_bypass_setting, mode_button_bypass_characteristics_value);
        temp_param_write_handle = param_write_handle;
    }
    
    // Notify Airlock Setting
    else if (BLE_COMM_DATABASE_TABLE[mode_button_bypass_characteristics_value] == param_write_handle)
    {
        BLE_notify_data(airlock_setting_char, airlock_characteristics_value);
        temp_param_write_handle = param_write_handle;
    }

    // First CONN Data Transmit to Bike Complete
    else if (BLE_COMM_DATABASE_TABLE[airlock_characteristics_value] == param_write_handle)
    {
        BLE_notify_data(ASK_TELEMETRY_TO_APP, acknowledgement_characteristics_value);
        temp_param_write_handle = param_write_handle;
    }

    /* First CONN Data Receive from App starts here, also data transmit to VCU starts here */
    // Get username from app
    if (BLE_COMM_DATABASE_TABLE[username_characteristics_value] == param_write_handle)
    {
        printf("BLE_STATUS :: RECEIVED USERNAME FROM APP :: %s \n", BLE_write_value);
        vcu_user_name_send(BLE_write_value, strlen(BLE_write_value));
        BLE_STATE = BUSY;
        BLE_notify_data(RECEIVE_SUCCESSFUL, username_characteristics_value);
        temp_param_write_handle = param_write_handle;
    }

    // Get birthday from app
    else if (BLE_COMM_DATABASE_TABLE[birthday_characteristics_value] == param_write_handle)
    {
        char rx_buffer[30];
        sprintf(rx_buffer, "%s", BLE_write_value);
        char *data_array[30] = {0};
        int i = 0;
        char delimiter[] = ",";
        char *ptr = strtok((char *)rx_buffer, delimiter);
        while (ptr != NULL)
        {
            data_array[i] = ptr;
            ptr = strtok(NULL, delimiter);
            printf(data_array[i]);
            printf("\n");
            i++;
        }
        printf("BLE_STATUS :: RECEIVED DATE_OF_BIRTH FROM APP :: %s \n", BLE_write_value);
        vcu_user_dob_send(atoi(data_array[0]), atoi(data_array[1]), atoi(data_array[2]));
        BLE_notify_data(RECEIVE_SUCCESSFUL, birthday_characteristics_value);
        temp_param_write_handle = param_write_handle;
        BLE_STATE = BUSY;
    }

    // Get bikename from app
    else if (BLE_COMM_DATABASE_TABLE[bikename_characteristics_value] == param_write_handle)
    {
        printf("BLE_STATUS :: RECEIVED BIKENAME FROM APP :: %s \n", BLE_write_value);
        vcu_bike_name_send(BLE_write_value, strlen(BLE_write_value));
        BLE_STATE = BUSY;
        BLE_notify_data(RECEIVE_SUCCESSFUL, bikename_characteristics_value);
        temp_param_write_handle = param_write_handle;
    }
}

/**
 * @brief    : READ CAN DATA SENT FROM VCU
 * @param id : CAN_ID
 */
void ble_can_read(uint32_t id)
{
    // First Connect all data received
    if (id == END_OF_DATA)
    {
        ble_task_flag = 1;
        esp_rec_getData(&end_of_data);

        // Got all data at first connect, NOTIFY that connection is successful
        ble_get_data = CONN_SUCCESS;

        got_all_data_from_VCU_to_be_notified = 1;
        BLE_notify_data(CONN_SUCCESS_STATUS, acknowledgement_characteristics_value);
    }

    // Bike Power Status
    else if (id == BIKE_POWER_STATUS)
    {
        ble_task_flag = 1;
        esp_rec_getData(&user_bike_power_status);

        Load_Bike_Power_Data();
        ble_get_data = GOT_BIKE_POWER_STATUS;
    }

    // Bike Settings
    else if (id == BIKE_SETTINGS)
    {
        ble_task_flag = 1;
        esp_rec_getData(&user_bike_setting_struct);

        Load_Bike_Settings_Data();
        ble_get_data = GOT_BIKE_SETTINGS;
    }

    // Dash Settings
    else if (id == BIKE_DASH_SETTINGS)
    {
        ble_task_flag = 1;
        esp_rec_getData(&bike_dash_setting_struct);

        Load_Bike_Dash_Settings_Data();
        ble_get_data = GOT_BIKE_DASH_SETTINGS;
    }

    // Overall data
    else if (id == OVERALL_DATA)
    {
        ble_task_flag = 1;
        esp_rec_getData(&overall_odo_struct);

        Load_Overall_Data();
        ble_get_data = GOT_OVERALL_DATA;
    }

    // Suste data
    else if (id == SUSTE_DATA)
    {
        ble_task_flag = 1;
        esp_rec_getData(&suste_odo_struct);

        Load_Suste_Data();
        ble_get_data = GOT_SUSTE_DATA;
    }

    // Thikka data
    else if (id == THIKKA_DATA)
    {
        ble_task_flag = 1;
        esp_rec_getData(&thikka_odo_struct);

        Load_Thikka_Data();
        ble_get_data = GOT_THIKKA_DATA;
    }

    // Babbal data
    else if (id == BABBAL_DATA)
    {
        ble_task_flag = 1;
        esp_rec_getData(&babbal_odo_struct);

        Load_Babbal_Data();
        ble_get_data = GOT_BABBAL_DATA;
    }

    // SoC and estimated range data
    else if (id == SOC_AND_RANGE)
    {
        ble_task_flag = 1;
        esp_rec_getData(&soc_and_range);

        Load_SoC_and_Range_Data();
        ble_get_data = GOT_SOC_AND_RANGE;
    }

    // ETA and Charging Status data
    else if (id == ETA_AND_CHARGING_STATUS)
    {
        ble_task_flag = 1;
        esp_rec_getData(&eta_and_charging_status);

        Load_ETA_and_ChargingStatus_Data();
        ble_get_data = GOT_ETA_AND_CHARGING_STATUS;
    }

    // Trip Telemetry data
    else if (id == TRIP_TELEMETRY)
    {
        ble_task_flag = 1;
        esp_rec_getData(&trip_telemetry);

        Load_Trip_Telemetry_Data();
        ble_get_data = GOT_TRIP_TELEMETRY;
    }

    /* ------------ BLE CAN RECEIVE ENDS HERE ------------- */
    if (got_all_data_from_VCU_to_be_notified == 1)
    {
        BLE_Notify_Modified_Data();
    }
}

void BLE_Notify_Modified_Data(void)
{
    if (ble_get_data == GOT_OVERALL_DATA)
    {
        for (int i = 1; i <= number_of_chunks_overall_data; i++)
        {
            BLE_notify_data(_overall_data[i], overall_data_characteristics_value);
        }
        ble_get_data = 0;
    }

    else if (ble_get_data == GOT_SUSTE_DATA)
    {
        for (int i = 1; i <= number_of_chunks_overall_data; i++)
        {
            BLE_notify_data(_suste_data[i], suste_data_characteristics_value);
        }
        ble_get_data = 0;
    }

    else if (ble_get_data == GOT_THIKKA_DATA)
    {
        for (int i = 1; i <= number_of_chunks_overall_data; i++)
        {
            BLE_notify_data(_thikka_data[i], thikka_data_characteristics_value);
        }
        ble_get_data = 0;
    }

    else if (ble_get_data == GOT_BABBAL_DATA)
    {
        for (int i = 1; i <= number_of_chunks_overall_data; i++)
        {
            BLE_notify_data(_babal_data[i], babbal_data_characteristics_value);
        }
        ble_get_data = 0;
    }

    else if (ble_get_data == GOT_SOC_AND_RANGE)
    {
        BLE_notify_data(SoC_and_range, SoC_estRange_characteristics_value);
        ble_get_data = 0;
    }

    else if (ble_get_data == GOT_ETA_AND_CHARGING_STATUS)
    {
        BLE_notify_data(ETA_and_chargingStatus, eta_ChargingStatus_characteristics_value);
        ble_get_data = 0;
    }

    else if (ble_get_data == GOT_TRIP_TELEMETRY)
    {
        BLE_notify_data(tripTelemetry, trip_telemetry_characteristics_value);
        ble_get_data = 0;
    }

    else if (ble_get_data == GOT_BIKE_POWER_STATUS)
    {
        ble_get_data = 0;
        if (BLE_CONNECTION_STATUS == FIRST_CONN_DATA_EXCHANGE_COMPLETE)
        {
            switch (RECEIVED_CHANGE_OF_SETTING)
            {
            case (SETTING_BIKE_POWER):
                BLE_notify_data(RECEIVE_SUCCESSFUL, bike_ON_OFF_characteristics_value);
                break;

            case (SETTING_KILL_SWITCH_BYPASS):
                BLE_notify_data(RECEIVE_SUCCESSFUL, kill_switch_bypass_characteristics_value);
                break;

            case (SETTING_MODE_BUTTON_BYPASS):
                BLE_notify_data(RECEIVE_SUCCESSFUL, mode_button_bypass_characteristics_value);
                break;

            case (SETTING_STAND_BYPASS):
                BLE_notify_data(RECEIVE_SUCCESSFUL, auto_timer_characteristics_value);
                break;

            default:
                break;
            }
        }
    }

    else if (ble_get_data == GOT_BIKE_SETTINGS)
    {
        ble_get_data = 0;
        if (BLE_CONNECTION_STATUS == FIRST_CONN_DATA_EXCHANGE_COMPLETE)
        {
            switch (RECEIVED_CHANGE_OF_SETTING)
            {
            case (SETTING_SHAKE_MODE):
                BLE_notify_data(RECEIVE_SUCCESSFUL, shake_mode_characteristics_value);
                break;

            case (SETTING_ANTI_THEFT):
                BLE_notify_data(RECEIVE_SUCCESSFUL, anti_theft_characteristics_value);
                break;

            default:
                break;
            }
        }
    }

    else if (ble_get_data == GOT_BIKE_DASH_SETTINGS)
    {
        ble_get_data = 0;
        switch (RECEIVED_CHANGE_OF_SETTING)
        {
        case (SETTING_AUTO_TIMER):
            BLE_notify_data(RECEIVE_SUCCESSFUL, auto_timer_characteristics_value);
            break;

        case (SETTING_TIME_FORMAT):
            BLE_notify_data(RECEIVE_SUCCESSFUL, time_format_characteristics_value);
            break;

        case (SETTING_UI_MODE):
            BLE_notify_data(RECEIVE_SUCCESSFUL, ui_mode_characteristics_value);
            break;

        case (SETTING_BRIGHTNESS):
            BLE_notify_data(RECEIVE_SUCCESSFUL, brightness_level_characteristics_value);
            break;

        default:
            break;
        }
    }
}

// timer callback for id verification
static void periodic_timer_callback(void *arg)
{
    timer_count++;
    timerStoppedFlag = 1;
    printf("Timer count in multiple of 5 Seconds: %d\n", timer_count);

#if 1
    if (ble_conn_maintain == 0)
    {
        BLE_notify_data(CONN_PACKET, maintain_ble_connection_characteristics_value);
        ble_conn_maintain = 1;
    }
    else if (ble_conn_maintain == 1)
    {
        // Disconnect App
        printf("\n------------------------ BLE connection not being maintained ------------------------\n");
        esp_ble_gatts_close(3, 0);
    }
#endif
}