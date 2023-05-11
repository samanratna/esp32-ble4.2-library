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
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x18, 0x0E, 0x00, 0x00,
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

    [maintain_ble_connection_characteristics_index] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
    [maintain_ble_connection_characteristics_value] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&MAINTAIN_BLE_CONNECTION_CHARACTERISTIC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, HRPS_HT_MEAS_MAX_LEN, 0, NULL}},
    [maintain_ble_connection_characteristics_descriptor] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_measurement_cc), (uint8_t *)ble_measurement_cc}},
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
        printf("Your Passkey is : %d", _passkey);
        break;

    case ESP_GAP_BLE_KEY_EVT:

        // shows the ble key info share with peer device to the user.
        ESP_LOGI(GATTS_TABLE_TAG, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));

        // Pairing Successful EVENT that confirms bonding
        pairing_status = 1;
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
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "auth mode = %s", esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));

            // Set Pairing Status to SUCCESSFUL
            pairing_status = 1;

            // Notify pairing success to YatriHub
            vTaskDelay(1500 / portTICK_PERIOD_MS);

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
        break;

    case ESP_GATTS_DISCONNECT_EVT:

        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);

        /* start advertising again when missing the connect */
        esp_ble_gap_start_advertising(&ble_adv_params);
        BLE_CONNECTION_STATUS = FIRST_CONN_DATA_EXCHANGE_COMPLETE;

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


// timer callback for id verification
static void periodic_timer_callback(void *arg)
{
    timer_count++;
    timerStoppedFlag = 1;
    printf("Timer count in multiple of 2 Seconds: %d\n", timer_count);

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