#include "main.h"

//=======================IMPORTANT START-UP FUNCTIONS INITIALIZER=========================//
void func_init()
{
    gpio_reset_pin(SIM_POWER_PIN);
    gpio_set_direction(SIM_POWER_PIN, GPIO_MODE_OUTPUT);
    uart_init();
    twai_init();
    sim7600_init();

    sim7600_reset();

    // sim7600_powerdown();
    // vTaskDelay(10000 / portTICK_PERIOD_MS);
    // sim7600_init();

    // batt_check();
    // ssl_init();
    gps_init();
    sms_init();

    // send CAN message to VCU for reset ack!
    twai_tx_data[0] = 0; // 1 for about to reset and 0 for module restarted
    twaiTxData(esp_gps_post_start_can_id, twai_tx_data, 1);

    ble_conn_timer_init();
    // pNumber_check();
    // balance_check();
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    uart_flush(2);
}

//=======================APP MAIN=========================//
void app_main(void)
{
    uint8_t data_buf[8];
    esp_vcu_init();
    esp_rec_init(twaiTxData);
    number_of_chunks_overall_data = 3;
    number_of_chunks_suste_data = 3;
    number_of_chunks_thikka_data = 3;
    number_of_chunks_babbal_data = 3;

#if 0
    /* STATIC DUMMY DATA FOR TEST */

    // Bike Setting variable
    char shake_mode_setting[15] = "SHAKE,EN";
    printf("-----------------------SETTING SHAKE MODE VARIABLE\n");
    
    char anti_theft_setting[15] = "THEFT,DIS";
    char kill_switch_bypass_setting[15] = "KILL,EN";
    char mode_button_bypass_setting[15] = "DRIVE,DIS";
    char stand_bypass_setting[15] = "STAND,DIS";

    // Bike Dash Setting
    char auto_timer_setting[15] = "A_TIMER,30";
    char time_format_setting[15] = "T_FORMAT,12";
    char ui_mode_setting[15] = "UI_MODE,2";
    char bike_power_state[15] = "BIKE,ON";

    char _overall_data[5][30] = {"C1,1,2,3,4", "C2,5,6,7,8", "C3,12,13,14"};
    char _suste_data[5][30] = {"C1,1,2,3,4", "C2,5,6,7,8", "C3,12,13,14"};
    char _thikka_data[5][30] = {"C1,11,22,33,44", "C2,5,6,7,8", "C3,12,13,14"};
    char _babal_data[5][30] = {"C1,0,2,3,4", "C2,5,6,7,8", "C3,12,13,14"};

    char SoC_and_range[10] = "10,20";
    char ETA_and_chargingStatus[20] = "30,40";
    char tripTelemetry[15] = "50,60,70";
/* ************************** */
#endif

    // Initialize twai and SIM
    func_init();

    // Initialize TWDT
    ESP_ERROR_CHECK(esp_task_wdt_init(WDT_TIMEOUT, false));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    xTaskCreate(twai_task, "twai_task", 4096, NULL, 10, NULL); // Twai task starting early for receiving bike credentials

    /* For recovering from ESP_OTA failure */
    data_buf[0] = 1;
    twaiTxData(ESP_OTA_STATUS, data_buf, 1); // Send ESP Update Failed
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    get_bike_credentials();
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // #if 1
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth", __func__);
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        esp_restart();
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(ESP_BLE_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND; // bonding with peer device after authentication
    // esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;                   // set the IO capability to No output No input
    esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT; // set the IO capability to No output No input
    uint8_t key_size = 16;                   // the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    // set static passkey
    uint32_t passkey = rand();
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;

    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    /* Check for updates */
    limit = 0;
    first_check = 1;
    if (check_version(1) == 1) // global
    {
        data_buf[0] = 1;
        twaiTxData(OTA_POP_REQ, data_buf, 1);
        send_release_notes();
        limit = 1;
    }
    else
    {
        if (check_version(0) == 1) // local
        {
            data_buf[0] = 1;
            twaiTxData(OTA_POP_REQ, data_buf, 1);
            limit = 0;
        }
    }
    first_check = 0;
    flag_sim_task = true; // SIM task activate flag

    // #endif
    //=======Data Post Structures for CAN (TWAI)==========//
    xTaskCreate(sim_task, "sim_task", 4096, NULL, 2, NULL);
    xTaskCreate(ota_task, "ota_task", 4096, NULL, 1, NULL);
    xTaskCreate(obd_task, "obd_task", 4096, NULL, 1, NULL);
}
