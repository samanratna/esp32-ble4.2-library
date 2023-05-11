#include "OBD.h"

#include "errno.h"
#include "esp_event.h"
#include "esp_ota_ops.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"

#define Wait 10000000
#define Max_OBD_Data 1024

static const char *TAG = "ESP_OBD";
static uint8_t data_buf[8] = {0};

uint64_t start_time = 0;
char buf_OBD[Max_OBD_Data + 200] = {0};

uint32_t filesize_ESP_OBD;
bool FLAG_OBD_END;
bool Flag_File_OBD;
bool Flag_OBD_Data;
int count_OBD;
int value_OBD;

bool OBD_ACTIVATE;

int ESP_FLASH_OBD(void)
{
    uint8_t sha_256[HASH_LEN] = {0};
    esp_partition_t partition;

    partition.address = ESP_PARTITION_TABLE_OFFSET;
    partition.size = ESP_PARTITION_TABLE_MAX_LEN;
    partition.type = ESP_PARTITION_TYPE_DATA;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for the partition table: ");

    partition.address = ESP_BOOTLOADER_OFFSET;
    partition.size = ESP_PARTITION_TABLE_OFFSET;
    partition.type = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for bootloader: ");

    esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
    print_sha256(sha_256, "SHA-256 for current firmware: ");

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t obd_state;
    if (esp_ota_get_state_partition(running, &obd_state) == ESP_OK)
    {
        if (obd_state == ESP_OTA_IMG_PENDING_VERIFY)
        {
            // run diagnostic function ...
            bool diagnostic_is_ok = diagnostic();
            if (diagnostic_is_ok)
            {
                ESP_LOGI(TAG, "Diagnostics completed successfully! Continuing execution ...");
                esp_ota_mark_app_valid_cancel_rollback();
            }
            else
            {
                ESP_LOGE(TAG, "Diagnostics failed! Start rollback to the previous version ...");
                esp_ota_mark_app_invalid_rollback_and_reboot();
            }
        }
    }

    esp_err_t err;
    /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = NULL;

    ESP_LOGI(TAG, "Starting ESP OBD");

    const esp_partition_t *configured = esp_ota_get_boot_partition();
    // const esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running)
    {
        ESP_LOGW(TAG, "Configured OBD boot partition at offset 0x%08x, but running from offset 0x%08x",
                 configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             running->type, running->subtype, running->address);

    update_partition = esp_ota_get_next_update_partition(NULL);
    assert(update_partition != NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %x at offset 0x%x",
             update_partition->subtype, update_partition->address);

    uint32_t i = 0;
    bool image_header_was_checked = false;
    FLAG_OBD_END = false;

    data_buf[0] = 0;
    twaiTxData(ESP_OBD_SEND, data_buf, 1); // First ACK to VCU

    previous_time = esp_timer_get_time();
    while (Flag_File_OBD == false)
    {
        current_time = esp_timer_get_time();
        if ((current_time - previous_time) > Wait)
        {
            printf("OBD Update Failed!!!\nVCU did not response\n");
            return 0;
        }
    }
    Flag_File_OBD = false;

    data_buf[0] = 1;
    twaiTxData(ESP_OBD_SEND, data_buf, 1); // File Size ACK to VCU
    count_OBD = 0;

    while (!FLAG_OBD_END)
    {
        ESP_ERROR_CHECK(esp_task_wdt_reset());

        // memset(out, '\0', sizeof(out));
        memset(buf_OBD, '\0', sizeof(buf_OBD));

        if ((filesize_ESP_OBD - i) > Max_OBD_Data)
        {
            value_OBD = Max_OBD_Data;
        }
        else
        {
            value_OBD = filesize_ESP_OBD - i;
            FLAG_OBD_END = true;
        }

        previous_time = esp_timer_get_time();

        while (Flag_OBD_Data == false)
        {
            current_time = esp_timer_get_time();
            if ((current_time - previous_time) > Wait)
            {
                printf("OBD Update Failed!!!\nData not received\n");
                return 0;
            }
        }

        Flag_OBD_Data = false;
        if (count_OBD < 0)
        {
            ESP_LOGE(TAG, "Error: SSL data read error");
            return 0;
        }
        else if (count_OBD > 0)
        {
            if (image_header_was_checked == false)
            {
                esp_app_desc_t new_app_info;
                int len1 = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t);
                printf("lenght of the strange headers:: %d\n\n", len1);
                if (value_OBD > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
                {
                    // check current version with downloading
                    memcpy(&new_app_info, &buf_OBD[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                    ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);

                    esp_app_desc_t running_app_info;
                    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
                    {
                        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
                    }

                    const esp_partition_t *last_invalid_app = esp_ota_get_last_invalid_partition();
                    esp_app_desc_t invalid_app_info;
                    if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK)
                    {
                        ESP_LOGI(TAG, "Last invalid firmware version: %s", invalid_app_info.version);
                    }

                    // check current version with last invalid partition
                    if (last_invalid_app != NULL)
                    {
                        if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0)
                        {
                            ESP_LOGW(TAG, "New version is the same as invalid version.");
                            ESP_LOGW(TAG, "Previously, there was an attempt to launch the firmware with %s version, but it failed.", invalid_app_info.version);
                            ESP_LOGW(TAG, "The firmware has been rolled back to the previous version.");
                            // http_cleanup(client);
                            // infinite_loop();
                        }
                    }
#ifndef CONFIG_EXAMPLE_SKIP_VERSION_CHECK
                    if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) == 0)
                    {
                        ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
                        // issue = 19;
                        return 0;
                        // http_cleanup(client);
                        // infinite_loop();
                    }
#endif

                    image_header_was_checked = true;

                    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
                    if (err != ESP_OK)
                    {
                        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                        esp_ota_abort(update_handle);
                    }
                    ESP_LOGI(TAG, "esp_ota_begin succeeded");
                }
                else
                {
                    ESP_LOGE(TAG, "received package is not fit len");
                    esp_ota_abort(update_handle);
                }
            }
            err = esp_ota_write(update_handle, (const void *)buf_OBD, count_OBD);
            if (err != ESP_OK)
            {
                printf("Unsuccessful Write\n");
                return 0;
            }

            else if (err == ESP_ERR_OTA_VALIDATE_FAILED)
            {
                printf("Magic byte error\n");
                return 0;
            }

            // binary_file_length += value_OBD;
            i += value_OBD;
            count_OBD = 0;

            printf("Data = %d\n", (i * 100) / filesize_ESP_OBD);

            data_buf[0] = 2;
            twaiTxData(ESP_OBD_SEND, data_buf, 1); // Data ACK to VCU
        }
    }
    err = esp_ota_end(update_handle);
    if (err != ESP_OK)
    {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED)
        {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        }
        else
        {
            ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
        }
        return 0;
    }

    return 1;
}

// OBD Task
void obd_task(void *arg)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_ERROR_CHECK(esp_task_wdt_status(NULL));
    while (1)
    {
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        vTaskDelay(1 / portTICK_PERIOD_MS);

        if (OBD_ACTIVATE)
        {
            flag_sim_task = false;
            int return_value = ESP_FLASH_OBD();
            if (return_value == 1)
            {
                const esp_partition_t *update_partition = NULL;
                esp_err_t err;
                update_partition = esp_ota_get_next_update_partition(NULL);
                err = esp_ota_set_boot_partition(update_partition);
                if (err != ESP_OK)
                {
                    printf("Set partition failed\n");
                    FLAG_OBD_END = false;
                    data_buf[0] = 3;
                    twaiTxData(ESP_OBD_SEND, data_buf, 1);
                    break;
                }
                printf("Updated\n");
                // http_post_update_remark(20, 20); // post version change notice in backend
                // esp_restart();
            }
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            OBD_ACTIVATE = false;
            flag_sim_task = true;
        }

        vTaskDelay(1);
    }
}

void obd_can(uint32_t struct_id, uint32_t can_id, uint8_t data[], uint8_t data_len) // CAN receive funtion for OBD Task
{
    if (can_id == ESP_OBD_RECEIVE)
    {
        if (data[0] == 0)
        {
            printf("OBD ACK received\n");
            OBD_ACTIVATE = true;
        }
        else if (data[0] == 1)
        {
            Flag_File_OBD = true;
            filesize_ESP_OBD = (data[4] << 24) + (data[3] << 16) + (data[2] << 8) + data[1];
            printf("File Size Recevied = %d\n", filesize_ESP_OBD);
        }
    }
    else if (can_id == OBD_DATA)
    {
        for (int i = 0; i < data_len; i++)
        {
            buf_OBD[count_OBD + i] = data[i];
            // printf("%u", data[i]);
        }
        count_OBD += data_len;
        printf("\ncount = %d\n", count_OBD);

        if (count_OBD >= value_OBD)
        {
            Flag_OBD_Data = true;
            printf("\n\n");
        }
    }
    else if (can_id == OBD_POST_REQ) // post to update success case in the back end if there's not obd case for esp32
    {
        OBD_ACTIVATE = true;
        flag_sim_task = true;
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        http_post_update_remark(20, 20);
        esp_restart();
        // OBD_ACTIVATE = false;
    }
}