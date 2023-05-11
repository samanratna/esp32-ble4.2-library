#include "OTA.h"

#include "errno.h"
#include "esp_event.h"
#include "esp_ota_ops.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"

#define CONFIG_EXAMPLE_GPIO_DIAGNOSTIC GPIO_NUM_4
#define BUFFSIZE 3000

/* URL for OTA Updates */
char File_URL[] = "https://s3.ap-south-1.amazonaws.com/yatri.static/embedded/Bike_Updates/";
char VCU_Board[] = "First_OTA_on_VCU_test.txt";
char Display_Board[] = "First_OTA_on_Display_Test.txt";
char ESP_Board[] = "First_OTA_on_COMM_Test.txt";
char URL_issue[] = "http://api.embedded.yatrimotorcycles.com/api/v1/bikes/bike-software/error";
char URL_success[] = "http://api.embedded.yatrimotorcycles.com/api/v1/bikes/bike-software";
char version_global[] = "http://api.embedded.yatrimotorcycles.com/api/v1/bikes/bike-software/latest";
char version_local[] = "http://api.embedded.yatrimotorcycles.com/api/v1/bikes/bike-software";
char global_updates[] = "Global_Updates/";
char local_updates[] = "Local_Updates/";
char CRC_File[] = "Global_Updates/CRC_File.txt";

const int uartport0 = UART_NUM_0;
const int uartport2 = UART_NUM_2;
uint8_t rxbuffer[3000] = {0};

bool OTA_ACTIVATE;
bool Bike_Status;
bool Flag_Bike_Status;
bool Flag_OTA;
bool FLAG_FIRST_ACK;
bool FLAG_TOTAL_FILESIZE;
bool FLAG_CRC_ACK;
bool FLAG_END;
bool Flag_Data_Start;

bool VCU_FLAG;
bool DISPLAY_FLAG;
bool COMM_FLAG;
bool FLAG_Diagnostic;
bool Flag_File_End;
bool Flag_ACK_VCU;
bool Flag_Start_Display;
bool FLAG_CRC_V;
bool FLAG_CRC_D;
bool Flag_Send_version;
bool Flag_Send_version_length;
bool Flag_Send_version_data;
bool FLAG_DISPLAY_ROLL;
bool FLAG_VCU_ROLL;

int first_check;
int limit;
int new_verlen;
int OTA_Verification_Status;

uint32_t VCU_CRC;
uint32_t Display_CRC;
uint32_t COMM_CRC;
uint32_t get_display_crc_value;
uint64_t sum;
uint64_t fileSize_ESP;
uint64_t File_Size_ESP;
uint64_t fileSize;
uint64_t File_Size;

uint64_t send_esp_size;
uint64_t VCU_FILE_SIZE;

bike_status_ota bikestatus;
releasenotes update_detais;

static uint8_t data_buf[8] = {0};
char decode_data[1026] = {0};
char CRC_Data[100] = {0};
static char ota_write_data[BUFFSIZE + 1] = {0};
static char out[BUFFSIZE + 1] = {0};

static const char *TAG = "ESP_OTA";

uint64_t previous_time;
uint64_t current_time;

int issue;
int issue_0;

char URL_ESP_Board[200];
char URL_Board[220];
char URL_CRC[220];

size_t out_len;
int b64invs[] = {62, -1, -1, -1, 63, 52, 53, 54, 55, 56, 57, 58,
                 59, 60, 61, -1, -1, -1, -1, -1, -1, -1, 0, 1, 2, 3, 4, 5,
                 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
                 21, 22, 23, 24, 25, -1, -1, -1, -1, -1, -1, 26, 27, 28,
                 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42,
                 43, 44, 45, 46, 47, 48, 49, 50, 51};

void ota_can(uint32_t struct_id, uint32_t can_id, uint8_t data[], uint8_t data_len) // CAN receive funtion for OTA Task
{
    if (struct_id > 0)
    {
        if (struct_id == state_check_ota)
        {
            printf("Can Received for Current Bike Status\n");
            esp_rec_getData(&bikestatus); // Receive bike status in structure

            char bike_state[50];

            printf("Charging Status = %d\n", bikestatus.charging_status);
            printf("Bike Key = %d\n", bikestatus.bike_status);
            printf("Current SOC = %d\n", bikestatus.current_soc);
            printf("Current Speeed = %d\n", bikestatus.current_speed);

            memset(bike_state, 0, sizeof(bike_state));

            if (bikestatus.current_soc < 30)
            {
                strcpy(bike_state, "OTA_ACK,4");
                printf("OTA_ACK,4\n");

                // data_buf[0] = 8;
                data_buf[0] = 2;
                twaiTxData(OTA_UI_DISPLAY, data_buf, 2); // send display insufficient SOC UI
            }
            else
            {

                if (bikestatus.current_speed == 0)
                {
                    if ((bikestatus.bike_status == 0) && (bikestatus.charging_status == 1))
                    {
                        strcpy(bike_state, "OTA_ACK,0");
                        printf("OTA_ACK,0\n");
                    }
                    else if (bikestatus.bike_status == 0)
                    {
                        strcpy(bike_state, "OTA_ACK,1");
                        printf("OTA_ACK,1\n");
                    }
                    else if (bikestatus.charging_status == 1)
                    {
                        strcpy(bike_state, "OTA_ACK,2");
                        printf("OTA_ACK,2\n");
                    }
                    else if ((bikestatus.bike_status == 1) && (bikestatus.charging_status == 0))
                    {
                        strcpy(bike_state, "OTA_ACK,3");
                        printf("OTA_ACK,3\n");
                        Flag_Bike_Status = true;
                    }
                }
                else
                {
                    strcpy(bike_state, "OTA_ACK,5");
                    printf("OTA_ACK,5\n");
                }
            }
            BLE_notify_data(bike_state, ota_characteristics_value); // Bike status report to APP
            Bike_Status = true;
        }
    }
    else if (can_id == Get_Display_CRC)
    {
        get_display_crc_value = (data[0] << 24) + (data[1] << 16) + (data[2] << 8) + data[3];
        printf("\n\n\nDisplay CRC HEX = %x\n\n", get_display_crc_value);
        printf("\n\n\nDisplay CRC INT= %d\n\n", get_display_crc_value);
    }
    else if (can_id == VCU_TO_ESP_OTA)
    {
        if (data[0] == 0)
        {
            FLAG_FIRST_ACK = true;
            printf("OTA ACK received from Bootloader0\n");
        }
        else if (data[0] == 1)
        {
            printf("File size ACK received \n");
            Flag_Data_Start = true;
        }
        else if (data[0] == 2)
        {
            Flag_ACK_VCU = true;
        }
        else if (data[0] == 3)
        {
            printf("Update Download for VCU Successfull\n");
            // Flag_Download_VCU = false;
            Flag_Start_Display = true;
        }
        else if (data[0] == 13)
        {
            FLAG_CRC_ACK = true;
        }
        else if (data[0] == 7)
        {
            Flag_Send_version = true;
            printf("Updates Flashed, Sending new version \n");
        }
        else if (data[0] == 8)
        {
            Flag_Send_version_length = true;
            printf("Version Length ACK received \n");
        }
        else if (data[0] == 9)
        {
            FLAG_TOTAL_FILESIZE = true;
        }
        else if (data[0] == 11)
        {
            Flag_Send_version_data = true;
            printf("Version Data ACK received \n");
        }
        else if (data[0] == 10)
        {
            printf("CRC check failed for VCU \n");
            FLAG_CRC_V = true;
        }
        else if (data[0] == 12)
        {
            printf("CRC check failed for Display \n");
            FLAG_CRC_D = true;
        }
        else if (data[0] == 15)
        {
            printf("DISPLAY + VCU Rolled Back \n");
            FLAG_DISPLAY_ROLL = true;
            issue_0 = 25;
        }
        else if (data[0] == 14)
        {
            printf("VCU Rolled Back \n");
            FLAG_VCU_ROLL = true;
            issue_0 = 25;
        }
    }
    else if (can_id == VCU_TO_ESP_ISSUE)
    {
        if (data[0] == 0)
        {
            printf("OTA request denied! \nSD card mount failed! \n");
            issue_0 = 0;
        }
        else if (data[0] == 1)
        {
            printf("OTA request denied! \nEmpty File Size received \n");
            issue_0 = 1;
        }
        else if (data[0] == 2)
        {
            printf("OTA request denied! \nError in opening file\n");
            issue_0 = 2;
        }
        else if (data[0] == 3)
        {
            printf("OTA request denied! \nError in Writing file\n");
            issue_0 = 3;
        }
        else if (data[0] == 4)
        {
            printf("OTA request denied! \nVCU Erase Failed\n");
            issue_0 = 4;
        }
        else if (data[0] == 5)
        {
            printf("OTA request denied! \nError in reading VCU bin file\n");
            issue_0 = 5;
        }
        else if (data[0] == 6)
        {
            printf("OTA request denied! \nError in flashing VCU\n");
            issue_0 = 6;
        }
        else if (data[0] == 7)
        {
            printf("OTA request denied! \nNo response from Display\n");
            issue_0 = 7;
        }
        else if (data[0] == 8)
        {
            printf("OTA request denied! \nFlash failed in Display\n");
            issue_0 = 8;
        }
    }
}

// OTA Task
void ota_task(void *arg)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_ERROR_CHECK(esp_task_wdt_status(NULL));
    while (1)
    {
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        vTaskDelay(1 / portTICK_PERIOD_MS);

        if (OTA_ACTIVATE)
        {
            // data_buf[0] = 8;
            data_buf[0] = 1;
            twaiTxData(OTA_UI_DISPLAY, data_buf, 1); // send display to checking SOC UI
            vTaskDelay(2000 / portTICK_PERIOD_MS);

            // BLE EVENT OFF

            int return_value = Yatri_OTA();
            if (return_value == 1)
            {
                printf("Updated\n");
                esp_restart();
            }
            else if (return_value == 2) // Bike status not verfiied or VCU did not response
            {
                // data_buf[0] = 8;
                // data_buf[0] = 12;
                // twaiTxData(OTA_UI_DISPLAY, data_buf, 2); // send display to Update failed UI
                printf("Update Fail \n");
            }
            else if (return_value == 4) // No network or same version found
            {
                printf("Update Fail \n");
            }
            else
            {
                // data_buf[0] = 1;
                // twaiTxData(ESP_OTA_STATUS, data_buf, 8); // Send ESP Update Failed

                // data_buf[0] = 8;
                data_buf[0] = 12;
                twaiTxData(OTA_UI_DISPLAY, data_buf, 2); // send display to Update failed UI
                printf("Update Fail \n");

                ESP_ERROR_CHECK(esp_task_wdt_reset());
                http_post_update_remark(issue, issue_0);
                //     Flag_OTA = false;     // For differentiating OTA mode and just Version check to request ID to display
                //     OTA_ESP_HOLD = false; // For holding ESP32 other tasks
                //     FLAG_HOLD = false;
                //     FLAG_SIM = false;
                vTaskDelay(2000 / portTICK_PERIOD_MS);
            }
            OTA_ACTIVATE = false;
            flag_sim_task = true;

            // BLE EVENT ON
        }
        vTaskDelay(1);
    }
}

int b64_decode(const char *in /*, unsigned char *out*/, size_t outlen)
{
    size_t len;
    size_t i;
    size_t j;
    int v;

    if (in == NULL || out == NULL)
    // if (in == NULL)
    {
        printf("NULL\n");
        if (out == NULL)
        {
            printf("Out is NULL \n");
        }
        return 0;
    }

    len = strlen(in);
    if (outlen < b64_decoded_size(in) || len % 4 != 0)
    {
        printf("Invalid input data size\n");
        return 0;
    }

    for (i = 0; i < len; i++)
    {
        if (!b64_isvalidchar(in[i]))
        {
            printf("Invalid char\n");
            return 0;
        }
    }

    b64_generate_decode_table();

    for (i = 0, j = 0; i < len; i += 4, j += 3)
    {
        v = b64invs[in[i] - 43];
        v = (v << 6) | b64invs[in[i + 1] - 43];
        v = in[i + 2] == '=' ? v << 6 : (v << 6) | b64invs[in[i + 2] - 43];
        v = in[i + 3] == '=' ? v << 6 : (v << 6) | b64invs[in[i + 3] - 43];

        out[j] = (v >> 16) & 0xFF;
        // printf(out[j]);
        // printf("Converted Value %d: %c \n", j, out[j]);
        if (in[i + 2] != '=')
            out[j + 1] = (v >> 8) & 0xFF;
        // printf(out[j+1]);
        // printf("Converted Value %d: %c \n", j+1, out[j+1]);
        if (in[i + 3] != '=')
            out[j + 2] = v & 0xFF;
        // printf(out[j+2]);
        // printf("Converted Value %d: %c \n", j+2, out[j+2]);
    }

    return 1;
}

int b64_isvalidchar(char c)
{
    if (c >= '0' && c <= '9')
        // printf("c = %c\r\n", c);
        return 1;
    if (c >= 'A' && c <= 'Z')
        // printf("c = %c\r\n", c);
        return 1;
    if (c >= 'a' && c <= 'z')
        // printf("c = %c\r\n", c);
        return 1;
    if (c == '+' || c == '/' || c == '=')
        // printf("c = %c\r\n", c);
        return 1;
    return 0;
}

size_t b64_decoded_size(const char *in)
{
    size_t len;
    size_t ret;
    size_t i;

    if (in == NULL)
        return 0;

    len = strlen(in);
    ret = len / 4 * 3;

    for (i = len; i-- > 0;)
    {
        if (in[i] == '=')
        {
            ret--;
        }
        else
        {
            break;
        }
    }

    return ret;
}

void b64_generate_decode_table()
{
    int inv[80];
    size_t i;

    memset(inv, -1, sizeof(inv));
    for (i = 0; i < sizeof(b64invs) - 1; i++)
    {
        inv[b64invs[i] - 43] = i;
    }
}

void print_sha256(const uint8_t *image_hash, const char *label)
{
    char hash_print[HASH_LEN * 2 + 1];
    hash_print[HASH_LEN * 2] = 0;
    for (int i = 0; i < HASH_LEN; ++i)
    {
        sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    }
    ESP_LOGI(TAG, "%s: %s", label, hash_print);
}

bool diagnostic(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Diagnostics (5 sec)...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    bool diagnostic_is_ok = gpio_get_level(CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);

    gpio_reset_pin(CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);
    return diagnostic_is_ok;
}

int send_version()
{
    previous_time = esp_timer_get_time();
    while (Flag_Send_version_length == false)
    {
        current_time = esp_timer_get_time();
        if ((current_time - previous_time) > WAIT_PERIOD)
        {
            printf("OTA Update Failed!!!!!!!!!!\n");
            issue = 18;
            return 0;
            // TODO: ping issue to backend and restart
            // esp_restart();
        }
    }

    Flag_Send_version_length = false;
    printf("Sending Data\n");
    printf("verlen = %d\n", new_verlen);
    printf("new_version = %s\n", new_version);

    for (int l = 0; l < new_verlen; l++)
    {
        if ((new_verlen - l) >= 8)
        {
            for (int i = 0; i < 8; i++)
            {
                data_buf[i] = new_version[i + l];
            }
            twaiTxData(Send_Version, data_buf, 8);
            memset(data_buf, 0, 8);
            l += 8;
        }
        else
        {
            for (int i = 0; i < (new_verlen - l); i++)
            {
                data_buf[i] = new_version[i + l];
            }
            twaiTxData(Send_Version, data_buf, new_verlen - l);
            l += (new_verlen - l);
            memset(data_buf, 0, new_verlen - l);
        }
        previous_time = esp_timer_get_time();
        while (Flag_Send_version_data == false)
        {
            current_time = esp_timer_get_time();
            if ((current_time - previous_time) > WAIT_PERIOD)
            {
                printf("OTA Update Failed!!!!!!!!!!!!!!!!!!!!\n");
                issue = 18;
                return 0;
                // TODO: ping issue to backend and restart
                // esp_restart();
            }
        }
        Flag_Send_version_data = false;
    }
    printf("Version Send Complete\n");
    return 1;
}

int check_version(int limits)
{
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    int connection_lost = 0;

retry:
    printf("Inside Check Version\n");
    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPINIT\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    // if (Flag_OTA == true)
    // {
    //     printf("Sending Network UI request to display\n");
    //     data_buf[0] = 2;
    //     twaiTxData(OTA_UI_DISPLAY, data_buf, 2); // send display to checking Version UI
    // }

    if (limits == 1)
    {
        // printf("Global\n");
        sprintf(ATcommand, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", version_global); // Get bike software version
    }
    else
    {
        // printf("Local\n");
        sprintf(ATcommand, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", version_local); // Get latest vehicle software version
    }
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"USERDATA\",\"%s\"\r\n", b_apiKey);
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    vTaskDelay(100 / portTICK_PERIOD_MS);
    sprintf(ATcommand, "AT+HTTPACTION=0\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 10000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 500 / portTICK_PERIOD_MS);
    memset(rxbuffer, 0, sizeof(rxbuffer));
    uart_read(rxbuffer, BUFFSIZE, 120000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    if (strstr((char *)rxbuffer, "+CCH"))
    {
        memset(rxbuffer, 0, sizeof(rxbuffer));
        uart_read(rxbuffer, BUFFSIZE, 120000 / portTICK_PERIOD_MS);
        uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    }
    // memset(ATcommand, 0, sizeof(ATcommand));

    // if (strstr((char *)rxbuffer, "200"))
    // {
    //     printf("Successfully Posted!\n");
    // }
    // else
    // {
    //     printf("Posting Failed!\n");
    //     sprintf(ATcommand, "AT+HTTPTERM\r\n");
    //     uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    //     uart_read(rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
    //     uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    //     uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    //     memset(rxbuffer, 0, sizeof(rxbuffer));
    //     memset(ATcommand, 0, sizeof(ATcommand));
    //     return 0;
    // }
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    sprintf(ATcommand, "AT+HTTPREAD=0,1024\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));

    if (strstr((char *)rxbuffer, "true"))
    {
        FLAG_Diagnostic = true;
        printf("\n\nDiagnostic Mode\n\n");
    }

    if (strstr((char *)rxbuffer, "success"))
    {
        char *data3[30] = {0};
        int i3 = 0;
        char delim3[] = "\"";
        char *ptr3 = strtok((char *)rxbuffer, delim3);
        while (ptr3 != NULL)
        {
            data3[i3] = ptr3;
            ptr3 = strtok(NULL, delim3);
            i3++;
        }

        sprintf(new_version, "%s", data3[13]); /*new_version = data3[13];*/
        if (limits)
        {
            sprintf(update_detais.release_notes, "%s", data3[21]);
        }
        VCU_FLAG = false;
        DISPLAY_FLAG = false;
        COMM_FLAG = false;

        if (strstr(data3[17], "VCU"))
        {
            VCU_FLAG = true;
            printf("VCU\n");
        }
        if (strstr(data3[17], "DISPLAY"))
        {
            DISPLAY_FLAG = true;
            printf("Display\n");
        }
        if (strstr(data3[17], "COMM"))
        {
            COMM_FLAG = true;
            printf("COMM\n");
        }

        char *data4[10] = {0};
        int i4 = 0;
        char delim4[5];

        if ((VCU_FLAG == true) && (DISPLAY_FLAG == true))
        {
            sprintf(delim4, ", =");
        }
        else if ((VCU_FLAG == true) && (COMM_FLAG == true))
        {
            sprintf(delim4, ", =");
        }
        else if ((DISPLAY_FLAG == true) && (COMM_FLAG == true))
        {
            sprintf(delim4, ", =");
        }
        else
        {
            sprintf(delim4, "=");
        }

        char *ptr4 = strtok(data3[17], delim4);
        while (ptr4 != NULL)
        {
            data4[i4] = ptr4;
            ptr4 = strtok(NULL, delim4);
            i4++;
        }

        sum = 0;
        int board_CRC = 0;
        uint32_t CRC[3];
        for (int u = 0; u < i4; u++)
        {
            printf("%d = %s\n", u, data4[u]);
            if (strstr(data4[u], "VCU"))
            {
                board_CRC = 0;
            }
            else if (strstr(data4[u], "DISPLAY"))
            {
                board_CRC = 1;
            }
            else if (strstr(data4[u], "COMM"))
            {
                board_CRC = 2;
            }
            if ((u % 2) != 0)
            {
                sum += atoi(data4[u]);
                char *data5[5] = {0};
                int i5 = 0;
                char delim5[2];
                sprintf(delim5, "|");
                char *ptr5 = strtok(data4[u], delim5);
                while (ptr5 != NULL)
                {
                    data5[i5] = ptr5;
                    ptr5 = strtok(NULL, delim5);
                    i5++;
                }
                CRC[board_CRC] = atoll(data5[1]);
            }
        }
        VCU_CRC = CRC[0];
        Display_CRC = CRC[1];
        COMM_CRC = CRC[2];

        printf("VCU CRC = %x\n", VCU_CRC);
        printf("Display CRC = %x\n", Display_CRC);
        printf("COMM CRC = %x\n", COMM_CRC);

        printf("sum(base64)= %lld\n", sum);
        printf("sum(hex)= %lld\n", (sum / 4) * 3);
    }

    else
    {
        OTA_Verification_Status = 2;
        memset(rxbuffer, 0, sizeof(rxbuffer));
        memset(ATcommand, 0, sizeof(ATcommand));
        sprintf(ATcommand, "AT+HTTPTERM\r\n");
        uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
        // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
        uart_read(rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
        uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
        uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
        memset(rxbuffer, 0, sizeof(rxbuffer));
        memset(ATcommand, 0, sizeof(ATcommand));
        vTaskDelay(50 / portTICK_PERIOD_MS);
        printf("Connection Error!\n");
        connection_lost++;

        if (connection_lost < 3)
        {
            printf("From Check Version Terminate\n");
            goto retry;
        }
        else
        {
            printf("Couldn't check the update available for pop-up due to connection error!\n");
            // FLAG_SIM = false;
            // printf("Check version Completed\n");
            issue = 10;
            return 0;
        }
    }

    if (Flag_OTA == true)
    {
        printf("Send Checking version UI request to display\n");
        // data_buf[0] = 8;
        data_buf[0] = 5;
        twaiTxData(OTA_UI_DISPLAY, data_buf, 2); // send display to checking Version UI
    }

    printf("current version = %s\n", current_version);
    printf("new version = %s\n", new_version);
    if (limits)
    {
        printf("Release Notes = %s\n", update_detais.release_notes);
    }
    if (strcmp(new_version, current_version) || (FLAG_Diagnostic == true))
    {

        printf("New update available\n");
        new_verlen = strlen(new_version);
        memset(rxbuffer, 0, sizeof(rxbuffer));
        memset(ATcommand, 0, sizeof(ATcommand));
        sprintf(ATcommand, "AT+HTTPTERM\r\n");
        uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
        // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
        uart_read(rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
        uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
        uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
        memset(rxbuffer, 0, sizeof(rxbuffer));
        memset(ATcommand, 0, sizeof(ATcommand));

        limit = limits;
        FLAG_Diagnostic = false;

        if (limit) // During global update, check if the previous update was skipped
        {
            if (check_version_skip())
            {
                return 0;
            }
        }

        // FLAG_SIM = false;
        return 1;
    }
    else
    {
        printf("Already installed version\n");

        if (first_check == 0)
        {
            if (limits == 0) // By default random updates are local updates, so check for global update too for
            {
                limits = 1;
                goto retry;
            }
            limit = 0; // If no updates are availble for both cases for random update, set local to default
        }

        OTA_Verification_Status = 5;
        memset(rxbuffer, 0, sizeof(rxbuffer));
        memset(ATcommand, 0, sizeof(ATcommand));
        sprintf(ATcommand, "AT+HTTPTERM\r\n");
        uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
        // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
        uart_read(rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
        uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
        uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
        memset(rxbuffer, 0, sizeof(rxbuffer));
        memset(ATcommand, 0, sizeof(ATcommand));

        // FLAG_SIM = false;
        printf("Check version Completed\n");
        http_post_update_remark(20, 20);
    }
    return 0;
}

int check_version_skip()
{
    char *data6[5] = {0};
    int i6 = 0;
    char delim6[2];
    sprintf(delim6, ".");
    char *ptr6 = strtok(new_version, delim6);
    while (ptr6 != NULL)
    {
        data6[i6] = ptr6;
        ptr6 = strtok(NULL, delim6);
        i6++;
    }

    char *data7[5] = {0};
    int i7 = 0;
    char delim7[2];
    sprintf(delim7, ".");
    char *ptr7 = strtok(current_version, delim7);
    while (ptr7 != NULL)
    {
        data7[i7] = ptr7;
        ptr7 = strtok(NULL, delim7);
        i7++;
    }

    // printf("%s");

    int diff = abs(atoi(data6[2]) - atoi(data7[2]));

    if (diff >= 2)
    {
        printf("\nPrevious version was skipped\n");
    }
    else
    {
        printf("\nPrevious version wasn't skipped\n");
        return 0;
    }

    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 5000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPINIT\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    strcat(URL_CRC, File_URL);
    strcat(URL_CRC, CRC_File);

    sprintf(ATcommand, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", URL_CRC);
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    memset(URL_ESP_Board, 0, sizeof(URL_ESP_Board));

    esp_task_wdt_reset(); // OTA task refresh
    sprintf(ATcommand, "AT+HTTPACTION=0\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 20000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 500 / portTICK_PERIOD_MS);
    memset(rxbuffer, 0, sizeof(rxbuffer));
    uart_read(rxbuffer, BUFFSIZE, 120000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    if (strstr((char *)rxbuffer, "+CCH"))
    {
        memset(rxbuffer, 0, sizeof(rxbuffer));
        uart_read(rxbuffer, BUFFSIZE, 120000 / portTICK_PERIOD_MS);
        uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    }

    int length_data = 0;

    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    sprintf(ATcommand, "AT+HTTPREAD=%d,%d\r\n", 0, 1024);
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    uart_get_buffered_data_len(uartport2, (size_t *)&length_data);

    previous_time = esp_timer_get_time();
    while ((int)length_data < (64 + 63)) // Get full data from SIM to uart // 60 offset for response data
    {
        current_time = esp_timer_get_time();
        if ((current_time - previous_time) > WAIT_PERIOD * 2.5) // 1 min wait
        {

            printf("Network Delay\n");
            data_buf[0] = 3;
            twaiTxData(OTA_UI_DISPLAY, data_buf, 1); // Send No Network UI request to display
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            issue = 13;
            return 1;
            //}
        }
        uart_get_buffered_data_len(uartport2, (size_t *)&length_data);
    }
    uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 0);
    uart_flush(uartport2);
    uart_flush(uartport0);
    int a;

    if (strstr((char *)rxbuffer, "+HTTPREAD"))
    {
        char *data2[10] = {0};
        int i2 = 0;
        char delim2[] = "\n";
        char *ptr2 = strtok((char *)rxbuffer, delim2);
        while (ptr2 != NULL)
        {
            data2[i2] = ptr2;
            ptr2 = strtok(NULL, delim2);
            i2++;
        }

        a = 0;
        for (int j = 0; j < (i2 - 1); j++)
        {
            if (strstr(data2[j], "+HTTPREAD: DATA,"))
            {
                for (int h = 0; h < (strlen(data2[j + 1]) - 1); h++)
                {
                    CRC_Data[h + a] = (char)data2[j + 1][h];
                }
                a = strlen(data2[j + 1]) - 1;
            }
        }
    }
    else
    {
        return 1;
    }

    char *data4[10] = {0};
    int i4 = 0;
    char delim4[5];
    sprintf(delim4, ", =");
    char *ptr4 = strtok(CRC_Data, delim4);
    while (ptr4 != NULL)
    {
        data4[i4] = ptr4;
        ptr4 = strtok(NULL, delim4);
        i4++;
    }

    sum = 0;
    int board_CRC = 0;
    uint32_t CRC[3];
    for (int u = 0; u < i4; u++)
    {
        printf("%d = %s\n", u, data4[u]);
        if (strstr(data4[u], "VCU"))
        {
            board_CRC = 0;
        }
        else if (strstr(data4[u], "DISPLAY"))
        {
            board_CRC = 1;
        }
        else if (strstr(data4[u], "COMM"))
        {
            board_CRC = 2;
        }
        if ((u % 2) != 0)
        {
            sum += atoi(data4[u]);
            char *data5[5] = {0};
            int i5 = 0;
            char delim5[2];
            sprintf(delim5, "|");
            char *ptr5 = strtok(data4[u], delim5);
            while (ptr5 != NULL)
            {
                data5[i5] = ptr5;
                ptr5 = strtok(NULL, delim5);
                i5++;
            }
            CRC[board_CRC] = atoll(data5[1]);
        }
    }
    VCU_CRC = CRC[0];
    Display_CRC = CRC[1];
    COMM_CRC = CRC[2];

    printf("VCU CRC = %x\n", VCU_CRC);
    printf("Display CRC = %x\n", Display_CRC);
    printf("COMM CRC = %x\n", COMM_CRC);

    printf("sum(base64)= %lld\n", sum);
    printf("sum(hex)= %lld\n", (sum / 4) * 3);
    VCU_FLAG = true;
    COMM_FLAG = true;
    DISPLAY_FLAG = true;
    return 0;
}

int ESP_OTA()
{
    // if (FLAG_SIM == false)
    // {
    //     FLAG_SIM = true;

    // uint8_t data_buf[8];
    uint8_t sha_256[HASH_LEN] = {0};
    esp_partition_t partition;

    // get sha256 digest for the partition table
    partition.address = ESP_PARTITION_TABLE_OFFSET;
    partition.size = ESP_PARTITION_TABLE_MAX_LEN;
    partition.type = ESP_PARTITION_TYPE_DATA;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for the partition table: ");

    // get sha256 digest for bootloader
    partition.address = ESP_BOOTLOADER_OFFSET;
    partition.size = ESP_PARTITION_TABLE_OFFSET;
    partition.type = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for bootloader: ");

    // get sha256 digest for running partition
    esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
    print_sha256(sha_256, "SHA-256 for current firmware: ");

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK)
    {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
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

    ESP_LOGI(TAG, "Starting ESP OTA");

    const esp_partition_t *configured = esp_ota_get_boot_partition();
    // const esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running)
    {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
                 configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             running->type, running->subtype, running->address);

    update_partition = esp_ota_get_next_update_partition(NULL);
    assert(update_partition != NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %x at offset 0x%x",
             update_partition->subtype, update_partition->address);

    int binary_file_length = 0;
    /*deal with all receive packet*/

    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    data_buf[0] = 0;
    twaiTxData(ESP_UPDATE_WAIT_ACK, data_buf, 1);

    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 5000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPINIT\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    strcat(URL_ESP_Board, File_URL);
    if (limit == 0) // local
    {
        printf("local\n");
        strcat(URL_ESP_Board, local_updates);
        strcat(URL_ESP_Board, bike_id);
        strcat(URL_ESP_Board, "/");
    }
    else
    {
        strcat(URL_ESP_Board, global_updates);
    }
    strcat(URL_ESP_Board, ESP_Board);

    sprintf(ATcommand, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", URL_ESP_Board);
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    memset(URL_ESP_Board, 0, sizeof(URL_ESP_Board));

    // if (!(strstr((char *)rxbuffer, "OK")))
    // {
    //     issue = 22;
    //     return 0;
    // }

    esp_task_wdt_reset(); // OTA task refresh
    sprintf(ATcommand, "AT+HTTPACTION=0\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 20000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 500 / portTICK_PERIOD_MS);
    memset(rxbuffer, 0, sizeof(rxbuffer));
    uart_read(rxbuffer, BUFFSIZE, 120000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    if (strstr((char *)rxbuffer, "+CCH"))
    {
        memset(rxbuffer, 0, sizeof(rxbuffer));
        uart_read(rxbuffer, BUFFSIZE, 120000 / portTICK_PERIOD_MS);
        uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    }
    // uart_flush(uartport2);
    // uart_flush(uartport0);

    // if (check_403() == 1)
    //     return 0;

    if (strstr((char *)rxbuffer, "+HTTPACTION"))
    {
        char *data1[10] = {0};
        int i1 = 0;
        char delim1[] = ",";
        char *ptr1 = strtok((char *)rxbuffer, delim1);
        while (ptr1 != NULL)
        {
            data1[i1] = ptr1;
            ptr1 = strtok(NULL, delim1);
            i1++;
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
        fileSize_ESP = atoi(data1[2]);
        File_Size_ESP = ((fileSize_ESP * 3) / 4);
        printf("file size ESP (base64): %lld\n", fileSize_ESP);
        printf("file size ESP(hex): %lld\n", File_Size_ESP);
        memset(rxbuffer, 0, sizeof(rxbuffer));
    }
    else
    {
        printf("Network Error\n");
        issue = 12;
        return 0;
    }

    __int32_t len = 0;
    __int32_t i = 0;
    __int32_t data_bytes = 0;
    __int32_t data_read = 0;
    int length_data = 0;
    int a;
    bool image_header_was_checked = false;
    bool FLAG_END;
    FLAG_END = false;

    uint64_t start_time = 0;
    uint32_t Estimated_time = 0;
    float diff = 0;

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    printf("Sum = %lld\n", sum);

    while (!FLAG_END)
    {
        memset(out, '\0', sizeof(out));
        memset(rxbuffer, '\0', sizeof(rxbuffer));
        memset(ota_write_data, '\0', sizeof(ota_write_data));

        esp_task_wdt_reset();

        data_buf[0] = 0;
        twaiTxData(ESP_UPDATE_WAIT_ACK, data_buf, 1); // ACK to VCU to acknowledge that ESP is being downloaded suessfully

        start_time = esp_timer_get_time();

        // vTaskDelay(10 / portTICK_PERIOD_MS);
        if ((fileSize_ESP - i) > 1024)
        {
            data_bytes = 1024;
            data_read = 768;
        }
        else
        {
            data_bytes = fileSize_ESP - i;
            data_read = ((data_bytes * 3) / 4);
            FLAG_END = true;
        }
        memset(rxbuffer, 0, sizeof(rxbuffer));
        memset(ATcommand, 0, sizeof(ATcommand));
        sprintf(ATcommand, "AT+HTTPREAD=%d,%d\r\n", i, data_bytes);
        // sprintf(ATcommand, "AT+HTTPREAD\r\n");
        uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
        ////////////////////////////////
        uart_get_buffered_data_len(uartport2, (size_t *)&length_data);

        previous_time = esp_timer_get_time();
        while ((int)length_data < (data_bytes + 63)) // Get full data from SIM to uart // 60 offset for response data
        {
            // printf("length_data = %d\n", (int)length_data);

            current_time = esp_timer_get_time();
            if ((current_time - previous_time) > WAIT_PERIOD * 2.5) // 1 min wait
            {

                printf("Network Delay\n");
                data_buf[0] = 3;
                twaiTxData(OTA_UI_DISPLAY, data_buf, 1); // Send No Network UI request to display
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                issue = 13;
                return 0;
                //}
            }

            // vTaskDelay(2000 / portTICK_PERIOD_MS);
            // esp_task_wdt_reset();
            uart_get_buffered_data_len(uartport2, (size_t *)&length_data);
        }
        ///////////////////////////////
        // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 1 / portTICK_PERIOD_MS);
        uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 0);
        uart_flush(uartport2);
        // uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
        // uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
        uart_flush(uartport0);

    redo_0:
        if (strstr((char *)rxbuffer, "+HTTPREAD"))
        {
            char *data2[10] = {0};
            int i2 = 0;
            char delim2[] = "\n";
            char *ptr2 = strtok((char *)rxbuffer, delim2);
            while (ptr2 != NULL)
            {
                data2[i2] = ptr2;
                ptr2 = strtok(NULL, delim2);
                i2++;
            }

            a = 0;
            for (int j = 0; j < (i2 - 1); j++)
            {
                if (strstr(data2[j], "+HTTPREAD: DATA,"))
                {
                    for (int h = 0; h < data_bytes; h++)
                    {
                        decode_data[h + a] = (char)data2[j + 1][h];
                        // printf("%c", decode_data[h + a]);
                    }
                    a = data_bytes;
                }
            }
        }
        else
        {
            data_buf[0] = 0;
            twaiTxData(ESP_UPDATE_WAIT_ACK, data_buf, 1); // ACK to VCU to acknowledge that ESP is being downloaded suessfully

            uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 30000 / portTICK_PERIOD_MS);
            // data_buf[0] = 0;
            // twaiTxData(ESP_UPDATE_WAIT_ACK, data_buf, 1); // ACK to VCU to acknowledge that ESP is being downloaded suessfully
            if ((strlen((char *)rxbuffer) > data_bytes))
            {
                printf("Delay in response overcome!\n");
                goto redo_0;
            }
            printf("Failed to fetch data\nNetwork Error!!\n");
            issue = 13;
            return 0;
        }
        data_buf[0] = 0;
        twaiTxData(ESP_UPDATE_WAIT_ACK, data_buf, 1); // ACK to VCU to acknowledge that ESP is being downloaded suessfully

        len = strlen(decode_data);
        i += len;
        printf(" i = %d\n", i);
        out_len = len;
        if (!b64_decode(decode_data, data_bytes))
        {
            printf("Decode Failure\nNetwork Error\n");
            issue = 14;
            return 0; // break; // esp_restart();
        }
        for (int y = 0; y < data_read; y++) // TODO: Remove this line
        {
            ota_write_data[y] = out[y];
        }

        // if (data_read < 0)
        // {
        //     ESP_LOGE(TAG, "Error: SSL data read error");
        //     return 0; // break;
        //               // http_cleanup(client);
        //               // task_fatal_error();
        // }
        // else
        if (data_read > 0)
        {
            if (image_header_was_checked == false)
            {
                esp_app_desc_t new_app_info;
                int len1 = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t);
                printf("lenght of the strange headers:: %d\n\n", len1);
                if (data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
                {
                    // check current version with downloading
                    memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
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
                    // #ifndef CONFIG_EXAMPLE_SKIP_VERSION_CHECK
                    //                     if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) == 0)
                    //                     {
                    //                         ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
                    //                         issue = 19;
                    //                         return 0;
                    //                         // http_cleanup(client);
                    //                         // infinite_loop();
                    //                     }
                    // #endif

                    image_header_was_checked = true;

                    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
                    if (err != ESP_OK)
                    {
                        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                        // http_cleanup(client);
                        esp_ota_abort(update_handle);
                        // task_fatal_error();
                    }
                    ESP_LOGI(TAG, "esp_ota_begin succeeded");
                }
                else
                {
                    ESP_LOGE(TAG, "received package is not fit len");
                    // http_cleanup(client);
                    esp_ota_abort(update_handle);
                    // task_fatal_error();
                }
            }
            err = esp_ota_write(update_handle, (const void *)ota_write_data, data_read);
            // if (err == ESP_OK)
            // {
            //     printf("Successfuly Write\n");
            // }

            // else
            if (err == ESP_ERR_OTA_VALIDATE_FAILED)
            {
                printf("Magic byte error\n");
                issue = 15;
                return 0; // break;
            }

            // vTaskDelay(10 / portTICK_PERIOD_MS);
            binary_file_length += data_read;
            ESP_LOGD(TAG, "Written image length %d", binary_file_length);
            // vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        // else if (data_read == 0)
        // {

        //     if (errno == ECONNRESET || errno == ENOTCONN)
        //     {
        //         ESP_LOGE(TAG, "Connection closed, errno = %d", errno);
        //         break;
        //     }

        //     break;
        // }

        memset(data_buf, 0, sizeof(data_buf));
        // data_buf[0] = (i * 100) / sum; // send Download percentage to display
        diff = (esp_timer_get_time() - start_time) / 10000.0;
        // printf("diff = %.2f\n", diff);
        float temp_calculation = (sum - (i * 0.75));
        Estimated_time = ceil(((diff * temp_calculation) / (6000.0 * data_read)) + 1.0); // Estimated time

        // printf("\n\n\n%d\n\n", i);
        if (i == 1024) // Only for the first time
        {
            // printf("\n\n\nHere\n\n\n\n");
            data_buf[0] = (i * 0.75 * 100) / sum;
            data_buf[1] = Estimated_time;
            twaiTxData(OTA_EST_TIME, data_buf, 8);

            // data_buf[0] = 8;
            data_buf[0] = 9;
            twaiTxData(OTA_UI_DISPLAY, data_buf, 2); // send display to Downloading UI
        }

        data_buf[0] = (i * 0.75 * 100) / sum;
        data_buf[1] = Estimated_time;
        twaiTxData(OTA_EST_TIME, data_buf, 8);
    }

    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 5000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    // uart_flush(uartport2);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    // uart_flush(uartport0);
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    ESP_LOGI(TAG, "Total Write binary data length: %d", binary_file_length);

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
        issue = 16;
        return 0;
    }

    // ESP_LOGI(TAG, "Prepare to restart system!");
    printf("Downloading updates for ESP Completed\n");
    // FLAG_SIM = false;
    return 1;
    // }
    // else
    // {
    //     printf("SIM is busy for Downloading ESP Updates\n");
    //     return 0;
    // }
}

int Download_Updates(int board) // Download, Convert and Send to VCU fot OTA
{
    // activating LTE service and then initializing ssl configurations
    // if (FLAG_SIM == false)
    // {
    //     FLAG_SIM = true;

    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 5000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    // uart_flush(uartport2);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    // uart_flush(uartport0);
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPINIT\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    strcat(URL_Board, File_URL);

    if (limit == 0) // local
    {
        printf("local\n");
        strcat(URL_ESP_Board, local_updates);
        strcat(URL_Board, bike_id);
        strcat(URL_Board, "/");
    }
    else
    {
        strcat(URL_ESP_Board, global_updates);
    }

    if (board == 1)
    {
        strcat(URL_Board, VCU_Board);
    }
    else if (board == 2)
    {
        strcat(URL_Board, Display_Board);
    }

    sprintf(ATcommand, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", URL_Board);
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    memset(URL_Board, 0, sizeof(URL_Board));

    // if (!(strstr((char *)rxbuffer, "OK")))
    // {
    //     issue = 22;
    //     return 0;
    // }
    ESP_ERROR_CHECK(esp_task_wdt_reset()); // OTA task refresh
    sprintf(ATcommand, "AT+HTTPACTION=0\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 20000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 500 / portTICK_PERIOD_MS);
    memset(rxbuffer, 0, sizeof(rxbuffer));
    uart_read(rxbuffer, BUFFSIZE, 120000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    if (strstr((char *)rxbuffer, "+CCH"))
    {
        memset(rxbuffer, 0, sizeof(rxbuffer));
        uart_read(rxbuffer, BUFFSIZE, 120000 / portTICK_PERIOD_MS);
        uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    }
    uart_flush(uartport2);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    uart_flush(uartport0);

    // if (check_403() == 1)
    //     return 0;

    if (strstr((char *)rxbuffer, "+HTTPACTION"))
    {
        char *data1[30] = {0};
        int i1 = 0;
        char delim1[] = ",";
        char *ptr1 = strtok((char *)rxbuffer, delim1);
        while (ptr1 != NULL)
        {
            data1[i1] = ptr1;
            ptr1 = strtok(NULL, delim1);
            i1++;
        }
        uart_write_bytes(uartport0, (uint8_t *)"\n", strlen("\n"));
        uart_write_bytes(uartport0, (uint8_t *)data1[2], strlen((char *)data1[2]));
        uart_write_bytes(uartport0, (uint8_t *)"\n", strlen("\n"));
        vTaskDelay(50 / portTICK_PERIOD_MS);
        fileSize = atoi(data1[2]);
        // TODO: Convert sending file size to hex
        File_Size = ((fileSize * 3) / 4);
        // VCU_FILE_SIZE = File_Size;
        printf("file size (base64): %lld\n", fileSize);
        printf("file size (hex): %lld\n", File_Size);
        memset(data_buf, 0, sizeof(data_buf));
        Flag_Data_Start = false;
        data_buf[0] = (File_Size >> 24) & 0xFF;
        data_buf[1] = (File_Size >> 16) & 0xFF;
        data_buf[2] = (File_Size >> 8) & 0xFF;
        data_buf[3] = File_Size & 0xFF;
        twaiTxData(SEND_File_Size, data_buf, 8);
        memset(data_buf, 0, sizeof(data_buf));
        memset(rxbuffer, 0, sizeof(rxbuffer));
    }
    else
    {
        issue = 12;
        printf("Network Error\n");
        return 0; // Terminate_OTA();
    }

    previous_time = esp_timer_get_time();
    while (Flag_Data_Start == false)
    {
        current_time = esp_timer_get_time();
        if ((current_time - previous_time) > WAIT_PERIOD)
        {
            printf("OTA Update Failed!!!\nVCU did not response\n");
            issue = 11;
            return 0; // Terminate_OTA();
            // esp_restart();
        }
    }
    Flag_Data_Start = false;

    printf("START FILE TRANSMISSION \n");

    __int32_t i = 0;
    int len = 0;
    __int32_t data_bytes = 0;
    int h = 0;
    int length_data;
    int a;

    uint64_t start_time = 0;
    uint32_t Estimated_time = 0;
    float diff = 0;

    while (!Flag_File_End)
    {
        start_time = esp_timer_get_time();

        ESP_ERROR_CHECK(esp_task_wdt_reset()); // OTA task refresh
        if ((fileSize - i) >= 1024)
        {
            data_bytes = 1024;
        }
        else
        {
            data_bytes = fileSize - i;
        }

        if ((i + data_bytes) == fileSize)
        {
            Flag_File_End = 1;
        }

        memset(rxbuffer, 0, sizeof(rxbuffer));
        sprintf(ATcommand, "AT+HTTPREAD=%d,%d\r\n", i, data_bytes);
        // sprintf(ATcommand, "AT+HTTPREAD=0,1024\r\n");
        uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
        ////////////////////////////////
        uart_get_buffered_data_len(uartport2, (size_t *)&length_data);

        previous_time = esp_timer_get_time();
        while ((int)length_data < (data_bytes + 63)) // Get full data from SIM to uart // 60 offset for response data
        {
            // printf("length_data = %d\n", (int)length_data);

            current_time = esp_timer_get_time();
            if ((current_time - previous_time) > WAIT_PERIOD * 2.5) // 1 min wait
            {
                printf("Network Delay\n");
                // data_buf[0] = 8;
                data_buf[0] = 4;
                twaiTxData(OTA_UI_DISPLAY, data_buf, 2); // Send No Network UI request to display
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                issue = 13;
                return 0;
            }
            // vTaskDelay(2000 / portTICK_PERIOD_MS);
            // ESP_ERROR_CHECK(esp_task_wdt_reset());
            uart_get_buffered_data_len(uartport2, (size_t *)&length_data);
        }
        ///////////////////////////////
        uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 0);
        uart_flush(uartport2);
        // uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
        // uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
        uart_flush(uartport0);

    redo:
        if (strstr((char *)rxbuffer, "+HTTPREAD"))
        {
            // printf("Here\n");
            char *data2[10] = {0};
            int i2 = 0;
            char delim2[] = "\n";
            char *ptr2 = strtok((char *)rxbuffer, delim2);
            while (ptr2 != NULL)
            {
                data2[i2] = ptr2;
                ptr2 = strtok(NULL, delim2);
                i2++;
            }

            // max = 0;
            a = 0;
            for (int j = 0; j < (i2 - 1); j++)
            {
                if (strstr(data2[j], "+HTTPREAD: DATA,"))
                {
                    for (int h = 0; h < data_bytes; h++)
                    {
                        decode_data[h + a] = (char)data2[j + 1][h];
                    }
                    a = data_bytes;
                }
            }
        }
        else
        {
            uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 20000 / portTICK_PERIOD_MS);
            if ((strlen((char *)rxbuffer) > data_bytes))
            {
                printf("Delay in response overcome!\n");
                goto redo;
            }
            printf("Failed to fetch data\nNetwork Error!!\n");
            issue = 13;
            return 0;
            // Terminate_OTA();
        }

        // uart_write_bytes(uartport0, decode_data, strlen((char *)decode_data));
        len = strlen(decode_data);
        i += len;

        out_len = len;

        if (!b64_decode(decode_data, data_bytes))
        {
            printf("Decode Failure\nNetwork Error\n");
            issue = 14;
            return 0; // Terminate_OTA();
        }

        Flag_ACK_VCU = false;
        if (data_bytes == 1024)
        {
            for (int e = 0; e < 768;)
            {
                for (int a = 0; a < 8; a++)
                {
                    data_buf[a] = out[e + a];
                    // printf("%x", out[e + a]);
                }
                twaiTxData(Data_To_VCU, data_buf, 8);
                memset(data_buf, 0, sizeof(data_buf));
                // vTaskDelay(10 / portTICK_PERIOD_MS);
                e += 8;
            }
            printf("Data Sent \n");
        }
        else
        {
            // vTaskDelay(2000 / portTICK_PERIOD_MS);
            int v = data_bytes * 3 / 4;
            printf("v = %d\n", v);
            for (int y = 0; (y < v);)
            {
                // printf("y = %d\n", y);

                h = v - y;

                if (h >= 8)
                {
                    // printf("normal h = 8");
                    for (int j = 0; j < 8; j++)
                    {
                        data_buf[j] = out[y + j];
                    }
                    twaiTxData(Data_To_VCU, data_buf, 8);
                    memset(data_buf, 0, sizeof(data_buf));
                    // vTaskDelay(10 / portTICK_PERIOD_MS);

                    y += 8;
                }
                else
                {
                    // printf("last = %d\n", h);
                    for (int j = 0; j < h; j++)
                    {
                        data_buf[j] = out[y + j];
                    }
                    twaiTxData(Data_To_VCU, data_buf, h);
                    memset(data_buf, 0, sizeof(data_buf));
                    y += h;
                }
                // printf("Data Last Sent \n");
            }
        }

        previous_time = esp_timer_get_time();
        while (Flag_ACK_VCU == false)
        {
            current_time = esp_timer_get_time();
            if ((current_time - previous_time) > WAIT_PERIOD)
            {
                printf("OTA Update Failed!!!!!\nVCU did not responded\n");
                data_buf[0] = 1;
                twaiTxData(ESP_OTA_STATUS, data_buf, 1); // Send ESP Update Failed

                issue = 11;
                return 0; // Terminate_OTA();
                // esp_restart();
            }
        }
        Flag_ACK_VCU = false;

        float temp_calculation = (sum - ((i * 0.75) + send_esp_size + VCU_FILE_SIZE));
        diff = (esp_timer_get_time() - start_time) / 10000.0;
        Estimated_time = ceil(((diff * temp_calculation) / (6000.0 * data_bytes))); // Estimated time

        data_buf[0] = (((i * 0.75) + (send_esp_size + VCU_FILE_SIZE)) * 100) / sum;
        data_buf[1] = Estimated_time;
        twaiTxData(OTA_EST_TIME, data_buf, 8);

        if ((i == 1024) && (COMM_FLAG == false)) // Only for the first times
        {
            if ((VCU_FLAG == true)) // When VCU update
            {
                data_buf[0] = 9;
                twaiTxData(OTA_UI_DISPLAY, data_buf, 2); // send display to Downloading UI
            }
            else if ((VCU_FLAG == false) && (DISPLAY_FLAG == true))
            {
                data_buf[0] = 9;
                twaiTxData(OTA_UI_DISPLAY, data_buf, 2); // send display to Downloading UI
            }
        }

        // data_buf[0] = 8;
        // data_buf[0] = 9;
        // twaiTxData(OTA_UI_DISPLAY, data_buf, 2); // send display to Downloading UI
    }

    // sum = sum - File_Size;
    Flag_File_End = 0;

    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 5000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    vTaskDelay(50 / portTICK_PERIOD_MS);

    // FLAG_SIM = false;
    printf("Downloading_updates task Completed\n");
    // }
    // else
    // {
    //     printf("SIM is busy for Downloading Updates\n");
    //     return 0;
    // }

    return 1;
}

int Yatri_OTA() // Main OTA Function for all
{
    Flag_OTA = true; // For differentiating OTA mode and just Version check to request ID to display
    int board_count = 0;

    printf("Send Bike status request\n");
    data_buf[0] = 0;
    twaiTxData(ask_bike_status, data_buf, 1); // Send Data receive ack to VCU

    previous_time = esp_timer_get_time();
    while (Bike_Status == false)
    {
        current_time = esp_timer_get_time();
        if ((current_time - previous_time) > WAIT_PERIOD)
        {
            printf("OTA Update Failed!!!!!!\nNo response from VCU for verifying Bike status\n");
            // issue = 9;
            return 2;
        }
    }
    Bike_Status = false;

    if (Flag_Bike_Status == true)
    {
        Flag_Bike_Status = false;
        OTA_Verification_Status = 1;

        // data_buf[0] = 8;
        data_buf[0] = 3;
        twaiTxData(OTA_UI_DISPLAY, data_buf, 1); // send display to checking Network UI
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        if (check_version(limit) == 1)
        {
            Flag_OTA = false;
            if ((VCU_FLAG == true) & (DISPLAY_FLAG == true) & (COMM_FLAG == true))
            {
                data_buf[0] = 5;
                board_count = 3;
                printf("All three boards\n");
            }
            else if ((VCU_FLAG == true) & (DISPLAY_FLAG == true))
            {
                data_buf[0] = 3;
                board_count = 2;
                printf("VCU and Display boards\n");
            }
            else if ((VCU_FLAG == true) & (COMM_FLAG == true))
            {
                data_buf[0] = 6;
                board_count = 2;
                printf("VCU and COMM boards\n");
            }

            else if ((COMM_FLAG == true) & (DISPLAY_FLAG == true))
            {
                data_buf[0] = 7;
                board_count = 2;
                printf("COMM and Display boards\n");
            }
            else if (DISPLAY_FLAG == true)
            {
                data_buf[0] = 2;
                board_count = 1;
                printf("Only Display\n");
            }
            else if (VCU_FLAG == true)
            {
                data_buf[0] = 1;
                board_count = 1;
                printf("Only VCU\n");
            }
            else if (COMM_FLAG == true)
            {
                data_buf[0] = 4;
                board_count = 1;
                printf("Only COMM \n");
            }

            FLAG_FIRST_ACK = false;
            twaiTxData(OTA_REQ_JUMP, data_buf, 1); // Send OTA Request
            printf("Sending OTA request \n");

            // wait for OTA mode ACK from VCU
            previous_time = esp_timer_get_time();
            while (FLAG_FIRST_ACK == false)
            {
                current_time = esp_timer_get_time();
                if ((current_time - previous_time) > WAIT_PERIOD)
                {
                    printf("OTA Update Failed!!!!!!\nNo response from VCU\n");
                    issue = 11;
                    return 0; // Terminate_OTA();
                }
            }
            FLAG_FIRST_ACK = false;
            printf("OTA ACK received from Bootloader1\n");
            fileSize_ESP = 0;
            sum = (sum / 4) * 3; // converting base64 filesize to hex
            send_esp_size = 0;

            ESP_ERROR_CHECK(esp_task_wdt_reset()); // OTA task refresh

            if (COMM_FLAG == true)
            {
                if (ESP_OTA() == 1)
                {
                    memset(data_buf, 0, sizeof(data_buf));
                    send_esp_size = ((fileSize_ESP / 4) * 3);

                    data_buf[0] = (send_esp_size >> 24) & 0xFF;
                    data_buf[1] = (send_esp_size >> 16) & 0xFF;
                    data_buf[2] = (send_esp_size >> 8) & 0xFF;
                    data_buf[3] = send_esp_size & 0xFF;
                    twaiTxData(ESP_File_Size, data_buf, 4); // Send Total FileSize
                    printf("ESP file Size sent\n");
                    board_count--;
                    // sum = sum - send_esp_size;
                }
                else
                {
                    data_buf[0] = 1;
                    twaiTxData(ESP_OTA_STATUS, data_buf, 1); // Send ESP Update Failed
                    printf("ESP update failed\n");
                    return 0;
                }
            }

            vTaskDelay(2000 / portTICK_PERIOD_MS);
            memset(data_buf, 0, sizeof(data_buf));
            data_buf[0] = 0;
            twaiTxData(ESP_OTA_STATUS, data_buf, 1); // ESP Update ACK

            vTaskDelay(2000 / portTICK_PERIOD_MS);
            FLAG_TOTAL_FILESIZE = false;

            memset(data_buf, 0, sizeof(data_buf));
            data_buf[0] = (sum >> 24) & 0xFF;
            data_buf[1] = (sum >> 16) & 0xFF;
            data_buf[2] = (sum >> 8) & 0xFF;
            data_buf[3] = sum & 0xFF;
            twaiTxData(TOTAL_File_Size, data_buf, 4); // Send Total Rem FileSize
            printf("Total File Size sent\n");

            previous_time = esp_timer_get_time();
            while (FLAG_TOTAL_FILESIZE == false)
            {
                current_time = esp_timer_get_time();
                if ((current_time - previous_time) > WAIT_PERIOD)
                {
                    printf("OTA Update Failed!!!!!!\n File Size ACK not received\n");
                    issue = 11;
                    return 0;
                    // Terminate_OTA();
                }
            }
            FLAG_TOTAL_FILESIZE = false;

            Flag_Send_version = false;
            VCU_FILE_SIZE = 0;

            if (board_count != 0)
            {
                if (VCU_FLAG == true)
                {
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    memset(data_buf, 0, sizeof(data_buf));
                    data_buf[0] = (VCU_CRC >> 24) & 0xFF;
                    data_buf[1] = (VCU_CRC >> 16) & 0xFF;
                    data_buf[2] = (VCU_CRC >> 8) & 0xFF;
                    data_buf[3] = VCU_CRC & 0xFF;
                    twaiTxData(CRC_SEND_VCU, data_buf, 8); // Send CRc of VCU
                    printf("VCU CRC Sent\n");

                    previous_time = esp_timer_get_time();
                    while (FLAG_CRC_ACK == false)
                    {
                        current_time = esp_timer_get_time();
                        if ((current_time - previous_time) > WAIT_PERIOD)
                        {
                            printf("OTA Update Failed!!!!!!\n CRC ACK not received\n");
                            issue = 11;
                            return 0;
                            // Terminate_OTA();
                        }
                    }
                    FLAG_CRC_ACK = false;

                    ESP_ERROR_CHECK(esp_task_wdt_reset()); // OTA task refresh
                    printf("Downloading for VCU\n");
                    Flag_Start_Display = false;
                    if (Download_Updates(1) == 0)
                    {
                        return 0;
                    }
                    VCU_FILE_SIZE = File_Size;
                    ESP_ERROR_CHECK(esp_task_wdt_reset()); // OTA Task Refresh
                    previous_time = esp_timer_get_time();
                    while (Flag_Start_Display == false)
                    {
                        current_time = esp_timer_get_time();
                        if ((current_time - previous_time) > WAIT_PERIOD)
                        {
                            printf("OTA Update Failed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                            issue = 11;
                            return 0; // Terminate_OTA();
                        }
                    }
                    Flag_Start_Display = false;
                    board_count--;
                }

                if (DISPLAY_FLAG == true)
                {
                    memset(data_buf, 0, sizeof(data_buf));
                    FLAG_CRC_ACK = false;
                    data_buf[0] = (Display_CRC >> 24) & 0xFF;
                    data_buf[1] = (Display_CRC >> 16) & 0xFF;
                    data_buf[2] = (Display_CRC >> 8) & 0xFF;
                    data_buf[3] = Display_CRC & 0xFF;
                    twaiTxData(CRC_SEND_DISPLAY, data_buf, 8); // Send CRC of Display
                    printf("Display CRC Sent\n");

                    previous_time = esp_timer_get_time();
                    while (FLAG_CRC_ACK == false)
                    {
                        current_time = esp_timer_get_time();
                        if ((current_time - previous_time) > WAIT_PERIOD)
                        {
                            printf("OTA Update Failed!!!!!!\n CRC ACK not received\n");
                            issue = 11;
                            return 0;
                            // Terminate_OTA();
                        }
                    }
                    FLAG_CRC_ACK = false;
                    printf("Downloading for Display\n");
                    //                         FLAG_HOLD = true;
                    if (Download_Updates(2) == 0)
                    {
                        return 0;
                    }
                }
            }

            printf("Downloadings Completed\n");
            data_buf[0] = 10;
            twaiTxData(OTA_UI_DISPLAY, data_buf, 1);

            FLAG_CRC_V = false;
            FLAG_CRC_D = false;

            FLAG_VCU_ROLL = false;
            FLAG_DISPLAY_ROLL = false;

            previous_time = esp_timer_get_time();
            while (Flag_Send_version == false)
            {
                ESP_ERROR_CHECK(esp_task_wdt_reset()); // OTA task refresh
                current_time = esp_timer_get_time();
                if ((current_time - previous_time) > WAIT_PERIOD * 3)
                {
                    // printf("Flashing delayed!\n");
                    // printf("OTA Update Failed!\n");
                    issue = 17;
                    return 0;
                }
                if (FLAG_CRC_V == true)
                {
                    issue = 22;
                    break;
                }
                if (FLAG_CRC_D == true)
                {
                    issue = 23;
                    break;
                }
            }

            if ((FLAG_CRC_V == true) || (FLAG_CRC_D == true))
            {
                if (FLAG_CRC_V == true)
                {
                    while (FLAG_VCU_ROLL == false)
                    {
                        ESP_ERROR_CHECK(esp_task_wdt_reset()); // OTA task refresh
                        current_time = esp_timer_get_time();
                        if ((current_time - previous_time) > WAIT_PERIOD * 2)
                        {
                            printf("Failed to roll back\n"); // TODO ping issue
                        }
                    }
                }

                if (FLAG_CRC_D == true)
                {
                    while (FLAG_DISPLAY_ROLL == false)
                    {
                        ESP_ERROR_CHECK(esp_task_wdt_reset()); // OTA task refresh
                        current_time = esp_timer_get_time();
                        if ((current_time - previous_time) > WAIT_PERIOD * 2)
                        {
                            printf("Failed to roll back\n"); // TODO ping issue
                        }
                    }
                }
                FLAG_VCU_ROLL = false;
                FLAG_DISPLAY_ROLL = false;
                FLAG_CRC_D = false;
                FLAG_CRC_V = false;
                return 0;
            }
            Flag_Send_version = false;

            Flag_Send_version_length = false;
            Flag_Send_version_data = false;

            data_buf[0] = new_verlen;
            twaiTxData(Version_Length, data_buf, 1);

            vTaskDelay(2000 / portTICK_PERIOD_MS);

            printf("Request for sending new version received\n");
            if (send_version() == 1)
            {

                if (COMM_FLAG == true)
                {
                    const esp_partition_t *update_partition = NULL;
                    esp_err_t err;
                    update_partition = esp_ota_get_next_update_partition(NULL);
                    err = esp_ota_set_boot_partition(update_partition);
                    if (err != ESP_OK)
                    {
                        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
                        // Terminate_OTA();
                        return 0;
                    }
                }
                //                 FLAG_SIM = false;
                printf("Update Successfull\n");
                ESP_ERROR_CHECK(esp_task_wdt_reset()); // OTA TASK Refresh

                // data_buf[0] = 8; //                 // http_post_software_version();
                data_buf[0] = 11;
                twaiTxData(OTA_UI_DISPLAY, data_buf, 1); // Send OTA Success UI request to display
                http_post_update_remark(20, 20);

                return 1;
            }
            return 2;
        }
        else
        {
            if (OTA_Verification_Status == 2)
            {
                // data_buf[0] = 8;
                data_buf[0] = 4;
                twaiTxData(OTA_UI_DISPLAY, data_buf, 2); // send display to No Network UI
            }
            else
            {
                // data_buf[0] = 8;
                data_buf[0] = 6;
                twaiTxData(OTA_UI_DISPLAY, data_buf, 2); // send display to Version Up to date UI
            }

            Flag_OTA = false; // For differentiating OTA mode and just Version check to request ID to display
            //                 // OTA_ESP_HOLD = false; // For holding ESP32 other tasks
            //                 // FLAG_HOLD = false;
            return 4;
        }
    }
    else
    {
        printf("Bike Status not Suitable for an Update\n");
        return 2;
    }

    //     FLAG_SIM = false;
    //     FLAG_HOLD = false;
    //     Flag_OTA = false;     // For differentiating OTA mode and just Version check to request ID to display
    //     OTA_ESP_HOLD = false; // For holding ESP32 other tasks

    return 0;
}

void http_post_update_remark(int issue, int issue_0)
{
    int download_OK = 0;
    int retry_Count = 0;
retry:
    uart_write_bytes(uartport0, "INSIDE ISSUE POST\n", strlen("INSIDE ISSUE POST\n"));

    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPINIT\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    if (issue == 20)
    {
        sprintf(ATcommand, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", URL_success);
    }
    else
    {
        sprintf(ATcommand, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", URL_issue);
    }

    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    ESP_ERROR_CHECK(esp_task_wdt_reset()); // OTA task refresh

    sprintf(ATcommand, "AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"USERDATA\",\"%s\"\r\n", b_apiKey);
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 3000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    if (issue == 20)
    {
        sprintf(ATcommand, "{\"status\":\"success\"}%c", 0x0A);
    }
    else
    {
        sprintf(ATcommand, "{\"remarks\":\"%d,%d\"}%c", issue, issue_0, 0x0A);
    }

    uint8_t len = strlen(ATcommand);
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPDATA=%d,1024\r\n", len);
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    ESP_ERROR_CHECK(esp_task_wdt_reset()); // OTA task refresh

    if (issue == 20)
    {
        sprintf(ATcommand, "{\"status\":\"success\"}%c", 0x0A);
    }
    else
    {
        sprintf(ATcommand, "{\"remarks\":\"%d,%d\"}%c", issue, issue_0, 0x0A);
    }

    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, "INSIDE WHILE\n", strlen("INSIDE WHILE\n"));
    while (!download_OK)
    {
        // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 500 / portTICK_PERIOD_MS);
        uart_read(rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
        uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
        if (strstr((char *)rxbuffer, "OK"))
        {
            download_OK = 1;
        }
    }
    uart_write_bytes(uartport0, "OUTSIDE WHILE\n", strlen("OUTSIDE WHILE\n"));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPACTION=1\r\n");
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    ESP_ERROR_CHECK(esp_task_wdt_reset()); // OTA task refresh
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 60000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 500 / portTICK_PERIOD_MS);
    memset(rxbuffer, 0, sizeof(rxbuffer));
    uart_read(rxbuffer, BUFFSIZE, 120000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    if (strstr((char *)rxbuffer, "+CCH"))
    {
        memset(rxbuffer, 0, sizeof(rxbuffer));
        uart_read(rxbuffer, BUFFSIZE, 120000 / portTICK_PERIOD_MS);
        uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    }

    if (strstr((char *)rxbuffer, "200"))
    {
        uart_write_bytes(uartport0, "SUCCESSFULLY POSTED\n", strlen("SUCCESSFULLY POSTED\n"));
    }
    else
    {
        if (retry_Count < 2)
        {
            retry_Count++;
            ESP_ERROR_CHECK(esp_task_wdt_reset()); // OTA task refresh
            uart_write_bytes(uartport0, "POSTING FAILED, RETRY\n", strlen("POSTING FAILED, RETRY\n"));
            goto retry;
        }
        else
        {
            uart_write_bytes(uartport0, "RETRY FAILED, RESUMING TASK\n", strlen("RETRY FAILED, RESUMING TASK\n"));
            retry_Count = 0;
        }
    }

    ESP_ERROR_CHECK(esp_task_wdt_reset()); // OTA task refresh

    sprintf(ATcommand, "AT+HTTPREAD=0,200\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 5000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uartport2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uartport2, rxbuffer, BUFFSIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rxbuffer, BUFFSIZE, 2000 / portTICK_PERIOD_MS);
    uart_write_bytes(uartport0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uartport0, rxbuffer, strlen((char *)rxbuffer));
    memset(rxbuffer, 0, sizeof(rxbuffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    retry_Count = 0;

    // get_time();

    vTaskDelay(1);
}
