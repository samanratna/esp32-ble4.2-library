#include "credentials.h"

bool bike_start;        // To ack the data receival
uint64_t previous_time; // Variables for timer
uint64_t current_time;
uint8_t data_buf[8] = {0}; // can tramsit data buffer
char b_apiKey[60];         // Bike Credentials variables
char EXAMPLE_DEVICE_NAME[20];
char bike_id[60];
char current_version[10];
char new_version[10];

OTA_EspVehicleCredential_Handler bike_credentials;

//======================================BIKE CREDENTIALS FOR PRODUCTION SERVER========================================//

char bikeVerifyUrl[] = "http://api.embedded.yatrimotorcycles.com/api/v1/charge-stations/payments/verify-user";
// char b_apiKey[] = "key:d8b9c9670c9717be1f8c74c9e0eb83ec82020fd791c8d0ac";
char crashReportUrl[] = "http://api.embedded.yatrimotorcycles.com/api/v1/bikes/crash";
char crashReportApiKey[] = "key:1c599aff61d7395ef020ac3e6f5777b3d9f7f521fb47ddf2";
char bikeLocationUrl[] = "http://api.embedded.yatrimotorcycles.com/api/v1/bikes/update-location";
char bikeLocationApiKey[] = "key:af8c5e643481fcb4388e336b5621306737998dde08e3fc91";

char mobileNumber[] = "+9779818266299"; // Enter the Mobile Number you want to send to (EMERGENCY CONTACT ONLY)

// for bike
char b_socUrl[] = "http://api.embedded.yatrimotorcycles.com/api/v1/bikes/batteries/soc";
char b_chargeCompleteUrl[] = "http://api.embedded.yatrimotorcycles.com/api/v1/bikes/charge-complete";
char b_chargerPluggedUrl[] = "http://api.embedded.yatrimotorcycles.com/api/v1/bikes/charge-plugged";
char b_chargerUnpluggedUrl[] = "http://api.embedded.yatrimotorcycles.com/api/v1/bikes/charge-unplugged";
char b_telemetryUrl[] = "http://api.embedded.yatrimotorcycles.com/api/v1/bikes/telemetries";
char b_theftAlertUrl[] = "http://api.embedded.yatrimotorcycles.com/api/v1/bikes/theft-alert";
char b_batteryLogUrl[] = "http://api.embedded.yatrimotorcycles.com/api/v1/bikes/battery-log";

//=====================================================================================================================//

void read_bike_credentials(uint32_t id)
{
    if (id == bike_cred_ack)
    {
        // data_buf[0] = 0;
        printf("Can Received for bike Credentials\n");
        esp_rec_getData(&bike_credentials); // Receive bike credentials in structure
        // twaiTxData(bike_cred_receive_ack, data_buf, 1); // Send Data receive ack to VCU

        memcpy(current_version, bike_credentials.b_software_version, sizeof(bike_credentials.b_software_version));
        memcpy(b_apiKey, bike_credentials.b_apiKey, sizeof(bike_credentials.b_apiKey));
        memcpy(bike_id, bike_credentials.b_bike_uuid, sizeof(bike_credentials.b_bike_uuid));
        memcpy(BLE_DEVICE_NAME, bike_credentials.b_device_name, sizeof(bike_credentials.b_device_name));

        printf("Software Version = %s\n", current_version);
        printf("Bike API Key = %s\n", b_apiKey);
        printf("Bike UUID = %s\n", bike_id);
        printf("Device Name = %s\n", BLE_DEVICE_NAME);

        bike_start = true;
    }
}

void get_bike_credentials(void)
{
    data_buf[0] = 1;
    twaiTxData(Bike_cred_req, data_buf, 1); // Request Bike credentials to VCU
    previous_time = esp_timer_get_time();
    int count = 0;

    while (bike_start == false)
    {
        current_time = esp_timer_get_time();
        if ((current_time - previous_time) > WAIT_PERIOD)
        {
            if (count == 3)
            {
                printf("Failed to receive data upon 3rd retry\n");
                esp_restart();
            }
            twaiTxData(Bike_cred_req, data_buf, 1);
            previous_time = esp_timer_get_time();
            count++;
        }
    }

    bike_start = false;
}