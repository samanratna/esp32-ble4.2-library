#ifndef CREDENTIALS_H_
#define CREDENTIALS_H_

#include "main.h"

// Bike Credentials variables
extern char b_apiKey[60];                   
extern char EXAMPLE_DEVICE_NAME[20];
extern char bike_id[60];
extern char current_version[10];
extern char new_version[10];

extern char bikeVerifyUrl[];
extern char ApiKey[];
extern char crashReportUrl[];
extern char crashReportApiKey[];
extern char bikeLocationUrl[];
extern char bikeLocationApiKey[];
extern char mobileNumber[]; 
extern char b_socUrl[];
extern char b_chargeCompleteUrl[];
extern char b_chargerPluggedUrl[];
extern char b_chargerUnpluggedUrl[];
extern char b_telemetryUrl[];
extern char b_theftAlertUrl[];
extern char b_batteryLogUrl[];

typedef struct {
	char b_software_version[10];
	char b_apiKey[60];
	char b_bike_uuid[60];
	char b_device_name[20];
} OTA_EspVehicleCredential_Handler;         // Structure to Receive Bike Credentials in defined Sequence

extern OTA_EspVehicleCredential_Handler bike_credentials;
void read_bike_credentials(uint32_t id);
void get_bike_credentials(void);

#endif