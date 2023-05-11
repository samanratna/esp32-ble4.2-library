#include "main.h"
#include "can.h"

const int uart_port0 = UART_NUM_0;
const int uart_port2 = UART_NUM_2;

//=============================================================================================================//

char ATcommand[300];
float latitude;
float longitude;

bool flag_sim_task;

uint8_t rx_buffer[200] = {0};
uint8_t ATisOK = 0;
uint8_t count = 0;
uint8_t retryCount = 0;
uint8_t gps_count = 0;
uint8_t downloadOK = 0;
uint8_t battFlag = 0;
uint8_t gps_flag = 0;
uint8_t allowFlag = 0;
uint8_t allowFlag1 = 1;
uint8_t gps_count1 = 0;
uint8_t gps_count2 = 0;
uint8_t rxbuff[256];
uint8_t signalStrength;
float battVoltage;
float initial_latitude;
float initial_longitude;
float final_latitude;
float final_longitude;
float distance;

portTickType g_ticks_start = 0;
TickType_t g_ticks_end = 0;

//============DATA POST STRUCTURES FOR CAN (TWAI)============//
typedef struct
{
  uint8_t charger_status;
  uint8_t soc;
  uint8_t time2fullCharge;
} EspSimSocHandle;

typedef struct
{
  float overall_odo;
  uint8_t overall_max_speed;
  uint8_t overall_avg_speed;
  float total_energy;
  float suste_overall_odo;
  float thikka_overall_odo;
  float babbal_overall_odo;
} EspSimTeleHandle;

typedef struct
{
  float batt_soh;
  float batt_cycles;
  bool ac_charger_type;
  uint8_t start_charge_soc;
  uint8_t end_charge_soc;
  uint8_t odo_difference;
  float overall_distance;
} Esp_BatLog_Handle;

typedef struct
{
  uint8_t charger_unplug;
  uint8_t end_soc;
} ESP_ChargerUnPlug_Handle;

typedef struct
{
  float acc_x;
  float acc_y;
  float acc_z;
  char bike_name[20];
} ESP_Crash_Handle;

typedef struct
{
  uint8_t charger_plug;
} ESP_ChargerPlug_Handle;

typedef struct
{
  uint8_t full_charge;
} ESP_FullCharge_Handle;

typedef struct
{
  uint8_t theft_notification;
} ESP_Theft_Handle;

typedef struct
{
  uint8_t get_time;
} ESP_GetTime_Handle;

typedef struct
{
  uint8_t gps_post;
} ESP_PostGPSLocation_Handle;

//=======================SIM7600 POWER-UP SEQUENCE=========================//
void sim7600_powerup()
{
  uart_write_bytes(uart_port0, "POWERING UP\n", strlen("POWERING UP\n"));
  gpio_set_level(SIM_POWER_PIN, 1);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  gpio_set_level(SIM_POWER_PIN, 0);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  uart_write_bytes(uart_port0, "LEAVING POWERING UP\n", strlen("LEAVING POWERING UP\n"));
}

//=======================SIM7600 POWER-DOWN SEQUENCE=========================//
void sim7600_powerdown()
{
  ATisOK = 0;
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  uart_write_bytes(uart_port0, "POWERING DOWN\n", strlen("POWERING DOWN\n"));
  sprintf(ATcommand, "AT+CPOF\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  while (!ATisOK)
  {
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS); //
    if (strstr((char *)rx_buffer, "OK"))
    {
      ATisOK = 1;
      uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    }
    vTaskDelay(1);
  }
  // uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, "LEAVING POWERING DOWN\n", strlen("LEAVING POWERING DOWN\n"));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
}

//=======================SIM7600 INITIALIZATION TASK=========================//
void sim7600_init()
{
  ATisOK = 0;
  uart_write_bytes(uart_port0, "VERIFYING SIM MODULE STATUS\n", strlen("VERIFYING SIM MODULE STATUS\n"));
  while (!ATisOK)
  {
    sprintf(ATcommand, "AT\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    if (strstr((char *)rx_buffer, "OK"))
    {
      ATisOK = 1;
      count = 0;
      uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    }
    else
    {
      uart_write_bytes(uart_port0, "COUNTING\n", strlen("COUNTING\n"));
      count++;
      if (count == 5)
      {
        uart_write_bytes(uart_port0, "POWERING UP\n", strlen("POWERING UP\n"));
        sim7600_powerup();
      }
    }
    vTaskDelay(1);
  }
  uart_write_bytes(0, "VERIFIED\n", strlen("VERIFIED\n"));
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  ATisOK = 0;
  sprintf(ATcommand, "AT+CTZU=1\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  vTaskDelay(50 / portTICK_PERIOD_MS);
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  sprintf(ATcommand, "AT+CTZR=1\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  vTaskDelay(50 / portTICK_PERIOD_MS);
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(50 / portTICK_PERIOD_MS);

  // delete all sms from sim module storage //
  sprintf(ATcommand, "AT+CMGD=1,4\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  vTaskDelay(50 / portTICK_PERIOD_MS);
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));

  sprintf(ATcommand, "AT+UIMHOTSWAPON=1\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  vTaskDelay(50 / portTICK_PERIOD_MS);
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
}

//=======================INTERNAL BATTERY VOLTAGE CHECK=========================//
void batt_check()
{
  sprintf(ATcommand, "AT+CBC\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));

  char *data1[5] = {0};
  int i1 = 0;
  char delim1[] = " ";
  char *ptr1 = strtok((char *)rx_buffer, delim1);
  while (ptr1 != NULL)
  {
    data1[i1] = ptr1;
    ptr1 = strtok(NULL, delim1);
    i1++;
  }

  char *data2[5] = {0};
  int i2 = 0;
  char delim2[] = "V";
  char *ptr2 = strtok((char *)data1[1], delim2);
  while (ptr2 != NULL)
  {
    data2[i2] = ptr2;
    ptr2 = strtok(NULL, delim2);
    i2++;
  }
  printf("Current Battery Voltage:\n");
  uart_write_bytes(uart_port0, (uint8_t *)data2[0], strlen((char *)data2[0]));
  uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));
  battVoltage = atof(data2[0]);

  if (battVoltage < 3.6 && battFlag == 0)
  {
    twai_tx_data[0] = 1;
    twai_tx_data[1] = (int)battVoltage * 10;
    twaiTxData(esp_sim_voltage_can_id, twai_tx_data, 2);
    battFlag = 1;
  }
  if (battVoltage >= 3.6)
  {
    battFlag = 0;
  }
}

//=======================LTE SIGNAL STRENGTH TEST=========================//
void signal_strength_check()
{
  sprintf(ATcommand, "AT+CSQ\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));

  if (strstr((char *)rx_buffer, "+CSQ"))
  {
    if (strstr((char *)rx_buffer, "ERROR"))
    {
      printf("ERROR in reading signal strength for now\n");
    }

    else if (strstr((char *)rx_buffer, "CLOSED") || strstr((char *)rx_buffer, "CCHOPEN") || strstr((char *)rx_buffer, "CMGS"))
    {
      printf("CCH PEER CLOSED CASE!\n");
      char *data1[5] = {0};
      int i1 = 0;
      char delim1[] = " ";
      char *ptr1 = strtok((char *)rx_buffer, delim1);
      while (ptr1 != NULL)
      {
        data1[i1] = ptr1;
        ptr1 = strtok(NULL, delim1);
        i1++;
      }

      char *data2[5] = {0};
      int i2 = 0;
      char delim2[] = ",";
      char *ptr2 = strtok((char *)data1[2], delim2);
      while (ptr2 != NULL)
      {
        data2[i2] = ptr2;
        ptr2 = strtok(NULL, delim2);
        i2++;
      }
      printf("RSSI Signal Strength:\n");
      uart_write_bytes(uart_port0, (uint8_t *)data2[0], strlen((char *)data2[0]));
      uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));
      signalStrength = atoi(data2[0]);
    }

    else
    {
      char *data1[5] = {0};
      int i1 = 0;
      char delim1[] = " ";
      char *ptr1 = strtok((char *)rx_buffer, delim1);
      while (ptr1 != NULL)
      {
        data1[i1] = ptr1;
        ptr1 = strtok(NULL, delim1);
        i1++;
      }

      char *data2[5] = {0};
      int i2 = 0;
      char delim2[] = ",";
      char *ptr2 = strtok((char *)data1[1], delim2);
      while (ptr2 != NULL)
      {
        data2[i2] = ptr2;
        ptr2 = strtok(NULL, delim2);
        i2++;
      }
      printf("RSSI Signal Strength:\n");
      uart_write_bytes(uart_port0, (uint8_t *)data2[0], strlen((char *)data2[0]));
      uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));
      signalStrength = atoi(data2[0]);
    }
  }
  else
  {
    signalStrength = 0;
  }
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
}

//======================SMS TEST=========================//
void sms_task()
{
  sprintf(ATcommand, "AT+CMGF=1\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(100 / portTICK_PERIOD_MS);
  sprintf(ATcommand, "AT+CMGS=\"%s\"\r\n", mobileNumber);
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(100 / portTICK_PERIOD_MS);
  sprintf(ATcommand, "http://maps.google.com/maps?q=loc:27.7736,85.5536 %c", 0x1a);
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  uart_write_bytes(uart_port0, "SMS SENT\n", strlen("SMS SENT\n"));
}

//=======================GPS DISTANCE CALCULATION=========================//
void calc_distance(float lat2, float lat1, float lon2, float lon1) // (initial latitude, destination latitude, initial longitude, destination longitude)
{
  // Haversine formula
  float a = (sin((lat2 - lat1) * C) / 2) * (sin((lat2 - lat1) * C) / 2) + cos(lat1 * C) * cos(lat2 * C) * ((sin((lon2 - lon1) * C)) / 2) * ((sin((lon2 - lon1) * C)) / 2);
  distance = (2 * 6371 * atan2(sqrt(a), sqrt(1 - a)));
  printf("Calculated distance: %f Km\n", distance);
}

//=======================GPS LOCATION CHECK=========================//
void gps_location()
{
  printf("inside GPS Location check!\n");
  int flag = 0;
  sprintf(ATcommand, "AT+CGPSINFO\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));
  memset(ATcommand, 0, sizeof(ATcommand));
  if (strstr((char *)rx_buffer, "SMS") || strstr((char *)rx_buffer, "CCH_PEER_CLOSED"))
  {
    flag = 1;
  }
  if (strlen((char *)rx_buffer) > 80 && flag != 1) // only parse coordinates if the gps has received the coordinates and use accordingly
  {
    if (allowFlag == 1)
    {
      printf("GPS Signal Available to VCU for Display!\n");
      twai_tx_data[0] = 1;
      twaiTxData(esp_gps_signal_status, twai_tx_data, 1);
      gps_count1 = 0;
    }
    allowFlag = 0;
    allowFlag1 = 1;

    gps_count = 0;
    char *data1[30] = {0};
    int i1 = 0;
    char delim1[] = " ";
    char *ptr1 = strtok((char *)rx_buffer, delim1);
    while (ptr1 != NULL)
    {
      data1[i1] = ptr1;
      ptr1 = strtok(NULL, delim1);
      i1++;
    }
    uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));

    char *data2[30] = {0};
    int i2 = 0;
    char delim2[] = ",";
    char *ptr2 = strtok((char *)data1[1], delim2);
    while (ptr2 != NULL)
    {
      data2[i2] = ptr2;
      ptr2 = strtok(NULL, delim2);
      i2++;
    }

    float lat1;
    lat1 = atof(data2[0]) / 100;

    float lon1;
    lon1 = atof(data2[2]) / 100;

    printf("converted lat1: %f,  long1: %f\n", lat1, lon1);
    uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));

    latitude = (int)lat1 + (((lat1 - (int)lat1) * 100) / 60); // final latitude and longitude are saved in Latitude and Longitude global float variables
    longitude = (int)lon1 + (((lon1 - (int)lon1) * 100) / 60);

    printf("Latitide: %f,  longitude: %f\n", latitude, longitude);
    uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));

    // http_post_location_data();
    if (gps_flag == 0)
    {
      initial_latitude = latitude;
      initial_longitude = longitude;
      final_latitude = latitude;
      final_longitude = longitude;

      // tell VCU GPS post started via CAN!
      twai_tx_data[0] = 1;
      twaiTxData(esp_gps_post_start_can_id, twai_tx_data, 1);
      http_post_location_data();
      gps_flag = 1;
    }
    else
    {
      final_latitude = latitude;
      final_longitude = longitude;
    }
    calc_distance(initial_latitude, final_latitude, initial_longitude, final_longitude);
    if (distance >= 0.15)
    {
      // tell VCU GPS post started via CAN!
      twai_tx_data[0] = 1;
      twaiTxData(esp_gps_post_start_can_id, twai_tx_data, 1);
      http_post_location_data();
      gps_flag = 0;
    }
  }
  else
  {
    if (allowFlag1 == 1)
    {
      printf("No GPS Signal to VCU for Display!\n");
      twai_tx_data[0] = 0;
      twaiTxData(esp_gps_signal_status, twai_tx_data, 1);
      gps_count2 = 0;
    }
    allowFlag1 = 0;
    allowFlag = 1;

    gps_count++;
    if (gps_count == 120) // value of count to re-initialize gps after a while if no gps signal
    {
      gps_init();
      gps_count = 0;
    }
    printf("No GPS Signal Yet!\n");
  }
  memset(rx_buffer, 0, sizeof(rx_buffer));
  vTaskDelay(1);
}

//=======================GPS LOCATION POST=========================//
void gps_location_post()
{
  printf("INISDE GPS LOCATION\n");

  int flag = 0;
  sprintf(ATcommand, "AT+CGPSINFO\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));
  memset(ATcommand, 0, sizeof(ATcommand));
  if (strstr((char *)rx_buffer, "SMS") || strstr((char *)rx_buffer, "CCH_PEER_CLOSED"))
  {
    flag = 1;
  }
  if (strlen((char *)rx_buffer) > 80 && flag != 1) // only parse coordinates if the gps has received the coordinates and use accordingly
  {
    char *data1[30] = {0};
    int i1 = 0;
    char delim1[] = " ";
    char *ptr1 = strtok((char *)rx_buffer, delim1);
    while (ptr1 != NULL)
    {
      data1[i1] = ptr1;
      ptr1 = strtok(NULL, delim1);
      i1++;
    }
    uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));

    char *data2[30] = {0};
    int i2 = 0;
    char delim2[] = ",";
    char *ptr2 = strtok((char *)data1[1], delim2);
    while (ptr2 != NULL)
    {
      data2[i2] = ptr2;
      ptr2 = strtok(NULL, delim2);
      i2++;
    }

    float lat1;
    lat1 = atof(data2[0]) / 100;

    float lon1;
    lon1 = atof(data2[2]) / 100;

    printf("converted lat1: %f,  long1: %f\n", lat1, lon1);
    uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));

    latitude = (int)lat1 + (((lat1 - (int)lat1) * 100) / 60); // final latitude and longitude are saved in Latitude and Longitude global float variables
    longitude = (int)lon1 + (((lon1 - (int)lon1) * 100) / 60);

    printf("Latitide: %f,  longitude: %f\n", latitude, longitude);
    uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));
    http_post_location_data();
  }
  else
  {
    printf("No GPS Signal!\n");
    twai_tx_data[0] = 8;
    twai_tx_data[1] = 4;
    twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
  }
  memset(rx_buffer, 0, sizeof(rx_buffer));
  vTaskDelay(1);
}

//=======================SMS ADXL VALUE TO  TASK=========================//
void sms_adxl_value(char xVal[5], char yVal[5], char zVal[5])
{
  sprintf(ATcommand, "AT+CGPSINFO\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));
  memset(ATcommand, 0, sizeof(ATcommand));

  if (strlen((char *)rx_buffer) > 80) // only parse coordinates if the gps has received the coordinates and use accordingly
  {
    char *data1[30] = {0};
    int i1 = 0;
    char delim1[] = " ";
    char *ptr1 = strtok((char *)rx_buffer, delim1);
    while (ptr1 != NULL)
    {
      data1[i1] = ptr1;
      ptr1 = strtok(NULL, delim1);
      i1++;
    }
    uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));

    char *data2[30] = {0};
    int i2 = 0;
    char delim2[] = ",";
    char *ptr2 = strtok((char *)data1[1], delim2);
    while (ptr2 != NULL)
    {
      data2[i2] = ptr2;
      ptr2 = strtok(NULL, delim2);
      i2++;
    }
    uart_write_bytes(uart_port0, (uint8_t *)data2[0], strlen((char *)data2[0]));
    uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));
    uart_write_bytes(uart_port0, (uint8_t *)data2[2], strlen((char *)data2[2]));
    uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));

    float long1, lat1;
    lat1 = atof(data2[0]) / 100;
    long1 = atof(data2[2]) / 100;

    printf("converted lat1: %f,  long1: %f\n", lat1, long1);
    uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));

    latitude = (int)lat1 + (((lat1 - (int)lat1) * 100) / 60); // final latitude and longitude are saved in Latitude and Longitude global float variables
    longitude = (int)long1 + (((long1 - (int)long1) * 100) / 60);

    printf("Latitide: %f,  longitude: %f\n", latitude, longitude);
    uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));

    sprintf(ATcommand, "AT+CMGF=1\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    sprintf(ATcommand, "AT+CMGS=\"%s\"\r\n", mobileNumber);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    sprintf(ATcommand, "SUSPECTING ACCIDENT OR UNUSUAL ORIENTATION\nX-Val: %sG\nY-Val: %sG\nZ-Val: %sG\nhttp://maps.google.com/maps?q=loc:%f,%f%c", xVal, yVal, zVal, latitude, longitude, 0x1a);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    uart_write_bytes(uart_port0, "SMS SENT\n", strlen("SMS SENT\n"));
    uart_write_bytes(uart_port0, "POSTING CRASH COORDINATES\n", strlen("POSTING CRASH COORDINATES\n"));

    http_post_CR_data();
  }

  else
  {
    sprintf(ATcommand, "AT+CMGF=1\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    vTaskDelay(100 / portTICK_PERIOD_MS);
    sprintf(ATcommand, "AT+CMGS=\"%s\"\r\n", mobileNumber);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    sprintf(ATcommand, "SUSPECTING ACCIDENT OR UNUSUAL ORIENTATION FOR : %s \n\nX-Val: %sG\nY-Val: %sG\nZ-Val: %sG%c", g_bikeName, xVal, yVal, zVal, 0x1a);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    uart_write_bytes(uart_port0, "SMS SENT\n", strlen("SMS SENT\n"));

    twai_transmit(&tx_msg, 10); // success via can
  }
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
}

//=======================SMS INITIALIZATION TASK=========================//
void sms_init()
{
  sprintf(ATcommand, "AT+CSMP=17,167,0,0\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(50 / portTICK_PERIOD_MS);
  sprintf(ATcommand, "AT+CMGF=1\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  uart_write_bytes(uart_port0, "SMS INITIALIZED\n", strlen("SMS INITIALIZED\n"));
}

//=======================GPS INITIALIZATION TASK=========================//
void gps_init()
{
  sprintf(ATcommand, "AT+CGPS=0\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(50 / portTICK_PERIOD_MS);

  sprintf(ATcommand, "AT+CGPS=1,1\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(50 / portTICK_PERIOD_MS);
  uart_write_bytes(uart_port0, "GPS INITIALIZED\n", strlen("GPS INITIALIZED\n"));
}

//=======================DATA AND TIME=========================//
void get_time()
{
  uart_write_bytes(uart_port0, "INSIDE GET TIME\n", strlen("INSIDE GET TIME\n"));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(50 / portTICK_PERIOD_MS);
  sprintf(ATcommand, "AT+CCLK?\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  char *_data1[5] = {0};
  int _i1 = 0;
  char _delim1[] = ",";
  char *_ptr1 = strtok((char *)rx_buffer, _delim1);
  while (_ptr1 != NULL)
  {
    _data1[_i1] = _ptr1;
    _ptr1 = strtok(NULL, _delim1);
    _i1++;
  }

  uart_write_bytes(uart_port0, (uint8_t *)_data1[0], strlen((char *)_data1[0]));
  uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));
  uart_write_bytes(uart_port0, (uint8_t *)_data1[1], strlen((char *)_data1[1]));
  uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));

  uart_write_bytes(uart_port0, "PARSING DATE\n", strlen("PARSING DATE\n"));
  char *_data2[5] = {0};
  int _i2 = 0;
  char _delim2[] = "\"";
  char *_ptr2 = strtok((char *)_data1[0], _delim2);
  while (_ptr2 != NULL)
  {
    _data2[_i2] = _ptr2;
    _ptr2 = strtok(NULL, _delim2);
    _i2++;
  }
  uart_write_bytes(uart_port0, (uint8_t *)_data2[1], strlen((char *)_data2[1]));
  uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));

  char *_data3[5] = {0};
  int _i3 = 0;
  char _delim3[] = "/";
  char *_ptr3 = strtok((char *)_data2[1], _delim3);
  while (_ptr3 != NULL)
  {
    _data3[_i3] = _ptr3;
    _ptr3 = strtok(NULL, _delim3);
    _i3++;
  }
  printf("Date: \n");
  uart_write_bytes(uart_port0, (uint8_t *)_data3[0], strlen((char *)_data3[0]));
  uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));
  uart_write_bytes(uart_port0, (uint8_t *)_data3[1], strlen((char *)_data3[1]));
  uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));
  uart_write_bytes(uart_port0, (uint8_t *)_data3[2], strlen((char *)_data3[2]));
  uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));

  uart_write_bytes(uart_port0, "PARSING TIME\n", strlen("PARSING TIME\n"));
  char *_data4[5] = {0};
  int _i4 = 0;
  char _delim4[] = "+";
  char *_ptr4 = strtok((char *)_data1[1], _delim4);
  while (_ptr4 != NULL)
  {
    _data4[_i4] = _ptr4;
    _ptr4 = strtok(NULL, _delim4);
    _i4++;
  }
  uart_write_bytes(uart_port0, (uint8_t *)_data4[0], strlen((char *)_data4[0]));
  uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));

  uart_write_bytes(uart_port0, "PARSING TIME AND SENDING TO F4 VIA CAN\n", strlen("PARSING TIME AND SENDING TO F4 VIA CAN\n"));
  char *_data5[5] = {0};
  int _i5 = 0;
  char _delim5[] = ":";
  char *_ptr5 = strtok((char *)_data4[0], _delim5);
  while (_ptr5 != NULL)
  {
    _data5[_i5] = _ptr5;
    _ptr5 = strtok(NULL, _delim5);
    _i5++;
  }
  uart_write_bytes(uart_port0, (uint8_t *)_data5[0], strlen((char *)_data5[0]));
  uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));
  uart_write_bytes(uart_port0, (uint8_t *)_data5[1], strlen((char *)_data5[1]));
  uart_write_bytes(uart_port0, (uint8_t *)"\n", strlen("\n"));

  //==============sending time to VCU via CAN=================//

  // esp_vcu_addTask(0, vcu_data_time_send);
  vcu_data_time_send(atoi(_data5[0]), atoi(_data5[1]), atoi(_data5[2]), atoi(_data3[2]), atoi(_data3[1]), atoi(_data3[0]));

  //=========================================================//

  memset(ATcommand, 0, sizeof(ATcommand));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  vTaskDelay(50 / portTICK_PERIOD_MS);
}

//=======================SIM PHONE NUMBER CHECK=========================//
void pNumber_check()
{
  sprintf(ATcommand, "AT+CREG?\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(50 / portTICK_PERIOD_MS);

  sprintf(ATcommand, "AT+COPS?\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(50 / portTICK_PERIOD_MS);

  sprintf(ATcommand, "AT+CUSD=1\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(50 / portTICK_PERIOD_MS);

  sprintf(ATcommand, "AT+CUSD=1,\"*903#\"\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 5000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 5000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
}

//=======================SIM BALANCE CHECK=========================//
void balance_check()
{
  sprintf(ATcommand, "AT+CREG?\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(50 / portTICK_PERIOD_MS);

  sprintf(ATcommand, "AT+COPS?\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(50 / portTICK_PERIOD_MS);

  sprintf(ATcommand, "AT+CUSD=1\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(50 / portTICK_PERIOD_MS);

  sprintf(ATcommand, "AT+CUSD=1,\"*901#\"\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 5000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 5000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));

  if (strstr((char *)rx_buffer, "ERROR") || strstr((char *)rx_buffer, "unavailable"))
  {
    printf("NETWORK PROBLEM, TRY LATER\n");
  }

  else
  {
    char *_data1[5] = {0};
    int _i1 = 0;
    char _delim1[] = ".";
    char *_ptr1 = strtok((char *)rx_buffer, _delim1);
    while (_ptr1 != NULL)
    {
      _data1[_i1] = _ptr1;
      _ptr1 = strtok(NULL, _delim1);
      _i1++;
    }
    char balance[5];
    sprintf(balance, "%s.%s", _data1[1], _data1[2]);
    g_balance = atof(balance);
    printf("Balance: %0.2f", g_balance);
    printf("\n");
  }
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
}

//=======================SSL COFIGURATION AND LTE ACTIVATION TASK=========================//
void ssl_init()
{
  // activating LTE service and then initializing ssl configurations
  sprintf(ATcommand, "AT+CNMP=2\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(50 / portTICK_PERIOD_MS);

  sprintf(ATcommand, "AT+CSSLCFG=\"sslversion\",0,4\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(50 / portTICK_PERIOD_MS);

  sprintf(ATcommand, "AT+CSSLCFG=\"authmode\",0,0\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(50 / portTICK_PERIOD_MS);

  sprintf(ATcommand, "AT+CCHSET=1\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(50 / portTICK_PERIOD_MS);

  sprintf(ATcommand, "AT+CCHSTART\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(50 / portTICK_PERIOD_MS);

  sprintf(ATcommand, "AT+CCHOPEN=0,\"www.yatrimotorcycles.com\",443,2\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
  // uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  // memset(rx_buffer, 0, sizeof(rx_buffer));
  // uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(50 / portTICK_PERIOD_MS);
}

void sim7600_reset()
{
  ATisOK = 0;
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  sprintf(ATcommand, "AT+CRESET\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  while (!ATisOK)
  {
    sprintf(ATcommand, "AT\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    if (strstr((char *)rx_buffer, "OK"))
    {
      ATisOK = 1;
      count = 0;
      uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    }
  }
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  sprintf(ATcommand, "AT+CMGD=1,4\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  vTaskDelay(50 / portTICK_PERIOD_MS);
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  ssl_init();
}

//=======================USER CRASH-REPORT POST=========================//
void http_post_CR_data()
{
  int successFlag = 0;
retry:

  uart_write_bytes(uart_port0, "INSIDE CRASH REPORT POST TASK\n", strlen("INSIDE CRASH REPORT POST TASK\n"));
  downloadOK = 0;
  successFlag = 0;
  g_ticks_start = 0;
  g_ticks_end = 0;
  sprintf(ATcommand, "AT+HTTPTERM\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));

  sprintf(ATcommand, "AT+HTTPINIT\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));

  sprintf(ATcommand, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", crashReportUrl);
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));

  sprintf(ATcommand, "AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));

  sprintf(ATcommand, "AT+HTTPPARA=\"USERDATA\",\"%s\"\r\n", b_apiKey);
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));

  sprintf(ATcommand, "{\"x\":%s,\"y\":%s,\"z\":%s,\"location\":{\"coordinates\":[\"%f\",\"%f\"]}}%c", g_xVal, g_yVal, g_zVal, longitude, latitude, 0x0A);
  uint8_t len = strlen(ATcommand);
  memset(ATcommand, 0, sizeof(ATcommand));

  sprintf(ATcommand, "AT+HTTPDATA=%d,10000\r\n", len);
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));

  sprintf(ATcommand, "{\"x\":%s,\"y\":%s,\"z\":%s,\"location\":{\"coordinates\":[\"%f\",\"%f\"]}}%c", g_xVal, g_yVal, g_zVal, longitude, latitude, 0x0A);
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, "INSIDE WHILE\n", strlen("INSIDE WHILE\n"));

  g_ticks_start = xTaskGetTickCount();
  while (!downloadOK)
  {
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    if (strstr((char *)rx_buffer, "OK"))
    {
      downloadOK = 1;
    }
    g_ticks_end = xTaskGetTickCount();
    if (g_ticks_end - g_ticks_start >= 5000 / portTICK_PERIOD_MS)
    {
      printf("WAIT TIMEOUT CONDITION\n");
      downloadOK = 1;
      g_ticks_start = 0;
      g_ticks_end = 0;

      // send CAN message to VCU for reset ack!
      twai_tx_data[0] = 1; // 1 for about to reset and 0 for module restarted
      twaiTxData(esp_gps_post_start_can_id, twai_tx_data, 1);

      sim7600_reset();

      // send CAN message to VCU for reset ack!
      twai_tx_data[0] = 0; // 1 for about to reset and 0 for module restarted
      twaiTxData(esp_gps_post_start_can_id, twai_tx_data, 1);
      return;
    }
  }
  g_ticks_start = 0;
  g_ticks_end = 0;

  uart_write_bytes(uart_port0, "OUTSIDE WHILE\n", strlen("OUTSIDE WHILE\n"));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));

  sprintf(ATcommand, "AT+HTTPACTION=1\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 10000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS);
  // uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  if (strstr((char *)rx_buffer, "+CCH"))
  {
    memset(rx_buffer, 0, sizeof(rx_buffer));
    uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  }
  if (strstr((char *)rx_buffer, "200"))
  {
    uart_write_bytes(uart_port0, "SUCCESSFULLY POSTED\n", strlen("SUCCESSFULLY POSTED\n"));
    successFlag = 1;
  }
  else if (strstr((char *)rx_buffer, "302"))
  {
    uart_write_bytes(uart_port0, "INSUFFICIENT SIM BALANCE\n", strlen("INSUFFICIENT SIM BALANCE\n"));
    twai_tx_data[0] = 9;
    twai_tx_data[1] = 3;
    twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
  }
  else
  {
    if (retryCount < 1)
    {
      retryCount++;
      uart_write_bytes(uart_port0, "POSTING FAILED, RETRY\n", strlen("POSTING FAILED, RETRY\n"));
      goto retry;
    }
    else
    {
      uart_write_bytes(uart_port0, "RETRY FAILED, RESUMING TASK\n", strlen("RETRY FAILED, RESUMING TASK\n"));
      retryCount = 0;
      successFlag = 0;
    }
  }

  // sprintf(ATcommand, "AT+HTTPREAD=0,200\r\n");
  // uart_write_bytes(uart_port2, ATcommand, strlen((char*)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 5000 / portTICK_PERIOD_MS);
  // uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  // uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  // memset(rx_buffer, 0, sizeof(rx_buffer));
  // memset(ATcommand, 0, sizeof(ATcommand));

  sprintf(ATcommand, "AT+HTTPTERM\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));

  if (successFlag == 1) // post success
  {
    twai_tx_data[0] = 9;
    twai_tx_data[1] = 1;
    twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
  }
  else if (successFlag == 0) // post failed
  {
    twai_tx_data[0] = 9;
    twai_tx_data[1] = 2;
    twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
  }
  vTaskDelay(1);
}

//=======================USER LOCATION POST=========================//
void http_post_location_data()
{
retry:
  // check if network is available or not before posting and if not available, do not post //
  signal_strength_check();
  if (signalStrength > 10)
  {
    int successFlag = 0;
    g_ticks_start = 0;
    g_ticks_end = 0;
    uart_write_bytes(uart_port0, "INSIDE LOCATION POST TASK\n", strlen("INSIDE LOCATION POST TASK\n"));
    downloadOK = 0;
    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPINIT\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", bikeLocationUrl);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"USERDATA\",\"%s\"\r\n", b_apiKey);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "{\"location\":{\"coordinates\":[\"%f\",\"%f\"]}}%c", longitude, latitude, 0x0A);
    uint8_t len = strlen(ATcommand);
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPDATA=%d,10000\r\n", len);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "{\"location\":{\"coordinates\":[\"%f\",\"%f\"]}}%c", longitude, latitude, 0x0A);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, "INSIDE WHILE\n", strlen("INSIDE WHILE\n"));

    g_ticks_start = xTaskGetTickCount();
    while (!downloadOK)
    {
      // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS);
      uart_read(rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS); //
      uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
      if (strstr((char *)rx_buffer, "OK"))
      {
        downloadOK = 1;
      }
      g_ticks_end = xTaskGetTickCount();
      if (g_ticks_end - g_ticks_start >= 5000 / portTICK_PERIOD_MS)
      {
        printf("WAIT TIMEOUT CONDITION\n");
        downloadOK = 1;
        g_ticks_start = 0;
        g_ticks_end = 0;

        // send CAN message to VCU for reset ack!
        twai_tx_data[0] = 1; // 1 for about to reset and 0 for module restarted
        twaiTxData(esp_gps_post_start_can_id, twai_tx_data, 1);

        sim7600_reset();

        // send CAN message to VCU for reset ack!
        twai_tx_data[0] = 0; // 1 for about to reset and 0 for module restarted
        twaiTxData(esp_gps_post_start_can_id, twai_tx_data, 1);
        return;
      }
    }
    g_ticks_start = 0;
    g_ticks_end = 0;

    // vTaskDelay(1);
    uart_write_bytes(uart_port0, "OUTSIDE WHILE\n", strlen("OUTSIDE WHILE\n"));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPACTION=1\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 10000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS);
    // uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    if (strstr((char *)rx_buffer, "+CCH"))
    {
      memset(rx_buffer, 0, sizeof(rx_buffer));
      uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
      uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    }
    if (strstr((char *)rx_buffer, "200"))
    {
      uart_write_bytes(uart_port0, "SUCCESSFULLY POSTED\n", strlen("SUCCESSFULLY POSTED\n"));
      successFlag = 1;
    }
    else if (strstr((char *)rx_buffer, "302"))
    {
      uart_write_bytes(uart_port0, "INSUFFICIENT SIM BALANCE\n", strlen("INSUFFICIENT SIM BALANCE\n"));
      twai_tx_data[0] = 8;
      twai_tx_data[1] = 3;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
    else
    {
      if (retryCount < 1)
      {
        retryCount++;
        uart_write_bytes(uart_port0, "POSTING FAILED, RETRY\n", strlen("POSTING FAILED, RETRY\n"));
        goto retry;
      }
      else
      {
        uart_write_bytes(uart_port0, "RETRY FAILED, RESUMING TASK\n", strlen("RETRY FAILED, RESUMING TASK\n"));
        retryCount = 0;
        successFlag = 0;
      }
    }

    // sprintf(ATcommand, "AT+HTTPREAD=0,200\r\n");
    // uart_write_bytes(uart_port2, ATcommand, strlen((char*)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 5000 / portTICK_PERIOD_MS);
    // uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    // uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    // memset(rx_buffer, 0, sizeof(rx_buffer));
    // memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    if (successFlag == 1) // post success
    {
      twai_tx_data[0] = 8;
      twai_tx_data[1] = 1;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
    else if (successFlag == 0) // post failed
    {
      twai_tx_data[0] = 8;
      twai_tx_data[1] = 2;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
  }

  // CAN to VCU informing GPS Post Finish!
  twai_tx_data[0] = 0;
  twaiTxData(esp_gps_post_start_can_id, twai_tx_data, 1);
  vTaskDelay(1);
}

//=======================BIKE ID VERIFICATION POST=========================//
void http_post_BV_data(char _bikeId[50])
{
retry:
  uart_write_bytes(uart_port0, "INSIDE BIKE VERIFICATION\n", strlen("INSIDE BIKE VERIFICATION\n"));
  downloadOK = 0;
  g_ticks_start = 0;
  g_ticks_end = 0;
  sprintf(ATcommand, "AT+HTTPTERM\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));

  sprintf(ATcommand, "AT+HTTPINIT\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));

  sprintf(ATcommand, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", bikeVerifyUrl);
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));

  sprintf(ATcommand, "AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));

  sprintf(ATcommand, "AT+HTTPPARA=\"USERDATA\",\"%s\"\r\n", ApiKey);
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));

  sprintf(ATcommand, "{\"bikeUUID\":\"%s\"}%c", _bikeId, 0x0A);
  uint8_t len = strlen(ATcommand);
  memset(ATcommand, 0, sizeof(ATcommand));

  sprintf(ATcommand, "AT+HTTPDATA=%d,10000\r\n", len);
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));

  sprintf(ATcommand, "{\"bikeUUID\":\"%s\"}%c", _bikeId, 0x0A);
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, "INSIDE WHILE\n", strlen("INSIDE WHILE\n"));

  g_ticks_start = xTaskGetTickCount();
  while (!downloadOK)
  {
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    if (strstr((char *)rx_buffer, "OK"))
    {
      downloadOK = 1;
    }
    g_ticks_end = xTaskGetTickCount();
    if (g_ticks_end - g_ticks_start >= 5000 / portTICK_PERIOD_MS)
    {
      printf("WAIT TIMEOUT CONDITION\n");
      downloadOK = 1;
      g_ticks_start = 0;
      g_ticks_end = 0;

      // send CAN message to VCU for reset ack!
      twai_tx_data[0] = 1; // 1 for about to reset and 0 for module restarted
      twaiTxData(esp_gps_post_start_can_id, twai_tx_data, 1);

      sim7600_reset();

      // send CAN message to VCU for reset ack!
      twai_tx_data[0] = 0; // 1 for about to reset and 0 for module restarted
      twaiTxData(esp_gps_post_start_can_id, twai_tx_data, 1);
      return;
    }
  }
  g_ticks_start = 0;
  g_ticks_end = 0;

  uart_write_bytes(uart_port0, "OUTSIDE WHILE\n", strlen("OUTSIDE WHILE\n"));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));

  sprintf(ATcommand, "AT+HTTPACTION=1\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 10000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS);
  // uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  if (strstr((char *)rx_buffer, "+CCH"))
  {
    memset(rx_buffer, 0, sizeof(rx_buffer));
    uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  }
  if (strstr((char *)rx_buffer, "200"))
  {
    uart_write_bytes(uart_port0, "SUCCESSFULLY POSTED\n", strlen("SUCCESSFULLY POSTED\n"));
  }
  else if (strstr((char *)rx_buffer, "302"))
  {
    uart_write_bytes(uart_port0, "INSUFFICIENT SIM BALANCE\n", strlen("INSUFFICIENT SIM BALANCE\n"));
    twai_tx_data[0] = 3;
    twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
  }
  else
  {
    if (retryCount < 1)
    {
      retryCount++;
      uart_write_bytes(uart_port0, "POSTING FAILED, RETRY\n", strlen("POSTING FAILED, RETRY\n"));
      goto retry;
    }
    else
    {
      uart_write_bytes(uart_port0, "RETRY FAILED, RESUMING TASK\n", strlen("RETRY FAILED, RESUMING TASK\n"));
      retryCount = 0;
    }
  }

  // sprintf(ATcommand, "AT+HTTPREAD=0,200\r\n");
  // uart_write_bytes(uart_port2, ATcommand, strlen((char*)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 5000 / portTICK_PERIOD_MS);
  // uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  // uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  // memset(rx_buffer, 0, sizeof(rx_buffer));
  // memset(ATcommand, 0, sizeof(ATcommand));

  sprintf(ATcommand, "AT+HTTPTERM\r\n");
  uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
  // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
  uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
  uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
  uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(ATcommand, 0, sizeof(ATcommand));
  retryCount = 0;
  vTaskDelay(1);
}

//=======================SOC, ETA AND CHARGING STATUS POST=========================//
void http_post_soc_data(char *_soc, char *_chargeStatus, char *_eta)
{
retry:
  // check if network is available or not before posting and if not available, do not post //
  signal_strength_check();
  if (signalStrength > 10)
  {
    int successFlag = 0;
    g_ticks_start = 0;
    g_ticks_end = 0;
    char eta1[40];
    sprintf(eta1, "%d Hours and %d Minutes", atoi(_eta) / 60, atoi(_eta) % 60);
    uart_write_bytes(uart_port0, "INSIDE SOC POST TASK\n", strlen("INSIDE SOC POST TASK\n"));
    memset(ATcommand, 0, sizeof(ATcommand));
    downloadOK = 0;
    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPINIT\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", b_socUrl);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"USERDATA\",\"%s\"\r\n", b_apiKey);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "{\"soc\":%s,\"isCharging\":%s,\"eta\":\"%s\"}%c", _soc, _chargeStatus, eta1, 0x0A);
    uint8_t len = strlen(ATcommand);
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPDATA=%d,10000\r\n", len);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "{\"soc\":%s,\"isCharging\":%s,\"eta\":\"%s\"}%c", _soc, _chargeStatus, eta1, 0x0A);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, "INSIDE WHILE\n", strlen("INSIDE WHILE\n"));

    g_ticks_start = xTaskGetTickCount();
    while (!downloadOK)
    {
      // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS);
      uart_read(rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS); //
      uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
      if (strstr((char *)rx_buffer, "OK"))
      {
        downloadOK = 1;
      }
      g_ticks_end = xTaskGetTickCount();
      if (g_ticks_end - g_ticks_start >= 5000 / portTICK_PERIOD_MS)
      {
        printf("WAIT TIMEOUT CONDITION\n");
        downloadOK = 1;
        g_ticks_start = 0;
        g_ticks_end = 0;

        // send CAN message to VCU for reset ack!
        twai_tx_data[0] = 1; // 1 for about to reset and 0 for module restarted
        twaiTxData(esp_gps_post_start_can_id, twai_tx_data, 1);

        sim7600_reset();

        // send CAN message to VCU for reset ack!
        twai_tx_data[0] = 0; // 1 for about to reset and 0 for module restarted
        twaiTxData(esp_gps_post_start_can_id, twai_tx_data, 1);
        return;
      }
    }
    g_ticks_start = 0;
    g_ticks_end = 0;

    uart_write_bytes(uart_port0, "OUTSIDE WHILE\n", strlen("OUTSIDE WHILE\n"));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPACTION=1\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 10000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS);
    // uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    if (strstr((char *)rx_buffer, "+CCH"))
    {
      memset(rx_buffer, 0, sizeof(rx_buffer));
      uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
      uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    }
    if (strstr((char *)rx_buffer, "200"))
    {
      uart_write_bytes(uart_port0, "SUCCESSFULLY POSTED\n", strlen("SUCCESSFULLY POSTED\n"));
      successFlag = 1;
    }
    else if (strstr((char *)rx_buffer, "302"))
    {
      uart_write_bytes(uart_port0, "INSUFFICIENT SIM BALANCE\n", strlen("INSUFFICIENT SIM BALANCE\n"));
      twai_tx_data[0] = 1;
      twai_tx_data[1] = 3;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
    else
    {
      if (retryCount < 1)
      {
        retryCount++;
        uart_write_bytes(uart_port0, "POSTING FAILED, RETRY\n", strlen("POSTING FAILED, RETRY\n"));
        goto retry;
      }
      else
      {
        uart_write_bytes(uart_port0, "RETRY FAILED, RESUMING TASK\n", strlen("RETRY FAILED, RESUMING TASK\n"));
        retryCount = 0;
        successFlag = 0;
      }
    }

    // memset(rx_buffer, 0, sizeof(rx_buffer));
    // memset(ATcommand, 0, sizeof(ATcommand));
    // sprintf(ATcommand, "AT+HTTPREAD=0,200\r\n");
    // uart_write_bytes(uart_port2, ATcommand, strlen((char*)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 5000 / portTICK_PERIOD_MS);
    // uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    // uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    // memset(rx_buffer, 0, sizeof(rx_buffer));
    // memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    retryCount = 0;

    if (successFlag == 1) // post success
    {
      twai_tx_data[0] = 1;
      twai_tx_data[1] = 1;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
    else if (successFlag == 0) // post failed
    {
      twai_tx_data[0] = 1;
      twai_tx_data[1] = 2;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
  }
  vTaskDelay(1);
}

//=======================BATTERY LOG POST=========================//
void http_post_battery_log(char *_soh, char *_lifeCycle, char *_chargeType, char *_startCharge, char *_endCharge, char *_currentODO, char *_ODOvariance)
{
retry:
  // check if network is available or not before posting and if not available, do not post //
  signal_strength_check();
  if (signalStrength > 10)
  {
    int successFlag = 0;
    g_ticks_start = 0;
    g_ticks_end = 0;
    uart_write_bytes(uart_port0, "INSIDE BATTERY LOG POST TASK\n", strlen("INSIDE BATTERY LOG POST TASK\n"));
    memset(ATcommand, 0, sizeof(ATcommand));
    downloadOK = 0;
    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPINIT\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", b_batteryLogUrl);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"USERDATA\",\"%s\"\r\n", b_apiKey);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "{\"soh\":%s,\"lifecycle\":%s,\"chargingType\":\"%s\",\"startingCharge\":%s,\"endingCharge\":%s,\"currentOdo\":%s,\"odoVariance\":%s}%c", _soh, _lifeCycle, _chargeType, _startCharge, _endCharge, _currentODO, _ODOvariance, 0x0A);
    uint8_t len = strlen(ATcommand);
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPDATA=%d,10000\r\n", len);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "{\"soh\":%s,\"lifecycle\":%s,\"chargingType\":\"%s\",\"startingCharge\":%s,\"endingCharge\":%s,\"currentOdo\":%s,\"odoVariance\":%s}%c", _soh, _lifeCycle, _chargeType, _startCharge, _endCharge, _currentODO, _ODOvariance, 0x0A);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, "INSIDE WHILE\n", strlen("INSIDE WHILE\n"));

    g_ticks_start = xTaskGetTickCount();
    while (!downloadOK)
    {
      // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS);
      uart_read(rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS); //
      uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
      if (strstr((char *)rx_buffer, "OK"))
      {
        downloadOK = 1;
      }
      g_ticks_end = xTaskGetTickCount();
      if (g_ticks_end - g_ticks_start >= 5000 / portTICK_PERIOD_MS)
      {
        printf("WAIT TIMEOUT CONDITION\n");
        downloadOK = 1;
        g_ticks_start = 0;
        g_ticks_end = 0;

        // send CAN message to VCU for reset ack!
        twai_tx_data[0] = 1; // 1 for about to reset and 0 for module restarted
        twaiTxData(esp_gps_post_start_can_id, twai_tx_data, 1);

        sim7600_reset();

        // send CAN message to VCU for reset ack!
        twai_tx_data[0] = 0; // 1 for about to reset and 0 for module restarted
        twaiTxData(esp_gps_post_start_can_id, twai_tx_data, 1);
        return;
      }
    }
    g_ticks_start = 0;
    g_ticks_end = 0;

    uart_write_bytes(uart_port0, "OUTSIDE WHILE\n", strlen("OUTSIDE WHILE\n"));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPACTION=1\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 10000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS);
    // uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    if (strstr((char *)rx_buffer, "+CCH"))
    {
      memset(rx_buffer, 0, sizeof(rx_buffer));
      uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
      uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    }
    if (strstr((char *)rx_buffer, "200"))
    {
      uart_write_bytes(uart_port0, "SUCCESSFULLY POSTED\n", strlen("SUCCESSFULLY POSTED\n"));
      successFlag = 1;
    }
    else if (strstr((char *)rx_buffer, "302"))
    {
      uart_write_bytes(uart_port0, "INSUFFICIENT SIM BALANCE\n", strlen("INSUFFICIENT SIM BALANCE\n"));
      twai_tx_data[0] = 2;
      twai_tx_data[1] = 3;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
    else
    {
      if (retryCount < 1)
      {
        retryCount++;
        uart_write_bytes(uart_port0, "POSTING FAILED, RETRY\n", strlen("POSTING FAILED, RETRY\n"));
        goto retry;
      }
      else
      {
        uart_write_bytes(uart_port0, "RETRY FAILED, RESUMING TASK\n", strlen("RETRY FAILED, RESUMING TASK\n"));
        retryCount = 0;
      }
    }

    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    // sprintf(ATcommand, "AT+HTTPREAD=0,200\r\n");
    // uart_write_bytes(uart_port2, ATcommand, strlen((char*)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 5000 / portTICK_PERIOD_MS);
    // uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    // uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    // memset(rx_buffer, 0, sizeof(rx_buffer));
    // memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    retryCount = 0;

    if (successFlag == 1) // post success
    {
      twai_tx_data[0] = 2;
      twai_tx_data[1] = 1;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
    else if (successFlag == 0) // post failed
    {
      twai_tx_data[0] = 2;
      twai_tx_data[1] = 2;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
  }

  vTaskDelay(1);
}

//=======================BIKE TELEMETRY POST=========================//
void http_post_tele_data(char _disTravelled[5], char _maxSpeed[5], char _avgSpeed[5], char _totalEnergy[10], char _odoSuste[10], char _odoThikka[10], char _odoBabbal[10])
{
retry:
  // check if network is available or not before posting and if not available, do not post //
  signal_strength_check();
  if (signalStrength > 10)
  {
    int successFlag = 0;
    g_ticks_start = 0;
    g_ticks_end = 0;
    uart_write_bytes(uart_port0, "POSTING TELEMETRY DATA\n", strlen("POSTING TELEMETRY DATA\n"));
    memset(ATcommand, 0, sizeof(ATcommand));
    downloadOK = 0;
    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPINIT\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", b_telemetryUrl);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"USERDATA\",\"%s\"\r\n", b_apiKey);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "{\"distanceTravelled\":%s,\"maxSpeed\":%s,\"avgSpeed\":%s,\"totalEUnit\":%s,\"susteOdo\":%s,\"thikkaOdo\":%s,\"babbalOdo\":%s}%c", _disTravelled, _maxSpeed, _avgSpeed, _totalEnergy, _odoSuste, _odoThikka, _odoBabbal, 0x0A);
    uint8_t len = strlen(ATcommand);
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPDATA=%d,10000\r\n", len);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "{\"distanceTravelled\":%s,\"maxSpeed\":%s,\"avgSpeed\":%s,\"totalEUnit\":%s,\"susteOdo\":%s,\"thikkaOdo\":%s,\"babbalOdo\":%s}%c", _disTravelled, _maxSpeed, _avgSpeed, _totalEnergy, _odoSuste, _odoThikka, _odoBabbal, 0x0A);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, "INSIDE WHILE\n", strlen("INSIDE WHILE\n"));

    g_ticks_start = xTaskGetTickCount();
    while (!downloadOK)
    {
      // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS);
      uart_read(rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS); //
      uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
      if (strstr((char *)rx_buffer, "OK"))
      {
        downloadOK = 1;
      }
      g_ticks_end = xTaskGetTickCount();
      if (g_ticks_end - g_ticks_start >= 5000 / portTICK_PERIOD_MS)
      {
        printf("WAIT TIMEOUT CONDITION\n");
        downloadOK = 1;
        g_ticks_start = 0;
        g_ticks_end = 0;

        // send CAN message to VCU for reset ack!
        twai_tx_data[0] = 1; // 1 for about to reset and 0 for module restarted
        twaiTxData(esp_gps_post_start_can_id, twai_tx_data, 1);

        sim7600_reset();

        // send CAN message to VCU for reset ack!
        twai_tx_data[0] = 0; // 1 for about to reset and 0 for module restarted
        twaiTxData(esp_gps_post_start_can_id, twai_tx_data, 1);
        return;
      }
    }
    g_ticks_start = 0;
    g_ticks_end = 0;

    uart_write_bytes(uart_port0, "OUTSIDE WHILE\n", strlen("OUTSIDE WHILE\n"));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPACTION=1\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 10000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS);
    // uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    if (strstr((char *)rx_buffer, "+CCH"))
    {
      memset(rx_buffer, 0, sizeof(rx_buffer));
      uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
      uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    }
    if (strstr((char *)rx_buffer, "200"))
    {
      uart_write_bytes(uart_port0, "SUCCESSFULLY POSTED\n", strlen("SUCCESSFULLY POSTED\n"));
      successFlag = 1;
    }
    else if (strstr((char *)rx_buffer, "302"))
    {
      uart_write_bytes(uart_port0, "INSUFFICIENT SIM BALANCE\n", strlen("INSUFFICIENT SIM BALANCE\n"));
      twai_tx_data[0] = 3;
      twai_tx_data[1] = 3;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
    else
    {
      if (retryCount < 1)
      {
        retryCount++;
        uart_write_bytes(uart_port0, "POSTING FAILED, RETRY\n", strlen("POSTING FAILED, RETRY\n"));
        goto retry;
      }
      else
      {
        uart_write_bytes(uart_port0, "RETRY FAILED, RESUMING TASK\n", strlen("RETRY FAILED, RESUMING TASK\n"));
        retryCount = 0;
        successFlag = 0;
      }
    }

    // sprintf(ATcommand, "AT+HTTPREAD=0,200\r\n");
    // uart_write_bytes(uart_port2, ATcommand, strlen((char*)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 5000 / portTICK_PERIOD_MS);
    // uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    // uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    // memset(rx_buffer, 0, sizeof(rx_buffer));
    // memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    retryCount = 0;

    if (successFlag == 1) // post success
    {
      twai_tx_data[0] = 3;
      twai_tx_data[1] = 1;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
    else if (successFlag == 0) // post failed
    {
      twai_tx_data[0] = 3;
      twai_tx_data[1] = 2;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
  }

  vTaskDelay(1);
}

//=======================CHARGE COMPLETION (100%) NOTIFICATION POST=========================//
void http_post_charge_completion()
{
retry:
  // check if network is available or not before posting and if not available, do not post //
  signal_strength_check();
  if (signalStrength > 10)
  {
    int successFlag = 0;
    g_ticks_start = 0;
    g_ticks_end = 0;
    uart_write_bytes(uart_port0, "INSIDE CHARGE COMPLETION NOTIFICATION\n", strlen("INSIDE CHARGE COMPLETION NOTIFICATION\n"));
    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPINIT\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", b_chargeCompleteUrl);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"USERDATA\",\"%s\"\r\n", b_apiKey);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPACTION=1\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 10000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS);
    memset(rx_buffer, 0, sizeof(rx_buffer));
    uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    if (strstr((char *)rx_buffer, "+CCH"))
    {
      memset(rx_buffer, 0, sizeof(rx_buffer));
      uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
      uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    }
    if (strstr((char *)rx_buffer, "200"))
    {
      uart_write_bytes(uart_port0, "SUCCESSFULLY POSTED\n", strlen("SUCCESSFULLY POSTED\n"));
      successFlag = 1;
    }
    else if (strstr((char *)rx_buffer, "302"))
    {
      uart_write_bytes(uart_port0, "INSUFFICIENT SIM BALANCE\n", strlen("INSUFFICIENT SIM BALANCE\n"));
      twai_tx_data[0] = 4;
      twai_tx_data[1] = 3;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
    else
    {
      if (retryCount < 1)
      {
        retryCount++;
        uart_write_bytes(uart_port0, "POSTING FAILED, RETRY\n", strlen("POSTING FAILED, RETRY\n"));
        goto retry;
      }
      else
      {
        uart_write_bytes(uart_port0, "RETRY FAILED, RESUMING TASK\n", strlen("RETRY FAILED, RESUMING TASK\n"));
        retryCount = 0;
        successFlag = 0;
      }
    }

    // sprintf(ATcommand, "AT+HTTPREAD=0,200\r\n");
    // uart_write_bytes(uart_port2, ATcommand, strlen((char*)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 5000 / portTICK_PERIOD_MS);
    // uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    // uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    // memset(rx_buffer, 0, sizeof(rx_buffer));
    // memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    retryCount = 0;

    if (successFlag == 1) // post success
    {
      twai_tx_data[0] = 4;
      twai_tx_data[1] = 1;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
    else if (successFlag == 0) // post failed
    {
      twai_tx_data[0] = 4;
      twai_tx_data[1] = 2;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
  }
  vTaskDelay(1);
}

//=======================CHARGER UNPLUGGED NOTIFICATION POST=========================//
void http_post_charge_unplugged(char *_soc)
{
  memset(ATcommand, 0, sizeof(ATcommand));
  downloadOK = 0;
retry:
  // check if network is available or not before posting and if not available, do not post //
  signal_strength_check();
  if (signalStrength > 10)
  {
    int successFlag = 0;
    g_ticks_start = 0;
    g_ticks_end = 0;
    uart_write_bytes(uart_port0, "INSIDE CHARGE UN-PLUGGED NOTIFICATION\n", strlen("INSIDE CHARGE UN-PLUGGED NOTIFICATION\n"));
    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPINIT\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", b_chargerUnpluggedUrl);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"USERDATA\",\"%s\"\r\n", b_apiKey);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "{\"soc\":%s}%c", _soc, 0x0A);
    uint8_t len = strlen(ATcommand);
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPDATA=%d,10000\r\n", len);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "{\"soc\":%s}%c", _soc, 0x0A);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, "INSIDE WHILE\n", strlen("INSIDE WHILE\n"));

    g_ticks_start = xTaskGetTickCount();
    while (!downloadOK)
    {
      // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS);
      uart_read(rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS); //
      uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
      if (strstr((char *)rx_buffer, "OK"))
      {
        downloadOK = 1;
      }
      g_ticks_end = xTaskGetTickCount();
      if (g_ticks_end - g_ticks_start >= 5000 / portTICK_PERIOD_MS)
      {
        printf("WAIT TIMEOUT CONDITION\n");
        downloadOK = 1;
        g_ticks_start = 0;
        g_ticks_end = 0;

        // send CAN message to VCU for reset ack!
        twai_tx_data[0] = 1; // 1 for about to reset and 0 for module restarted
        twaiTxData(esp_gps_post_start_can_id, twai_tx_data, 1);

        sim7600_reset();

        // send CAN message to VCU for reset ack!
        twai_tx_data[0] = 0; // 1 for about to reset and 0 for module restarted
        twaiTxData(esp_gps_post_start_can_id, twai_tx_data, 1);

        return;
      }
    }
    g_ticks_start = 0;
    g_ticks_end = 0;

    uart_write_bytes(uart_port0, "OUTSIDE WHILE\n", strlen("OUTSIDE WHILE\n"));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPACTION=1\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS);
    memset(rx_buffer, 0, sizeof(rx_buffer));
    uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    if (strstr((char *)rx_buffer, "+CCH"))
    {
      memset(rx_buffer, 0, sizeof(rx_buffer));
      uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
      uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    }
    if (strstr((char *)rx_buffer, "200"))
    {
      uart_write_bytes(uart_port0, "SUCCESSFULLY POSTED\n", strlen("SUCCESSFULLY POSTED\n"));
      successFlag = 1;
    }
    else if (strstr((char *)rx_buffer, "302"))
    {
      uart_write_bytes(uart_port0, "INSUFFICIENT SIM BALANCE\n", strlen("INSUFFICIENT SIM BALANCE\n"));
      twai_tx_data[0] = 5;
      twai_tx_data[1] = 3;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
    else
    {
      if (retryCount < 1)
      {
        retryCount++;
        uart_write_bytes(uart_port0, "POSTING FAILED, RETRY\n", strlen("POSTING FAILED, RETRY\n"));
        goto retry;
      }
      else
      {
        uart_write_bytes(uart_port0, "RETRY FAILED, RESUMING TASK\n", strlen("RETRY FAILED, RESUMING TASK\n"));
        retryCount = 0;
        successFlag = 0;
      }
    }

    // sprintf(ATcommand, "AT+HTTPREAD=0,200\r\n");
    // uart_write_bytes(uart_port2, ATcommand, strlen((char*)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 5000 / portTICK_PERIOD_MS);
    // uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    // uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    // memset(rx_buffer, 0, sizeof(rx_buffer));
    // memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); // v
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    retryCount = 0;

    if (successFlag == 1) // post success
    {
      twai_tx_data[0] = 5;
      twai_tx_data[1] = 1;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
    else if (successFlag == 0) // post failed
    {
      twai_tx_data[0] = 5;
      twai_tx_data[1] = 2;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
  }
  vTaskDelay(1);
}

//=======================CHARGER PLUGGED NOTIFICATION POST=========================//
void http_post_charge_plugged()
{
retry:
  // check if network is available or not before posting and if not available, do not post //
  signal_strength_check();
  if (signalStrength > 10)
  {
    int successFlag = 0;
    g_ticks_start = 0;
    g_ticks_end = 0;
    uart_write_bytes(uart_port0, "INSIDE CHARGE PLUGGED NOTIFICATION\n", strlen("INSIDE CHARGE PLUGGED NOTIFICATION\n"));
    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPINIT\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", b_chargerPluggedUrl);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"USERDATA\",\"%s\"\r\n", b_apiKey);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPACTION=1\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 10000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS);
    // uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    if (strstr((char *)rx_buffer, "+CCH"))
    {
      memset(rx_buffer, 0, sizeof(rx_buffer));
      uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
      uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    }
    if (strstr((char *)rx_buffer, "200"))
    {
      uart_write_bytes(uart_port0, "SUCCESSFULLY POSTED\n", strlen("SUCCESSFULLY POSTED\n"));
      successFlag = 1;
    }
    else if (strstr((char *)rx_buffer, "302"))
    {
      uart_write_bytes(uart_port0, "INSUFFICIENT SIM BALANCE\n", strlen("INSUFFICIENT SIM BALANCE\n"));
      twai_tx_data[0] = 6;
      twai_tx_data[1] = 3;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
    else
    {
      if (retryCount < 1)
      {
        retryCount++;
        uart_write_bytes(uart_port0, "POSTING FAILED, RETRY\n", strlen("POSTING FAILED, RETRY\n"));
        goto retry;
      }
      else
      {
        uart_write_bytes(uart_port0, "RETRY FAILED, RESUMING TASK\n", strlen("RETRY FAILED, RESUMING TASK\n"));
        retryCount = 0;
        successFlag = 0;
      }
    }

    // sprintf(ATcommand, "AT+HTTPREAD=0,200\r\n");
    // uart_write_bytes(uart_port2, ATcommand, strlen((char*)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 5000 / portTICK_PERIOD_MS);
    // uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    // uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    // memset(rx_buffer, 0, sizeof(rx_buffer));
    // memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    retryCount = 0;

    if (successFlag == 1) // post success
    {
      twai_tx_data[0] = 6;
      twai_tx_data[1] = 1;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
    else if (successFlag == 0) // post failed
    {
      twai_tx_data[0] = 6;
      twai_tx_data[1] = 2;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
  }
  vTaskDelay(1);
}

//=======================THEFT ALERT NOTIFICATION POST=========================//
void theft_alert()
{
retry:
  // check if network is available or not before posting and if not available, do not post //
  signal_strength_check();
  if (signalStrength > 10)
  {
    int successFlag = 0;
    g_ticks_start = 0;
    g_ticks_end = 0;
    uart_write_bytes(uart_port0, "INSIDE THEFT ALERT NOTIFICATION\n", strlen("INSIDE THEFT ALERT NOTIFICATION\n"));
    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPINIT\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", b_theftAlertUrl);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPPARA=\"USERDATA\",\"%s\"\r\n", b_apiKey);
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 3000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPACTION=1\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 10000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 500 / portTICK_PERIOD_MS);
    memset(rx_buffer, 0, sizeof(rx_buffer));
    uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    if (strstr((char *)rx_buffer, "+CCH"))
    {
      memset(rx_buffer, 0, sizeof(rx_buffer));
      uart_read(rx_buffer, BUF_SIZE, 120000 / portTICK_PERIOD_MS);
      uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    }
    if (strstr((char *)rx_buffer, "200"))
    {
      uart_write_bytes(uart_port0, "SUCCESSFULLY POSTED\n", strlen("SUCCESSFULLY POSTED\n"));
      successFlag = 1;
    }
    else if (strstr((char *)rx_buffer, "302"))
    {
      uart_write_bytes(uart_port0, "INSUFFICIENT SIM BALANCE\n", strlen("INSUFFICIENT SIM BALANCE\n"));
      twai_tx_data[0] = 7;
      twai_tx_data[1] = 3;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
    else
    {
      if (retryCount < 1)
      {
        retryCount++;
        uart_write_bytes(uart_port0, "POSTING FAILED, RETRY\n", strlen("POSTING FAILED, RETRY\n"));
        goto retry;
      }
      else
      {
        uart_write_bytes(uart_port0, "RETRY FAILED, RESUMING TASK\n", strlen("RETRY FAILED, RESUMING TASK\n"));
        retryCount = 0;
        successFlag = 0;
      }
    }

    // sprintf(ATcommand, "AT+HTTPREAD=0,200\r\n");
    // uart_write_bytes(uart_port2, ATcommand, strlen((char*)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 5000 / portTICK_PERIOD_MS);
    // uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    // uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));

    sprintf(ATcommand, "AT+HTTPTERM\r\n");
    uart_write_bytes(uart_port2, ATcommand, strlen((char *)ATcommand));
    // uart_read_bytes(uart_port2, rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    uart_read(rx_buffer, BUF_SIZE, 1000 / portTICK_PERIOD_MS); //
    uart_write_bytes(uart_port0, ATcommand, strlen((char *)ATcommand));
    uart_write_bytes(uart_port0, rx_buffer, strlen((char *)rx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(ATcommand, 0, sizeof(ATcommand));
    retryCount = 0;

    if (successFlag == 1) // post success
    {
      twai_tx_data[0] = 7;
      twai_tx_data[1] = 1;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
    else if (successFlag == 0) // post failed
    {
      twai_tx_data[0] = 7;
      twai_tx_data[1] = 2;
      twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
    }
  }
  vTaskDelay(1);
}

//=======================CUSTOM UART READ API for better timeout solution=========================//
void uart_read(void *buf, uint32_t length, TickType_t ticks_to_wait)
{
  uint8_t whileFlag = 0;
  uint8_t len = 0;
  portTickType ticks_start = xTaskGetTickCount();
  while (!whileFlag)
  {
    len = uart_read_bytes(uart_port2, buf, length, 150 / portTICK_PERIOD_MS); // edit uart_port in this line for required uart port
    if (len > 0)
    {
      // printf("rx data length: %d\n", len);
      whileFlag = 1;
      ticks_start = 0;
    }
    TickType_t ticks_end = xTaskGetTickCount();
    if (ticks_end - ticks_start >= ticks_to_wait)
    {
      whileFlag = 1;
      ticks_start = 0;
      ticks_end = 0;
    }
  }
  uart_flush(uart_port2);
}

//=======================UART INITIALIZATION=========================//
void uart_init()
{
  uart_config_t uart_config0 = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

  uart_config_t uart_config2 = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
  uart_param_config(uart_port0, &uart_config0);
  uart_param_config(uart_port2, &uart_config2);

  uart_set_pin(uart_port0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_set_pin(uart_port2, UART_2_TX, UART_2_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  uart_driver_install(uart_port0, BUF_SIZE * 2, 0, 0, NULL, 0);
  uart_driver_install(uart_port2, BUF_SIZE * 2, 0, 0, NULL, 0);
}

//=======================SIM CAN READ=========================//
void sim_can_read(uint32_t id)
{
  if (id == POST_SOC) // for SOC Post
  {
    EspSimSocHandle SOC_data;
    esp_rec_getData(&SOC_data);
    printf("SOC and ETA Data:\n");
    printf("\nStatus : %d\n", SOC_data.charger_status);
    printf("SOC : %d\n", SOC_data.soc);
    printf("Estm : %d\n\n", SOC_data.time2fullCharge);

    //=======data assign to arguments==========//
    sprintf(g_soc, "%d", SOC_data.soc);
    if (SOC_data.charger_status == 01)
    {
      ptrChargeStatus = (uint8_t *)"true";
    }
    else
    {
      ptrChargeStatus = (uint8_t *)"false";
    }
    sprintf(g_eta, "%d", SOC_data.time2fullCharge);
    //==========================================//
    sim_task_flag = 1;
    state = SOC_ETA_POST;
    printf("data: %s,  %s,  %s", g_soc, ptrChargeStatus, g_eta);
  }

  else if (id == POST_TELEMETRY) // for TELEMETRY Post
  {
    EspSimTeleHandle TELE_data;
    esp_rec_getData(&TELE_data);
    printf("TELEMETRY Data:\n");
    printf("ODO : %f\n", TELE_data.overall_odo);
    printf("MAX SPEED : %d\n", TELE_data.overall_max_speed);
    printf("AVG SPEED : %d\n", TELE_data.overall_avg_speed);
    printf("ENERGY : %f\n", TELE_data.total_energy);
    printf("SUSTE ODO : %f\n", TELE_data.suste_overall_odo);
    printf("THIKKA ODO : %f\n", TELE_data.thikka_overall_odo);
    printf("BABBAL ODO : %f\n", TELE_data.babbal_overall_odo);

    //=======data assign to arguments==========//
    sprintf(g_disTravelled, "%f", TELE_data.overall_odo);
    sprintf(g_maxSpeed, "%d", TELE_data.overall_max_speed);
    sprintf(g_avgSpeed, "%d", TELE_data.overall_avg_speed);
    sprintf(g_totalEnergy, "%f", TELE_data.total_energy);
    sprintf(g_odoSuste, "%f", TELE_data.suste_overall_odo);
    sprintf(g_odoThikka, "%f", TELE_data.thikka_overall_odo);
    sprintf(g_odoBabbal, "%f", TELE_data.babbal_overall_odo);
    //==========================================//
    sim_task_flag = 1;
    state = TELE_DATA_POST;
    printf("data: %s,  %s,  %s, %s,  %s,  %s, %s", g_disTravelled, g_maxSpeed, g_avgSpeed, g_totalEnergy, g_odoSuste, g_odoThikka, g_odoBabbal);
  }

  else if (id == POST_BATTERY_LOG) // for BATTERY LOG Post
  {
    Esp_BatLog_Handle BATTLOG_data;
    esp_rec_getData(&BATTLOG_data);
    printf("BATTERY LOG Data:\n");
    printf("SOH : %f\n", BATTLOG_data.batt_soh);
    printf("BATT CYCLE : %f\n", BATTLOG_data.batt_cycles);
    printf("CHARGER TYPE : %d\n", BATTLOG_data.ac_charger_type);
    printf("START CHARGE : %d\n", BATTLOG_data.start_charge_soc);
    printf("END CHARGE : %d\n", BATTLOG_data.end_charge_soc);
    printf("ODO DIFF : %d\n", BATTLOG_data.odo_difference);
    printf("TOTAL ODO : %f\n", BATTLOG_data.overall_distance);

    //=======data assign to arguments==========//
    sprintf(g_soh, "%f", BATTLOG_data.batt_soh);
    sprintf(g_LifeCycle, "%f", BATTLOG_data.batt_cycles);
    if (BATTLOG_data.ac_charger_type == 0)
    {
      ptrChargeType = (uint8_t *)"AC charge";
    }
    else if (BATTLOG_data.ac_charger_type == 1)
    {
      ptrChargeType = (uint8_t *)"Yatri fast charge";
    }
    sprintf(g_StartCharge, "%d", BATTLOG_data.start_charge_soc);
    sprintf(g_EndCharge, "%d", BATTLOG_data.end_charge_soc);
    sprintf(g_ODOvariance, "%d", BATTLOG_data.odo_difference);
    sprintf(g_disTravelled, "%f", BATTLOG_data.overall_distance);
    //==========================================//
    sim_task_flag = 1;
    state = BATTERY_LOG;
    printf("data: %s,  %s,  %s, %s,  %s,  %s, %s\n", g_soh, g_LifeCycle, ptrChargeType, g_StartCharge, g_EndCharge, g_ODOvariance, g_disTravelled);
  }

  else if (id == POST_CHARGE_UNPLUGGED) // for CHARGE UNPLUGGED Post
  {
    ESP_ChargerUnPlug_Handle CU_data;
    esp_rec_getData(&CU_data);
    printf("CHARGE UNPLUGED Data:\n");
    printf("END SOC : %d\n", CU_data.end_soc);

    //=======data assign to arguments==========//
    sprintf(g_soc, "%d", CU_data.end_soc);
    //==========================================//
    sim_task_flag = 1;
    state = CHARGE_UNPLUGGED_POST;
    printf("data: %s\n", g_soc);
  }

  else if (id == POST_CHARGE_PLUGGED) // for CHARGE PLUGGED Post
  {
    ESP_ChargerPlug_Handle CP_data;
    esp_rec_getData(&CP_data);
    printf("CHARGE PLUGGED Post Request\n");
    sim_task_flag = 1;
    state = CHARGE_PLUGGED_POST;
  }

  else if (id == POST_CHARGE_COMPLETION) // for CHARGE COMPLETION Post
  {
    ESP_FullCharge_Handle FC_data;
    esp_rec_getData(&FC_data);
    printf("FULLY CHARGED Post Request\n");
    sim_task_flag = 1;
    state = CHARGE_COMPLETION_POST;
  }

  else if (id == POST_CRASH_DETECTION) // for CRASH DETECTION SMS and Post
  {
    ESP_Crash_Handle CD_data;
    esp_rec_getData(&CD_data);
    printf("CRASH / FALL DETECTION Data:\n");
    printf("G-X : %f\n", CD_data.acc_x);
    printf("G-Y : %f\n", CD_data.acc_y);
    printf("G-Z : %f\n", CD_data.acc_z);
    printf("BIKE NAME: %s\n", CD_data.bike_name);

    //=======data assign to arguments==========//
    sprintf(g_xVal, "%0.2f", CD_data.acc_x);
    sprintf(g_yVal, "%0.2f", CD_data.acc_y);
    sprintf(g_zVal, "%0.2f", CD_data.acc_z);
    sprintf(g_bikeName, "%s", CD_data.bike_name);
    //==========================================//

    sim_task_flag = 1;
    state = SMS_ADXL;
    printf("data: %s, %s, %s, %s\n", g_xVal, g_yVal, g_zVal, g_bikeName);
  }

  else if (id == POST_THEFT_ALERT) // for THEFT ALERT Post
  {
    ESP_Theft_Handle TN_data;
    esp_rec_getData(&TN_data);
    printf("THEFT ALERT Post Request\n");
    sim_task_flag = 1;
    state = THEFT_ALERT;
  }

  else if (id == POST_GPS_LOCATION) // for GPS LOCATION Post
  {
    ESP_PostGPSLocation_Handle GPS_data;
    esp_rec_getData(&GPS_data);
    printf("GPS LOCATION Post Request\n");
    sim_task_flag = 1;
    state = GET_LOCATION;
  }

  else if (id == UPDATE_TIME) // for TIME UPDATE Post
  {
    ESP_GetTime_Handle TIME_data;
    esp_rec_getData(&TIME_data);
    printf("DATE and TIME Request\n");
    sim_task_flag = 1;
    state = GET_TIME;
  }
}

//=======================SIM TASK MAIN=========================//
void sim_task(void *arg)
{
  // Subscribe this task to TWDT, then check if it is subscribed
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
  ESP_ERROR_CHECK(esp_task_wdt_status(NULL));
  while (1)
  {
    ESP_ERROR_CHECK(esp_task_wdt_reset());
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if ((sim_task_flag == 1) && (flag_sim_task == true))
    {
      switch (state)
      {
      case SOC_ETA_POST: // state 1

        sim_task_flag = 0;
        http_post_soc_data(g_soc, (char *)ptrChargeStatus, g_eta);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        break;

      case TELE_DATA_POST: // state 2

        sim_task_flag = 0;
        http_post_tele_data(g_disTravelled, g_maxSpeed, g_avgSpeed, g_totalEnergy, g_odoSuste, g_odoThikka, g_odoBabbal);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        break;

      case CHARGE_COMPLETION_POST: // state 3

        sim_task_flag = 0;
        http_post_charge_completion();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        break;

      case CHARGE_UNPLUGGED_POST: // state 4

        sim_task_flag = 0;
        http_post_charge_unplugged(g_soc);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        break;

      case CHARGE_PLUGGED_POST: // state 5

        sim_task_flag = 0;
        http_post_charge_plugged();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        break;

      case GET_TIME: // state 6

        sim_task_flag = 0;
        get_time();
        vTaskDelay(100 / portTICK_PERIOD_MS);
        // sending CAN message to VCU to assure the transmission
        twai_tx_data[0] = 10;
        twai_tx_data[1] = 1;
        twaiTxData(esp_post_status_can_id, twai_tx_data, 2);
        break;

      case GET_LOCATION: // state 7

        sim_task_flag = 0;
        gps_location_post();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        break;

      case SMS_ADXL: // state 8

        sim_task_flag = 0;
        // sms_adxl_value(g_xVal, g_yVal, g_zVal);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        break;

      case THEFT_ALERT: // state 9

        sim_task_flag = 0;
        theft_alert();
        vTaskDelay(50 / portTICK_PERIOD_MS);
        break;

      case BATTERY_LOG: // state 10

        sim_task_flag = 0;
        http_post_battery_log(g_soh, g_LifeCycle, (char *)ptrChargeType, g_StartCharge, g_EndCharge, g_disTravelled, g_ODOvariance);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        break;
      }
      vTaskDelay(1);
    }
    if (flag_sim_task == true)
    {
      gps_location();
    }
    vTaskDelay(1);
  }
}
