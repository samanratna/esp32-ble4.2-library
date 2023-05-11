#ifndef SIM7600_H_
#define SIM7600_H_

#define BUF_SIZE (1024)
#define LED_1 2
#define LED_2 4
#define LED_3 15
#define UART_2_RX 16
#define UART_2_TX 17
#define SIM_POWER_PIN 13

#define SOC_ETA_POST 1
#define TELE_DATA_POST 2
#define CHARGE_COMPLETION_POST 3
#define CHARGE_UNPLUGGED_POST 4
#define CHARGE_PLUGGED_POST 5
#define GET_TIME 6
#define GET_LOCATION 7
#define SMS_ADXL 8
#define THEFT_ALERT 9
#define BATTERY_LOG 10

#define SEND_MAX_SPEED 1
#define SEND_AVG_SPEED 2
#define SEND_TRAVELLED_DISTANCE 3
#define SEND_CARBON_OFFSET 4
#define SEND_SOC 5
#define SEND_CHARGE_STATUS 6
#define SEND_CHARGE_COST 7

#define LIGHTS "A"
#define LIGHTS_AND_HORN "B"
#define TRIP_RESET "C"

#define ENABLE_GPS_TRACKING "K"
#define DISABLE_GPS_TRACKING "k"

#define ENABLE_SHAKE_MODE "L"
#define DISABLE_SHAKE_MODE "l"

#define ENABLE_ANTI_THEFT "M"
#define DISABLE_ANTI_THEFT "m"

#define BIKE_TURN_ON "1"
#define BIKE_TURN_OFF "0"

#define C 0.017453292519943295

extern uint8_t signalStrength;

char gps_tracking[2];
char shake_mode[2];
char anti_theft[2];

char app_features[10];

// uint8_t simStatus;

uint8_t g_maxSpeed_trip;
uint8_t g_maxSpeed_overall;

uint8_t g_avgSpeed_trip;
uint8_t g_avgSpeed_overall;

uint8_t g_travelledDistance_trip;
uint8_t g_travelledDistance_overall;

uint8_t g_carbonOffset_trip;
uint8_t g_carbonOffset_overall;

uint8_t g_soc_ble;
uint8_t g_charging_Status;
uint8_t g_eta_ble;
uint8_t g_IsFastCharging;
uint8_t g_estRange;

uint8_t g_chargeStatus_trip;
uint8_t g_chargeStatus_overall;

uint8_t g_carbonCost_trip;
uint8_t g_carbonCost_overall;

uint8_t state;
uint8_t sim_task_flag;
// uint8_t ble_task_flag;
uint8_t vcu_gps_flag;

uint32_t odo_distance;
uint32_t trip_distance;
float totalEnergy;
int odo_avg_speed;
int odo_max_speed;
int trip_avg_speed;
int trip_max_speed;

uint32_t odoBabbal;
uint32_t odoThikka;
uint32_t odoSuste;

int odo_suste;
int first_write_for_security;

uint8_t *ptrBikeID;
uint8_t *ptrSOC;
// char ptrSOC[5];
uint8_t *ptrChargeStatus;
uint8_t *ptrChargeType;
uint8_t *ptrETA;
uint8_t ptrSOH_L;
uint8_t ptrSOH_H;
double ptrSOH;

uint8_t ptrLifeCycle_L;
uint8_t ptrLifeCycle_H;
double ptrLifeCycle;

uint8_t *ptrStartCharge;
uint8_t *ptrEndCharge;

uint8_t *ptrODOvariance;

uint8_t *ptrUnit;

uint8_t *ptr_xVal;
uint8_t *ptr_yVal;
uint8_t *ptr_zVal;
char g_bikeName[20];

char g_soc[5];
char g_eta[5];
char g_disTravelled[10];
char g_totalEnergy[10];
char g_odoBabbal[10];
char g_odoThikka[10];
char g_odoSuste[10];

char g_maxSpeed[5];
char g_avgSpeed[5];
char g_xVal[20];
char g_yVal[20];
char g_zVal[20];

char g_soh[400];
char g_LifeCycle[400];
char g_StartCharge[5];
char g_EndCharge[5];
char g_ODOvariance[5];

char g_last_username[50];
char g_current_username[50];

char g_last_bikename[50];
char g_current_bikename[50];

char g_last_bday_year[10];
char g_last_bday_month[10];
char g_last_bday_day[10];

char g_current_bday_year[10];
char g_current_bday_month[10];
char g_current_bday_day[10];

char username_HEX[80];
char bikename_HEX[80];

float g_balance;

int VCU_EPROM_writeStatus;
int username_received_from_BLE;
int bikename_received_from_BLE;
int birthdate_received_from_BLE;

char received_bikename[10];
char received_username[10];

extern char ATcommand[300];
extern bool flag_sim_task;

void sim7600_powerup();
void sim7600_powerdown();
void sim7600_reset();
void gps_init();
void get_time();
void batt_check();
void sms_task();
void balance_check();
void gps_location();
void gps_location_post();
void calc_distance(float lat2, float lat1, float lon2, float lon1);
void http_post_location_data();
void sim7600_init();
void ssl_init();
void sms_init();
void pNumber_check();
void signal_strength_check();
void http_post_charge_completion();
void http_post_charge_unplugged(char _soc[5]);
void http_post_charge_plugged();
void theft_alert();
void http_post_BV_data(char _bikeId[50]);
void http_post_soc_data(char _soc[5], char _chargeStatus[10], char _eta[40]);
void http_post_tele_data(char _disTravelled[5], char _maxSpeed[5], char _avgSpeed[5], char _totalEnergy[10], char _odoSuste[10], char _odoThikka[10], char _odoBabbal[10]);
void http_post_battery_log(char *_soh, char *_lifeCycle, char *_chargeType, char *_startCharge, char *_endCharge, char *_currentODO, char *_ODOvariance);
void sms_adxl_value(char xVal[5], char yVal[5], char zVal[5]);
void http_post_CR_data();
void uart_init();
void interrupt_init();

void uart_read(void *buf, uint32_t length, TickType_t ticks_to_wait);
void sim_task(void *arg);
void busy_task(void);
void ideal_task(void);
void sim_can_read(uint32_t id);

#endif