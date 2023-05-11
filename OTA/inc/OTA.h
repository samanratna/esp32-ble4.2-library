#ifndef _OTA_H_
#define _OTA_H_

#include "main.h"

#define WAIT_PERIOD 30000000                      // 30 sec  /*used in obd*/
#define CONFIG_EXAMPLE_GPIO_DIAGNOSTIC GPIO_NUM_4 // used in obd
#define HASH_LEN 32                               /* SHA-256 digest length */

extern bool OTA_ACTIVATE; // in ble

extern int first_check; // in main
extern int limit;

extern uint64_t previous_time; // in obd
extern uint64_t current_time;

typedef struct
{
    uint8_t charging_status;
    uint8_t bike_status;
    uint8_t current_soc;
    uint8_t current_speed;
} bike_status_ota;

typedef struct
{
    char release_notes[200];
} releasenotes;

extern releasenotes update_detais;
extern bike_status_ota bikestatus;

int Download_Updates(int board);
int ESP_OTA();
void b64_generate_decode_table();
int b64_isvalidchar(char c);
size_t b64_decoded_size(const char *in);
int send_version();

void http_post_update_remark(int issue, int issue_0);
// // int check_403();

void ota_task(void *arg);
int Yatri_OTA();
void ota_can(uint32_t struct_id, uint32_t can_id, uint8_t data[], uint8_t data_len);
int check_version(int limit);
int check_version_skip();
bool diagnostic(void);
void print_sha256(const uint8_t *image_hash, const char *label);

extern bool FLAG_FIRST_ACK;
extern bool FLAG_TOTAL_FILESIZE;
extern bool FLAG_CRC_V;
extern bool Flag_Send_version;
extern bool FLAG_DISPLAY_ROLL;
extern bool FLAG_CRC_ACK;
extern uint32_t VCU_CRC;
extern uint64_t fileSize_ESP;
extern uint64_t sum;

#endif