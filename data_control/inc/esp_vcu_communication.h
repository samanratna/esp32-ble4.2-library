#ifndef INC_ESP_VCU_COMMUNICATION_H_
#define INC_ESP_VCU_COMMUNICATION_H_

#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "esp_log.h"
#include "combiner.h"

extern bool Response_OK;
extern bool ACK_OK;
extern bool Stop_ACK;
extern bool halt_receive;

#define Response_Send 0
#define Structure_Send 1
#define Data_Finished_Send 2
#define Task_End 3

typedef enum
{
	TRANSMIT_FAIL,
	TRANSMIT_QUEUE_FULL,
	TRANSMIT_TIME_OUT,
	CAN_INVALID_ARG,
	TRANSMIT_OK
} CAN_StatusType_Def;

typedef enum
{
	VCU_RESPONSE_SEND_ERROR,
	VCU_RESPONSE_SEND_OK
} VCU_CanResponse_StatusTypeDef;

typedef enum
{
	ESP_VCU_OK = 1,
	ESP_VCU_FAILED = 0,
	ESP_VCU_BUSY = -1
} ESP_VCU_Status;
typedef enum
{
	VCU_CAN_TRANSMIT_ERROR,
	VCU_CAN_TRANSMIT_OK
} VCU_CANTransmitStatusDef;

typedef enum
{
	VCU_STOP_CAN_ERROR,
	VCU_STOP_CAN_OK
} VCU_StopCan_StatusTypeDef;

typedef struct
{
	uint8_t response[8];
} VCU_CanResponse_Handel;

typedef struct
{
	uint8_t stop[8];
} VCU_StopCan_Handel;

int vcu_can_task(void *structure, uint8_t size_of_struct, uint32_t can_id);
VCU_CanResponse_StatusTypeDef vcu_response_send(VCU_CanResponse_Handel *response_data);
VCU_StopCan_StatusTypeDef vcu_stop_can_send(VCU_StopCan_Handel *stop_data);
VCU_CANTransmitStatusDef vcu_can_send(uint32_t can_id, uint8_t data[], uint8_t data_len);
CAN_StatusType_Def transmit_can_message(uint32_t can_id, uint8_t data_in[], uint8_t data_len);
void get_can_state(void);
void set_alerts(esp_err_t can_state);
void recover_can_communication_after_bus_off(void);
void can_bus_status_info(void);

#endif