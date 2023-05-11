
#include "main.h"
#include "can.h"

#define CAN_TRANSMIT_TIME_OUT 2000
#define Response_Send 0
#define Structure_Send 1
#define Data_Finished_Send 2
#define Task_End 3

static const char *CONSOLE_TAG = "ESP_VCU_COMMUNICATION";
// static uint32_t tx_counter=0;
// static uint32_t rx_counter=0;
// static uint32_t bus_off_counter=0;
// static uint32_t alerts=0;

bool Response_OK;
bool ACK_OK;
bool Stop_ACK;
bool halt_receive;
static bool response_flag = false;
static bool res_flag = false;
static uint8_t data_send_count = 0;
static uint8_t struct_len;
uint8_t data_buffer[100]; // check later
uint32_t alerts = 0;
int Recovery_flag;

int vcu_can_task(void *structure, uint8_t size_of_struct, uint32_t can_id)
{
	static ESP_VCU_Status vcu_can_send_status = ESP_VCU_BUSY;
	static uint8_t task_state_level = 0;
	static uint32_t vcu_can_send_timer = 0;
	uint32_t current_time = esp_timer_get_time() / 1000;

	switch (task_state_level)
	{
	case Response_Send:
		ESP_LOGI(CONSOLE_TAG, "VCU Response Can Send");
		Response_OK = false;
		ACK_OK = false;
		Stop_ACK = false;
		task_state_level++;
		struct_len = size_of_struct;
		VCU_CanResponse_Handel response_data;
		response_data.response[0] = size_of_struct;
		response_data.response[1] = can_id >> 24 & 0xFF;
		response_data.response[2] = can_id >> 16 & 0xFF;
		response_data.response[3] = can_id >> 8 & 0xFF;
		response_data.response[4] = can_id >> 0 & 0xFF;
		if (vcu_response_send(&response_data) == VCU_RESPONSE_SEND_OK)
		{
			vcu_can_send_status = ESP_VCU_BUSY;
			vcu_can_send_timer = current_time + CAN_TRANSMIT_TIME_OUT;
			ESP_LOGI(CONSOLE_TAG, "Successfully Send Response Can to VCU\n");
			return vcu_can_send_status;
		}
		else
		{
			vcu_can_send_status = ESP_VCU_FAILED;
			task_state_level = 0;
			return vcu_can_send_status;
		}
		break;

	case Structure_Send:

		if ((Response_OK && !res_flag) || (ACK_OK && response_flag))
		{
			ACK_OK = false;
			if (Response_OK)
			{
				response_flag = true;
				Response_OK = false;
				ESP_LOGI(CONSOLE_TAG, "From Memory Allocation\n");
				combiner_genericDataToBytes(structure, data_buffer, struct_len);
			}
			uint8_t send_data[8];
			if (struct_len > 8)
			{
				res_flag = true;
				memcpy(send_data, data_buffer + data_send_count, 8);
				if (vcu_can_send(can_id, send_data, 8) == VCU_CAN_TRANSMIT_OK)
				{
					vcu_can_send_status = ESP_VCU_BUSY;
					vcu_can_send_timer = current_time + CAN_TRANSMIT_TIME_OUT;
				}
				else
				{
					vcu_can_send_status = ESP_VCU_FAILED;
					task_state_level = 0;
					data_send_count = 0;
					res_flag = false;
					response_flag = false;
				}
				struct_len = struct_len - 8;
				data_send_count += 8;
			}
			else
			{
				ACK_OK = false;
				response_flag = false;
				res_flag = false;
				task_state_level++;
				memcpy(send_data, data_buffer + data_send_count, struct_len);
				if (vcu_can_send(can_id, send_data, struct_len) == VCU_CAN_TRANSMIT_OK)
				{
					vcu_can_send_status = ESP_VCU_BUSY;
					vcu_can_send_timer = current_time + CAN_TRANSMIT_TIME_OUT;
					data_send_count = 0;
				}
				else
				{
					vcu_can_send_status = ESP_VCU_FAILED;
					task_state_level = 0;
					data_send_count = 0;
				}
				ESP_LOGI(CONSOLE_TAG, "From Memory Free\n");
			}
			return vcu_can_send_status;
		}
		else
		{
			if (current_time > vcu_can_send_timer)
			{
				ESP_LOGI(CONSOLE_TAG, "No ACK/Response From the VCU\n");
				vcu_can_send_timer = 0;
				response_flag = false;
				res_flag = false;
				vcu_can_send_status = ESP_VCU_FAILED;
				task_state_level = 0;
				data_send_count = 0;
				ESP_LOGI(CONSOLE_TAG, "From Memory Free\n");
				return vcu_can_send_status;
			}
			return vcu_can_send_status;
		}
		break;

	case Data_Finished_Send:
		if (ACK_OK)
		{
			ESP_LOGI(CONSOLE_TAG, "From the Data End Can Send\n");
			Stop_ACK = false;
			task_state_level++;
			VCU_StopCan_Handel stop_data;
			stop_data.stop[0] = size_of_struct;
			stop_data.stop[1] = can_id >> 24 & 0xFF;
			stop_data.stop[2] = can_id >> 16 & 0xFF;
			stop_data.stop[3] = can_id >> 8 & 0xFF;
			stop_data.stop[4] = can_id >> 0 & 0xFF;
			if (vcu_stop_can_send(&stop_data) == VCU_STOP_CAN_OK)
			{
				vcu_can_send_status = ESP_VCU_BUSY;
				vcu_can_send_timer = current_time + CAN_TRANSMIT_TIME_OUT;
				return vcu_can_send_status;
			}
			else
			{
				vcu_can_send_status = ESP_VCU_FAILED;
				task_state_level = 0;
				return vcu_can_send_status;
			}
		}
		else
		{
			if (current_time > vcu_can_send_timer)
			{
				ESP_LOGI(CONSOLE_TAG, "No ACK From the VCU\n");
				vcu_can_send_timer = 0;
				vcu_can_send_status = ESP_VCU_FAILED;
				task_state_level = 0;
				return vcu_can_send_status;
			}
			return vcu_can_send_status;
		}
		break;

	case Task_End:
		if (Stop_ACK)
		{
			ESP_LOGI(CONSOLE_TAG, "Successfully Send the Structure\n");
			task_state_level = 0;
			vcu_can_send_status = ESP_VCU_OK;
			return vcu_can_send_status;
		}
		else
		{
			if (current_time > vcu_can_send_timer)
			{
				ESP_LOGI(CONSOLE_TAG, "No Stop ACK From the VCU\n");
				vcu_can_send_timer = 0;
				vcu_can_send_status = ESP_VCU_FAILED;
				task_state_level = 0;
				return vcu_can_send_status;
			}
			return vcu_can_send_status;
		}
		break;
	}
	return vcu_can_send_status;
}

VCU_CanResponse_StatusTypeDef vcu_response_send(VCU_CanResponse_Handel *response_data)
{
	VCU_CanResponse_StatusTypeDef can_response_state = VCU_RESPONSE_SEND_ERROR;
	uint8_t response_data_buff[8];
	for (int i = 0; i < 5; i++)
	{
		response_data_buff[i] = response_data->response[i];
	}
	if (transmit_can_message(response_can_id, response_data_buff, 5) == TRANSMIT_OK)
	{
		can_response_state = VCU_RESPONSE_SEND_OK;
	}
	return can_response_state;
}
VCU_StopCan_StatusTypeDef vcu_stop_can_send(VCU_StopCan_Handel *stop_data)
{
	VCU_StopCan_StatusTypeDef stop_can_state = VCU_STOP_CAN_ERROR;
	uint8_t data_data_buff[8];
	for (int i = 0; i < 5; i++)
	{
		data_data_buff[i] = stop_data->stop[i];
	}
	if (transmit_can_message(stop_can_id, data_data_buff, 5) == TRANSMIT_OK)
	{
		stop_can_state = VCU_STOP_CAN_OK;
	}
	return stop_can_state;
}

VCU_CANTransmitStatusDef vcu_can_send(uint32_t can_id, uint8_t data[], uint8_t data_len)
{
	VCU_CANTransmitStatusDef vcu_can_transmit_status = VCU_CAN_TRANSMIT_ERROR;
	if (transmit_can_message(can_id, data, data_len) == TRANSMIT_OK)
	{
		vcu_can_transmit_status = VCU_CAN_TRANSMIT_OK;
	}
	return vcu_can_transmit_status;
}

CAN_StatusType_Def transmit_can_message(uint32_t can_id, uint8_t data_in[], uint8_t data_len)
{

	CAN_StatusType_Def can_transmit_status = TRANSMIT_FAIL;
	static twai_message_t tx_msg;
	tx_msg.identifier = can_id;
	tx_msg.ss = 0;
	tx_msg.dlc_non_comp = 0;
	tx_msg.self = 0;
	tx_msg.extd = 1;
	tx_msg.rtr = 0;
	tx_msg.data_length_code = data_len;
	for (int i = 0; i < data_len; i++)
	{
		tx_msg.data[i] = data_in[i];
	}
	esp_err_t can_state = twai_transmit(&tx_msg, 1000 / portTICK_PERIOD_MS);
	if (can_state == ESP_OK)
	{
		can_transmit_status = TRANSMIT_OK;
	}
	// set_alerts(can_state);
	get_can_state();
	// can_bus_status_info();
	// printf("after send message\n");

	return can_transmit_status;
}

void get_can_state(void)
{

	Recovery_flag = 0;
	if (twai_read_alerts(&alerts, pdMS_TO_TICKS(0)) != ESP_OK)
	{
		// printf("unable to read the alerts\n");
		return;
	}
	if (alerts & TWAI_ALERT_TX_IDLE)
	{
		// printf("No more messages queued for transmission\n");
		Recovery_flag = 1;
	}
	if (alerts & TWAI_ALERT_TX_SUCCESS)
	{
		// printf("The previous transmission was successful\n");
		Recovery_flag = 1;
	}
	if (alerts & TWAI_ALERT_BELOW_ERR_WARN)
	{
		printf("Both error counters have dropped below error warning limit\n");
		Recovery_flag = 1;
	}
	if (alerts & TWAI_ALERT_ERR_ACTIVE)
	{
		printf("TWAI controller has become error active\n");
		Recovery_flag = 1;
	}
	if (alerts & TWAI_ALERT_RECOVERY_IN_PROGRESS)
	{
		printf("TWAI controller is undergoing bus recovery\n");
		Recovery_flag = 1;
	}
	if (alerts & TWAI_ALERT_BUS_RECOVERED)
	{
		printf("TWAI controller has successfully completed bus recovery\n");
		Recovery_flag = 1;
	}
	if (alerts & TWAI_ALERT_ARB_LOST)
	{
		printf("The previous transmission lost arbitration\n");
		Recovery_flag = 1;
	}
	if (alerts & TWAI_ALERT_ABOVE_ERR_WARN)
	{
		printf("One of the error counters have exceeded the error warning limit\n");
		Recovery_flag = 1;
	}
	if (alerts & TWAI_ALERT_BUS_ERROR)
	{
		printf("A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus\n");
		Recovery_flag = 1;
	}
	if (alerts & TWAI_ALERT_TX_FAILED)
	{
		Recovery_flag = 1;
		printf("The previous transmission has failed\n");
	}
	if (alerts & TWAI_ALERT_RX_QUEUE_FULL)
	{
		printf("The RX queue is full causing a received frame to be lost\n");
		Recovery_flag = 1;
		twai_clear_receive_queue();
	}
	if (alerts & TWAI_ALERT_ERR_PASS)
	{
		printf("TWAI controller has become error passive\n");
		Recovery_flag = 1;
	}
	if (alerts & TWAI_ALERT_BUS_OFF)
	{
		printf("Bus-off condition occurred. TWAI controller can no longer influence bus\n");
		Recovery_flag = 1;
	}

	if (Recovery_flag == 0)
	{
		// recover_can_communication_after_bus_off();
	}
}
void set_alerts(esp_err_t can_state)
{
	switch (can_state)
	{
	case ESP_OK:
		// printf("Transmission successfully queued/initiated\n");
		break;
	case ESP_ERR_INVALID_ARG:
		printf("Arguments are invalid\n");
		break;
	case ESP_ERR_TIMEOUT:
		printf("Timed out waiting for space on TX queue\n");
		break;
	case ESP_FAIL:
		printf("TX queue is disabled and another message is currently transmitting\n");
		break;
	case ESP_ERR_INVALID_STATE:
		printf("TWAI driver is not in running state, or is not installed\n");
		break;
	case ESP_ERR_NOT_SUPPORTED:
		printf("Listen Only Mode does not support transmissions\n");
		break;
	}
}
void recover_can_communication_after_bus_off(void)
{
	halt_receive = true;
	ESP_ERROR_CHECK(twai_stop());
	ESP_LOGI(CONSOLE_TAG, "Stop Can communication");
	vTaskDelay(pdMS_TO_TICKS(1000));
	ESP_ERROR_CHECK(twai_driver_uninstall());
	ESP_LOGI(CONSOLE_TAG, "Driver uninstalled");
	vTaskDelay(pdMS_TO_TICKS(1000));
	//   ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, & f_config));
	twai_init();
	ESP_LOGI(CONSOLE_TAG, "Driver installed");
	vTaskDelay(pdMS_TO_TICKS(1000));
	// ESP_ERROR_CHECK(twai_start());
	ESP_LOGI(CONSOLE_TAG, "Driver started");
	ESP_LOGI(CONSOLE_TAG, "Starting transmissions");
	halt_receive = false;
}
void can_bus_status_info(void)
{
	twai_status_info_t status_inf;
	twai_get_status_info(&status_inf);
	switch (status_inf.state)
	{
	case TWAI_STATE_STOPPED:
		printf("Stopped state. The TWAI controller will not participate in any TWAI bus activities\n");
		break;
	case TWAI_STATE_RUNNING:
		printf("Running state. The TWAI controller can transmit and receive messages\n");
		break;
	case TWAI_STATE_BUS_OFF:
		printf("Bus-off state. The TWAI controller cannot participate in bus activities until it has recovered\n");
		break;
	case TWAI_STATE_RECOVERING:
		printf("Recovering state. The TWAI controller is undergoing bus recovery\n");
		break;
	}
	printf("Number of messages queued for transmission or awaiting transmission completion=%u\n", status_inf.msgs_to_tx);
	printf("Number of messages in RX queue waiting to be read=%u\n", status_inf.msgs_to_rx);
	printf("Current value of Transmit Error Counter=%u\n", status_inf.tx_error_counter);
	printf("Current value of Receive Error Counter=%u\n", status_inf.rx_error_counter);
	printf("Number of messages that failed transmissions=%u\n", status_inf.tx_failed_count);
	printf("Number of messages that were lost due to a full RX queue=%u\n", status_inf.rx_missed_count);
	printf("Number of instances arbitration was lost=%u\n", status_inf.arb_lost_count);
	printf("Number of instances a bus error has occurred=%u\n", status_inf.bus_error_count);
}
