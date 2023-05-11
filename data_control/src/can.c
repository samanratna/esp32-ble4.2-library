#include "main.h"

static const char *ERROR_TAG = "FROM CAN.C";
uint8_t twai_tx_data[8];
twai_message_t message;
// const twai_message_t tx_msg = {.identifier = 0X18CF28FA, .ss = 0, .dlc_non_comp = 1, .self = 0, .extd = 1, .rtr = 0, .data_length_code = 8, .data = {0, 0, 0, 0, 0, 0, 0, 0}};
xQueueHandle can_rx_queue;
//=======================TWAI INITIALIZATION=========================//
void twai_init(void)
{
  // Initialize configuration structures using macro initializers
  static const twai_general_config_t g_config = {
      .mode = TWAI_MODE_NORMAL,
      .tx_io = 5,
      .rx_io = 4,
      .clkout_io = TWAI_IO_UNUSED,
      .bus_off_io = TWAI_IO_UNUSED,
      .tx_queue_len = 1600,
      .rx_queue_len = 1000,
      .alerts_enabled = TWAI_ALERT_ALL,
      .clkout_divider = 0,
      .intr_flags = ESP_INTR_FLAG_LEVEL1};

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = {.acceptance_code = (0x0000FBFA << 3), .acceptance_mask = ~(0x1300FFFF << 3), .single_filter = true};
  // twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    printf("Driver installed\n");
  }
  else
  {
    printf("Failed to install driver\n");
    return;
  }
  // Start TWAI driver
  if (twai_start() == ESP_OK)
  {
    printf("Driver started\n");
  }
  else
  {
    printf("Failed to start driver\n");
    return;
  }
}

int twaiTxData(uint32_t id, const uint8_t *tx_buff, uint8_t data_len)
{
  uint8_t send_buf[8];
  for (int i = 0; i < data_len; i++)
  {
    send_buf[i] = tx_buff[i];
  }

  if (transmit_can_message(id, send_buf, data_len) == 4)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

//=======================TWAI ALERT READ=========================//
void twai_alert_read(void)
{
  uint32_t alerts_to_enable = TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF | TWAI_ALERT_TX_IDLE | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK)
  {
    printf("Alerts Reconfigured\n");
  }
  else
  {
    printf("Failed to Reconfigure Alerts");
  }
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(1000));
}

//=======================TWAI TASK MAIN=========================//
void twai_task(void *arg)
{
  printf("TWAI function\n");
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
  ESP_ERROR_CHECK(esp_task_wdt_status(NULL));
  while (1)
  {
    ESP_ERROR_CHECK(esp_task_wdt_reset());
    // vTaskDelay(1 / portTICK_PERIOD_MS);

    twai_message_t message;

    esp_vcu_exec();
    // printf("Inside the twia task\n");
    if (twai_receive(&message, 10 / portMAX_DELAY) == ESP_OK)
    {
      printf("TWAI Message Recieved!\n");
      char data_print[100];
      int n = 0;
      printf("%x\n", message.identifier);
      for (int i = 0; i < message.data_length_code; i++)
      {
        n += sprintf(data_print + n, "%d,", message.data[i]);
      }
      printf(data_print);
      printf("\n");
      esp_rec_interrupt(message.identifier, message.data, message.data_length_code);
      esp_rec_errorFixTimer();
      uint32_t id = esp_rec_getCanID();
      switch (message.identifier)
      {
      case response_can_id:
        Response_OK = true;
        ESP_LOGE(ERROR_TAG, "FROM RESPONSE OK");
        break;
      case ack_can_id:
        ACK_OK = true;
        ESP_LOGE(ERROR_TAG, "FROM ACK OK");
        break;
      case stop_can_id:
        Stop_ACK = true;
        ESP_LOGE(ERROR_TAG, "FROM STOP ACK");
        break;
      }

      ota_can(id, message.identifier, message.data, message.data_length_code);
      obd_can(id, message.identifier, message.data, message.data_length_code);
      // printf("INSIDE TWAI RECEIVE\n\n");

      if (id > 0)
      {
        ble_can_read(id);
        sim_can_read(id);
        read_bike_credentials(id);
      }
    }
    else
    {
      // printf("CAN_STATUS :: Failed to receive message, Retrying...\n");
    }
    memset(message.data, 0, 8);
    vTaskDelay(1);
  }
  vTaskDelay(1);
}
