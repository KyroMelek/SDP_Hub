#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <string.h>
#include <xbee_api.hpp>
#include <iostream>
#include <vector>

// #include <sstream> // Require to use std::stringstream

using std::cout;

static const char *TAG = "UART Handler";

void performOutletAction(json *json_payload);

// UART Xbee
#define USB_UART UART_NUM_0      // uart0 to communicate between xbee and esp32
#define ESP_XBEE_UART UART_NUM_2 // uart0 to communicate between xbee and esp32
#define TX_u (GPIO_NUM_1)        // uart0 TX
#define RX_u (GPIO_NUM_3)        // uart0 RX
#define RX (GPIO_NUM_18)         // uart0 TX
#define TX (GPIO_NUM_19)         // uart0 RX

static const int RX_BUFFER_SIZE = 1024; // 1KB RX buffer

static QueueHandle_t uart0_queue;
#define PATTERN_CHR_NUM (3) /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

void xbee_uart_setup(void)
{
  // uart configuration
  const uart_config_t uart2_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 122,
      .source_clk = UART_SCLK_APB,
  };

  // install UART driver
  ESP_ERROR_CHECK(uart_driver_install(ESP_XBEE_UART, 4098, 4098, 20, &uart0_queue, 0));
  ESP_ERROR_CHECK(uart_param_config(ESP_XBEE_UART, &uart2_config));
  ESP_ERROR_CHECK(uart_set_pin(ESP_XBEE_UART, TX, RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
};

void uart_event_task(void *pvParameters)
{
  uart_event_t event;
  size_t buffered_size;
  uint8_t *dtmp = (uint8_t *)malloc(RD_BUF_SIZE);
  for (;;)
  {
    // Waiting for UART event.
    if (xQueueReceive(uart0_queue, (void *)&event, (TickType_t)portMAX_DELAY))
    {
      bzero(dtmp, RD_BUF_SIZE);
      ESP_LOGI(TAG, "uart[%d] event:", ESP_XBEE_UART);
      switch (event.type)
      {
      // Event of UART receving data
      /*We'd better handler data event fast, there would be much more data events than
      other types of events. If we take too much time on data event, the queue might
      be full.*/
      case UART_DATA:
      {
        ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
        uart_read_bytes(ESP_XBEE_UART, dtmp, event.size, portMAX_DELAY);

        ESP_LOGI(TAG, "[DATA EVT]: ");
        // for (int i = 0; i < 128; i++)
        // {
        //   cout << dtmp[i];
        // }
        // cout << std::endl;

        // json with all information about xbee frame
        json *j = readFrame(dtmp);

        cout << j->dump() << std::endl;

        // handle error cases
        // unrecognized frame
        if (*j == -1)
        {
          ESP_LOGI(TAG, "recieved an unrecognized frame");
        }
        // invalid frame
        else if (*j == -2)
        {
          ESP_LOGI(TAG, "recieved an invalid frame");
        }
        // otherwise do something
        else
        {
          // get type of frame
          uint8_t frameType = j->at("FRAME TYPE").get<int>();
          // switch based on type of frame
          switch (frameType)
          {
          // rx response                                  -- transmit request will include data -> means this is an outlet action
          case 0x90:
          {
            performOutletAction(j);
            break;
          }
          // explicit rx response                         -- transmit request will include data -> means this is an outlet action
          case 0x91:
          {
            performOutletAction(j);
            break;
          }
          // all other cases                              -- other operations deal with XBee behavior -> do not need ESP32's attention
          default:
            break;
          };
          vTaskDelay(1);
        }
        vTaskDelay(1);

        break;
      }
        // Event of HW FIFO overflow detected
      case UART_FIFO_OVF:
      {
        ESP_LOGI(TAG, "hw fifo overflow");
        // If fifo overflow happened, you should consider adding flow control for your application.
        // The ISR has already reset the rx FIFO,
        // As an example, we directly flush the rx buffer here in order to read more data.
        uart_flush_input(ESP_XBEE_UART);
        xQueueReset(uart0_queue);
        break;
      }
      // Event of UART ring buffer full
      case UART_BUFFER_FULL:
      {
        ESP_LOGI(TAG, "ring buffer full");
        // If buffer full happened, you should consider increasing your buffer size
        // As an example, we directly flush the rx buffer here in order to read more data.
        uart_flush_input(ESP_XBEE_UART);
        xQueueReset(uart0_queue);
        break;
      }
      // Event of UART RX break detected
      case UART_BREAK:
      {
        ESP_LOGI(TAG, "uart rx break");
        break;
      }
      // Event of UART parity check error
      case UART_PARITY_ERR:
      {
        ESP_LOGI(TAG, "uart parity error");
        break;
      }
      // Event of UART frame error
      case UART_FRAME_ERR:
      {
        ESP_LOGI(TAG, "uart frame error");
        break;
      // UART_PATTERN_DET
      case UART_PATTERN_DET:
        uart_get_buffered_data_len(ESP_XBEE_UART, &buffered_size);
        int pos = uart_pattern_pop_pos(ESP_XBEE_UART);
        ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
        if (pos == -1)
        {
          // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
          // record the position. We should set a larger queue size.
          // As an example, we directly flush the rx buffer here.
          uart_flush_input(ESP_XBEE_UART);
        }
        else
        {
          uart_read_bytes(ESP_XBEE_UART, dtmp, pos, 100 / portTICK_PERIOD_MS);
          uint8_t pat[PATTERN_CHR_NUM + 1];
          memset(pat, 0, sizeof(pat));
          uart_read_bytes(ESP_XBEE_UART, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
          ESP_LOGI(TAG, "read data: %s", dtmp);
          ESP_LOGI(TAG, "read pat : %s", pat);
        }
        break;
      }
      // Others
      default:
      {
        ESP_LOGI(TAG, "uart event type: %d", event.type);
        break;
      }
        vTaskDelay(1);
      }
      vTaskDelay(1);
    }
    vTaskDelay(1);
  }

  free(dtmp);
  dtmp = NULL;
  vTaskDelete(NULL);
}
/*
 * xbee frames arrive in the following format
        {
            "FRAME TYPE", X,                                    -- type of xbee frame, IE at command response, transmit request, etc
            "FRAME OVERHEAD", {                                 -- data relevant to xbee protocol, IE frame ID, destination, etc
                XXX, XXX
                .
                .
                .
            },
            "FRAME DATA", {                                     -- data to perform outlwt interactions with
                "data"{                                         -- data to act on, necessary data for operations will be found here in the expected key-value pairs
                    "value", X,
                    .
                    .
                }
                "op", x,                                        -- operation, IE toggle receptacles, set power limit, etc
            }
        }
*/

// perform an action in the smart power outlet based on the contents of recieved frame
void performOutletAction(json *j)
{
  json json_payload = (*j)["FRAME DATA"];

  if (!json_payload["op"].is_null() && json_payload["op"].is_number())
  {
    //  get type of JSON
    int json_type = json_payload["op"].get<int>();
    // get data from json packet
    json json_data = json_payload["data"];
    switch (json_type)
    {
    // outlet will not recieve measurement packets, ignore type == 0
    // handle changes in receptacle state
    case 1:
    {
      cout << "Case 1, Power data: " << std::endl;
      break;
    }
    // handle setting maximum instaneous power draw
    case 4:
    {
      cout << "Case 4, Time Request: " << std::endl;

      time_t now;
      time(&now);
      std::string stringTime = std::to_string(now);
      cout << json_payload << std::endl;

      uint64_t destAddr = (*j)["FRAME OVERHEAD"]["DST64"].get<uint64_t>();
      uint32_t destAddrLowOrder = (*j)["FRAME OVERHEAD"]["DST16"].get<uint32_t>();

      json j = {
          {"op", 4},
          "data",
          {"s", now, "us", 0, "tz", "EST+5EDT,M3.2.0/2,M11.1.0/2"}};

      std::string message = j.dump();

      std::vector<uint8_t> *messageUART = formTXFrame(message, destAddr, destAddrLowOrder, NULL, NULL);

      // timeval tv;
      // tv.tv_sec = now;
      // sntp_sync_time(&tv);

      int response = uart_write_bytes(ESP_XBEE_UART, &messageUART, messageUART->size());
      cout << "Uart bytes written " << response << std::endl;

      break;
    }
    default:
    {
      cout << "Default case: " << std::endl;
      break;
    }
    break;
    };
  };
};
