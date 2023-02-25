// Joseph Garro Kyrolos Melek
// jmg289@uakron.edu
// 2/14/2023
// v1.0
// firmware for esp32 and xbee based smart power outlet
// resources used https://github.com/theElementZero/ESP32-UART-interrupt-handling/blob/master/uart_interrupt.c
//---------------------------------
// additional classes for functionality
#include "xbee_api.hpp"
#include "json.hpp"
#include "WiFiSetup.h"
#include "timeSetup.hpp"
#include "HTTPServer.hpp"

// esp32 classes
#include "stdio.h"                                      // standard io
#include "driver/gpio.h"                                // esp GPIO pin control
#include "driver/uart.h"                                // esp UART driver
#include "driver/spi_master.h"                          // esp SPI driver
#include "driver/spi_common.h"
#include "freertos/FreeRTOS.h"                          // freeRTOS for multitasking
#include "freertos/task.h"                              // create and schedudle tasks
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_sntp.h"
#include "nvs_flash.h"

// c++ classes
#include <iostream>
#include <queue>
#include <ctime>
#include <time.h>
#include <map>
#include <tuple>

// link json class
using json = nlohmann::json;

//---------------------------------
// global definitions


// UART definitions
#define BUFFER_SIZE (1024 * 4)                          // hold 2048 bytes in each buffer
static QueueHandle_t xbee_queue;                        // queue to handle xbee events

// queue for pointers to incoming XBEE frames
std::queue<std::vector<uint8_t>> xbee_incoming;
// queue for pointers to outgoing XBEE frames
std::queue<std::vector<uint8_t>> xbee_outgoing;

// map to hold outlet names
std::map<std::string, std::tuple<uint16_t, uint64_t>> outletZigbeeAddress;

// GPIO pin definitions
#define XBEE_UART (UART_NUM_2)                          // uart2 to communicate between xbee and esp32
#define XBEE_UART_RX (GPIO_NUM_18)                      // uart2 TX
#define XBEE_UART_TX (GPIO_NUM_19)                      // uart2 RX

// logging tags
static const char *XBEE_TAG = "xbee uart";
static const char *PWIC_TAG = "PWIC uart";
static const char *RECEPTACLE_TAG = "receptacle state";
static const char *MAX_INSTANTANEOUS_POWER_DRAW_TAG = "set max instantanous power draw";
static const char *MAX_SUSTAINED_POWER_DRAW_TAG = "set max sustained power draw";
static const char *SET_SYSTEM_TIME = "set system time";
static const char *GET_SNTP_TIME = "get time";
static const char *PARSE_FRAME = "parse xbee frame";
static const char *SETUP = "setup";

// struct to store maeausrements from the PWICs
struct PWIC_measurements{
    float instantaneousVoltage;
    float instantaneousCurrent;
    float instantaneousPower;
    float powerFactor;
};

//---------------------------------
// function definitions

// configure UART connection to xbee module
static void xbee_uart_init(void){
    ESP_LOGI(XBEE_TAG, "configuring xbee uart connection");
    // UART configuration settings
    const uart_config_t xbee_uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB};

    // install UART driver
    ESP_ERROR_CHECK(uart_driver_install(XBEE_UART, BUFFER_SIZE, BUFFER_SIZE, 64, &xbee_queue, 0));                  // install UART driver on pins connected to xbee, buffer of 2048 bytes, event queue enabled
    ESP_ERROR_CHECK(uart_param_config(XBEE_UART, &xbee_uart_config));                                               // write xbee_uart_config to xbee UART
    ESP_ERROR_CHECK(uart_set_pin(XBEE_UART, XBEE_UART_TX, XBEE_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));   // assign TX and RX pins to xbee UART
};

// UART event handler for XBEE
// synthesized from https://github.com/espressif/esp-idf/blob/49551cc48cb3cdd5563059028749616de313f0ec/examples/peripherals/uart/uart_events/main/uart_events_example_main.c
static void xbee_uart_event_task(void *pvParameters){
    uart_event_t xbee_event;                                                                // hold UART event
    uint8_t* dtmp = (uint8_t*) malloc(BUFFER_SIZE);                                         // temporary buffer
    for(;;){
        // activate when a UART event is detected
        if(xQueueReceive(xbee_queue, (void*)&xbee_event, (TickType_t)portMAX_DELAY)){
            size_t eventsize = xbee_event.size;
            bzero(dtmp, BUFFER_SIZE); 
            ESP_LOGI(XBEE_TAG, "uart[%d] event:", XBEE_UART);                               // zero buffer
            switch(xbee_event.type){                                                        // handle different UART events
                // read incoming UART data
                case UART_DATA:
                    ESP_LOGI(XBEE_TAG, "[UART DATA]: %d", eventsize);
                    uart_read_bytes(XBEE_UART, dtmp, eventsize, portMAX_DELAY);             // write data to dtmp
                        xbee_incoming.push(std::vector<uint8_t>(dtmp, dtmp + eventsize));   // push frame to incoming xbee queue              
                    break;

                // HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(XBEE_TAG, "hw fifo overflow");
                    uart_flush_input(XBEE_UART);
                    xQueueReset(xbee_queue);
                    break;

                // UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(XBEE_TAG, "ring buffer full");
                    uart_flush_input(XBEE_UART);
                    xQueueReset(xbee_queue);
                    break;

                // UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(XBEE_TAG, "uart rx break");
                    break;

                // UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(XBEE_TAG, "uart parity error");
                    break;

                // UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(XBEE_TAG, "uart frame error");
                    break;

                default:
                    ESP_LOGI(XBEE_TAG, "xbee UART event: %d", xbee_event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
};

// perform an action in the hub based on the contents of recieved frame
void performHubAction(json* json_object){
    json frame_payload = (*json_object)["FRAME DATA"];
    json frame_overhead = (*json_object)["FRAME OVERHEAD"];
    if(!frame_payload["op"].is_null() && frame_payload["op"].is_number()){
        //  get type of JSON
        int frame_type = frame_payload["op"].get<int>();
        // get data from json packet
        switch(frame_type){
             // outlet will not recieve measurement packets, ignore type == 0
             // process measurements
             case 101:
                //toggleReceptacles(json_data);
                break;
                    
            //handle setting maximum instaneous power draw
            case 102:
                //setMaximumInstantaneousPowerDraw(json_data);
                break;

            // handle maximum sustained power draw
            case 103:
                //setMaximumSustainedPowerDraw(json_data);
                break;

            // handle time set
            case 104:
                returnTime(frame_overhead);
                break;
            default:
                break;
       };       
    };
};

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
// determine action to take based on recieved XBEE frame
static void parseFrame(void *pvParameters){
    for(;;){
        // work on existing xbee frames
        while(!xbee_incoming.empty()){
            // get oldest xbee frame
            std::vector<uint8_t> xbee_frame = xbee_incoming.front();                               
            // remove xbee frame from queue                    
            xbee_incoming.pop();
            // copy vector to a uint8_t array
            size_t dataSize = xbee_frame.size();
            uint8_t carray[dataSize];
            std::copy(xbee_frame.begin(), xbee_frame.end(), carray);
            // json with all information about xbee frame
            json* j = readFrame(carray);

            // handle error cases
            // unrecognized frame
            if(*j == -1){
                ESP_LOGI(PARSE_FRAME, "recieved an unrecognized frame");
            }
            // invalid frame
            else if(*j == -2){
                ESP_LOGI(PARSE_FRAME, "recieved an invalid frame");
            }
            // otherwise do something
            else{
                // get type of frame
                uint8_t frameType = j->at("FRAME TYPE").get<int>();
                // should I change these to dynamiclly allocated vars????
                //json frame_payload = (*j)["FRAME DATA"];
                //json frame_overhead = (*j)["FRAME OVERHEAD"];
                // switch based on type of frame
                switch(frameType){
                    // rx response                                  -- transmit request will include data -> means this is an outlet action
                    case 0x90:                                      
                        performHubAction(j);
                        break;
                    
                    // explicit rx response                         -- transmit request will include data -> means this is an outlet action
                    case 0x91:
                        performHubAction(j);
                        break;
                    
                    // add support for xbee configuration???

                    // all other cases                              -- other operations deal with XBee behavior -> do not need ESP32's attention
                    default:                                        
                        break;
                };
                vTaskDelay(1);
            }
            vTaskDelay(1);
        }
    vTaskDelay(1);       
    }
};

// send xbee frames stored in queue
static void sendFrame(void *pvParameters){
    for(;;){
        // work on existing xbee frames
        while(!xbee_outgoing.empty()){
            // get oldest xbee frame
            std::vector<uint8_t> xbee_frame = xbee_outgoing.front();                               
            // remove xbee frame from queue                    
            xbee_outgoing.pop();
            // copy vector to a uint8_t array
            uart_write_bytes(XBEE_UART, xbee_frame.data(), xbee_frame.size());
            vTaskDelay(1);
        }
        vTaskDelay(1);
    }
};

// main function
extern "C" void app_main() {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // initalize wifi 
    wifi_init_sta();
    // get time from SNTP server
    obtain_time();

    // initalize serial connections
    xbee_uart_init();                                                                       // initalize xbee UART connection

    // webserver
    start_webserver();

    // begin multitasking
    xTaskCreatePinnedToCore(xbee_uart_event_task, "handle xbee", 4*1024, NULL, 12, NULL, 0);
    xTaskCreatePinnedToCore(parseFrame, "parse incoming frames", 2*32768, NULL, 13, NULL, 0);
    xTaskCreatePinnedToCore(sendFrame, "parse incoming frames", 32768, NULL, 13, NULL, 0);

}
