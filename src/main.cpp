#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <nvs_flash.h>

#include "HTTPServer.h"
#include "WIFISetup.h"
#include "UARTHandler.hpp"
#include "timeSetup.h"

static const char *TAG = "HUB";

#define TOP_GPIO GPIO_NUM_32
#define BOTTOM_GPIO GPIO_NUM_33

extern "C"
{
  void app_main(void)
  {
    ESP_LOGI(TAG, "Hub Main");
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    gpio_set_direction(TOP_GPIO, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(BOTTOM_GPIO, GPIO_MODE_INPUT_OUTPUT);

    xbee_uart_setup();
    wifi_init_sta();

    obtain_time();

    start_webserver();
    xTaskCreatePinnedToCore(uart_event_task, "uart_event_task", 32768, NULL, 12, NULL, 1);
  }
}
