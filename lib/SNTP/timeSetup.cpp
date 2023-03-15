#include <esp_sntp.h>
#include <esp_log.h>
#include "timeSetup.hpp"
#include "json.hpp"
#include <queue>
#include "xbee_api.hpp"
#include <map>
#include <tuple>
#include <iostream>

// tag for time library
static const char *GET_SNTP_TIME = "get time";

extern std::queue<std::vector<uint8_t>> xbee_outgoing;
extern std::map<std::string, std::tuple<uint16_t, uint64_t>> outletZigbeeAddress;
extern std::map<uint64_t, std::string> zigbeeAddressOutlet;

void time_sync_notification_cb(struct timeval *tv)
{
  ESP_LOGI(GET_SNTP_TIME, "Notification of a time synchronization event");
  time_t now;
  struct tm *timeinfo;

  time(&now);
  // Set timezone to Eastern Standard Time
  setenv("TZ", "EST5EDT,M3.2.0,M11.1.0", 1);
  tzset();
  timeinfo = localtime(&now);
  printf("Current local time and date: %s", asctime(timeinfo));
}

static void initialize_sntp(void)
{
  ESP_LOGI(GET_SNTP_TIME, "Initializing SNTP");
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");
  sntp_set_time_sync_notification_cb(time_sync_notification_cb);
  sntp_init();
}

void obtain_time()
{
  initialize_sntp();

  // wait for time to be set
  time_t now = 0;
  struct tm timeinfo = {};
  int retry = 0;
  const int retry_count = 10;
  while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count)
  {
    ESP_LOGI(GET_SNTP_TIME, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
  time(&now);
  localtime_r(&now, &timeinfo);
}

// return current time
void returnTime(json frame_overhead_)
{
  // get return info
  uint64_t destAddr = frame_overhead_["DST64"].get<uint64_t>();
  uint16_t destAddrLowOrder = frame_overhead_["DST16"].get<uint32_t>();

  std::string outletName = "-1";
  if (!zigbeeAddressOutlet.count(destAddr))
    outletZigbeeAddress[outletName] = std::make_tuple(destAddrLowOrder, destAddr);
  zigbeeAddressOutlet[destAddr] = outletName;

  // get current time
  time_t now;
  time(&now);
  std::string stringTime = std::to_string(now);

  // json object to hold time information
  json j = {
      {"op", 4},
      {"data",
       {{"s", now}, {"us", 0}, {"tz", "EST+5EDT,M3.2.0/2,M11.1.0/2"}}}};

  // get string representation of json object
  std::string json_bytes = j.dump();
  // add frame to outgoing xbee queue
  std::vector<uint8_t> *frame = formTXFrame(json_bytes, destAddr, destAddrLowOrder, NULL, NULL);
  xbee_outgoing.push(*frame);
};