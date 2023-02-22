#include <esp_sntp.h>
#include <esp_log.h>
#include <time.h>

static const char *TAG = "Time";

void time_sync_notification_cb(struct timeval *tv)
{
  ESP_LOGI(TAG, "Notification of a time synchronization event");
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
  ESP_LOGI(TAG, "Initializing SNTP");
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
  struct tm timeinfo = {0};
  int retry = 0;
  const int retry_count = 10;
  while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count)
  {
    ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
  time(&now);
  localtime_r(&now, &timeinfo);
}