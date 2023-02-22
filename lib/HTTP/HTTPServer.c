#include <esp_http_server.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <time.h>
static const char *TAG = "HTTP Server";

#define TOP_GPIO GPIO_NUM_32
#define BOTTOM_GPIO GPIO_NUM_33

//  URI Handler functions
/* Our URI handler function to be called during GET /uri request */
esp_err_t change_rec_state(httpd_req_t *req)
{
  if (strcmp(req->uri, "/top/on") == 0)
  {
    // Write data to UART.
    // char *test_str = "address top on \n";
    // uart_write_bytes(ESP_XBEE_UART, (const char *)test_str, strlen(test_str));

    gpio_set_level(TOP_GPIO, 1);
    /* Send a simple response */
    const char resp[] = "Top Outlet Turned On";
    time_t now;
    char strftime_buf[64];
    struct tm timeinfo;

    time(&now);
    // Set timezone to Eastern Standard Time
    setenv("TZ", "GMT+5", 1);
    tzset();

    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Cleveland is: %s", strftime_buf);

    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
  }
  else if (strcmp(req->uri, "/top/off") == 0)
  {
    gpio_set_level(TOP_GPIO, 0);
    /* Send a simple response */
    const char resp[] = "Top Outlet Turned Off";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
  }
  else if (strcmp(req->uri, "/bottom/on") == 0)
  {
    gpio_set_level(BOTTOM_GPIO, 1);
    /* Send a simple response */
    const char resp[] = "Bottom Outlet Turned On";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
  }
  else if (strcmp(req->uri, "/bottom/off") == 0)
  {
    gpio_set_level(BOTTOM_GPIO, 0);
    /* Send a simple response */
    const char resp[] = "Bottom Outlet Turned Off";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
  }
  else
  {
    return ESP_FAIL;
  }
}

/* Our URI handler function to be called during POST /uri request */
esp_err_t post_handler(httpd_req_t *req)
{
  /* Destination buffer for content of HTTP POST request.
   * httpd_req_recv() accepts char* only, but content could
   * as well be any binary data (needs type casting).
   * In case of string data, null termination will be absent, and
   * content length would give length of string */
  char content[100];

  /* Truncate if content length larger than the buffer */
  size_t recv_size = req->content_len > sizeof(content) ? sizeof(content) : req->content_len; // MIN(req->content_len, sizeof(content));

  int ret = httpd_req_recv(req, content, recv_size);
  ESP_LOGI(TAG, "Post message: %s", content);
  if (ret <= 0)
  { /* 0 return value indicates connection closed */
    /* Check if timeout occurred */
    if (ret == HTTPD_SOCK_ERR_TIMEOUT)
    {
      /* In case of timeout one can choose to retry calling
       * httpd_req_recv(), but to keep it simple, here we
       * respond with an HTTP 408 (Request Timeout) error */
      httpd_resp_send_408(req);
    }
    /* In case of error, returning ESP_FAIL will
     * ensure that the underlying socket is closed */
    return ESP_FAIL;
  }

  /* Send a simple response */
  const char resp[] = "URI POST Response";
  httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

/* URI handler structure for top receptacle on */
httpd_uri_t uri_top_on = {
    .uri = "/top/on",
    .method = HTTP_GET,
    .handler = change_rec_state,
    .user_ctx = NULL};

/* URI handler structure for top receptacle off */
httpd_uri_t uri_top_off = {
    .uri = "/top/off",
    .method = HTTP_GET,
    .handler = change_rec_state,
    .user_ctx = NULL};

/* URI handler structure for bottom receptacle on */
httpd_uri_t uri_bottom_on = {
    .uri = "/bottom/on",
    .method = HTTP_GET,
    .handler = change_rec_state,
    .user_ctx = NULL};

/* URI handler structure for  bottom receptacle off */
httpd_uri_t uri_bottom_off = {
    .uri = "/bottom/off",
    .method = HTTP_GET,
    .handler = change_rec_state,
    .user_ctx = NULL};

/* URI handler structure for POST /uri */
httpd_uri_t uri_post = {
    .uri = "/uri",
    .method = HTTP_POST,
    .handler = post_handler,
    .user_ctx = NULL};

/* Function for starting the webserver */
httpd_handle_t start_webserver(void)
{
  /* Generate default configuration */
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  /* Empty handle to esp_http_server */
  httpd_handle_t server = NULL;

  /* Start the httpd server */
  if (httpd_start(&server, &config) == ESP_OK)
  {
    /* Register URI handlers */
    httpd_register_uri_handler(server, &uri_top_on);
    httpd_register_uri_handler(server, &uri_top_off);
    httpd_register_uri_handler(server, &uri_bottom_on);
    httpd_register_uri_handler(server, &uri_bottom_off);

    httpd_register_uri_handler(server, &uri_post);
  }
  /* If server failed to start, handle will be NULL */
  return server;
}

/* Function for stopping the webserver */
void stop_webserver(httpd_handle_t server)
{
  if (server)
  {
    /* Stop the httpd server */
    httpd_stop(server);
  }
}