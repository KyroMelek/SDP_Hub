#include <string.h>
#include <sys/param.h>
#include <stdlib.h>
#include <ctype.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_tls.h"
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include "esp_crt_bundle.h"
#endif
#include "iostream"
#if !CONFIG_IDF_TARGET_LINUX
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#endif

#include "esp_http_client.h"

#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 2048
static const char *TAG = "HTTP_CLIENT";

// Define client certificate
extern const char howsmyssl_com_root_cert_pem_start[] asm("_binary_howsmyssl_com_root_cert_pem_start");
extern const char howsmyssl_com_root_cert_pem_end[]   asm("_binary_howsmyssl_com_root_cert_pem_end");


esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static char *output_buffer;  // Buffer to store response of http request from event handler
    static int output_len;       // Stores number of bytes read
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
        {
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        }
        case HTTP_EVENT_ON_CONNECTED:
        {
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        }
        case HTTP_EVENT_HEADER_SENT:
        {
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        }
        case HTTP_EVENT_ON_HEADER:
        {
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        }
        case HTTP_EVENT_ON_DATA:
        {
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            /*
             *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
             *  However, event handler can also be used in case chunked encoding is used.
             */
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // If user_data buffer is configured, copy the response into the buffer
                int copy_len = 0;
                if (evt->user_data) {
                    copy_len = MIN(evt->data_len, (MAX_HTTP_OUTPUT_BUFFER - output_len));
                    if (copy_len) {
                        memcpy(evt->user_data + output_len, evt->data, copy_len);
                    }
                } else {
                    const int buffer_len = esp_http_client_get_content_length(evt->client);
                    if (output_buffer == NULL) {
                        output_buffer = (char *) malloc(buffer_len);
                        output_len = 0;
                        if (output_buffer == NULL) {
                            ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                            return ESP_FAIL;
                        }
                    }
                    copy_len = MIN(evt->data_len, (buffer_len - output_len));
                    if (copy_len) {
                        memcpy(output_buffer + output_len, evt->data, copy_len);
                    }
                }
                output_len += copy_len;
            }

            break;
        }
        case HTTP_EVENT_ON_FINISH:
        {
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            if (output_buffer != NULL) {
                // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
                // ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        }
        case HTTP_EVENT_DISCONNECTED:
        {
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error((esp_tls_error_handle_t)evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
                ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            if (output_buffer != NULL) {
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        }
        case HTTP_EVENT_REDIRECT:
        {
            ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
            esp_http_client_set_header(evt->client, "From", "user@example.com");
            esp_http_client_set_header(evt->client, "Accept", "text/html");
            esp_http_client_set_redirection(evt->client);
            break;
        }

    }
    return ESP_OK;
}

#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
void https_with_url(uint64_t address, std::string body)
{
    std::cout << "Entering Post method" << std::endl;
    std::string URL = "https://us-east-1.aws.data.mongodb-api.com/app/sdppoweroutlet-ogswv/endpoint/outlet?Address=" + std::to_string(address);
    std::cout << "URL is: " << URL << std::endl;
    char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};

    esp_http_client_config_t config = {};
    config.url = URL.c_str();//"https://us-east-1.aws.data.mongodb-api.com/app/sdppoweroutlet-ogswv/endpoint/outlet?Address=8";
    config.event_handler = _http_event_handler;
    config.user_data = local_response_buffer;
    config.cert_pem = howsmyssl_com_root_cert_pem_start;
    //config.port = 443;
    // config.is_async = true;
    // config.timeout_ms = 10000;
    //config.buffer_size = 


    std::cout << "Before Init handle" << std::endl;

    esp_http_client_handle_t client = esp_http_client_init(&config);

    std::cout << "Init handle" << std::endl;

    // POST
    const char *post_data = body.c_str(); //"{\"TopP\":\"6\", \"BottomP\":\"7\", \"EpochTime\":\"1680634492\"}";
    std::cout << "Data to be sent to DB: " << post_data << std::endl;
    //esp_http_client_set_url(client, "https://us-east-1.aws.data.mongodb-api.com/app/sdppoweroutlet-ogswv/endpoint/outlet?Address=6");
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    std::cout << "Before perform: " << std::endl;
    esp_err_t err = esp_http_client_perform(client);
    std::cout << "after perform: " << err << std::endl;

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %" PRIu64,
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}
#endif // CONFIG_MBEDTLS_CERTIFICATE_BUNDLE