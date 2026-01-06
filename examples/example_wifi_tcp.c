/**
 * @file example_wifi_tcp.c
 * @brief WiFi TCP transport example
 * 
 * This example demonstrates how to use the WiFi TCP transport for
 * network communication. It can operate as either a TCP server
 * waiting for connections or as a client connecting to a remote host.
 * 
 * Note: WiFi must be initialized and connected before using this transport.
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "transport.h"
#include "transport_wifi_tcp.h"

static const char *TAG = "example_wifi_tcp";

#define WIFI_SSID       "your_wifi_ssid"
#define WIFI_PASSWORD   "your_wifi_password"
#define TCP_PORT        3333

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

/**
 * @brief WiFi event handler
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi disconnected, reconnecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief Initialize WiFi in station mode
 */
static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to WiFi...");
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, 
                        pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TAG, "WiFi connected");
}

/**
 * @brief Callback for transport events
 */
static void on_transport_event(transport_t *transport, 
                                const transport_event_t *event, 
                                void *user_data)
{
    (void)transport;
    (void)user_data;
    
    switch (event->type) {
        case TRANSPORT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Client connected!");
            break;
            
        case TRANSPORT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "Client disconnected");
            break;
            
        case TRANSPORT_EVENT_DATA_RECEIVED:
            ESP_LOGI(TAG, "Received %zu bytes", event->data_received.len);
            break;
            
        case TRANSPORT_EVENT_ERROR:
            ESP_LOGE(TAG, "Error: %s", event->error.message);
            break;
            
        default:
            break;
    }
}

/**
 * @brief TCP Server example
 */
static void tcp_server_task(void *pvParameters)
{
    (void)pvParameters;
    
    wifi_tcp_transport_t *tcp = wifi_tcp_transport_create();
    if (tcp == NULL) {
        ESP_LOGE(TAG, "Failed to create TCP transport");
        vTaskDelete(NULL);
        return;
    }
    
    wifi_tcp_config_t config;
    wifi_tcp_config_default(&config);
    config.role = WIFI_TCP_ROLE_SERVER;
    config.port = TCP_PORT;
    config.base.event_callback = on_transport_event;
    
    transport_err_t err = transport_init(&tcp->base, &config);
    if (err != TRANSPORT_OK) {
        ESP_LOGE(TAG, "TCP init failed: %s", transport_err_to_str(err));
        wifi_tcp_transport_destroy(tcp);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Starting TCP server on port %d...", TCP_PORT);
    
    err = wifi_tcp_listen(tcp);
    if (err != TRANSPORT_OK) {
        ESP_LOGE(TAG, "TCP listen failed: %s", transport_err_to_str(err));
        transport_deinit(&tcp->base);
        wifi_tcp_transport_destroy(tcp);
        vTaskDelete(NULL);
        return;
    }
    
    uint8_t buffer[256];
    size_t bytes_read;
    
    while (1) {
        if (transport_get_state(&tcp->base) != TRANSPORT_STATE_CONNECTED) {
            ESP_LOGI(TAG, "Waiting for client connection...");
            err = wifi_tcp_accept(tcp, 30000);
            if (err == TRANSPORT_ERR_TIMEOUT) {
                continue;
            } else if (err != TRANSPORT_OK) {
                ESP_LOGE(TAG, "Accept failed: %s", transport_err_to_str(err));
                continue;
            }
            
            char remote_ip[16];
            uint16_t remote_port;
            wifi_tcp_get_remote_addr(tcp, remote_ip, &remote_port);
            ESP_LOGI(TAG, "Client connected from %s:%d", remote_ip, remote_port);
        }
        
        err = transport_read(&tcp->base, buffer, sizeof(buffer) - 1, 
                             &bytes_read, 1000);
        
        if (err == TRANSPORT_OK && bytes_read > 0) {
            buffer[bytes_read] = '\0';
            ESP_LOGI(TAG, "Received: %s", buffer);
            
            const char *response = "TCP Echo: ";
            transport_write(&tcp->base, (uint8_t *)response, strlen(response), 
                           NULL, 1000);
            transport_write(&tcp->base, buffer, bytes_read, NULL, 1000);
        } else if (err == TRANSPORT_ERR_NOT_CONNECTED) {
            ESP_LOGI(TAG, "Client disconnected");
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== WiFi TCP Transport Example ===");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    wifi_init_sta();
    
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
}
