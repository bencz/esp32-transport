/**
 * @file example_multi_transport.c
 * @brief Multi-transport simultaneous usage example
 * 
 * This example demonstrates how to use multiple transports simultaneously:
 * - Receive data from any transport
 * - Forward/broadcast data to other transports
 * - Use all transports concurrently
 * 
 * Use case: A gateway that receives data via Bluetooth and forwards
 * it to WiFi TCP, or receives from UART and broadcasts to all others.
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
#include "transport_bt_spp.h"
#include "transport_ble.h"
#include "transport_wifi_tcp.h"
#include "transport_uart.h"
#include "transport_manager.h"

static const char *TAG = "example_multi";

#define WIFI_SSID       "your_wifi_ssid"
#define WIFI_PASSWORD   "your_wifi_password"
#define TCP_PORT        3333
#define UART_TX_PIN     17
#define UART_RX_PIN     16

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static transport_handle_t h_spp = TRANSPORT_HANDLE_INVALID;
static transport_handle_t h_ble = TRANSPORT_HANDLE_INVALID;
static transport_handle_t h_tcp = TRANSPORT_HANDLE_INVALID;
static transport_handle_t h_uart = TRANSPORT_HANDLE_INVALID;

/**
 * @brief WiFi event handler
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief Initialize WiFi
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
}

/**
 * @brief Get transport name by handle
 */
static const char *get_transport_name(transport_handle_t handle)
{
    if (handle == h_spp) return "SPP";
    if (handle == h_ble) return "BLE";
    if (handle == h_tcp) return "TCP";
    if (handle == h_uart) return "UART";
    return "Unknown";
}

/**
 * @brief Global callback for data received on ANY transport
 * 
 * This is called automatically when data arrives on any registered transport.
 * Perfect for implementing a gateway/bridge between transports.
 */
static void on_data_received(transport_handle_t handle,
                              const uint8_t *data, size_t len,
                              void *user_data)
{
    (void)user_data;
    
    ESP_LOGI(TAG, "Data received on [%s]: %zu bytes", 
             get_transport_name(handle), len);
    
    // Example: Broadcast received data to ALL OTHER connected transports
    size_t sent_count = transport_manager_broadcast(data, len, handle, 1000);
    
    if (sent_count > 0) {
        ESP_LOGI(TAG, "Forwarded to %zu other transport(s)", sent_count);
    }
}

/**
 * @brief Print status of all transports
 */
static void print_status(void)
{
    ESP_LOGI(TAG, "=== Transport Status ===");
    ESP_LOGI(TAG, "Connected transports: %zu", transport_manager_connected_count());
    
    transport_handle_t handles[] = {h_spp, h_ble, h_tcp, h_uart};
    const char *names[] = {"SPP", "BLE", "TCP", "UART"};
    
    for (int i = 0; i < 4; i++) {
        if (handles[i] != TRANSPORT_HANDLE_INVALID) {
            bool connected = transport_manager_is_connected(handles[i]);
            ESP_LOGI(TAG, "  [%s] %s", names[i], connected ? "CONNECTED" : "disconnected");
            
            if (connected) {
                transport_stats_t stats;
                if (transport_manager_get_stats(handles[i], &stats) == TRANSPORT_OK) {
                    ESP_LOGI(TAG, "       TX: %llu bytes, RX: %llu bytes", 
                             stats.bytes_sent, stats.bytes_received);
                }
            }
        }
    }
}

/**
 * @brief Example 1: Read from any transport, forward to all others
 * 
 * This pattern is useful for creating a transparent bridge between
 * all communication interfaces.
 */
static void gateway_bridge_task(void *pvParameters)
{
    (void)pvParameters;
    
    uint8_t buffer[256];
    size_t bytes_read;
    transport_handle_t source;
    
    ESP_LOGI(TAG, "Gateway bridge task started");
    
    while (1) {
        // Read from ANY transport that has data
        transport_err_t err = transport_manager_read_any(
            buffer, sizeof(buffer) - 1,
            &bytes_read, &source,
            100  // Short timeout for responsive polling
        );
        
        if (err == TRANSPORT_OK && bytes_read > 0) {
            buffer[bytes_read] = '\0';
            
            ESP_LOGI(TAG, "[%s] -> Received: %s", 
                     get_transport_name(source), buffer);
            
            // Forward to all OTHER connected transports
            size_t forwarded = transport_manager_broadcast(
                buffer, bytes_read,
                source,  // Exclude source from broadcast
                1000
            );
            
            ESP_LOGI(TAG, "Forwarded to %zu transport(s)", forwarded);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Example 2: Specific routing - Bluetooth to TCP only
 * 
 * This pattern routes data from a specific source to a specific destination.
 */
static void bt_to_tcp_router_task(void *pvParameters)
{
    (void)pvParameters;
    
    uint8_t buffer[256];
    size_t bytes_forwarded;
    
    ESP_LOGI(TAG, "BT->TCP router task started");
    
    while (1) {
        // Check if both transports are connected
        if (transport_manager_is_connected(h_spp) && 
            transport_manager_is_connected(h_tcp)) {
            
            // Forward from SPP to TCP
            transport_err_t err = transport_manager_forward(
                h_spp,      // Source: Bluetooth SPP
                h_tcp,      // Destination: TCP
                buffer, sizeof(buffer),
                &bytes_forwarded,
                100
            );
            
            if (err == TRANSPORT_OK && bytes_forwarded > 0) {
                ESP_LOGI(TAG, "SPP -> TCP: %zu bytes", bytes_forwarded);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Example 3: Poll-based multi-transport handling
 * 
 * This pattern explicitly polls all transports and handles each one.
 */
static void poll_based_handler_task(void *pvParameters)
{
    (void)pvParameters;
    
    uint8_t buffer[256];
    size_t bytes_read;
    
    ESP_LOGI(TAG, "Poll-based handler task started");
    
    while (1) {
        // Poll all transports for available data
        transport_poll_result_t poll_results[TRANSPORT_MANAGER_MAX_TRANSPORTS];
        size_t count = transport_manager_poll(poll_results, TRANSPORT_MANAGER_MAX_TRANSPORTS);
        
        // Process each transport that has data
        for (size_t i = 0; i < count; i++) {
            transport_handle_t handle = poll_results[i].handle;
            size_t available = poll_results[i].bytes_available;
            
            ESP_LOGD(TAG, "[%s] has %zu bytes available", 
                     get_transport_name(handle), available);
            
            // Read from this specific transport
            transport_err_t err = transport_manager_read_handle(
                handle,
                buffer, sizeof(buffer) - 1,
                &bytes_read,
                100
            );
            
            if (err == TRANSPORT_OK && bytes_read > 0) {
                buffer[bytes_read] = '\0';
                
                // Handle data based on source
                if (handle == h_uart) {
                    // UART data: broadcast to wireless transports
                    ESP_LOGI(TAG, "UART command: %s", buffer);
                    transport_manager_broadcast(buffer, bytes_read, h_uart, 1000);
                    
                } else if (handle == h_spp || handle == h_ble) {
                    // Bluetooth data: send to TCP and UART
                    ESP_LOGI(TAG, "BT data: %s", buffer);
                    transport_manager_write_handle(h_tcp, buffer, bytes_read, NULL, 1000);
                    transport_manager_write_handle(h_uart, buffer, bytes_read, NULL, 1000);
                    
                } else if (handle == h_tcp) {
                    // TCP data: send to Bluetooth
                    ESP_LOGI(TAG, "TCP data: %s", buffer);
                    transport_manager_write_handle(h_spp, buffer, bytes_read, NULL, 1000);
                    transport_manager_write_handle(h_ble, buffer, bytes_read, NULL, 1000);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Initialize all transports
 */
static void init_transports(void)
{
    // Initialize Bluetooth SPP
    bt_spp_transport_t *spp = bt_spp_transport_create();
    if (spp != NULL) {
        bt_spp_config_t spp_config;
        bt_spp_config_default(&spp_config);
        spp_config.device_name = "ESP32_Gateway_SPP";
        
        if (transport_init(&spp->base, &spp_config) == TRANSPORT_OK) {
            h_spp = transport_manager_register(&spp->base, "spp");
            bt_spp_listen(spp);
            ESP_LOGI(TAG, "SPP initialized (handle=%d)", h_spp);
        }
    }
    
    // Initialize BLE
    ble_transport_t *ble = ble_transport_create();
    if (ble != NULL) {
        ble_config_t ble_config;
        ble_config_default(&ble_config);
        ble_config.device_name = "ESP32_Gateway_BLE";
        
        if (transport_init(&ble->base, &ble_config) == TRANSPORT_OK) {
            h_ble = transport_manager_register(&ble->base, "ble");
            ble_start_advertising(ble);
            ESP_LOGI(TAG, "BLE initialized (handle=%d)", h_ble);
        }
    }
    
    // Initialize WiFi TCP Server
    wifi_tcp_transport_t *tcp = wifi_tcp_transport_create();
    if (tcp != NULL) {
        wifi_tcp_config_t tcp_config;
        wifi_tcp_config_default(&tcp_config);
        tcp_config.role = WIFI_TCP_ROLE_SERVER;
        tcp_config.port = TCP_PORT;
        
        if (transport_init(&tcp->base, &tcp_config) == TRANSPORT_OK) {
            h_tcp = transport_manager_register(&tcp->base, "tcp");
            wifi_tcp_listen(tcp);
            ESP_LOGI(TAG, "TCP initialized (handle=%d), port %d", h_tcp, TCP_PORT);
        }
    }
    
    // Initialize UART
    uart_transport_t *uart = uart_transport_create();
    if (uart != NULL) {
        uart_transport_config_t uart_config;
        uart_transport_config_default(&uart_config);
        uart_config.uart_num = 1;
        uart_config.baud_rate = 115200;
        uart_config.tx_pin = UART_TX_PIN;
        uart_config.rx_pin = UART_RX_PIN;
        
        if (transport_init(&uart->base, &uart_config) == TRANSPORT_OK) {
            if (transport_connect(&uart->base, NULL, 0) == TRANSPORT_OK) {
                h_uart = transport_manager_register(&uart->base, "uart");
                ESP_LOGI(TAG, "UART initialized (handle=%d)", h_uart);
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== Multi-Transport Gateway Example ===");
    ESP_LOGI(TAG, "Demonstrates simultaneous usage of all transports");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize WiFi
    wifi_init_sta();
    
    // Initialize transport manager
    transport_err_t err = transport_manager_init(NULL);
    if (err != TRANSPORT_OK) {
        ESP_LOGE(TAG, "Manager init failed: %s", transport_err_to_str(err));
        return;
    }
    
    // Set global data callback (optional - for automatic handling)
    transport_manager_set_data_callback(on_data_received, NULL);
    
    // Initialize all transports
    init_transports();
    
    // Print initial status
    print_status();
    
    // Choose ONE of these task patterns:
    
    // Option 1: Simple gateway bridge (forward everything to everything)
    xTaskCreate(gateway_bridge_task, "gateway", 4096, NULL, 5, NULL);
    
    // Option 2: Specific routing (BT -> TCP only)
    // xTaskCreate(bt_to_tcp_router_task, "bt_tcp", 4096, NULL, 5, NULL);
    
    // Option 3: Poll-based with custom routing logic
    // xTaskCreate(poll_based_handler_task, "poll_handler", 4096, NULL, 5, NULL);
    
    // Main loop: accept TCP connections and print status periodically
    wifi_tcp_transport_t *tcp = (wifi_tcp_transport_t *)transport_manager_get(h_tcp);
    uint32_t status_counter = 0;
    
    while (1) {
        // Accept TCP connections if not connected
        if (tcp != NULL && !transport_manager_is_connected(h_tcp)) {
            wifi_tcp_accept(tcp, 100);
        }
        
        // Print status every 30 seconds
        if (++status_counter >= 300) {
            print_status();
            status_counter = 0;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
