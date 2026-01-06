/**
 * @file example_multi_transport.c
 * @brief Multi-transport example using the transport manager
 * 
 * This example demonstrates how to use multiple transports simultaneously
 * with the transport manager. It registers all available transports and
 * allows switching between them at runtime.
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
 * @brief Print transport information
 */
static void print_transport_info(transport_t *transport, const char *name)
{
    char info[128];
    transport_get_info(transport, info, sizeof(info));
    ESP_LOGI(TAG, "[%s] %s", name, info);
}

/**
 * @brief Print all registered transports
 */
static void print_all_transports(void)
{
    transport_handle_t handles[TRANSPORT_MANAGER_MAX_TRANSPORTS];
    size_t count = transport_manager_list(handles, TRANSPORT_MANAGER_MAX_TRANSPORTS);
    
    ESP_LOGI(TAG, "=== Registered Transports (%zu) ===", count);
    for (size_t i = 0; i < count; i++) {
        transport_t *t = transport_manager_get(handles[i]);
        if (t != NULL) {
            ESP_LOGI(TAG, "  [%d] Type: %s, State: %s",
                     handles[i],
                     transport_type_to_str(transport_get_type(t)),
                     transport_state_to_str(transport_get_state(t)));
        }
    }
}

/**
 * @brief Print statistics for a transport
 */
static void print_stats(transport_handle_t handle, const char *name)
{
    transport_stats_t stats;
    if (transport_manager_get_stats(handle, &stats) == TRANSPORT_OK) {
        ESP_LOGI(TAG, "[%s] Stats: TX=%llu bytes, RX=%llu bytes, Errors=%lu",
                 name, stats.bytes_sent, stats.bytes_received, 
                 (unsigned long)stats.errors);
    }
}

/**
 * @brief Process received command
 */
static void process_command(const char *cmd)
{
    if (strcmp(cmd, "spp") == 0 || strcmp(cmd, "SPP") == 0) {
        if (h_spp != TRANSPORT_HANDLE_INVALID) {
            transport_manager_set_active(h_spp);
            ESP_LOGI(TAG, "Switched to Bluetooth SPP");
        }
    } else if (strcmp(cmd, "ble") == 0 || strcmp(cmd, "BLE") == 0) {
        if (h_ble != TRANSPORT_HANDLE_INVALID) {
            transport_manager_set_active(h_ble);
            ESP_LOGI(TAG, "Switched to BLE");
        }
    } else if (strcmp(cmd, "tcp") == 0 || strcmp(cmd, "TCP") == 0) {
        if (h_tcp != TRANSPORT_HANDLE_INVALID) {
            transport_manager_set_active(h_tcp);
            ESP_LOGI(TAG, "Switched to WiFi TCP");
        }
    } else if (strcmp(cmd, "uart") == 0 || strcmp(cmd, "UART") == 0) {
        if (h_uart != TRANSPORT_HANDLE_INVALID) {
            transport_manager_set_active(h_uart);
            ESP_LOGI(TAG, "Switched to UART");
        }
    } else if (strcmp(cmd, "list") == 0) {
        print_all_transports();
    } else if (strcmp(cmd, "stats") == 0) {
        if (h_spp != TRANSPORT_HANDLE_INVALID) print_stats(h_spp, "SPP");
        if (h_ble != TRANSPORT_HANDLE_INVALID) print_stats(h_ble, "BLE");
        if (h_tcp != TRANSPORT_HANDLE_INVALID) print_stats(h_tcp, "TCP");
        if (h_uart != TRANSPORT_HANDLE_INVALID) print_stats(h_uart, "UART");
    } else if (strcmp(cmd, "help") == 0) {
        const char *help = 
            "Commands:\r\n"
            "  spp   - Switch to Bluetooth SPP\r\n"
            "  ble   - Switch to BLE\r\n"
            "  tcp   - Switch to WiFi TCP\r\n"
            "  uart  - Switch to UART\r\n"
            "  list  - List all transports\r\n"
            "  stats - Show statistics\r\n"
            "  help  - Show this help\r\n";
        transport_manager_write((uint8_t *)help, strlen(help), NULL, 1000);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== Multi-Transport Example ===");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    wifi_init_sta();
    
    transport_err_t err = transport_manager_init(NULL);
    if (err != TRANSPORT_OK) {
        ESP_LOGE(TAG, "Manager init failed: %s", transport_err_to_str(err));
        return;
    }
    
    bt_spp_transport_t *spp = bt_spp_transport_create();
    ble_transport_t *ble = ble_transport_create();
    wifi_tcp_transport_t *tcp = wifi_tcp_transport_create();
    uart_transport_t *uart = uart_transport_create();
    
    if (spp != NULL) {
        bt_spp_config_t spp_config;
        bt_spp_config_default(&spp_config);
        spp_config.device_name = "ESP32_Multi_SPP";
        
        if (transport_init(&spp->base, &spp_config) == TRANSPORT_OK) {
            h_spp = transport_manager_register(&spp->base, "spp");
            bt_spp_listen(spp);
            ESP_LOGI(TAG, "SPP registered (handle=%d)", h_spp);
        }
    }
    
    if (ble != NULL) {
        ble_config_t ble_config;
        ble_config_default(&ble_config);
        ble_config.device_name = "ESP32_Multi_BLE";
        
        if (transport_init(&ble->base, &ble_config) == TRANSPORT_OK) {
            h_ble = transport_manager_register(&ble->base, "ble");
            ble_start_advertising(ble);
            ESP_LOGI(TAG, "BLE registered (handle=%d)", h_ble);
        }
    }
    
    if (tcp != NULL) {
        wifi_tcp_config_t tcp_config;
        wifi_tcp_config_default(&tcp_config);
        tcp_config.role = WIFI_TCP_ROLE_SERVER;
        tcp_config.port = TCP_PORT;
        
        if (transport_init(&tcp->base, &tcp_config) == TRANSPORT_OK) {
            h_tcp = transport_manager_register(&tcp->base, "tcp");
            wifi_tcp_listen(tcp);
            ESP_LOGI(TAG, "TCP registered (handle=%d), port %d", h_tcp, TCP_PORT);
        }
    }
    
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
                ESP_LOGI(TAG, "UART registered (handle=%d)", h_uart);
            }
        }
    }
    
    print_all_transports();
    
    if (h_uart != TRANSPORT_HANDLE_INVALID) {
        transport_manager_set_active(h_uart);
        ESP_LOGI(TAG, "Default active transport: UART");
    } else if (h_ble != TRANSPORT_HANDLE_INVALID) {
        transport_manager_set_active(h_ble);
        ESP_LOGI(TAG, "Default active transport: BLE");
    }
    
    uint8_t buffer[256];
    size_t bytes_read;
    
    ESP_LOGI(TAG, "Ready! Send 'help' for commands.");
    
    while (1) {
        transport_t *active = transport_manager_get_active();
        
        if (active != NULL && transport_get_state(active) == TRANSPORT_STATE_CONNECTED) {
            err = transport_manager_read(buffer, sizeof(buffer) - 1, &bytes_read, 500);
            
            if (err == TRANSPORT_OK && bytes_read > 0) {
                buffer[bytes_read] = '\0';
                
                char *newline = strchr((char *)buffer, '\r');
                if (newline) *newline = '\0';
                newline = strchr((char *)buffer, '\n');
                if (newline) *newline = '\0';
                
                ESP_LOGI(TAG, "Received: %s", buffer);
                
                process_command((char *)buffer);
                
                if (strncmp((char *)buffer, "spp", 3) != 0 &&
                    strncmp((char *)buffer, "ble", 3) != 0 &&
                    strncmp((char *)buffer, "tcp", 3) != 0 &&
                    strncmp((char *)buffer, "uart", 4) != 0 &&
                    strcmp((char *)buffer, "list") != 0 &&
                    strcmp((char *)buffer, "stats") != 0 &&
                    strcmp((char *)buffer, "help") != 0) {
                    
                    const char *response = "Echo: ";
                    transport_manager_write((uint8_t *)response, strlen(response), NULL, 1000);
                    transport_manager_write(buffer, strlen((char *)buffer), NULL, 1000);
                    transport_manager_write((uint8_t *)"\r\n", 2, NULL, 1000);
                }
            }
        }
        
        if (h_tcp != TRANSPORT_HANDLE_INVALID && tcp != NULL) {
            if (transport_get_state(&tcp->base) != TRANSPORT_STATE_CONNECTED) {
                wifi_tcp_accept(tcp, 100);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
