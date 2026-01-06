/**
 * @file example_uart.c
 * @brief UART transport example
 * 
 * This example demonstrates how to use the UART transport for
 * serial communication. It configures a UART port and echoes
 * back any received data.
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "transport.h"
#include "transport_uart.h"

static const char *TAG = "example_uart";

#define UART_TX_PIN     17
#define UART_RX_PIN     16
#define UART_BAUD_RATE  115200

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
            ESP_LOGI(TAG, "UART opened");
            break;
            
        case TRANSPORT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "UART closed");
            break;
            
        case TRANSPORT_EVENT_DATA_RECEIVED:
            ESP_LOGI(TAG, "Received %zu bytes", event->data_received.len);
            break;
            
        case TRANSPORT_EVENT_ERROR:
            ESP_LOGE(TAG, "Error: %s", event->error.message);
            break;
            
        case TRANSPORT_EVENT_STATE_CHANGED:
            ESP_LOGI(TAG, "State: %s -> %s",
                     transport_state_to_str(event->state_changed.old_state),
                     transport_state_to_str(event->state_changed.new_state));
            break;
            
        default:
            break;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== UART Transport Example ===");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    uart_transport_t *uart = uart_transport_create();
    if (uart == NULL) {
        ESP_LOGE(TAG, "Failed to create UART transport");
        return;
    }
    
    uart_transport_config_t config;
    uart_transport_config_default(&config);
    config.uart_num = 1;
    config.baud_rate = UART_BAUD_RATE;
    config.tx_pin = UART_TX_PIN;
    config.rx_pin = UART_RX_PIN;
    config.base.event_callback = on_transport_event;
    config.base.user_data = NULL;
    
    transport_err_t err = transport_init(&uart->base, &config);
    if (err != TRANSPORT_OK) {
        ESP_LOGE(TAG, "UART init failed: %s", transport_err_to_str(err));
        uart_transport_destroy(uart);
        return;
    }
    
    ESP_LOGI(TAG, "UART initialized, opening port...");
    
    err = transport_connect(&uart->base, NULL, 0);
    if (err != TRANSPORT_OK) {
        ESP_LOGE(TAG, "UART open failed: %s", transport_err_to_str(err));
        transport_deinit(&uart->base);
        uart_transport_destroy(uart);
        return;
    }
    
    ESP_LOGI(TAG, "UART%d opened at %d baud", config.uart_num, config.baud_rate);
    ESP_LOGI(TAG, "TX: GPIO%d, RX: GPIO%d", UART_TX_PIN, UART_RX_PIN);
    
    uint8_t buffer[256];
    size_t bytes_read;
    
    const char *welcome = "UART Transport Ready!\r\n";
    transport_write(&uart->base, (uint8_t *)welcome, strlen(welcome), NULL, 1000);
    
    while (1) {
        err = transport_read(&uart->base, buffer, sizeof(buffer) - 1, 
                             &bytes_read, 1000);
        
        if (err == TRANSPORT_OK && bytes_read > 0) {
            buffer[bytes_read] = '\0';
            ESP_LOGI(TAG, "Received %zu bytes: %s", bytes_read, buffer);
            
            const char *response = "Echo: ";
            transport_write(&uart->base, (uint8_t *)response, strlen(response), 
                           NULL, 1000);
            transport_write(&uart->base, buffer, bytes_read, NULL, 1000);
            transport_write(&uart->base, (uint8_t *)"\r\n", 2, NULL, 1000);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
