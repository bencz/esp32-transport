/**
 * @file example_ble.c
 * @brief Bluetooth Low Energy transport example
 * 
 * This example demonstrates how to use the BLE transport for low-power
 * wireless communication. The ESP32 acts as a peripheral device with
 * a custom GATT service for bidirectional data transfer.
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "transport.h"
#include "transport_ble.h"

static const char *TAG = "example_ble";

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
            ESP_LOGI(TAG, "Central device connected!");
            break;
            
        case TRANSPORT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "Central device disconnected");
            break;
            
        case TRANSPORT_EVENT_DATA_RECEIVED:
            ESP_LOGI(TAG, "Received %zu bytes", event->data_received.len);
            ESP_LOG_BUFFER_HEXDUMP(TAG, event->data_received.data, 
                                   event->data_received.len, ESP_LOG_DEBUG);
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
    ESP_LOGI(TAG, "=== Bluetooth Low Energy Example ===");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ble_transport_t *ble = ble_transport_create();
    if (ble == NULL) {
        ESP_LOGE(TAG, "Failed to create BLE transport");
        return;
    }
    
    ble_config_t config;
    ble_config_default(&config);
    config.device_name = "ESP32_BLE_Demo";
    config.mtu = 512;
    config.base.event_callback = on_transport_event;
    config.base.user_data = NULL;
    
    transport_err_t err = transport_init(&ble->base, &config);
    if (err != TRANSPORT_OK) {
        ESP_LOGE(TAG, "BLE init failed: %s", transport_err_to_str(err));
        ble_transport_destroy(ble);
        return;
    }
    
    ESP_LOGI(TAG, "BLE initialized, starting advertising...");
    
    err = ble_start_advertising(ble);
    if (err != TRANSPORT_OK) {
        ESP_LOGE(TAG, "BLE advertising failed: %s", transport_err_to_str(err));
        transport_deinit(&ble->base);
        ble_transport_destroy(ble);
        return;
    }
    
    ESP_LOGI(TAG, "BLE advertising started");
    ESP_LOGI(TAG, "Device name: %s", config.device_name);
    ESP_LOGI(TAG, "Service UUID: 0x%04X", config.service_uuid);
    
    uint8_t buffer[256];
    size_t bytes_read;
    
    while (1) {
        if (transport_get_state(&ble->base) == TRANSPORT_STATE_CONNECTED) {
            uint16_t mtu = ble_get_mtu(ble);
            ESP_LOGI(TAG, "Connected with MTU: %d", mtu);
            
            err = transport_read(&ble->base, buffer, sizeof(buffer) - 1, 
                                 &bytes_read, 1000);
            
            if (err == TRANSPORT_OK && bytes_read > 0) {
                buffer[bytes_read] = '\0';
                ESP_LOGI(TAG, "Received: %s", buffer);
                
                const char *response = "BLE Echo: ";
                transport_write(&ble->base, (uint8_t *)response, strlen(response), 
                               NULL, 1000);
                transport_write(&ble->base, buffer, bytes_read, NULL, 1000);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
