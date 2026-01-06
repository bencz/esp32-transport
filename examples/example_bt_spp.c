/**
 * @file example_bt_spp.c
 * @brief Bluetooth Classic SPP transport example
 * 
 * This example demonstrates how to use the Bluetooth Classic SPP transport
 * for wireless serial communication. The ESP32 acts as a server waiting
 * for incoming connections and echoes back any received data.
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "transport.h"
#include "transport_bt_spp.h"

static const char *TAG = "example_bt_spp";

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
    ESP_LOGI(TAG, "=== Bluetooth Classic SPP Example ===");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    bt_spp_transport_t *spp = bt_spp_transport_create();
    if (spp == NULL) {
        ESP_LOGE(TAG, "Failed to create SPP transport");
        return;
    }
    
    bt_spp_config_t config;
    bt_spp_config_default(&config);
    config.device_name = "ESP32_SPP_Demo";
    config.base.event_callback = on_transport_event;
    config.base.user_data = NULL;
    
    transport_err_t err = transport_init(&spp->base, &config);
    if (err != TRANSPORT_OK) {
        ESP_LOGE(TAG, "SPP init failed: %s", transport_err_to_str(err));
        bt_spp_transport_destroy(spp);
        return;
    }
    
    ESP_LOGI(TAG, "SPP initialized, starting server...");
    
    err = bt_spp_listen(spp);
    if (err != TRANSPORT_OK) {
        ESP_LOGE(TAG, "SPP listen failed: %s", transport_err_to_str(err));
        transport_deinit(&spp->base);
        bt_spp_transport_destroy(spp);
        return;
    }
    
    ESP_LOGI(TAG, "SPP server started, waiting for connections...");
    ESP_LOGI(TAG, "Device name: %s", config.device_name);
    
    uint8_t buffer[256];
    size_t bytes_read;
    
    while (1) {
        if (transport_get_state(&spp->base) == TRANSPORT_STATE_CONNECTED) {
            err = transport_read(&spp->base, buffer, sizeof(buffer) - 1, 
                                 &bytes_read, 1000);
            
            if (err == TRANSPORT_OK && bytes_read > 0) {
                buffer[bytes_read] = '\0';
                ESP_LOGI(TAG, "Received: %s", buffer);
                
                const char *response = "Echo: ";
                transport_write(&spp->base, (uint8_t *)response, strlen(response), 
                               NULL, 1000);
                transport_write(&spp->base, buffer, bytes_read, NULL, 1000);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
