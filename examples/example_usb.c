/**
 * @file example_usb.c
 * @brief USB CDC transport example
 * 
 * This example demonstrates how to use the USB CDC transport for
 * virtual serial communication over USB. The ESP32 appears as a
 * serial port when connected to a computer.
 * 
 * Note: Requires ESP32-S2, ESP32-S3, or ESP32-C3 with native USB support.
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "transport.h"
#include "transport_usb.h"

static const char *TAG = "example_usb";

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
            ESP_LOGI(TAG, "USB host connected (DTR active)");
            break;
            
        case TRANSPORT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "USB host disconnected");
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
    ESP_LOGI(TAG, "=== USB CDC Transport Example ===");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    usb_transport_t *usb = usb_transport_create();
    if (usb == NULL) {
        ESP_LOGE(TAG, "Failed to create USB transport");
        return;
    }
    
    usb_transport_config_t config;
    usb_transport_config_default(&config);
    config.base.event_callback = on_transport_event;
    config.base.user_data = NULL;
    
    transport_err_t err = transport_init(&usb->base, &config);
    if (err != TRANSPORT_OK) {
        ESP_LOGE(TAG, "USB init failed: %s", transport_err_to_str(err));
        usb_transport_destroy(usb);
        return;
    }
    
    ESP_LOGI(TAG, "USB CDC initialized, opening...");
    
    err = transport_connect(&usb->base, NULL, 0);
    if (err != TRANSPORT_OK) {
        ESP_LOGE(TAG, "USB open failed: %s", transport_err_to_str(err));
        transport_deinit(&usb->base);
        usb_transport_destroy(usb);
        return;
    }
    
    ESP_LOGI(TAG, "USB CDC ready, waiting for host connection...");
    
    err = usb_transport_wait_for_host(usb, 30000);
    if (err == TRANSPORT_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "No host connected within timeout, continuing anyway...");
    }
    
    uint8_t buffer[256];
    size_t bytes_read;
    
    while (1) {
        if (usb_transport_is_host_connected(usb)) {
            if (transport_get_state(&usb->base) == TRANSPORT_STATE_CONNECTED) {
                usb_line_coding_t line_coding;
                usb_transport_get_line_coding(usb, &line_coding);
                ESP_LOGI(TAG, "Host connected at %lu baud", 
                         (unsigned long)line_coding.baud_rate);
            }
            
            err = transport_read(&usb->base, buffer, sizeof(buffer) - 1, 
                                 &bytes_read, 1000);
            
            if (err == TRANSPORT_OK && bytes_read > 0) {
                buffer[bytes_read] = '\0';
                ESP_LOGI(TAG, "Received: %s", buffer);
                
                const char *response = "USB Echo: ";
                transport_write(&usb->base, (uint8_t *)response, strlen(response), 
                               NULL, 1000);
                transport_write(&usb->base, buffer, bytes_read, NULL, 1000);
                transport_write(&usb->base, (uint8_t *)"\r\n", 2, NULL, 1000);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
