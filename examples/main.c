/**
 * @file main.c
 * @brief Exemplo de uso do sistema de abstração de transportes
 * 
 * Este exemplo demonstra como usar a camada de abstração para
 * alternar entre Bluetooth Classic SPP e BLE de forma transparente.
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "transport.h"
#include "transport_bt_spp.h"
#include "transport_ble.h"
#include "transport_manager.h"

static const char *TAG = "transport_example";

/**
 * @brief Callback para eventos de transporte
 */
static void on_transport_event(transport_t *transport, 
                                const transport_event_t *event, 
                                void *user_data)
{
    const char *name = (const char *)user_data;
    
    switch (event->type) {
        case TRANSPORT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "[%s] Connected!", name);
            break;
            
        case TRANSPORT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "[%s] Disconnected", name);
            break;
            
        case TRANSPORT_EVENT_DATA_RECEIVED:
            ESP_LOGI(TAG, "[%s] Received %zu bytes", name, event->data_received.len);
            ESP_LOG_BUFFER_HEXDUMP(TAG, event->data_received.data, 
                                   event->data_received.len, ESP_LOG_DEBUG);
            break;
            
        case TRANSPORT_EVENT_ERROR:
            ESP_LOGE(TAG, "[%s] Error: %s", name, event->error.message);
            break;
            
        case TRANSPORT_EVENT_STATE_CHANGED:
            ESP_LOGI(TAG, "[%s] State: %s -> %s", name,
                     transport_state_to_str(event->state_changed.old_state),
                     transport_state_to_str(event->state_changed.new_state));
            break;
            
        default:
            break;
    }
}

/**
 * @brief Exemplo usando Bluetooth Classic SPP
 */
static void example_bt_spp(void)
{
    ESP_LOGI(TAG, "=== Bluetooth Classic SPP Example ===");
    
    bt_spp_transport_t *spp = bt_spp_transport_create();
    if (spp == NULL) {
        ESP_LOGE(TAG, "Failed to create SPP transport");
        return;
    }
    
    bt_spp_config_t config;
    bt_spp_config_default(&config);
    config.device_name = "ESP32_SPP_Demo";
    config.base.event_callback = on_transport_event;
    config.base.user_data = (void *)"SPP";
    
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

/**
 * @brief Exemplo usando BLE
 */
static void example_ble(void)
{
    ESP_LOGI(TAG, "=== Bluetooth Low Energy Example ===");
    
    ble_transport_t *ble = ble_transport_create();
    if (ble == NULL) {
        ESP_LOGE(TAG, "Failed to create BLE transport");
        return;
    }
    
    ble_config_t config;
    ble_config_default(&config);
    config.device_name = "ESP32_BLE_Demo";
    config.base.event_callback = on_transport_event;
    config.base.user_data = (void *)"BLE";
    
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
    
    ESP_LOGI(TAG, "BLE advertising started, waiting for connections...");
    
    uint8_t buffer[256];
    size_t bytes_read;
    
    while (1) {
        if (transport_get_state(&ble->base) == TRANSPORT_STATE_CONNECTED) {
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

/**
 * @brief Exemplo usando o gerenciador de transportes
 * 
 * Demonstra como registrar múltiplos transportes e alternar entre eles.
 */
static void example_manager(void)
{
    ESP_LOGI(TAG, "=== Transport Manager Example ===");
    
    transport_err_t err = transport_manager_init(NULL);
    if (err != TRANSPORT_OK) {
        ESP_LOGE(TAG, "Manager init failed: %s", transport_err_to_str(err));
        return;
    }
    
    bt_spp_transport_t *spp = bt_spp_transport_create();
    ble_transport_t *ble = ble_transport_create();
    
    if (spp == NULL || ble == NULL) {
        ESP_LOGE(TAG, "Failed to create transports");
        if (spp) bt_spp_transport_destroy(spp);
        if (ble) ble_transport_destroy(ble);
        transport_manager_deinit();
        return;
    }
    
    bt_spp_config_t spp_config;
    bt_spp_config_default(&spp_config);
    spp_config.device_name = "ESP32_Multi_SPP";
    
    ble_config_t ble_config;
    ble_config_default(&ble_config);
    ble_config.device_name = "ESP32_Multi_BLE";
    
    err = transport_init(&spp->base, &spp_config);
    if (err != TRANSPORT_OK) {
        ESP_LOGE(TAG, "SPP init failed: %s", transport_err_to_str(err));
    }
    
    err = transport_init(&ble->base, &ble_config);
    if (err != TRANSPORT_OK) {
        ESP_LOGE(TAG, "BLE init failed: %s", transport_err_to_str(err));
    }
    
    transport_handle_t spp_handle = transport_manager_register(&spp->base, "spp");
    transport_handle_t ble_handle = transport_manager_register(&ble->base, "ble");
    
    ESP_LOGI(TAG, "Registered SPP (handle=%d) and BLE (handle=%d)", 
             spp_handle, ble_handle);
    
    transport_manager_set_active(ble_handle);
    
    transport_t *active = transport_manager_get_active();
    if (active != NULL) {
        char info[128];
        transport_get_info(active, info, sizeof(info));
        ESP_LOGI(TAG, "Active transport: %s", info);
    }
    
    transport_handle_t handles[TRANSPORT_MANAGER_MAX_TRANSPORTS];
    size_t count = transport_manager_list(handles, TRANSPORT_MANAGER_MAX_TRANSPORTS);
    
    ESP_LOGI(TAG, "Registered transports: %zu", count);
    for (size_t i = 0; i < count; i++) {
        transport_t *t = transport_manager_get(handles[i]);
        if (t != NULL) {
            ESP_LOGI(TAG, "  [%d] Type: %s, State: %s",
                     handles[i],
                     transport_type_to_str(transport_get_type(t)),
                     transport_state_to_str(transport_get_state(t)));
        }
    }
    
    ESP_LOGI(TAG, "Starting BLE advertising via manager...");
    ble_start_advertising(ble);
    
    uint8_t buffer[256];
    size_t bytes_read;
    
    while (1) {
        active = transport_manager_get_active();
        
        if (active != NULL && transport_get_state(active) == TRANSPORT_STATE_CONNECTED) {
            err = transport_manager_read(buffer, sizeof(buffer) - 1, &bytes_read, 1000);
            
            if (err == TRANSPORT_OK && bytes_read > 0) {
                buffer[bytes_read] = '\0';
                ESP_LOGI(TAG, "Manager received: %s", buffer);
                
                if (strcmp((char *)buffer, "switch") == 0) {
                    transport_handle_t current = 
                        (transport_manager_get_active() == &spp->base) ? ble_handle : spp_handle;
                    transport_manager_set_active(current);
                    ESP_LOGI(TAG, "Switched to %s", 
                             transport_type_to_str(transport_get_type(transport_manager_get_active())));
                } else {
                    const char *response = "Manager Echo: ";
                    transport_manager_write((uint8_t *)response, strlen(response), NULL, 1000);
                    transport_manager_write(buffer, bytes_read, NULL, 1000);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Exemplo de uso polimórfico com ponteiro base
 * 
 * Demonstra como usar a abstração de forma genérica.
 */
static void process_transport(transport_t *transport)
{
    char info[128];
    
    transport_get_info(transport, info, sizeof(info));
    ESP_LOGI(TAG, "Processing: %s", info);
    
    transport_type_t type = transport_get_type(transport);
    ESP_LOGI(TAG, "  Type: %s", transport_type_to_str(type));
    
    transport_state_t state = transport_get_state(transport);
    ESP_LOGI(TAG, "  State: %s", transport_state_to_str(state));
    
    if (state == TRANSPORT_STATE_CONNECTED) {
        size_t available;
        transport_available(transport, &available);
        ESP_LOGI(TAG, "  Bytes available: %zu", available);
    }
}

/**
 * @brief Demonstração de polimorfismo
 */
static void example_polymorphism(void)
{
    ESP_LOGI(TAG, "=== Polymorphism Example ===");
    
    bt_spp_transport_t *spp = bt_spp_transport_create();
    ble_transport_t *ble = ble_transport_create();
    
    if (spp == NULL || ble == NULL) {
        ESP_LOGE(TAG, "Failed to create transports");
        if (spp) bt_spp_transport_destroy(spp);
        if (ble) ble_transport_destroy(ble);
        return;
    }
    
    bt_spp_config_t spp_config;
    bt_spp_config_default(&spp_config);
    transport_init(&spp->base, &spp_config);
    
    ble_config_t ble_config;
    ble_config_default(&ble_config);
    transport_init(&ble->base, &ble_config);
    
    transport_t *transports[] = {
        &spp->base,
        &ble->base,
    };
    
    for (size_t i = 0; i < sizeof(transports) / sizeof(transports[0]); i++) {
        process_transport(transports[i]);
    }
    
    transport_deinit(&spp->base);
    transport_deinit(&ble->base);
    bt_spp_transport_destroy(spp);
    ble_transport_destroy(ble);
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 Transport Abstraction Layer Demo");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    #if defined(CONFIG_TRANSPORT_EXAMPLE_SPP)
        example_bt_spp();
    #elif defined(CONFIG_TRANSPORT_EXAMPLE_BLE)
        example_ble();
    #elif defined(CONFIG_TRANSPORT_EXAMPLE_MANAGER)
        example_manager();
    #else
        example_polymorphism();
    #endif
}