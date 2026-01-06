/**
 * @file transport_manager.c
 * @brief Transport manager implementation
 */

#include "transport_manager.h"

#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "transport_manager";

typedef struct {
    transport_t *transport;
    const char *name;
    transport_stats_t stats;
    int64_t connect_time;
    bool in_use;
} transport_entry_t;

typedef struct {
    transport_entry_t entries[TRANSPORT_MANAGER_MAX_TRANSPORTS];
    transport_handle_t active_handle;
    transport_manager_config_t config;
    SemaphoreHandle_t mutex;
    bool initialized;
    transport_manager_data_cb_t data_callback;
    void *data_callback_user_data;
} transport_manager_t;

static transport_manager_t g_manager = {0};

static void manager_event_callback(transport_t *transport, 
                                    const transport_event_t *event, 
                                    void *user_data)
{
    transport_handle_t handle = (transport_handle_t)(intptr_t)user_data;
    
    if (handle < 0 || handle >= TRANSPORT_MANAGER_MAX_TRANSPORTS) {
        return;
    }
    
    transport_entry_t *entry = &g_manager.entries[handle];
    
    switch (event->type) {
        case TRANSPORT_EVENT_CONNECTED:
            entry->connect_time = esp_timer_get_time();
            entry->stats.reconnects++;
            ESP_LOGI(TAG, "Transport '%s' connected", entry->name);
            break;
            
        case TRANSPORT_EVENT_DISCONNECTED:
            if (entry->connect_time > 0) {
                entry->stats.uptime_ms += (esp_timer_get_time() - entry->connect_time) / 1000;
                entry->connect_time = 0;
            }
            ESP_LOGI(TAG, "Transport '%s' disconnected", entry->name);
            break;
            
        case TRANSPORT_EVENT_DATA_RECEIVED:
            entry->stats.bytes_received += event->data_received.len;
            entry->stats.packets_received++;
            if (g_manager.data_callback != NULL) {
                g_manager.data_callback(handle, event->data_received.data, 
                                        event->data_received.len, 
                                        g_manager.data_callback_user_data);
            }
            break;
            
        case TRANSPORT_EVENT_DATA_SENT:
            entry->stats.bytes_sent += event->data_sent.bytes_sent;
            entry->stats.packets_sent++;
            break;
            
        case TRANSPORT_EVENT_ERROR:
            entry->stats.errors++;
            ESP_LOGE(TAG, "Transport '%s' error: %s", 
                     entry->name, event->error.message);
            break;
            
        default:
            break;
    }
}

transport_err_t transport_manager_init(const transport_manager_config_t *config)
{
    if (g_manager.initialized) {
        return TRANSPORT_ERR_ALREADY_INITIALIZED;
    }
    
    memset(&g_manager, 0, sizeof(g_manager));
    
    g_manager.mutex = xSemaphoreCreateMutex();
    if (g_manager.mutex == NULL) {
        return TRANSPORT_ERR_NO_MEM;
    }
    
    if (config != NULL) {
        memcpy(&g_manager.config, config, sizeof(transport_manager_config_t));
    } else {
        g_manager.config.auto_reconnect = false;
        g_manager.config.reconnect_delay_ms = 5000;
        g_manager.config.max_reconnect_attempts = 3;
        g_manager.config.selector = NULL;
        g_manager.config.selector_user_data = NULL;
    }
    
    g_manager.active_handle = TRANSPORT_HANDLE_INVALID;
    g_manager.initialized = true;
    
    ESP_LOGI(TAG, "Transport manager initialized");
    
    return TRANSPORT_OK;
}

transport_err_t transport_manager_deinit(void)
{
    if (!g_manager.initialized) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    xSemaphoreTake(g_manager.mutex, portMAX_DELAY);
    
    for (int i = 0; i < TRANSPORT_MANAGER_MAX_TRANSPORTS; i++) {
        if (g_manager.entries[i].in_use) {
            transport_t *t = g_manager.entries[i].transport;
            if (t != NULL && t->state == TRANSPORT_STATE_CONNECTED) {
                transport_disconnect(t);
            }
        }
    }
    
    xSemaphoreGive(g_manager.mutex);
    vSemaphoreDelete(g_manager.mutex);
    
    memset(&g_manager, 0, sizeof(g_manager));
    
    ESP_LOGI(TAG, "Transport manager deinitialized");
    
    return TRANSPORT_OK;
}

transport_handle_t transport_manager_register(transport_t *transport, const char *name)
{
    if (!g_manager.initialized) {
        return TRANSPORT_HANDLE_INVALID;
    }
    
    if (transport == NULL || name == NULL) {
        return TRANSPORT_HANDLE_INVALID;
    }
    
    xSemaphoreTake(g_manager.mutex, portMAX_DELAY);
    
    transport_handle_t handle = TRANSPORT_HANDLE_INVALID;
    
    for (int i = 0; i < TRANSPORT_MANAGER_MAX_TRANSPORTS; i++) {
        if (!g_manager.entries[i].in_use) {
            g_manager.entries[i].transport = transport;
            g_manager.entries[i].name = name;
            g_manager.entries[i].in_use = true;
            memset(&g_manager.entries[i].stats, 0, sizeof(transport_stats_t));
            g_manager.entries[i].connect_time = 0;
            
            transport->event_callback = manager_event_callback;
            transport->user_data = (void *)(intptr_t)i;
            
            handle = i;
            ESP_LOGI(TAG, "Registered transport '%s' with handle %d", name, handle);
            break;
        }
    }
    
    xSemaphoreGive(g_manager.mutex);
    
    return handle;
}

transport_err_t transport_manager_unregister(transport_handle_t handle)
{
    if (!g_manager.initialized) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (handle < 0 || handle >= TRANSPORT_MANAGER_MAX_TRANSPORTS) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(g_manager.mutex, portMAX_DELAY);
    
    transport_entry_t *entry = &g_manager.entries[handle];
    
    if (!entry->in_use) {
        xSemaphoreGive(g_manager.mutex);
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    if (entry->transport != NULL) {
        entry->transport->event_callback = NULL;
        entry->transport->user_data = NULL;
    }
    
    if (g_manager.active_handle == handle) {
        g_manager.active_handle = TRANSPORT_HANDLE_INVALID;
    }
    
    ESP_LOGI(TAG, "Unregistered transport '%s'", entry->name);
    
    memset(entry, 0, sizeof(transport_entry_t));
    
    xSemaphoreGive(g_manager.mutex);
    
    return TRANSPORT_OK;
}

transport_t *transport_manager_get(transport_handle_t handle)
{
    if (!g_manager.initialized) {
        return NULL;
    }
    
    if (handle < 0 || handle >= TRANSPORT_MANAGER_MAX_TRANSPORTS) {
        return NULL;
    }
    
    xSemaphoreTake(g_manager.mutex, portMAX_DELAY);
    
    transport_t *transport = NULL;
    
    if (g_manager.entries[handle].in_use) {
        transport = g_manager.entries[handle].transport;
    }
    
    xSemaphoreGive(g_manager.mutex);
    
    return transport;
}

transport_t *transport_manager_get_by_name(const char *name)
{
    if (!g_manager.initialized || name == NULL) {
        return NULL;
    }
    
    xSemaphoreTake(g_manager.mutex, portMAX_DELAY);
    
    transport_t *transport = NULL;
    
    for (int i = 0; i < TRANSPORT_MANAGER_MAX_TRANSPORTS; i++) {
        if (g_manager.entries[i].in_use && 
            strcmp(g_manager.entries[i].name, name) == 0) {
            transport = g_manager.entries[i].transport;
            break;
        }
    }
    
    xSemaphoreGive(g_manager.mutex);
    
    return transport;
}

transport_t *transport_manager_get_by_type(transport_type_t type)
{
    if (!g_manager.initialized) {
        return NULL;
    }
    
    xSemaphoreTake(g_manager.mutex, portMAX_DELAY);
    
    transport_t *transport = NULL;
    
    for (int i = 0; i < TRANSPORT_MANAGER_MAX_TRANSPORTS; i++) {
        if (g_manager.entries[i].in_use) {
            transport_t *t = g_manager.entries[i].transport;
            if (t != NULL && transport_get_type(t) == type) {
                transport = t;
                break;
            }
        }
    }
    
    xSemaphoreGive(g_manager.mutex);
    
    return transport;
}

transport_err_t transport_manager_set_active(transport_handle_t handle)
{
    if (!g_manager.initialized) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (handle < 0 || handle >= TRANSPORT_MANAGER_MAX_TRANSPORTS) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(g_manager.mutex, portMAX_DELAY);
    
    if (!g_manager.entries[handle].in_use) {
        xSemaphoreGive(g_manager.mutex);
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    g_manager.active_handle = handle;
    
    ESP_LOGI(TAG, "Set active transport to '%s'", g_manager.entries[handle].name);
    
    xSemaphoreGive(g_manager.mutex);
    
    return TRANSPORT_OK;
}

transport_t *transport_manager_get_active(void)
{
    if (!g_manager.initialized) {
        return NULL;
    }
    
    if (g_manager.active_handle == TRANSPORT_HANDLE_INVALID) {
        return NULL;
    }
    
    return transport_manager_get(g_manager.active_handle);
}

transport_err_t transport_manager_connect(const char *address, uint32_t timeout_ms)
{
    transport_t *transport = transport_manager_get_active();
    
    if (transport == NULL) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    return transport_connect(transport, address, timeout_ms);
}

transport_err_t transport_manager_disconnect(void)
{
    transport_t *transport = transport_manager_get_active();
    
    if (transport == NULL) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    return transport_disconnect(transport);
}

transport_err_t transport_manager_write(const uint8_t *data, size_t len, 
                                         size_t *bytes_written, uint32_t timeout_ms)
{
    transport_t *transport = transport_manager_get_active();
    
    if (transport == NULL) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    return transport_write(transport, data, len, bytes_written, timeout_ms);
}

transport_err_t transport_manager_read(uint8_t *buffer, size_t len, 
                                        size_t *bytes_read, uint32_t timeout_ms)
{
    transport_t *transport = transport_manager_get_active();
    
    if (transport == NULL) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    return transport_read(transport, buffer, len, bytes_read, timeout_ms);
}

size_t transport_manager_list(transport_handle_t *handles, size_t max_handles)
{
    if (!g_manager.initialized || handles == NULL || max_handles == 0) {
        return 0;
    }
    
    xSemaphoreTake(g_manager.mutex, portMAX_DELAY);
    
    size_t count = 0;
    
    for (int i = 0; i < TRANSPORT_MANAGER_MAX_TRANSPORTS && count < max_handles; i++) {
        if (g_manager.entries[i].in_use) {
            handles[count++] = i;
        }
    }
    
    xSemaphoreGive(g_manager.mutex);
    
    return count;
}

transport_err_t transport_manager_get_stats(transport_handle_t handle, 
                                             transport_stats_t *stats)
{
    if (!g_manager.initialized) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (handle < 0 || handle >= TRANSPORT_MANAGER_MAX_TRANSPORTS || stats == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(g_manager.mutex, portMAX_DELAY);
    
    transport_entry_t *entry = &g_manager.entries[handle];
    
    if (!entry->in_use) {
        xSemaphoreGive(g_manager.mutex);
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    memcpy(stats, &entry->stats, sizeof(transport_stats_t));
    
    if (entry->connect_time > 0) {
        stats->uptime_ms += (esp_timer_get_time() - entry->connect_time) / 1000;
    }
    
    xSemaphoreGive(g_manager.mutex);
    
    return TRANSPORT_OK;
}

transport_err_t transport_manager_reset_stats(transport_handle_t handle)
{
    if (!g_manager.initialized) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (handle < 0 || handle >= TRANSPORT_MANAGER_MAX_TRANSPORTS) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(g_manager.mutex, portMAX_DELAY);
    
    transport_entry_t *entry = &g_manager.entries[handle];
    
    if (!entry->in_use) {
        xSemaphoreGive(g_manager.mutex);
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    memset(&entry->stats, 0, sizeof(transport_stats_t));
    
    if (entry->transport != NULL && 
        transport_get_state(entry->transport) == TRANSPORT_STATE_CONNECTED) {
        entry->connect_time = esp_timer_get_time();
    }
    
    xSemaphoreGive(g_manager.mutex);
    
    return TRANSPORT_OK;
}

/* ============================================================================
 * Multi-Transport Simultaneous Operations Implementation
 * ============================================================================ */

transport_err_t transport_manager_connect_handle(transport_handle_t handle,
                                                  const char *address,
                                                  uint32_t timeout_ms)
{
    transport_t *transport = transport_manager_get(handle);
    
    if (transport == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    return transport_connect(transport, address, timeout_ms);
}

transport_err_t transport_manager_disconnect_handle(transport_handle_t handle)
{
    transport_t *transport = transport_manager_get(handle);
    
    if (transport == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    return transport_disconnect(transport);
}

transport_err_t transport_manager_write_handle(transport_handle_t handle,
                                                const uint8_t *data, size_t len,
                                                size_t *bytes_written, uint32_t timeout_ms)
{
    transport_t *transport = transport_manager_get(handle);
    
    if (transport == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    return transport_write(transport, data, len, bytes_written, timeout_ms);
}

transport_err_t transport_manager_read_handle(transport_handle_t handle,
                                               uint8_t *buffer, size_t len,
                                               size_t *bytes_read, uint32_t timeout_ms)
{
    transport_t *transport = transport_manager_get(handle);
    
    if (transport == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    return transport_read(transport, buffer, len, bytes_read, timeout_ms);
}

size_t transport_manager_broadcast(const uint8_t *data, size_t len,
                                    transport_handle_t exclude_handle,
                                    uint32_t timeout_ms)
{
    if (!g_manager.initialized || data == NULL || len == 0) {
        return 0;
    }
    
    size_t success_count = 0;
    
    xSemaphoreTake(g_manager.mutex, portMAX_DELAY);
    
    for (int i = 0; i < TRANSPORT_MANAGER_MAX_TRANSPORTS; i++) {
        if (!g_manager.entries[i].in_use) {
            continue;
        }
        
        if (i == exclude_handle) {
            continue;
        }
        
        transport_t *t = g_manager.entries[i].transport;
        if (t == NULL || transport_get_state(t) != TRANSPORT_STATE_CONNECTED) {
            continue;
        }
        
        xSemaphoreGive(g_manager.mutex);
        
        transport_err_t err = transport_write(t, data, len, NULL, timeout_ms);
        if (err == TRANSPORT_OK) {
            success_count++;
        }
        
        xSemaphoreTake(g_manager.mutex, portMAX_DELAY);
    }
    
    xSemaphoreGive(g_manager.mutex);
    
    return success_count;
}

size_t transport_manager_poll(transport_poll_result_t *results, size_t max_results)
{
    if (!g_manager.initialized || results == NULL || max_results == 0) {
        return 0;
    }
    
    size_t count = 0;
    
    xSemaphoreTake(g_manager.mutex, portMAX_DELAY);
    
    for (int i = 0; i < TRANSPORT_MANAGER_MAX_TRANSPORTS && count < max_results; i++) {
        if (!g_manager.entries[i].in_use) {
            continue;
        }
        
        transport_t *t = g_manager.entries[i].transport;
        if (t == NULL || transport_get_state(t) != TRANSPORT_STATE_CONNECTED) {
            continue;
        }
        
        size_t available = 0;
        if (transport_available(t, &available) == TRANSPORT_OK && available > 0) {
            results[count].handle = i;
            results[count].bytes_available = available;
            count++;
        }
    }
    
    xSemaphoreGive(g_manager.mutex);
    
    return count;
}

transport_err_t transport_manager_read_any(uint8_t *buffer, size_t len,
                                            size_t *bytes_read,
                                            transport_handle_t *source_handle,
                                            uint32_t timeout_ms)
{
    if (!g_manager.initialized || buffer == NULL || len == 0 || bytes_read == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    *bytes_read = 0;
    if (source_handle != NULL) {
        *source_handle = TRANSPORT_HANDLE_INVALID;
    }
    
    uint32_t elapsed = 0;
    const uint32_t poll_interval = 10;
    
    while (elapsed < timeout_ms || timeout_ms == 0) {
        transport_poll_result_t poll_results[TRANSPORT_MANAGER_MAX_TRANSPORTS];
        size_t poll_count = transport_manager_poll(poll_results, TRANSPORT_MANAGER_MAX_TRANSPORTS);
        
        if (poll_count > 0) {
            transport_handle_t handle = poll_results[0].handle;
            transport_t *t = transport_manager_get(handle);
            
            if (t != NULL) {
                transport_err_t err = transport_read(t, buffer, len, bytes_read, 
                                                      timeout_ms - elapsed);
                if (err == TRANSPORT_OK && *bytes_read > 0) {
                    if (source_handle != NULL) {
                        *source_handle = handle;
                    }
                    return TRANSPORT_OK;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(poll_interval));
        elapsed += poll_interval;
        
        if (timeout_ms == 0) {
            break;
        }
    }
    
    return TRANSPORT_ERR_TIMEOUT;
}

transport_err_t transport_manager_forward(transport_handle_t source_handle,
                                           transport_handle_t dest_handle,
                                           uint8_t *buffer, size_t buffer_len,
                                           size_t *bytes_forwarded,
                                           uint32_t timeout_ms)
{
    if (!g_manager.initialized || buffer == NULL || buffer_len == 0) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    if (bytes_forwarded != NULL) {
        *bytes_forwarded = 0;
    }
    
    transport_t *source = transport_manager_get(source_handle);
    if (source == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    size_t bytes_read = 0;
    transport_err_t err = transport_read(source, buffer, buffer_len, &bytes_read, timeout_ms);
    
    if (err != TRANSPORT_OK || bytes_read == 0) {
        return err;
    }
    
    size_t total_forwarded = 0;
    
    if (dest_handle == TRANSPORT_HANDLE_INVALID) {
        size_t sent = transport_manager_broadcast(buffer, bytes_read, source_handle, timeout_ms);
        if (sent > 0) {
            total_forwarded = bytes_read;
        }
    } else {
        transport_t *dest = transport_manager_get(dest_handle);
        if (dest != NULL && transport_get_state(dest) == TRANSPORT_STATE_CONNECTED) {
            size_t written = 0;
            err = transport_write(dest, buffer, bytes_read, &written, timeout_ms);
            if (err == TRANSPORT_OK) {
                total_forwarded = written;
            }
        }
    }
    
    if (bytes_forwarded != NULL) {
        *bytes_forwarded = total_forwarded;
    }
    
    return total_forwarded > 0 ? TRANSPORT_OK : TRANSPORT_ERR_IO;
}

size_t transport_manager_connected_count(void)
{
    if (!g_manager.initialized) {
        return 0;
    }
    
    size_t count = 0;
    
    xSemaphoreTake(g_manager.mutex, portMAX_DELAY);
    
    for (int i = 0; i < TRANSPORT_MANAGER_MAX_TRANSPORTS; i++) {
        if (g_manager.entries[i].in_use) {
            transport_t *t = g_manager.entries[i].transport;
            if (t != NULL && transport_get_state(t) == TRANSPORT_STATE_CONNECTED) {
                count++;
            }
        }
    }
    
    xSemaphoreGive(g_manager.mutex);
    
    return count;
}

bool transport_manager_is_connected(transport_handle_t handle)
{
    transport_t *transport = transport_manager_get(handle);
    
    if (transport == NULL) {
        return false;
    }
    
    return transport_get_state(transport) == TRANSPORT_STATE_CONNECTED;
}

transport_err_t transport_manager_set_data_callback(transport_manager_data_cb_t callback,
                                                     void *user_data)
{
    if (!g_manager.initialized) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    xSemaphoreTake(g_manager.mutex, portMAX_DELAY);
    
    g_manager.data_callback = callback;
    g_manager.data_callback_user_data = user_data;
    
    xSemaphoreGive(g_manager.mutex);
    
    return TRANSPORT_OK;
}