/**
 * @file transport_bt_spp.c
 * @brief Bluetooth Classic SPP backend implementation
 */

#include "transport_bt_spp.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/ringbuf.h"

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"

static const char *TAG = "bt_spp_transport";

#define BT_SPP_DEFAULT_MTU          (990)
#define BT_SPP_DEFAULT_BUFFER_SIZE  (4096)
#define BT_SPP_DEFAULT_TIMEOUT_MS   (5000)

/**
 * @brief SPP transport internal data
 */
typedef struct {
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t write_sem;
    SemaphoreHandle_t connect_sem;
    EventGroupHandle_t event_group;
    RingbufHandle_t rx_ringbuf;
    bool initialized;
    bool connected;
    uint32_t conn_handle;
} bt_spp_internal_t;

#define EVENT_CONNECTED     BIT0
#define EVENT_DISCONNECTED  BIT1
#define EVENT_WRITE_DONE    BIT2
#define EVENT_DATA_READY    BIT3

/**
 * @brief Current transport instance pointer
 * 
 * Note: ESP-IDF's Bluetooth callbacks (esp_bt_gap_register_callback,
 * esp_spp_register_callback) do not provide a user context parameter,
 * so we must store the transport instance here to access it from callbacks.
 * This limits the implementation to a single SPP transport instance at a time,
 * which aligns with ESP-IDF's single Bluetooth stack limitation.
 * 
 * The pointer is set during init and cleared during deinit.
 */
static bt_spp_transport_t *s_transport_instance = NULL;

/**
 * @brief Vtable function prototypes
 */
static transport_err_t bt_spp_init(transport_t *self, const void *config);
static transport_err_t bt_spp_deinit(transport_t *self);
static transport_err_t bt_spp_connect(transport_t *self, const char *address, uint32_t timeout_ms);
static transport_err_t bt_spp_disconnect(transport_t *self);
static transport_err_t bt_spp_write(transport_t *self, const uint8_t *data, size_t len, 
                                     size_t *bytes_written, uint32_t timeout_ms);
static transport_err_t bt_spp_read(transport_t *self, uint8_t *buffer, size_t len, 
                                    size_t *bytes_read, uint32_t timeout_ms);
static transport_err_t bt_spp_available(transport_t *self, size_t *available);
static transport_err_t bt_spp_flush(transport_t *self);
static transport_state_t bt_spp_get_state(transport_t *self);
static transport_type_t bt_spp_get_type(transport_t *self);
static transport_err_t bt_spp_get_info(transport_t *self, char *info, size_t len);
static transport_err_t bt_spp_set_option(transport_t *self, int option, 
                                          const void *value, size_t len);
static transport_err_t bt_spp_get_option(transport_t *self, int option, 
                                          void *value, size_t *len);

/**
 * @brief Bluetooth SPP transport vtable
 */
static const transport_vtable_t bt_spp_vtable = {
    .init       = bt_spp_init,
    .deinit     = bt_spp_deinit,
    .connect    = bt_spp_connect,
    .disconnect = bt_spp_disconnect,
    .write      = bt_spp_write,
    .read       = bt_spp_read,
    .available  = bt_spp_available,
    .flush      = bt_spp_flush,
    .get_state  = bt_spp_get_state,
    .get_type   = bt_spp_get_type,
    .get_info   = bt_spp_get_info,
    .set_option = bt_spp_set_option,
    .get_option = bt_spp_get_option,
};

/**
 * @brief GAP event callback
 */
static void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Authentication success: %s", param->auth_cmpl.device_name);
            } else {
                ESP_LOGE(TAG, "Authentication failed, status: %d", param->auth_cmpl.stat);
            }
            break;
            
        case ESP_BT_GAP_PIN_REQ_EVT:
            ESP_LOGI(TAG, "PIN request");
            if (s_transport_instance != NULL && s_transport_instance->config.pin_code != NULL) {
                esp_bt_pin_code_t pin;
                size_t pin_len = strlen(s_transport_instance->config.pin_code);
                if (pin_len > ESP_BT_PIN_CODE_LEN) {
                    pin_len = ESP_BT_PIN_CODE_LEN;
                }
                memcpy(pin, s_transport_instance->config.pin_code, pin_len);
                esp_bt_gap_pin_reply(param->pin_req.bda, true, pin_len, pin);
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief SPP event callback
 */
static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    bt_spp_transport_t *transport = s_transport_instance;
    
    if (transport == NULL) {
        return;
    }
    
    bt_spp_internal_t *internal = (bt_spp_internal_t *)transport->internal;
    
    if (internal == NULL) {
        return;
    }
    
    switch (event) {
        case ESP_SPP_INIT_EVT:
            if (param->init.status == ESP_SPP_SUCCESS) {
                ESP_LOGI(TAG, "SPP initialized");
                internal->initialized = true;
            } else {
                ESP_LOGE(TAG, "SPP init failed: %d", param->init.status);
            }
            break;
            
        case ESP_SPP_DISCOVERY_COMP_EVT:
            if (param->disc_comp.status == ESP_SPP_SUCCESS) {
                ESP_LOGI(TAG, "Discovery complete, scn_num=%d", param->disc_comp.scn_num);
                if (param->disc_comp.scn_num > 0) {
                    esp_spp_connect(transport->config.security, param->disc_comp.scn[0]);
                }
            } else {
                ESP_LOGE(TAG, "Discovery failed: %d", param->disc_comp.status);
                xSemaphoreGive(internal->connect_sem);
            }
            break;
            
        case ESP_SPP_OPEN_EVT:
            if (param->open.status == ESP_SPP_SUCCESS) {
                ESP_LOGI(TAG, "SPP connection opened, handle=%"PRIu32, param->open.handle);
                internal->conn_handle = param->open.handle;
                internal->connected = true;
                memcpy(transport->remote_addr, param->open.rem_bda, 6);
                transport_set_state(&transport->base, TRANSPORT_STATE_CONNECTED);
                xEventGroupSetBits(internal->event_group, EVENT_CONNECTED);
                xSemaphoreGive(internal->connect_sem);
                
                transport_event_t evt = {
                    .type = TRANSPORT_EVENT_CONNECTED,
                };
                transport_emit_event(&transport->base, &evt);
            } else {
                ESP_LOGE(TAG, "SPP connection failed: %d", param->open.status);
                transport_set_state(&transport->base, TRANSPORT_STATE_ERROR);
                xSemaphoreGive(internal->connect_sem);
            }
            break;
            
        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(TAG, "SPP connection closed");
            internal->connected = false;
            internal->conn_handle = 0;
            xEventGroupSetBits(internal->event_group, EVENT_DISCONNECTED);
            xEventGroupClearBits(internal->event_group, EVENT_CONNECTED);
            transport_set_state(&transport->base, TRANSPORT_STATE_INITIALIZED);
            
            transport_event_t evt = {
                .type = TRANSPORT_EVENT_DISCONNECTED,
            };
            transport_emit_event(&transport->base, &evt);
            break;
            
        case ESP_SPP_DATA_IND_EVT:
            if (param->data_ind.len > 0 && internal->rx_ringbuf != NULL) {
                if (xRingbufferSend(internal->rx_ringbuf, param->data_ind.data, 
                                     param->data_ind.len, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "RX buffer full, dropping %d bytes", param->data_ind.len);
                }
                xEventGroupSetBits(internal->event_group, EVENT_DATA_READY);
                
                transport_event_t data_evt = {
                    .type = TRANSPORT_EVENT_DATA_RECEIVED,
                    .data_received = {
                        .data = param->data_ind.data,
                        .len = param->data_ind.len,
                    },
                };
                transport_emit_event(&transport->base, &data_evt);
            }
            break;
            
        case ESP_SPP_WRITE_EVT:
            if (param->write.status == ESP_SPP_SUCCESS) {
                xEventGroupSetBits(internal->event_group, EVENT_WRITE_DONE);
                xSemaphoreGive(internal->write_sem);
            }
            break;
            
        case ESP_SPP_SRV_OPEN_EVT:
            if (param->srv_open.status == ESP_SPP_SUCCESS) {
                ESP_LOGI(TAG, "SPP server connection from client");
                internal->conn_handle = param->srv_open.handle;
                internal->connected = true;
                memcpy(transport->remote_addr, param->srv_open.rem_bda, 6);
                transport_set_state(&transport->base, TRANSPORT_STATE_CONNECTED);
                xEventGroupSetBits(internal->event_group, EVENT_CONNECTED);
                
                transport_event_t srv_evt = {
                    .type = TRANSPORT_EVENT_CONNECTED,
                };
                transport_emit_event(&transport->base, &srv_evt);
            }
            break;
            
        case ESP_SPP_START_EVT:
            if (param->start.status == ESP_SPP_SUCCESS) {
                ESP_LOGI(TAG, "SPP server started, handle=%"PRIu32, param->start.handle);
                transport->spp_handle = param->start.handle;
                
                if (transport->config.discoverable) {
                    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                } else {
                    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                }
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief Initializes the Bluetooth stack
 */
static transport_err_t init_bluetooth_stack(bt_spp_transport_t *transport)
{
    esp_err_t ret;
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller init failed: %s", esp_err_to_name(ret));
        return TRANSPORT_ERR_INTERNAL;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        esp_bt_controller_deinit();
        return TRANSPORT_ERR_INTERNAL;
    }
    
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        return TRANSPORT_ERR_INTERNAL;
    }
    
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        return TRANSPORT_ERR_INTERNAL;
    }
    
    ret = esp_bt_gap_register_callback(gap_callback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GAP callback registration failed: %s", esp_err_to_name(ret));
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        return TRANSPORT_ERR_INTERNAL;
    }
    
    ret = esp_spp_register_callback(spp_callback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPP callback registration failed: %s", esp_err_to_name(ret));
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        return TRANSPORT_ERR_INTERNAL;
    }
    
    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0,
    };
    
    ret = esp_spp_enhanced_init(&spp_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPP init failed: %s", esp_err_to_name(ret));
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        return TRANSPORT_ERR_INTERNAL;
    }
    
    if (transport->config.device_name != NULL) {
        esp_bt_dev_set_device_name(transport->config.device_name);
    }
    
    return TRANSPORT_OK;
}

/**
 * @brief Deinitializes the Bluetooth stack
 */
static void deinit_bluetooth_stack(void)
{
    esp_spp_deinit();
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
}

static transport_err_t bt_spp_init(transport_t *self, const void *config)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    bt_spp_transport_t *transport = (bt_spp_transport_t *)self;
    
    if (self->state != TRANSPORT_STATE_UNINITIALIZED) {
        return TRANSPORT_ERR_ALREADY_INITIALIZED;
    }
    
    // Create base mutex for thread safety
    transport->base.mutex = xSemaphoreCreateMutex();
    if (transport->base.mutex == NULL) {
        return TRANSPORT_ERR_NO_MEM;
    }
    
    if (config != NULL) {
        memcpy(&transport->config, config, sizeof(bt_spp_config_t));
    }
    
    bt_spp_internal_t *internal = calloc(1, sizeof(bt_spp_internal_t));
    if (internal == NULL) {
        return TRANSPORT_ERR_NO_MEM;
    }
    
    internal->mutex = xSemaphoreCreateMutex();
    internal->write_sem = xSemaphoreCreateBinary();
    internal->connect_sem = xSemaphoreCreateBinary();
    internal->event_group = xEventGroupCreate();
    
    if (internal->mutex == NULL || internal->write_sem == NULL || 
        internal->connect_sem == NULL || internal->event_group == NULL) {
        if (internal->mutex) vSemaphoreDelete(internal->mutex);
        if (internal->write_sem) vSemaphoreDelete(internal->write_sem);
        if (internal->connect_sem) vSemaphoreDelete(internal->connect_sem);
        if (internal->event_group) vEventGroupDelete(internal->event_group);
        free(internal);
        return TRANSPORT_ERR_NO_MEM;
    }
    
    size_t buffer_size = transport->config.base.rx_buffer_size;
    if (buffer_size == 0) {
        buffer_size = BT_SPP_DEFAULT_BUFFER_SIZE;
    }
    
    internal->rx_ringbuf = xRingbufferCreate(buffer_size, RINGBUF_TYPE_BYTEBUF);
    if (internal->rx_ringbuf == NULL) {
        vSemaphoreDelete(internal->mutex);
        vSemaphoreDelete(internal->write_sem);
        vSemaphoreDelete(internal->connect_sem);
        vEventGroupDelete(internal->event_group);
        free(internal);
        return TRANSPORT_ERR_NO_MEM;
    }
    
    transport->internal = internal;
    s_transport_instance = transport;
    
    transport_err_t err = init_bluetooth_stack(transport);
    if (err != TRANSPORT_OK) {
        vSemaphoreDelete(transport->base.mutex);
        transport->base.mutex = NULL;
        vRingbufferDelete(internal->rx_ringbuf);
        vSemaphoreDelete(internal->mutex);
        vSemaphoreDelete(internal->write_sem);
        vSemaphoreDelete(internal->connect_sem);
        vEventGroupDelete(internal->event_group);
        free(internal);
        transport->internal = NULL;
        s_transport_instance = NULL;
        return err;
    }
    
    int retries = 50;
    while (!internal->initialized && retries > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
        retries--;
    }
    
    if (!internal->initialized) {
        deinit_bluetooth_stack();
        vSemaphoreDelete(transport->base.mutex);
        transport->base.mutex = NULL;
        vRingbufferDelete(internal->rx_ringbuf);
        vSemaphoreDelete(internal->mutex);
        vSemaphoreDelete(internal->write_sem);
        vSemaphoreDelete(internal->connect_sem);
        vEventGroupDelete(internal->event_group);
        free(internal);
        transport->internal = NULL;
        s_transport_instance = NULL;
        return TRANSPORT_ERR_TIMEOUT;
    }
    
    transport_set_state(self, TRANSPORT_STATE_INITIALIZED);
    
    return TRANSPORT_OK;
}

static transport_err_t bt_spp_deinit(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    bt_spp_transport_t *transport = (bt_spp_transport_t *)self;
    bt_spp_internal_t *internal = (bt_spp_internal_t *)transport->internal;
    
    if (self->state == TRANSPORT_STATE_UNINITIALIZED) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (internal->connected) {
        bt_spp_disconnect(self);
    }
    
    deinit_bluetooth_stack();
    
    if (internal->rx_ringbuf) {
        vRingbufferDelete(internal->rx_ringbuf);
    }
    if (internal->mutex) {
        vSemaphoreDelete(internal->mutex);
    }
    if (internal->write_sem) {
        vSemaphoreDelete(internal->write_sem);
    }
    if (internal->connect_sem) {
        vSemaphoreDelete(internal->connect_sem);
    }
    if (internal->event_group) {
        vEventGroupDelete(internal->event_group);
    }
    
    free(internal);
    transport->internal = NULL;
    s_transport_instance = NULL;
    
    transport_set_state(self, TRANSPORT_STATE_UNINITIALIZED);
    
    return TRANSPORT_OK;
}

static transport_err_t bt_spp_connect(transport_t *self, const char *address, uint32_t timeout_ms)
{
    if (self == NULL || address == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    bt_spp_transport_t *transport = (bt_spp_transport_t *)self;
    bt_spp_internal_t *internal = (bt_spp_internal_t *)transport->internal;
    
    if (self->state == TRANSPORT_STATE_UNINITIALIZED) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (internal->connected) {
        return TRANSPORT_ERR_ALREADY_CONNECTED;
    }
    
    uint8_t remote_addr[6];
    transport_err_t err = bt_str_to_addr(address, remote_addr);
    if (err != TRANSPORT_OK) {
        return err;
    }
    
    transport_set_state(self, TRANSPORT_STATE_CONNECTING);
    
    esp_err_t ret = esp_spp_start_discovery(remote_addr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPP discovery failed: %s", esp_err_to_name(ret));
        transport_set_state(self, TRANSPORT_STATE_INITIALIZED);
        return TRANSPORT_ERR_IO;
    }
    
    if (timeout_ms == 0) {
        timeout_ms = BT_SPP_DEFAULT_TIMEOUT_MS;
    }
    
    if (xSemaphoreTake(internal->connect_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        transport_set_state(self, TRANSPORT_STATE_INITIALIZED);
        return TRANSPORT_ERR_TIMEOUT;
    }
    
    if (!internal->connected) {
        transport_set_state(self, TRANSPORT_STATE_INITIALIZED);
        return TRANSPORT_ERR_IO;
    }
    
    return TRANSPORT_OK;
}

static transport_err_t bt_spp_disconnect(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    bt_spp_transport_t *transport = (bt_spp_transport_t *)self;
    bt_spp_internal_t *internal = (bt_spp_internal_t *)transport->internal;
    
    if (!internal->connected) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    transport_set_state(self, TRANSPORT_STATE_DISCONNECTING);
    
    esp_err_t ret = esp_spp_disconnect(internal->conn_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPP disconnect failed: %s", esp_err_to_name(ret));
        return TRANSPORT_ERR_IO;
    }
    
    EventBits_t bits = xEventGroupWaitBits(internal->event_group, 
                                            EVENT_DISCONNECTED,
                                            pdTRUE, pdFALSE,
                                            pdMS_TO_TICKS(BT_SPP_DEFAULT_TIMEOUT_MS));
    
    if (!(bits & EVENT_DISCONNECTED)) {
        return TRANSPORT_ERR_TIMEOUT;
    }
    
    return TRANSPORT_OK;
}

static transport_err_t bt_spp_write(transport_t *self, const uint8_t *data, size_t len, 
                                     size_t *bytes_written, uint32_t timeout_ms)
{
    if (self == NULL || data == NULL || len == 0) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    bt_spp_transport_t *transport = (bt_spp_transport_t *)self;
    bt_spp_internal_t *internal = (bt_spp_internal_t *)transport->internal;
    
    if (internal == NULL) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (!internal->connected) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    if (timeout_ms == 0) {
        timeout_ms = BT_SPP_DEFAULT_TIMEOUT_MS;
    }
    
    if (xSemaphoreTake(internal->mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return TRANSPORT_ERR_TIMEOUT;
    }
    
    xEventGroupClearBits(internal->event_group, EVENT_WRITE_DONE);
    
    esp_err_t ret = esp_spp_write(internal->conn_handle, len, (uint8_t *)data);
    if (ret != ESP_OK) {
        xSemaphoreGive(internal->mutex);
        ESP_LOGE(TAG, "SPP write failed: %s", esp_err_to_name(ret));
        return TRANSPORT_ERR_IO;
    }
    
    if (xSemaphoreTake(internal->write_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        xSemaphoreGive(internal->mutex);
        return TRANSPORT_ERR_TIMEOUT;
    }
    
    if (bytes_written != NULL) {
        *bytes_written = len;
    }
    
    xSemaphoreGive(internal->mutex);
    
    return TRANSPORT_OK;
}

static transport_err_t bt_spp_read(transport_t *self, uint8_t *buffer, size_t len, 
                                    size_t *bytes_read, uint32_t timeout_ms)
{
    if (self == NULL || buffer == NULL || len == 0 || bytes_read == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    bt_spp_transport_t *transport = (bt_spp_transport_t *)self;
    bt_spp_internal_t *internal = (bt_spp_internal_t *)transport->internal;
    
    if (internal == NULL) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (!internal->connected) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    if (timeout_ms == 0) {
        timeout_ms = BT_SPP_DEFAULT_TIMEOUT_MS;
    }
    
    size_t item_size = 0;
    uint8_t *item = xRingbufferReceiveUpTo(internal->rx_ringbuf, &item_size, 
                                            pdMS_TO_TICKS(timeout_ms), len);
    
    if (item == NULL) {
        *bytes_read = 0;
        return TRANSPORT_ERR_TIMEOUT;
    }
    
    memcpy(buffer, item, item_size);
    vRingbufferReturnItem(internal->rx_ringbuf, item);
    *bytes_read = item_size;
    
    return TRANSPORT_OK;
}

static transport_err_t bt_spp_available(transport_t *self, size_t *available)
{
    if (self == NULL || available == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    bt_spp_transport_t *transport = (bt_spp_transport_t *)self;
    bt_spp_internal_t *internal = (bt_spp_internal_t *)transport->internal;
    
    if (internal == NULL) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (internal->rx_ringbuf == NULL) {
        *available = 0;
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    UBaseType_t items_waiting = 0;
    vRingbufferGetInfo(internal->rx_ringbuf, NULL, NULL, NULL, NULL, &items_waiting);
    *available = items_waiting;
    
    return TRANSPORT_OK;
}

static transport_err_t bt_spp_flush(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    bt_spp_transport_t *transport = (bt_spp_transport_t *)self;
    bt_spp_internal_t *internal = (bt_spp_internal_t *)transport->internal;
    
    if (internal == NULL) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (internal->rx_ringbuf == NULL) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    size_t item_size;
    void *item;
    while ((item = xRingbufferReceive(internal->rx_ringbuf, &item_size, 0)) != NULL) {
        vRingbufferReturnItem(internal->rx_ringbuf, item);
    }
    
    return TRANSPORT_OK;
}

static transport_state_t bt_spp_get_state(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_STATE_UNINITIALIZED;
    }
    return self->state;
}

static transport_type_t bt_spp_get_type(transport_t *self)
{
    (void)self;
    return TRANSPORT_TYPE_BT_CLASSIC;
}

static transport_err_t bt_spp_get_info(transport_t *self, char *info, size_t len)
{
    if (self == NULL || info == NULL || len == 0) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    bt_spp_transport_t *transport = (bt_spp_transport_t *)self;
    bt_spp_internal_t *internal = (bt_spp_internal_t *)transport->internal;
    
    if (internal == NULL) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    char addr_str[18] = "Not connected";
    if (internal != NULL && internal->connected) {
        bt_addr_to_str(transport->remote_addr, addr_str, sizeof(addr_str));
    }
    
    snprintf(info, len, "Bluetooth SPP - State: %s, Remote: %s",
             transport_state_to_str(self->state), addr_str);
    
    return TRANSPORT_OK;
}

static transport_err_t bt_spp_set_option(transport_t *self, int option, 
                                          const void *value, size_t len)
{
    if (self == NULL || value == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    bt_spp_transport_t *transport = (bt_spp_transport_t *)self;
    
    switch ((bt_spp_option_t)option) {
        case BT_SPP_OPT_DEVICE_NAME:
            if (len > 0) {
                esp_bt_dev_set_device_name((const char *)value);
            }
            break;
            
        case BT_SPP_OPT_DISCOVERABLE:
            if (len >= sizeof(bool)) {
                bool discoverable = *(const bool *)value;
                transport->config.discoverable = discoverable;
                if (discoverable) {
                    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                } else {
                    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                }
            }
            break;
            
        case BT_SPP_OPT_SECURITY_MODE:
            if (len >= sizeof(bt_spp_security_t)) {
                transport->config.security = *(const bt_spp_security_t *)value;
            }
            break;
            
        default:
            return TRANSPORT_ERR_NOT_SUPPORTED;
    }
    
    return TRANSPORT_OK;
}

static transport_err_t bt_spp_get_option(transport_t *self, int option, 
                                          void *value, size_t *len)
{
    if (self == NULL || value == NULL || len == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    bt_spp_transport_t *transport = (bt_spp_transport_t *)self;
    
    switch ((bt_spp_option_t)option) {
        case BT_SPP_OPT_DISCOVERABLE:
            if (*len >= sizeof(bool)) {
                *(bool *)value = transport->config.discoverable;
                *len = sizeof(bool);
            }
            break;
            
        case BT_SPP_OPT_SECURITY_MODE:
            if (*len >= sizeof(bt_spp_security_t)) {
                *(bt_spp_security_t *)value = transport->config.security;
                *len = sizeof(bt_spp_security_t);
            }
            break;
            
        default:
            return TRANSPORT_ERR_NOT_SUPPORTED;
    }
    
    return TRANSPORT_OK;
}

bt_spp_transport_t *bt_spp_transport_create(void)
{
    bt_spp_transport_t *transport = calloc(1, sizeof(bt_spp_transport_t));
    if (transport == NULL) {
        return NULL;
    }
    
    transport->base.vtable = &bt_spp_vtable;
    transport->base.state = TRANSPORT_STATE_UNINITIALIZED;
    transport->base.name = "Bluetooth SPP";
    
    bt_spp_config_default(&transport->config);
    
    return transport;
}

void bt_spp_transport_destroy(bt_spp_transport_t *transport)
{
    if (transport == NULL) {
        return;
    }
    
    if (transport->base.state != TRANSPORT_STATE_UNINITIALIZED) {
        bt_spp_deinit(&transport->base);
    }
    
    free(transport);
}

void bt_spp_config_default(bt_spp_config_t *config)
{
    if (config == NULL) {
        return;
    }
    
    memset(config, 0, sizeof(bt_spp_config_t));
    config->base.name = "ESP32_SPP";
    config->base.timeout_ms = BT_SPP_DEFAULT_TIMEOUT_MS;
    config->base.rx_buffer_size = BT_SPP_DEFAULT_BUFFER_SIZE;
    config->base.tx_buffer_size = BT_SPP_DEFAULT_BUFFER_SIZE;
    config->device_name = "ESP32_SPP";
    config->role = BT_SPP_ROLE_SLAVE;
    config->security = BT_SPP_SEC_AUTHENTICATE_ENCRYPT;
    config->pin_code = "1234";
    config->discoverable = true;
    config->connectable = true;
}

const transport_vtable_t *bt_spp_get_vtable(void)
{
    return &bt_spp_vtable;
}

transport_err_t bt_spp_listen(bt_spp_transport_t *transport)
{
    if (transport == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    if (transport->base.state == TRANSPORT_STATE_UNINITIALIZED) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    transport->is_server = true;
    
    esp_err_t ret = esp_spp_start_srv(transport->config.security, ESP_SPP_ROLE_SLAVE, 0, 
                                       transport->config.device_name);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPP server start failed: %s", esp_err_to_name(ret));
        return TRANSPORT_ERR_IO;
    }
    
    return TRANSPORT_OK;
}

transport_err_t bt_spp_stop_listen(bt_spp_transport_t *transport)
{
    if (transport == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = esp_spp_stop_srv();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPP server stop failed: %s", esp_err_to_name(ret));
        return TRANSPORT_ERR_IO;
    }
    
    transport->is_server = false;
    
    return TRANSPORT_OK;
}

transport_err_t bt_spp_get_remote_address(bt_spp_transport_t *transport, uint8_t *addr)
{
    if (transport == NULL || addr == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    bt_spp_internal_t *internal = (bt_spp_internal_t *)transport->internal;
    
    if (internal == NULL || !internal->connected) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    memcpy(addr, transport->remote_addr, 6);
    
    return TRANSPORT_OK;
}

char *bt_addr_to_str(const uint8_t *addr, char *str, size_t len)
{
    if (addr == NULL || str == NULL || len < 18) {
        return NULL;
    }
    
    snprintf(str, len, "%02X:%02X:%02X:%02X:%02X:%02X",
             addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    
    return str;
}

transport_err_t bt_str_to_addr(const char *str, uint8_t *addr)
{
    if (str == NULL || addr == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    unsigned int tmp[6];
    int ret = sscanf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
                     &tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4], &tmp[5]);
    
    if (ret != 6) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    for (int i = 0; i < 6; i++) {
        addr[i] = (uint8_t)tmp[i];
    }
    
    return TRANSPORT_OK;
}