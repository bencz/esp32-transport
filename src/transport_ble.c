/**
 * @file transport_ble.c
 * @brief Bluetooth Low Energy backend implementation
 */

#include "transport_ble.h"

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
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

static const char *TAG = "ble_transport";

#define BLE_DEFAULT_MTU             (512)
#define BLE_DEFAULT_BUFFER_SIZE     (4096)
#define BLE_DEFAULT_TIMEOUT_MS      (5000)
#define BLE_APP_ID                  (0)

typedef struct {
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t write_sem;
    SemaphoreHandle_t connect_sem;
    EventGroupHandle_t event_group;
    RingbufHandle_t rx_ringbuf;
    esp_gatt_if_t gatts_if;
    esp_gatt_if_t gattc_if;
    uint16_t service_handle;
    uint16_t descr_handle;
    bool advertising;
    bool scanning;
    bool registered;
    bool notify_enabled;
    ble_scan_result_cb_t scan_callback;
    void *scan_user_data;
} ble_internal_t;

#define EVENT_CONNECTED     BIT0
#define EVENT_DISCONNECTED  BIT1
#define EVENT_WRITE_DONE    BIT2
#define EVENT_DATA_READY    BIT3
#define EVENT_MTU_SET       BIT4

/**
 * @brief Current transport instance pointer
 * 
 * Note: ESP-IDF's BLE callbacks (esp_ble_gap_register_callback,
 * esp_ble_gatts_register_callback) do not provide a user context parameter,
 * so we must store the transport instance here to access it from callbacks.
 * This limits the implementation to a single BLE transport instance at a time,
 * which aligns with ESP-IDF's single Bluetooth stack limitation.
 * 
 * The pointer is set during init and cleared during deinit.
 */
static ble_transport_t *s_transport_instance = NULL;

static transport_err_t ble_init(transport_t *self, const void *config);
static transport_err_t ble_deinit(transport_t *self);
static transport_err_t ble_connect(transport_t *self, const char *address, uint32_t timeout_ms);
static transport_err_t ble_disconnect(transport_t *self);
static transport_err_t ble_write(transport_t *self, const uint8_t *data, size_t len, 
                                  size_t *bytes_written, uint32_t timeout_ms);
static transport_err_t ble_read(transport_t *self, uint8_t *buffer, size_t len, 
                                 size_t *bytes_read, uint32_t timeout_ms);
static transport_err_t ble_available(transport_t *self, size_t *available);
static transport_err_t ble_flush(transport_t *self);
static transport_state_t ble_get_state(transport_t *self);
static transport_type_t ble_get_type(transport_t *self);
static transport_err_t ble_get_info(transport_t *self, char *info, size_t len);
static transport_err_t ble_set_option(transport_t *self, int option, 
                                       const void *value, size_t len);
static transport_err_t ble_get_option(transport_t *self, int option, 
                                       void *value, size_t *len);

static const transport_vtable_t ble_vtable = {
    .init       = ble_init,
    .deinit     = ble_deinit,
    .connect    = ble_connect,
    .disconnect = ble_disconnect,
    .write      = ble_write,
    .read       = ble_read,
    .available  = ble_available,
    .flush      = ble_flush,
    .get_state  = ble_get_state,
    .get_type   = ble_get_type,
    .get_info   = ble_get_info,
    .set_option = ble_set_option,
    .get_option = ble_get_option,
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t char_decl_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t char_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read_write_notify = 
    ESP_GATT_CHAR_PROP_BIT_READ | 
    ESP_GATT_CHAR_PROP_BIT_WRITE | 
    ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ble_transport_t *transport = s_transport_instance;
    if (transport == NULL) {
        return;
    }
    
    ble_internal_t *internal = (ble_internal_t *)transport->internal;

    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Advertising data set complete");
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising start failed: %d", param->adv_start_cmpl.status);
            } else {
                ESP_LOGI(TAG, "Advertising started");
                internal->advertising = true;
            }
            break;
            
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising stop failed: %d", param->adv_stop_cmpl.status);
            } else {
                ESP_LOGI(TAG, "Advertising stopped");
                internal->advertising = false;
            }
            break;
            
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                if (internal->scan_callback != NULL) {
                    uint8_t *adv_name = NULL;
                    uint8_t adv_name_len = 0;
                    adv_name = esp_ble_resolve_adv_data(param->scan_rst.ble_adv,
                                                        ESP_BLE_AD_TYPE_NAME_CMPL,
                                                        &adv_name_len);
                    char name[32] = {0};
                    if (adv_name != NULL && adv_name_len > 0) {
                        size_t copy_len = adv_name_len < sizeof(name) - 1 ? 
                                          adv_name_len : sizeof(name) - 1;
                        memcpy(name, adv_name, copy_len);
                    }
                    internal->scan_callback(param->scan_rst.bda,
                                           param->scan_rst.ble_addr_type,
                                           param->scan_rst.rssi,
                                           name,
                                           internal->scan_user_data);
                }
            }
            break;
            
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Scan start failed: %d", param->scan_start_cmpl.status);
            } else {
                ESP_LOGI(TAG, "Scan started");
                internal->scanning = true;
            }
            break;
            
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Scan stop failed: %d", param->scan_stop_cmpl.status);
            } else {
                ESP_LOGI(TAG, "Scan stopped");
                internal->scanning = false;
            }
            break;
            
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "Connection params updated");
            break;
            
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                 esp_ble_gatts_cb_param_t *param)
{
    ble_transport_t *transport = s_transport_instance;
    if (transport == NULL) {
        return;
    }
    
    ble_internal_t *internal = (ble_internal_t *)transport->internal;

    switch (event) {
        case ESP_GATTS_REG_EVT:
            if (param->reg.status == ESP_GATT_OK) {
                internal->gatts_if = gatts_if;
                internal->registered = true;
                ESP_LOGI(TAG, "GATTS registered, app_id=%d", param->reg.app_id);
                
                esp_ble_gap_set_device_name(transport->config.device_name);
                
                uint8_t adv_data[] = {
                    0x02, 0x01, 0x06,
                    0x03, 0x03, 
                    (uint8_t)(transport->config.service_uuid & 0xFF),
                    (uint8_t)((transport->config.service_uuid >> 8) & 0xFF),
                };
                esp_ble_gap_config_adv_data_raw(adv_data, sizeof(adv_data));
                
                esp_gatts_attr_db_t gatt_db[4];
                memset(gatt_db, 0, sizeof(gatt_db));
                
                uint16_t service_uuid = transport->config.service_uuid;
                gatt_db[0].attr_control.auto_rsp = ESP_GATT_AUTO_RSP;
                gatt_db[0].att_desc.uuid_length = ESP_UUID_LEN_16;
                gatt_db[0].att_desc.uuid_p = (uint8_t *)&primary_service_uuid;
                gatt_db[0].att_desc.perm = ESP_GATT_PERM_READ;
                gatt_db[0].att_desc.max_length = sizeof(uint16_t);
                gatt_db[0].att_desc.length = sizeof(uint16_t);
                gatt_db[0].att_desc.value = (uint8_t *)&service_uuid;
                
                uint16_t tx_char_uuid = transport->config.tx_char_uuid;
                gatt_db[1].attr_control.auto_rsp = ESP_GATT_AUTO_RSP;
                gatt_db[1].att_desc.uuid_length = ESP_UUID_LEN_16;
                gatt_db[1].att_desc.uuid_p = (uint8_t *)&char_decl_uuid;
                gatt_db[1].att_desc.perm = ESP_GATT_PERM_READ;
                gatt_db[1].att_desc.max_length = sizeof(uint8_t);
                gatt_db[1].att_desc.length = sizeof(uint8_t);
                gatt_db[1].att_desc.value = (uint8_t *)&char_prop_read_write_notify;
                
                gatt_db[2].attr_control.auto_rsp = ESP_GATT_RSP_BY_APP;
                gatt_db[2].att_desc.uuid_length = ESP_UUID_LEN_16;
                gatt_db[2].att_desc.uuid_p = (uint8_t *)&tx_char_uuid;
                gatt_db[2].att_desc.perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE;
                gatt_db[2].att_desc.max_length = BLE_DEFAULT_MTU;
                gatt_db[2].att_desc.length = 0;
                gatt_db[2].att_desc.value = NULL;
                
                uint16_t ccc_val = 0x0000;
                gatt_db[3].attr_control.auto_rsp = ESP_GATT_AUTO_RSP;
                gatt_db[3].att_desc.uuid_length = ESP_UUID_LEN_16;
                gatt_db[3].att_desc.uuid_p = (uint8_t *)&char_client_config_uuid;
                gatt_db[3].att_desc.perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE;
                gatt_db[3].att_desc.max_length = sizeof(uint16_t);
                gatt_db[3].att_desc.length = sizeof(uint16_t);
                gatt_db[3].att_desc.value = (uint8_t *)&ccc_val;
                
                esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, 4, 0);
            } else {
                ESP_LOGE(TAG, "GATTS registration failed: %d", param->reg.status);
            }
            break;
            
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            if (param->add_attr_tab.status == ESP_GATT_OK) {
                internal->service_handle = param->add_attr_tab.handles[0];
                transport->attr_handle_tx = param->add_attr_tab.handles[2];
                internal->descr_handle = param->add_attr_tab.handles[3];
                esp_ble_gatts_start_service(internal->service_handle);
            }
            break;
            
        case ESP_GATTS_CONNECT_EVT: {
            ESP_LOGI(TAG, "Client connected, conn_id=%d", param->connect.conn_id);
            transport->conn_handle = param->connect.conn_id;
            memcpy(transport->peer_addr, param->connect.remote_bda, 6);
            transport_set_state(&transport->base, TRANSPORT_STATE_CONNECTED);
            xEventGroupSetBits(internal->event_group, EVENT_CONNECTED);
            
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, 6);
            conn_params.min_int = transport->config.conn_config.min_interval;
            conn_params.max_int = transport->config.conn_config.max_interval;
            conn_params.latency = transport->config.conn_config.latency;
            conn_params.timeout = transport->config.conn_config.timeout;
            esp_ble_gap_update_conn_params(&conn_params);
            
            transport_event_t conn_evt = { .type = TRANSPORT_EVENT_CONNECTED };
            transport_emit_event(&transport->base, &conn_evt);
            break;
        }
            
        case ESP_GATTS_DISCONNECT_EVT: {
            ESP_LOGI(TAG, "Client disconnected, reason=0x%x", param->disconnect.reason);
            transport->conn_handle = 0;
            internal->notify_enabled = false;
            xEventGroupSetBits(internal->event_group, EVENT_DISCONNECTED);
            xEventGroupClearBits(internal->event_group, EVENT_CONNECTED);
            transport_set_state(&transport->base, TRANSPORT_STATE_INITIALIZED);
            
            transport_event_t disc_evt = { .type = TRANSPORT_EVENT_DISCONNECTED };
            transport_emit_event(&transport->base, &disc_evt);
            
            if (internal->advertising) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        }
            
        case ESP_GATTS_WRITE_EVT:
            if (param->write.handle == transport->attr_handle_tx) {
                if (internal->rx_ringbuf != NULL && param->write.len > 0) {
                    xRingbufferSend(internal->rx_ringbuf, param->write.value, 
                                    param->write.len, 0);
                    xEventGroupSetBits(internal->event_group, EVENT_DATA_READY);
                    
                    transport_event_t data_evt = {
                        .type = TRANSPORT_EVENT_DATA_RECEIVED,
                        .data_received = {
                            .data = param->write.value,
                            .len = param->write.len,
                        },
                    };
                    transport_emit_event(&transport->base, &data_evt);
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                                param->write.trans_id, ESP_GATT_OK, NULL);
                }
            } else if (param->write.handle == internal->descr_handle) {
                if (param->write.len == 2) {
                    uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                    internal->notify_enabled = (descr_value == 0x0001);
                    ESP_LOGI(TAG, "Notifications %s", 
                             internal->notify_enabled ? "enabled" : "disabled");
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                                param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            break;
            
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(TAG, "MTU set to %d", param->mtu.mtu);
            transport->current_mtu = param->mtu.mtu;
            xEventGroupSetBits(internal->event_group, EVENT_MTU_SET);
            break;
            
        case ESP_GATTS_CONF_EVT:
            xEventGroupSetBits(internal->event_group, EVENT_WRITE_DONE);
            xSemaphoreGive(internal->write_sem);
            break;
            
        default:
            break;
    }
}

static transport_err_t init_ble_stack(ble_transport_t *transport)
{
    esp_err_t ret;
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller init failed: %s", esp_err_to_name(ret));
        return TRANSPORT_ERR_INTERNAL;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
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
    
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GAP callback registration failed: %s", esp_err_to_name(ret));
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        return TRANSPORT_ERR_INTERNAL;
    }
    
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTS callback registration failed: %s", esp_err_to_name(ret));
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        return TRANSPORT_ERR_INTERNAL;
    }
    
    ret = esp_ble_gatts_app_register(BLE_APP_ID);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTS app registration failed: %s", esp_err_to_name(ret));
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        return TRANSPORT_ERR_INTERNAL;
    }
    
    esp_ble_gatt_set_local_mtu(transport->config.mtu);
    
    return TRANSPORT_OK;
}

static void deinit_ble_stack(void)
{
    esp_ble_gatts_app_unregister(BLE_APP_ID);
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
}

static transport_err_t ble_init(transport_t *self, const void *config)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    ble_transport_t *transport = (ble_transport_t *)self;
    
    if (self->state != TRANSPORT_STATE_UNINITIALIZED) {
        return TRANSPORT_ERR_ALREADY_INITIALIZED;
    }
    
    // Create base mutex for thread safety
    transport->base.mutex = xSemaphoreCreateMutex();
    if (transport->base.mutex == NULL) {
        return TRANSPORT_ERR_NO_MEM;
    }
    
    if (config != NULL) {
        memcpy(&transport->config, config, sizeof(ble_config_t));
    }
    
    ble_internal_t *internal = calloc(1, sizeof(ble_internal_t));
    if (internal == NULL) {
        vSemaphoreDelete(transport->base.mutex);
        transport->base.mutex = NULL;
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
        vSemaphoreDelete(transport->base.mutex);
        transport->base.mutex = NULL;
        return TRANSPORT_ERR_NO_MEM;
    }
    
    size_t buffer_size = transport->config.base.rx_buffer_size;
    if (buffer_size == 0) {
        buffer_size = BLE_DEFAULT_BUFFER_SIZE;
    }
    
    internal->rx_ringbuf = xRingbufferCreate(buffer_size, RINGBUF_TYPE_BYTEBUF);
    if (internal->rx_ringbuf == NULL) {
        vSemaphoreDelete(internal->mutex);
        vSemaphoreDelete(internal->write_sem);
        vSemaphoreDelete(internal->connect_sem);
        vEventGroupDelete(internal->event_group);
        free(internal);
        vSemaphoreDelete(transport->base.mutex);
        transport->base.mutex = NULL;
        return TRANSPORT_ERR_NO_MEM;
    }
    
    transport->internal = internal;
    s_transport_instance = transport;
    
    transport_err_t err = init_ble_stack(transport);
    if (err != TRANSPORT_OK) {
        vRingbufferDelete(internal->rx_ringbuf);
        vSemaphoreDelete(internal->mutex);
        vSemaphoreDelete(internal->write_sem);
        vSemaphoreDelete(internal->connect_sem);
        vEventGroupDelete(internal->event_group);
        free(internal);
        transport->internal = NULL;
        s_transport_instance = NULL;
        vSemaphoreDelete(transport->base.mutex);
        transport->base.mutex = NULL;
        return err;
    }
    
    int retries = 50;
    while (!internal->registered && retries > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
        retries--;
    }
    
    if (!internal->registered) {
        deinit_ble_stack();
        vRingbufferDelete(internal->rx_ringbuf);
        vSemaphoreDelete(internal->mutex);
        vSemaphoreDelete(internal->write_sem);
        vSemaphoreDelete(internal->connect_sem);
        vEventGroupDelete(internal->event_group);
        free(internal);
        transport->internal = NULL;
        s_transport_instance = NULL;
        vSemaphoreDelete(transport->base.mutex);
        transport->base.mutex = NULL;
        return TRANSPORT_ERR_TIMEOUT;
    }
    
    transport->current_mtu = 23;
    transport_set_state(self, TRANSPORT_STATE_INITIALIZED);
    
    return TRANSPORT_OK;
}

static transport_err_t ble_deinit(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    ble_transport_t *transport = (ble_transport_t *)self;
    ble_internal_t *internal = (ble_internal_t *)transport->internal;
    
    if (self->state == TRANSPORT_STATE_UNINITIALIZED) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (self->state == TRANSPORT_STATE_CONNECTED) {
        ble_disconnect(self);
    }
    
    if (internal->advertising) {
        esp_ble_gap_stop_advertising();
    }
    
    deinit_ble_stack();
    
    if (internal->rx_ringbuf) vRingbufferDelete(internal->rx_ringbuf);
    if (internal->mutex) vSemaphoreDelete(internal->mutex);
    if (internal->write_sem) vSemaphoreDelete(internal->write_sem);
    if (internal->connect_sem) vSemaphoreDelete(internal->connect_sem);
    if (internal->event_group) vEventGroupDelete(internal->event_group);
    
    free(internal);
    transport->internal = NULL;
    s_transport_instance = NULL;
    
    // Delete base mutex
    if (transport->base.mutex) {
        vSemaphoreDelete(transport->base.mutex);
        transport->base.mutex = NULL;
    }
    
    transport_set_state(self, TRANSPORT_STATE_UNINITIALIZED);
    
    return TRANSPORT_OK;
}

static transport_err_t ble_connect(transport_t *self, const char *address, uint32_t timeout_ms)
{
    (void)address;
    (void)timeout_ms;
    
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    ble_transport_t *transport = (ble_transport_t *)self;
    
    if (transport->config.role == BLE_ROLE_PERIPHERAL) {
        return ble_start_advertising(transport);
    }
    
    return TRANSPORT_ERR_NOT_SUPPORTED;
}

static transport_err_t ble_disconnect(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    ble_transport_t *transport = (ble_transport_t *)self;
    ble_internal_t *internal = (ble_internal_t *)transport->internal;
    
    if (self->state != TRANSPORT_STATE_CONNECTED) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    transport_set_state(self, TRANSPORT_STATE_DISCONNECTING);
    
    esp_err_t ret = esp_ble_gatts_close(internal->gatts_if, transport->conn_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE disconnect failed: %s", esp_err_to_name(ret));
        return TRANSPORT_ERR_IO;
    }
    
    EventBits_t bits = xEventGroupWaitBits(internal->event_group, 
                                            EVENT_DISCONNECTED,
                                            pdTRUE, pdFALSE,
                                            pdMS_TO_TICKS(BLE_DEFAULT_TIMEOUT_MS));
    
    if (!(bits & EVENT_DISCONNECTED)) {
        return TRANSPORT_ERR_TIMEOUT;
    }
    
    return TRANSPORT_OK;
}

static transport_err_t ble_write(transport_t *self, const uint8_t *data, size_t len, 
                                  size_t *bytes_written, uint32_t timeout_ms)
{
    if (self == NULL || data == NULL || len == 0) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    ble_transport_t *transport = (ble_transport_t *)self;
    ble_internal_t *internal = (ble_internal_t *)transport->internal;
    
    if (internal == NULL) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (self->state != TRANSPORT_STATE_CONNECTED) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    if (!internal->notify_enabled) {
        ESP_LOGW(TAG, "Notifications not enabled by client");
        return TRANSPORT_ERR_NOT_SUPPORTED;
    }
    
    if (timeout_ms == 0) {
        timeout_ms = BLE_DEFAULT_TIMEOUT_MS;
    }
    
    if (xSemaphoreTake(internal->mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return TRANSPORT_ERR_TIMEOUT;
    }
    
    size_t mtu_payload = transport->current_mtu - 3;
    size_t offset = 0;
    
    while (offset < len) {
        size_t chunk_size = len - offset;
        if (chunk_size > mtu_payload) {
            chunk_size = mtu_payload;
        }
        
        xEventGroupClearBits(internal->event_group, EVENT_WRITE_DONE);
        
        esp_err_t ret = esp_ble_gatts_send_indicate(internal->gatts_if,
                                                     transport->conn_handle,
                                                     transport->attr_handle_tx,
                                                     chunk_size,
                                                     (uint8_t *)(data + offset),
                                                     false);
        if (ret != ESP_OK) {
            xSemaphoreGive(internal->mutex);
            ESP_LOGE(TAG, "BLE send failed: %s", esp_err_to_name(ret));
            return TRANSPORT_ERR_IO;
        }
        
        if (xSemaphoreTake(internal->write_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
            xSemaphoreGive(internal->mutex);
            return TRANSPORT_ERR_TIMEOUT;
        }
        
        offset += chunk_size;
    }
    
    if (bytes_written != NULL) {
        *bytes_written = len;
    }
    
    xSemaphoreGive(internal->mutex);
    
    return TRANSPORT_OK;
}

static transport_err_t ble_read(transport_t *self, uint8_t *buffer, size_t len, 
                                 size_t *bytes_read, uint32_t timeout_ms)
{
    if (self == NULL || buffer == NULL || len == 0 || bytes_read == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    ble_transport_t *transport = (ble_transport_t *)self;
    ble_internal_t *internal = (ble_internal_t *)transport->internal;
    
    if (self->state != TRANSPORT_STATE_CONNECTED) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    if (timeout_ms == 0) {
        timeout_ms = BLE_DEFAULT_TIMEOUT_MS;
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

static transport_err_t ble_available(transport_t *self, size_t *available)
{
    if (self == NULL || available == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    ble_transport_t *transport = (ble_transport_t *)self;
    ble_internal_t *internal = (ble_internal_t *)transport->internal;
    
    if (internal->rx_ringbuf == NULL) {
        *available = 0;
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    UBaseType_t items_waiting = 0;
    vRingbufferGetInfo(internal->rx_ringbuf, NULL, NULL, NULL, NULL, &items_waiting);
    *available = items_waiting;
    
    return TRANSPORT_OK;
}

static transport_err_t ble_flush(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    ble_transport_t *transport = (ble_transport_t *)self;
    ble_internal_t *internal = (ble_internal_t *)transport->internal;
    
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

static transport_state_t ble_get_state(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_STATE_UNINITIALIZED;
    }
    return self->state;
}

static transport_type_t ble_get_type(transport_t *self)
{
    (void)self;
    return TRANSPORT_TYPE_BLE;
}

static transport_err_t ble_get_info(transport_t *self, char *info, size_t len)
{
    if (self == NULL || info == NULL || len == 0) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    ble_transport_t *transport = (ble_transport_t *)self;
    
    snprintf(info, len, "BLE - State: %s, MTU: %d",
             transport_state_to_str(self->state), transport->current_mtu);
    
    return TRANSPORT_OK;
}

static transport_err_t ble_set_option(transport_t *self, int option, 
                                       const void *value, size_t len)
{
    if (self == NULL || value == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    ble_transport_t *transport = (ble_transport_t *)self;
    
    switch ((ble_option_t)option) {
        case BLE_OPT_DEVICE_NAME:
            if (len > 0) {
                esp_ble_gap_set_device_name((const char *)value);
            }
            break;
            
        case BLE_OPT_MTU:
            if (len >= sizeof(uint16_t)) {
                uint16_t mtu = *(const uint16_t *)value;
                esp_ble_gatt_set_local_mtu(mtu);
                transport->config.mtu = mtu;
            }
            break;
            
        default:
            return TRANSPORT_ERR_NOT_SUPPORTED;
    }
    
    return TRANSPORT_OK;
}

static transport_err_t ble_get_option(transport_t *self, int option, 
                                       void *value, size_t *len)
{
    if (self == NULL || value == NULL || len == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    ble_transport_t *transport = (ble_transport_t *)self;
    
    switch ((ble_option_t)option) {
        case BLE_OPT_MTU:
            if (*len >= sizeof(uint16_t)) {
                *(uint16_t *)value = transport->current_mtu;
                *len = sizeof(uint16_t);
            }
            break;
            
        default:
            return TRANSPORT_ERR_NOT_SUPPORTED;
    }
    
    return TRANSPORT_OK;
}

ble_transport_t *ble_transport_create(void)
{
    ble_transport_t *transport = calloc(1, sizeof(ble_transport_t));
    if (transport == NULL) {
        return NULL;
    }
    
    transport->base.vtable = &ble_vtable;
    transport->base.state = TRANSPORT_STATE_UNINITIALIZED;
    transport->base.name = "Bluetooth Low Energy";
    
    ble_config_default(&transport->config);
    
    return transport;
}

void ble_transport_destroy(ble_transport_t *transport)
{
    if (transport == NULL) {
        return;
    }
    
    if (transport->base.state != TRANSPORT_STATE_UNINITIALIZED) {
        ble_deinit(&transport->base);
    }
    
    free(transport);
}

void ble_config_default(ble_config_t *config)
{
    if (config == NULL) {
        return;
    }
    
    memset(config, 0, sizeof(ble_config_t));
    config->base.name = "ESP32_BLE";
    config->base.timeout_ms = BLE_DEFAULT_TIMEOUT_MS;
    config->base.rx_buffer_size = BLE_DEFAULT_BUFFER_SIZE;
    config->base.tx_buffer_size = BLE_DEFAULT_BUFFER_SIZE;
    config->device_name = "ESP32_BLE";
    config->role = BLE_ROLE_PERIPHERAL;
    config->service_uuid = BLE_TRANSPORT_SERVICE_UUID;
    config->tx_char_uuid = BLE_TRANSPORT_CHAR_TX_UUID;
    config->rx_char_uuid = BLE_TRANSPORT_CHAR_RX_UUID;
    config->mtu = BLE_DEFAULT_MTU;
    config->adv_config.min_interval = 0x20;
    config->adv_config.max_interval = 0x40;
    config->adv_config.connectable = true;
    config->adv_config.scannable = true;
    config->conn_config.min_interval = 0x10;
    config->conn_config.max_interval = 0x20;
    config->conn_config.latency = 0;
    config->conn_config.timeout = 400;
}

const transport_vtable_t *ble_get_vtable(void)
{
    return &ble_vtable;
}

transport_err_t ble_start_advertising(ble_transport_t *transport)
{
    if (transport == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    ble_internal_t *internal = (ble_internal_t *)transport->internal;
    
    if (transport->base.state == TRANSPORT_STATE_UNINITIALIZED) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (internal->advertising) {
        return TRANSPORT_OK;
    }
    
    adv_params.adv_int_min = transport->config.adv_config.min_interval;
    adv_params.adv_int_max = transport->config.adv_config.max_interval;
    
    esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Start advertising failed: %s", esp_err_to_name(ret));
        return TRANSPORT_ERR_IO;
    }
    
    return TRANSPORT_OK;
}

transport_err_t ble_stop_advertising(ble_transport_t *transport)
{
    if (transport == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    ble_internal_t *internal = (ble_internal_t *)transport->internal;
    
    if (!internal->advertising) {
        return TRANSPORT_OK;
    }
    
    esp_err_t ret = esp_ble_gap_stop_advertising();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Stop advertising failed: %s", esp_err_to_name(ret));
        return TRANSPORT_ERR_IO;
    }
    
    return TRANSPORT_OK;
}

transport_err_t ble_start_scan(ble_transport_t *transport, uint32_t duration_sec)
{
    if (transport == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    ble_internal_t *internal = (ble_internal_t *)transport->internal;
    
    if (transport->base.state == TRANSPORT_STATE_UNINITIALIZED) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (internal->scanning) {
        return TRANSPORT_OK;
    }
    
    esp_ble_scan_params_t scan_params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,
        .scan_window = 0x30,
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
    };
    
    esp_ble_gap_set_scan_params(&scan_params);
    
    esp_err_t ret = esp_ble_gap_start_scanning(duration_sec);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Start scanning failed: %s", esp_err_to_name(ret));
        return TRANSPORT_ERR_IO;
    }
    
    return TRANSPORT_OK;
}

transport_err_t ble_stop_scan(ble_transport_t *transport)
{
    if (transport == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    ble_internal_t *internal = (ble_internal_t *)transport->internal;
    
    if (!internal->scanning) {
        return TRANSPORT_OK;
    }
    
    esp_err_t ret = esp_ble_gap_stop_scanning();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Stop scanning failed: %s", esp_err_to_name(ret));
        return TRANSPORT_ERR_IO;
    }
    
    return TRANSPORT_OK;
}

uint16_t ble_get_mtu(ble_transport_t *transport)
{
    if (transport == NULL) {
        return 0;
    }
    return transport->current_mtu;
}

transport_err_t ble_request_mtu(ble_transport_t *transport, uint16_t mtu)
{
    if (transport == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    if (transport->base.state != TRANSPORT_STATE_CONNECTED) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    esp_err_t ret = esp_ble_gattc_send_mtu_req(transport->conn_handle, mtu);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MTU request failed: %s", esp_err_to_name(ret));
        return TRANSPORT_ERR_IO;
    }
    
    return TRANSPORT_OK;
}

transport_err_t ble_get_peer_address(ble_transport_t *transport, uint8_t *addr, uint8_t *addr_type)
{
    if (transport == NULL || addr == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    if (transport->base.state != TRANSPORT_STATE_CONNECTED) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    memcpy(addr, transport->peer_addr, 6);
    if (addr_type != NULL) {
        *addr_type = transport->peer_addr_type;
    }
    
    return TRANSPORT_OK;
}

transport_err_t ble_set_scan_callback(ble_transport_t *transport, 
                                       ble_scan_result_cb_t callback, 
                                       void *user_data)
{
    if (transport == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    ble_internal_t *internal = (ble_internal_t *)transport->internal;
    
    if (internal == NULL) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    internal->scan_callback = callback;
    internal->scan_user_data = user_data;
    
    return TRANSPORT_OK;
}