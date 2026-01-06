/**
 * @file transport_usb.c
 * @brief USB CDC transport backend implementation
 */

#include "transport_usb.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

static const char *TAG = "usb_transport";

#define USB_DEFAULT_BUFFER_SIZE     (4096)
#define USB_DEFAULT_TIMEOUT_MS      (1000)
#define USB_CDC_ITF                 (0)

typedef struct {
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t rx_sem;
    uint8_t *rx_buffer;
    size_t rx_buffer_size;
    size_t rx_head;
    size_t rx_tail;
    bool initialized;
} usb_internal_t;

/**
 * @brief Current transport instance pointer
 * 
 * Note: TinyUSB CDC callbacks do not provide a user context parameter,
 * so we must store the transport instance here to access it from callbacks.
 * This limits the implementation to a single USB CDC transport instance at a time.
 * 
 * The pointer is set during init and cleared during deinit.
 */
static usb_transport_t *s_transport_instance = NULL;

static transport_err_t usb_init(transport_t *self, const void *config);
static transport_err_t usb_deinit(transport_t *self);
static transport_err_t usb_connect(transport_t *self, const char *address, uint32_t timeout_ms);
static transport_err_t usb_disconnect(transport_t *self);
static transport_err_t usb_write(transport_t *self, const uint8_t *data, size_t len, 
                                  size_t *bytes_written, uint32_t timeout_ms);
static transport_err_t usb_read(transport_t *self, uint8_t *buffer, size_t len, 
                                 size_t *bytes_read, uint32_t timeout_ms);
static transport_err_t usb_available(transport_t *self, size_t *available);
static transport_err_t usb_flush(transport_t *self);
static transport_state_t usb_get_state(transport_t *self);
static transport_type_t usb_get_type(transport_t *self);
static transport_err_t usb_get_info(transport_t *self, char *info, size_t len);
static transport_err_t usb_set_option(transport_t *self, int option, 
                                       const void *value, size_t len);
static transport_err_t usb_get_option(transport_t *self, int option, 
                                       void *value, size_t *len);

static const transport_vtable_t usb_vtable = {
    .init       = usb_init,
    .deinit     = usb_deinit,
    .connect    = usb_connect,
    .disconnect = usb_disconnect,
    .write      = usb_write,
    .read       = usb_read,
    .available  = usb_available,
    .flush      = usb_flush,
    .get_state  = usb_get_state,
    .get_type   = usb_get_type,
    .get_info   = usb_get_info,
    .set_option = usb_set_option,
    .get_option = usb_get_option,
};

static size_t ringbuf_available(usb_internal_t *internal)
{
    if (internal->rx_head >= internal->rx_tail) {
        return internal->rx_head - internal->rx_tail;
    }
    return internal->rx_buffer_size - internal->rx_tail + internal->rx_head;
}

static size_t ringbuf_free(usb_internal_t *internal)
{
    return internal->rx_buffer_size - ringbuf_available(internal) - 1;
}

static void ringbuf_write(usb_internal_t *internal, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        internal->rx_buffer[internal->rx_head] = data[i];
        internal->rx_head = (internal->rx_head + 1) % internal->rx_buffer_size;
    }
}

static size_t ringbuf_read(usb_internal_t *internal, uint8_t *data, size_t max_len)
{
    size_t available = ringbuf_available(internal);
    size_t to_read = available < max_len ? available : max_len;
    
    for (size_t i = 0; i < to_read; i++) {
        data[i] = internal->rx_buffer[internal->rx_tail];
        internal->rx_tail = (internal->rx_tail + 1) % internal->rx_buffer_size;
    }
    
    return to_read;
}

static void cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    (void)itf;
    
    usb_transport_t *transport = s_transport_instance;
    if (transport == NULL) {
        return;
    }
    
    usb_internal_t *internal = (usb_internal_t *)transport->internal;
    if (internal == NULL) {
        return;
    }
    
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE];
    size_t rx_size = 0;
    
    esp_err_t ret = tinyusb_cdcacm_read(itf, buf, sizeof(buf), &rx_size);
    if (ret == ESP_OK && rx_size > 0) {
        size_t free_space = ringbuf_free(internal);
        size_t to_write = rx_size < free_space ? rx_size : free_space;
        
        if (to_write > 0) {
            ringbuf_write(internal, buf, to_write);
            xSemaphoreGive(internal->rx_sem);
            
            transport_event_t data_evt = {
                .type = TRANSPORT_EVENT_DATA_RECEIVED,
                .data_received = {
                    .data = buf,
                    .len = rx_size,
                },
            };
            transport_emit_event(&transport->base, &data_evt);
        }
    }
}

static void cdc_line_state_callback(int itf, cdcacm_event_t *event)
{
    (void)itf;
    
    usb_transport_t *transport = s_transport_instance;
    if (transport == NULL) {
        return;
    }
    
    bool dtr = event->line_state_changed_data.dtr;
    bool rts = event->line_state_changed_data.rts;
    
    bool was_connected = transport->dtr;
    transport->dtr = dtr;
    transport->rts = rts;
    
    ESP_LOGI(TAG, "Line state: DTR=%d, RTS=%d", dtr, rts);
    
    if (dtr && !was_connected) {
        transport_set_state(&transport->base, TRANSPORT_STATE_CONNECTED);
        transport_event_t evt = { .type = TRANSPORT_EVENT_CONNECTED };
        transport_emit_event(&transport->base, &evt);
    } else if (!dtr && was_connected) {
        transport_set_state(&transport->base, TRANSPORT_STATE_INITIALIZED);
        transport_event_t evt = { .type = TRANSPORT_EVENT_DISCONNECTED };
        transport_emit_event(&transport->base, &evt);
    }
}

static void cdc_line_coding_callback(int itf, cdcacm_event_t *event)
{
    (void)itf;
    
    usb_transport_t *transport = s_transport_instance;
    if (transport == NULL) {
        return;
    }
    
    cdc_line_coding_t const *coding = event->line_coding_changed_data.p_line_coding;
    
    transport->config.line_coding.baud_rate = coding->bit_rate;
    transport->config.line_coding.stop_bits = coding->stop_bits;
    transport->config.line_coding.parity = coding->parity;
    transport->config.line_coding.data_bits = coding->data_bits;
    
    ESP_LOGI(TAG, "Line coding: %lu baud, %d data bits, %d stop bits, parity %d",
             (unsigned long)coding->bit_rate, coding->data_bits, 
             coding->stop_bits, coding->parity);
}

static transport_err_t usb_init(transport_t *self, const void *config)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    usb_transport_t *transport = (usb_transport_t *)self;
    
    if (self->state != TRANSPORT_STATE_UNINITIALIZED) {
        return TRANSPORT_ERR_ALREADY_INITIALIZED;
    }
    
    if (config != NULL) {
        memcpy(&transport->config, config, sizeof(usb_transport_config_t));
    }
    
    usb_internal_t *internal = calloc(1, sizeof(usb_internal_t));
    if (internal == NULL) {
        return TRANSPORT_ERR_NO_MEM;
    }
    
    internal->mutex = xSemaphoreCreateMutex();
    internal->rx_sem = xSemaphoreCreateBinary();
    
    if (internal->mutex == NULL || internal->rx_sem == NULL) {
        if (internal->mutex) vSemaphoreDelete(internal->mutex);
        if (internal->rx_sem) vSemaphoreDelete(internal->rx_sem);
        free(internal);
        return TRANSPORT_ERR_NO_MEM;
    }
    
    internal->rx_buffer_size = transport->config.base.rx_buffer_size;
    if (internal->rx_buffer_size == 0) {
        internal->rx_buffer_size = USB_DEFAULT_BUFFER_SIZE;
    }
    
    internal->rx_buffer = malloc(internal->rx_buffer_size);
    if (internal->rx_buffer == NULL) {
        vSemaphoreDelete(internal->mutex);
        vSemaphoreDelete(internal->rx_sem);
        free(internal);
        return TRANSPORT_ERR_NO_MEM;
    }
    
    transport->internal = internal;
    s_transport_instance = transport;
    
    transport_set_state(self, TRANSPORT_STATE_INITIALIZED);
    
    ESP_LOGI(TAG, "USB CDC transport initialized");
    
    return TRANSPORT_OK;
}

static transport_err_t usb_deinit(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    usb_transport_t *transport = (usb_transport_t *)self;
    usb_internal_t *internal = (usb_internal_t *)transport->internal;
    
    if (self->state == TRANSPORT_STATE_UNINITIALIZED) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (transport->is_open) {
        usb_disconnect(self);
    }
    
    if (internal->mutex) vSemaphoreDelete(internal->mutex);
    if (internal->rx_sem) vSemaphoreDelete(internal->rx_sem);
    if (internal->rx_buffer) free(internal->rx_buffer);
    
    free(internal);
    transport->internal = NULL;
    s_transport_instance = NULL;
    
    transport_set_state(self, TRANSPORT_STATE_UNINITIALIZED);
    
    ESP_LOGI(TAG, "USB CDC transport deinitialized");
    
    return TRANSPORT_OK;
}

static transport_err_t usb_connect(transport_t *self, const char *address, uint32_t timeout_ms)
{
    (void)address;
    
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    usb_transport_t *transport = (usb_transport_t *)self;
    usb_internal_t *internal = (usb_internal_t *)transport->internal;
    
    if (self->state == TRANSPORT_STATE_UNINITIALIZED) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (transport->is_open) {
        return TRANSPORT_ERR_ALREADY_CONNECTED;
    }
    
    if (!internal->initialized) {
        const tinyusb_config_t tusb_cfg = {
            .device_descriptor = NULL,
            .string_descriptor = NULL,
            .external_phy = false,
            .configuration_descriptor = NULL,
        };
        
        esp_err_t ret = tinyusb_driver_install(&tusb_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "TinyUSB driver install failed: %s", esp_err_to_name(ret));
            return TRANSPORT_ERR_IO;
        }
        
        tinyusb_config_cdcacm_t acm_cfg = {
            .usb_dev = TINYUSB_USBDEV_0,
            .cdc_port = TINYUSB_CDC_ACM_0,
            .rx_unread_buf_sz = 256,
            .callback_rx = &cdc_rx_callback,
            .callback_rx_wanted_char = NULL,
            .callback_line_state_changed = &cdc_line_state_callback,
            .callback_line_coding_changed = &cdc_line_coding_callback,
        };
        
        ret = tusb_cdc_acm_init(&acm_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "CDC ACM init failed: %s", esp_err_to_name(ret));
            return TRANSPORT_ERR_IO;
        }
        
        internal->initialized = true;
    }
    
    transport->is_open = true;
    
    if (transport->dtr) {
        transport_set_state(self, TRANSPORT_STATE_CONNECTED);
        transport_event_t evt = { .type = TRANSPORT_EVENT_CONNECTED };
        transport_emit_event(self, &evt);
    } else {
        transport_set_state(self, TRANSPORT_STATE_INITIALIZED);
    }
    
    ESP_LOGI(TAG, "USB CDC opened, waiting for host connection");
    
    if (timeout_ms > 0) {
        return usb_transport_wait_for_host(transport, timeout_ms);
    }
    
    return TRANSPORT_OK;
}

static transport_err_t usb_disconnect(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    usb_transport_t *transport = (usb_transport_t *)self;
    
    if (!transport->is_open) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    transport->is_open = false;
    transport_set_state(self, TRANSPORT_STATE_INITIALIZED);
    
    transport_event_t evt = { .type = TRANSPORT_EVENT_DISCONNECTED };
    transport_emit_event(self, &evt);
    
    ESP_LOGI(TAG, "USB CDC closed");
    
    return TRANSPORT_OK;
}

static transport_err_t usb_write(transport_t *self, const uint8_t *data, size_t len, 
                                  size_t *bytes_written, uint32_t timeout_ms)
{
    if (self == NULL || data == NULL || len == 0) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    usb_transport_t *transport = (usb_transport_t *)self;
    usb_internal_t *internal = (usb_internal_t *)transport->internal;
    
    if (!transport->is_open || !transport->dtr) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    if (timeout_ms == 0) {
        timeout_ms = USB_DEFAULT_TIMEOUT_MS;
    }
    
    if (xSemaphoreTake(internal->mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return TRANSPORT_ERR_TIMEOUT;
    }
    
    size_t written = tinyusb_cdcacm_write_queue(USB_CDC_ITF, data, len);
    esp_err_t ret = tinyusb_cdcacm_write_flush(USB_CDC_ITF, pdMS_TO_TICKS(timeout_ms));
    
    xSemaphoreGive(internal->mutex);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB write flush failed: %s", esp_err_to_name(ret));
        return TRANSPORT_ERR_IO;
    }
    
    if (bytes_written != NULL) {
        *bytes_written = written;
    }
    
    return TRANSPORT_OK;
}

static transport_err_t usb_read(transport_t *self, uint8_t *buffer, size_t len, 
                                 size_t *bytes_read, uint32_t timeout_ms)
{
    if (self == NULL || buffer == NULL || len == 0 || bytes_read == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    usb_transport_t *transport = (usb_transport_t *)self;
    usb_internal_t *internal = (usb_internal_t *)transport->internal;
    
    if (!transport->is_open) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    if (timeout_ms == 0) {
        timeout_ms = USB_DEFAULT_TIMEOUT_MS;
    }
    
    size_t available = ringbuf_available(internal);
    if (available == 0) {
        if (xSemaphoreTake(internal->rx_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
            *bytes_read = 0;
            return TRANSPORT_ERR_TIMEOUT;
        }
    }
    
    *bytes_read = ringbuf_read(internal, buffer, len);
    
    return TRANSPORT_OK;
}

static transport_err_t usb_available(transport_t *self, size_t *available)
{
    if (self == NULL || available == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    usb_transport_t *transport = (usb_transport_t *)self;
    usb_internal_t *internal = (usb_internal_t *)transport->internal;
    
    if (!transport->is_open) {
        *available = 0;
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    *available = ringbuf_available(internal);
    
    return TRANSPORT_OK;
}

static transport_err_t usb_flush(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    usb_transport_t *transport = (usb_transport_t *)self;
    usb_internal_t *internal = (usb_internal_t *)transport->internal;
    
    if (!transport->is_open) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    internal->rx_head = 0;
    internal->rx_tail = 0;
    
    return TRANSPORT_OK;
}

static transport_state_t usb_get_state(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_STATE_UNINITIALIZED;
    }
    return self->state;
}

static transport_type_t usb_get_type(transport_t *self)
{
    (void)self;
    return TRANSPORT_TYPE_CUSTOM;
}

static transport_err_t usb_get_info(transport_t *self, char *info, size_t len)
{
    if (self == NULL || info == NULL || len == 0) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    usb_transport_t *transport = (usb_transport_t *)self;
    
    snprintf(info, len, "USB CDC - State: %s, DTR: %d, Baud: %lu",
             transport_state_to_str(self->state),
             transport->dtr,
             (unsigned long)transport->config.line_coding.baud_rate);
    
    return TRANSPORT_OK;
}

static transport_err_t usb_set_option(transport_t *self, int option, 
                                       const void *value, size_t len)
{
    (void)self;
    (void)option;
    (void)value;
    (void)len;
    
    return TRANSPORT_ERR_NOT_SUPPORTED;
}

static transport_err_t usb_get_option(transport_t *self, int option, 
                                       void *value, size_t *len)
{
    if (self == NULL || value == NULL || len == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    usb_transport_t *transport = (usb_transport_t *)self;
    
    switch ((usb_transport_option_t)option) {
        case USB_OPT_LINE_CODING:
            if (*len >= sizeof(usb_line_coding_t)) {
                memcpy(value, &transport->config.line_coding, sizeof(usb_line_coding_t));
                *len = sizeof(usb_line_coding_t);
            }
            break;
            
        case USB_OPT_DTR:
            if (*len >= sizeof(bool)) {
                *(bool *)value = transport->dtr;
                *len = sizeof(bool);
            }
            break;
            
        case USB_OPT_RTS:
            if (*len >= sizeof(bool)) {
                *(bool *)value = transport->rts;
                *len = sizeof(bool);
            }
            break;
            
        default:
            return TRANSPORT_ERR_NOT_SUPPORTED;
    }
    
    return TRANSPORT_OK;
}

usb_transport_t *usb_transport_create(void)
{
    usb_transport_t *transport = calloc(1, sizeof(usb_transport_t));
    if (transport == NULL) {
        return NULL;
    }
    
    transport->base.vtable = &usb_vtable;
    transport->base.state = TRANSPORT_STATE_UNINITIALIZED;
    transport->base.name = "USB CDC";
    
    usb_transport_config_default(&transport->config);
    
    return transport;
}

void usb_transport_destroy(usb_transport_t *transport)
{
    if (transport == NULL) {
        return;
    }
    
    if (transport->base.state != TRANSPORT_STATE_UNINITIALIZED) {
        usb_deinit(&transport->base);
    }
    
    free(transport);
}

void usb_transport_config_default(usb_transport_config_t *config)
{
    if (config == NULL) {
        return;
    }
    
    memset(config, 0, sizeof(usb_transport_config_t));
    config->base.name = "ESP32_USB";
    config->base.timeout_ms = USB_DEFAULT_TIMEOUT_MS;
    config->base.rx_buffer_size = USB_DEFAULT_BUFFER_SIZE;
    config->base.tx_buffer_size = USB_DEFAULT_BUFFER_SIZE;
    config->line_coding.baud_rate = 115200;
    config->line_coding.stop_bits = 0;
    config->line_coding.parity = 0;
    config->line_coding.data_bits = 8;
}

const transport_vtable_t *usb_transport_get_vtable(void)
{
    return &usb_vtable;
}

bool usb_transport_is_host_connected(usb_transport_t *transport)
{
    if (transport == NULL) {
        return false;
    }
    return transport->dtr;
}

transport_err_t usb_transport_get_line_coding(usb_transport_t *transport, 
                                               usb_line_coding_t *line_coding)
{
    if (transport == NULL || line_coding == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    memcpy(line_coding, &transport->config.line_coding, sizeof(usb_line_coding_t));
    
    return TRANSPORT_OK;
}

transport_err_t usb_transport_wait_for_host(usb_transport_t *transport, uint32_t timeout_ms)
{
    if (transport == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    if (transport->dtr) {
        return TRANSPORT_OK;
    }
    
    uint32_t elapsed = 0;
    const uint32_t poll_interval = 100;
    
    while (elapsed < timeout_ms || timeout_ms == 0) {
        if (transport->dtr) {
            return TRANSPORT_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(poll_interval));
        elapsed += poll_interval;
    }
    
    return TRANSPORT_ERR_TIMEOUT;
}
