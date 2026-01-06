/**
 * @file transport_uart.c
 * @brief UART transport backend implementation
 */

#include "transport_uart.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG = "uart_transport";

#define UART_DEFAULT_BAUD_RATE      (115200)
#define UART_DEFAULT_BUFFER_SIZE    (2048)
#define UART_DEFAULT_TIMEOUT_MS     (1000)
#define UART_DEFAULT_DATA_BITS      (8)
#define UART_DEFAULT_UART_NUM       (1)

typedef struct {
    SemaphoreHandle_t mutex;
    QueueHandle_t event_queue;
} uart_internal_t;

static transport_err_t uart_init(transport_t *self, const void *config);
static transport_err_t uart_deinit(transport_t *self);
static transport_err_t uart_connect(transport_t *self, const char *address, uint32_t timeout_ms);
static transport_err_t uart_disconnect(transport_t *self);
static transport_err_t uart_write(transport_t *self, const uint8_t *data, size_t len, 
                                   size_t *bytes_written, uint32_t timeout_ms);
static transport_err_t uart_read(transport_t *self, uint8_t *buffer, size_t len, 
                                  size_t *bytes_read, uint32_t timeout_ms);
static transport_err_t uart_available(transport_t *self, size_t *available);
static transport_err_t uart_flush(transport_t *self);
static transport_state_t uart_get_state(transport_t *self);
static transport_type_t uart_get_type(transport_t *self);
static transport_err_t uart_get_info(transport_t *self, char *info, size_t len);
static transport_err_t uart_set_option(transport_t *self, int option, 
                                        const void *value, size_t len);
static transport_err_t uart_get_option(transport_t *self, int option, 
                                        void *value, size_t *len);

static const transport_vtable_t uart_vtable = {
    .init       = uart_init,
    .deinit     = uart_deinit,
    .connect    = uart_connect,
    .disconnect = uart_disconnect,
    .write      = uart_write,
    .read       = uart_read,
    .available  = uart_available,
    .flush      = uart_flush,
    .get_state  = uart_get_state,
    .get_type   = uart_get_type,
    .get_info   = uart_get_info,
    .set_option = uart_set_option,
    .get_option = uart_get_option,
};

static uart_parity_t convert_parity(uart_transport_parity_t parity)
{
    switch (parity) {
        case UART_TRANSPORT_PARITY_ODD:
            return UART_PARITY_ODD;
        case UART_TRANSPORT_PARITY_EVEN:
            return UART_PARITY_EVEN;
        default:
            return UART_PARITY_DISABLE;
    }
}

static uart_stop_bits_t convert_stop_bits(uart_transport_stop_bits_t stop_bits)
{
    switch (stop_bits) {
        case UART_TRANSPORT_STOP_BITS_1_5:
            return UART_STOP_BITS_1_5;
        case UART_TRANSPORT_STOP_BITS_2:
            return UART_STOP_BITS_2;
        default:
            return UART_STOP_BITS_1;
    }
}

static uart_hw_flowcontrol_t convert_flow_ctrl(uart_transport_flow_ctrl_t flow_ctrl)
{
    switch (flow_ctrl) {
        case UART_TRANSPORT_FLOW_CTRL_RTS:
            return UART_HW_FLOWCTRL_RTS;
        case UART_TRANSPORT_FLOW_CTRL_CTS:
            return UART_HW_FLOWCTRL_CTS;
        case UART_TRANSPORT_FLOW_CTRL_RTS_CTS:
            return UART_HW_FLOWCTRL_CTS_RTS;
        default:
            return UART_HW_FLOWCTRL_DISABLE;
    }
}

static transport_err_t uart_init(transport_t *self, const void *config)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    uart_transport_t *transport = (uart_transport_t *)self;
    
    if (self->state != TRANSPORT_STATE_UNINITIALIZED) {
        return TRANSPORT_ERR_ALREADY_INITIALIZED;
    }
    
    if (config != NULL) {
        memcpy(&transport->config, config, sizeof(uart_transport_config_t));
    }
    
    uart_internal_t *internal = calloc(1, sizeof(uart_internal_t));
    if (internal == NULL) {
        return TRANSPORT_ERR_NO_MEM;
    }
    
    internal->mutex = xSemaphoreCreateMutex();
    if (internal->mutex == NULL) {
        free(internal);
        return TRANSPORT_ERR_NO_MEM;
    }
    
    transport->internal = internal;
    
    transport_set_state(self, TRANSPORT_STATE_INITIALIZED);
    
    ESP_LOGI(TAG, "UART transport initialized (UART%d)", transport->config.uart_num);
    
    return TRANSPORT_OK;
}

static transport_err_t uart_deinit(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    uart_transport_t *transport = (uart_transport_t *)self;
    uart_internal_t *internal = (uart_internal_t *)transport->internal;
    
    if (self->state == TRANSPORT_STATE_UNINITIALIZED) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (transport->is_open) {
        uart_disconnect(self);
    }
    
    if (internal->mutex) vSemaphoreDelete(internal->mutex);
    if (internal->event_queue) vQueueDelete(internal->event_queue);
    
    free(internal);
    transport->internal = NULL;
    
    transport_set_state(self, TRANSPORT_STATE_UNINITIALIZED);
    
    ESP_LOGI(TAG, "UART transport deinitialized");
    
    return TRANSPORT_OK;
}

static transport_err_t uart_connect(transport_t *self, const char *address, uint32_t timeout_ms)
{
    (void)address;
    (void)timeout_ms;
    
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    uart_transport_t *transport = (uart_transport_t *)self;
    uart_internal_t *internal = (uart_internal_t *)transport->internal;
    
    if (self->state == TRANSPORT_STATE_UNINITIALIZED) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (transport->is_open) {
        return TRANSPORT_ERR_ALREADY_CONNECTED;
    }
    
    transport_set_state(self, TRANSPORT_STATE_CONNECTING);
    
    uart_config_t uart_config = {
        .baud_rate = transport->config.baud_rate,
        .data_bits = transport->config.data_bits - 5,
        .parity = convert_parity(transport->config.parity),
        .stop_bits = convert_stop_bits(transport->config.stop_bits),
        .flow_ctrl = convert_flow_ctrl(transport->config.flow_ctrl),
        .rx_flow_ctrl_thresh = transport->config.rx_flow_ctrl_thresh,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t err = uart_param_config(transport->config.uart_num, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(err));
        transport_set_state(self, TRANSPORT_STATE_INITIALIZED);
        return TRANSPORT_ERR_IO;
    }
    
    err = uart_set_pin(transport->config.uart_num,
                       transport->config.tx_pin,
                       transport->config.rx_pin,
                       transport->config.rts_pin,
                       transport->config.cts_pin);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(err));
        transport_set_state(self, TRANSPORT_STATE_INITIALIZED);
        return TRANSPORT_ERR_IO;
    }
    
    size_t rx_buffer_size = transport->config.base.rx_buffer_size;
    size_t tx_buffer_size = transport->config.base.tx_buffer_size;
    if (rx_buffer_size == 0) rx_buffer_size = UART_DEFAULT_BUFFER_SIZE;
    if (tx_buffer_size == 0) tx_buffer_size = UART_DEFAULT_BUFFER_SIZE;
    
    err = uart_driver_install(transport->config.uart_num,
                              rx_buffer_size,
                              tx_buffer_size,
                              20,
                              &internal->event_queue,
                              0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        transport_set_state(self, TRANSPORT_STATE_INITIALIZED);
        return TRANSPORT_ERR_IO;
    }
    
    transport->is_open = true;
    transport_set_state(self, TRANSPORT_STATE_CONNECTED);
    
    transport_event_t evt = { .type = TRANSPORT_EVENT_CONNECTED };
    transport_emit_event(self, &evt);
    
    ESP_LOGI(TAG, "UART%d opened at %d baud", 
             transport->config.uart_num, transport->config.baud_rate);
    
    return TRANSPORT_OK;
}

static transport_err_t uart_disconnect(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    uart_transport_t *transport = (uart_transport_t *)self;
    
    if (!transport->is_open) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    transport_set_state(self, TRANSPORT_STATE_DISCONNECTING);
    
    uart_wait_tx_done(transport->config.uart_num, pdMS_TO_TICKS(1000));
    uart_driver_delete(transport->config.uart_num);
    
    transport->is_open = false;
    transport_set_state(self, TRANSPORT_STATE_INITIALIZED);
    
    transport_event_t evt = { .type = TRANSPORT_EVENT_DISCONNECTED };
    transport_emit_event(self, &evt);
    
    ESP_LOGI(TAG, "UART%d closed", transport->config.uart_num);
    
    return TRANSPORT_OK;
}

static transport_err_t uart_write(transport_t *self, const uint8_t *data, size_t len, 
                                   size_t *bytes_written, uint32_t timeout_ms)
{
    if (self == NULL || data == NULL || len == 0) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    uart_transport_t *transport = (uart_transport_t *)self;
    uart_internal_t *internal = (uart_internal_t *)transport->internal;
    
    if (!transport->is_open) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    if (timeout_ms == 0) {
        timeout_ms = UART_DEFAULT_TIMEOUT_MS;
    }
    
    if (xSemaphoreTake(internal->mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return TRANSPORT_ERR_TIMEOUT;
    }
    
    int written = uart_write_bytes(transport->config.uart_num, data, len);
    
    xSemaphoreGive(internal->mutex);
    
    if (written < 0) {
        ESP_LOGE(TAG, "UART write failed");
        return TRANSPORT_ERR_IO;
    }
    
    if (bytes_written != NULL) {
        *bytes_written = (size_t)written;
    }
    
    return TRANSPORT_OK;
}

static transport_err_t uart_read(transport_t *self, uint8_t *buffer, size_t len, 
                                  size_t *bytes_read, uint32_t timeout_ms)
{
    if (self == NULL || buffer == NULL || len == 0 || bytes_read == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    uart_transport_t *transport = (uart_transport_t *)self;
    
    if (!transport->is_open) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    if (timeout_ms == 0) {
        timeout_ms = UART_DEFAULT_TIMEOUT_MS;
    }
    
    int read_len = uart_read_bytes(transport->config.uart_num, buffer, len, 
                                    pdMS_TO_TICKS(timeout_ms));
    
    if (read_len < 0) {
        ESP_LOGE(TAG, "UART read failed");
        return TRANSPORT_ERR_IO;
    }
    
    *bytes_read = (size_t)read_len;
    
    if (read_len == 0) {
        return TRANSPORT_ERR_TIMEOUT;
    }
    
    return TRANSPORT_OK;
}

static transport_err_t uart_available(transport_t *self, size_t *available)
{
    if (self == NULL || available == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    uart_transport_t *transport = (uart_transport_t *)self;
    
    if (!transport->is_open) {
        *available = 0;
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    size_t buffered_size = 0;
    esp_err_t err = uart_get_buffered_data_len(transport->config.uart_num, &buffered_size);
    if (err != ESP_OK) {
        *available = 0;
        return TRANSPORT_ERR_IO;
    }
    
    *available = buffered_size;
    
    return TRANSPORT_OK;
}

static transport_err_t uart_flush(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    uart_transport_t *transport = (uart_transport_t *)self;
    
    if (!transport->is_open) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    esp_err_t err = uart_flush_input(transport->config.uart_num);
    if (err != ESP_OK) {
        return TRANSPORT_ERR_IO;
    }
    
    return TRANSPORT_OK;
}

static transport_state_t uart_get_state(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_STATE_UNINITIALIZED;
    }
    return self->state;
}

static transport_type_t uart_get_type(transport_t *self)
{
    (void)self;
    return TRANSPORT_TYPE_UART;
}

static transport_err_t uart_get_info(transport_t *self, char *info, size_t len)
{
    if (self == NULL || info == NULL || len == 0) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    uart_transport_t *transport = (uart_transport_t *)self;
    
    snprintf(info, len, "UART%d - State: %s, Baud: %d",
             transport->config.uart_num,
             transport_state_to_str(self->state),
             transport->config.baud_rate);
    
    return TRANSPORT_OK;
}

static transport_err_t uart_set_option(transport_t *self, int option, 
                                        const void *value, size_t len)
{
    if (self == NULL || value == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    uart_transport_t *transport = (uart_transport_t *)self;
    
    switch ((uart_transport_option_t)option) {
        case UART_OPT_BAUD_RATE:
            if (len >= sizeof(int)) {
                int baud_rate = *(const int *)value;
                transport->config.baud_rate = baud_rate;
                if (transport->is_open) {
                    uart_set_baudrate(transport->config.uart_num, baud_rate);
                }
            }
            break;
            
        default:
            return TRANSPORT_ERR_NOT_SUPPORTED;
    }
    
    return TRANSPORT_OK;
}

static transport_err_t uart_get_option(transport_t *self, int option, 
                                        void *value, size_t *len)
{
    if (self == NULL || value == NULL || len == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    uart_transport_t *transport = (uart_transport_t *)self;
    
    switch ((uart_transport_option_t)option) {
        case UART_OPT_BAUD_RATE:
            if (*len >= sizeof(int)) {
                if (transport->is_open) {
                    uint32_t baud_rate;
                    uart_get_baudrate(transport->config.uart_num, &baud_rate);
                    *(int *)value = (int)baud_rate;
                } else {
                    *(int *)value = transport->config.baud_rate;
                }
                *len = sizeof(int);
            }
            break;
            
        default:
            return TRANSPORT_ERR_NOT_SUPPORTED;
    }
    
    return TRANSPORT_OK;
}

uart_transport_t *uart_transport_create(void)
{
    uart_transport_t *transport = calloc(1, sizeof(uart_transport_t));
    if (transport == NULL) {
        return NULL;
    }
    
    transport->base.vtable = &uart_vtable;
    transport->base.state = TRANSPORT_STATE_UNINITIALIZED;
    transport->base.name = "UART";
    
    uart_transport_config_default(&transport->config);
    
    return transport;
}

void uart_transport_destroy(uart_transport_t *transport)
{
    if (transport == NULL) {
        return;
    }
    
    if (transport->base.state != TRANSPORT_STATE_UNINITIALIZED) {
        uart_deinit(&transport->base);
    }
    
    free(transport);
}

void uart_transport_config_default(uart_transport_config_t *config)
{
    if (config == NULL) {
        return;
    }
    
    memset(config, 0, sizeof(uart_transport_config_t));
    config->base.name = "ESP32_UART";
    config->base.timeout_ms = UART_DEFAULT_TIMEOUT_MS;
    config->base.rx_buffer_size = UART_DEFAULT_BUFFER_SIZE;
    config->base.tx_buffer_size = UART_DEFAULT_BUFFER_SIZE;
    config->uart_num = UART_DEFAULT_UART_NUM;
    config->baud_rate = UART_DEFAULT_BAUD_RATE;
    config->data_bits = UART_DEFAULT_DATA_BITS;
    config->parity = UART_TRANSPORT_PARITY_NONE;
    config->stop_bits = UART_TRANSPORT_STOP_BITS_1;
    config->flow_ctrl = UART_TRANSPORT_FLOW_CTRL_NONE;
    config->rx_flow_ctrl_thresh = 122;
    config->tx_pin = UART_PIN_NO_CHANGE;
    config->rx_pin = UART_PIN_NO_CHANGE;
    config->rts_pin = UART_PIN_NO_CHANGE;
    config->cts_pin = UART_PIN_NO_CHANGE;
}

const transport_vtable_t *uart_transport_get_vtable(void)
{
    return &uart_vtable;
}

transport_err_t uart_transport_set_baudrate(uart_transport_t *transport, int baud_rate)
{
    if (transport == NULL || baud_rate <= 0) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    transport->config.baud_rate = baud_rate;
    
    if (transport->is_open) {
        esp_err_t err = uart_set_baudrate(transport->config.uart_num, baud_rate);
        if (err != ESP_OK) {
            return TRANSPORT_ERR_IO;
        }
    }
    
    return TRANSPORT_OK;
}

int uart_transport_get_baudrate(uart_transport_t *transport)
{
    if (transport == NULL) {
        return -1;
    }
    
    if (transport->is_open) {
        uint32_t baud_rate;
        if (uart_get_baudrate(transport->config.uart_num, &baud_rate) == ESP_OK) {
            return (int)baud_rate;
        }
        return -1;
    }
    
    return transport->config.baud_rate;
}

transport_err_t uart_transport_send_break(uart_transport_t *transport, int duration_ms)
{
    if (transport == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    if (!transport->is_open) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    int break_cycles = (transport->config.baud_rate * duration_ms) / 1000;
    if (break_cycles < 13) break_cycles = 13;
    
    esp_err_t err = uart_set_line_inverse(transport->config.uart_num, UART_SIGNAL_TXD_INV);
    if (err != ESP_OK) {
        return TRANSPORT_ERR_IO;
    }
    
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    
    uart_set_line_inverse(transport->config.uart_num, 0);
    
    return TRANSPORT_OK;
}

transport_err_t uart_transport_wait_tx_done(uart_transport_t *transport, uint32_t timeout_ms)
{
    if (transport == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    if (!transport->is_open) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    esp_err_t err = uart_wait_tx_done(transport->config.uart_num, pdMS_TO_TICKS(timeout_ms));
    if (err == ESP_ERR_TIMEOUT) {
        return TRANSPORT_ERR_TIMEOUT;
    }
    if (err != ESP_OK) {
        return TRANSPORT_ERR_IO;
    }
    
    return TRANSPORT_OK;
}
