/**
 * @file transport.c
 * @brief Implementação das funções utilitárias do sistema de transporte
 */

#include "transport.h"
#include <string.h>

/**
 * @brief Tabela de strings para códigos de erro
 */
static const char *const err_strings[] = {
    [TRANSPORT_OK]                    = "OK",
    [TRANSPORT_ERR_INVALID_ARG]       = "Invalid argument",
    [TRANSPORT_ERR_NO_MEM]            = "Out of memory",
    [TRANSPORT_ERR_NOT_INITIALIZED]   = "Not initialized",
    [TRANSPORT_ERR_ALREADY_INITIALIZED] = "Already initialized",
    [TRANSPORT_ERR_NOT_CONNECTED]     = "Not connected",
    [TRANSPORT_ERR_ALREADY_CONNECTED] = "Already connected",
    [TRANSPORT_ERR_TIMEOUT]           = "Timeout",
    [TRANSPORT_ERR_IO]                = "I/O error",
    [TRANSPORT_ERR_NOT_SUPPORTED]     = "Not supported",
    [TRANSPORT_ERR_BUSY]              = "Busy",
    [TRANSPORT_ERR_INTERNAL]          = "Internal error",
};

/**
 * @brief Tabela de strings para estados
 */
static const char *const state_strings[] = {
    [TRANSPORT_STATE_UNINITIALIZED] = "Uninitialized",
    [TRANSPORT_STATE_INITIALIZED]   = "Initialized",
    [TRANSPORT_STATE_CONNECTING]    = "Connecting",
    [TRANSPORT_STATE_CONNECTED]     = "Connected",
    [TRANSPORT_STATE_DISCONNECTING] = "Disconnecting",
    [TRANSPORT_STATE_ERROR]         = "Error",
};

/**
 * @brief Tabela de strings para tipos
 */
static const char *const type_strings[] = {
    [TRANSPORT_TYPE_UNKNOWN]    = "Unknown",
    [TRANSPORT_TYPE_BT_CLASSIC] = "Bluetooth Classic",
    [TRANSPORT_TYPE_BLE]        = "Bluetooth Low Energy",
    [TRANSPORT_TYPE_WIFI_TCP]   = "WiFi TCP",
    [TRANSPORT_TYPE_WIFI_UDP]   = "WiFi UDP",
    [TRANSPORT_TYPE_UART]       = "UART",
    [TRANSPORT_TYPE_SPI]        = "SPI",
    [TRANSPORT_TYPE_I2C]        = "I2C",
    [TRANSPORT_TYPE_CUSTOM]     = "Custom",
};

const char *transport_err_to_str(transport_err_t err)
{
    if (err < 0 || err >= sizeof(err_strings) / sizeof(err_strings[0])) {
        return "Unknown error";
    }
    return err_strings[err];
}

const char *transport_state_to_str(transport_state_t state)
{
    if (state < 0 || state >= sizeof(state_strings) / sizeof(state_strings[0])) {
        return "Unknown state";
    }
    return state_strings[state];
}

const char *transport_type_to_str(transport_type_t type)
{
    if (type < 0 || type >= sizeof(type_strings) / sizeof(type_strings[0])) {
        return "Unknown type";
    }
    return type_strings[type];
}

void transport_emit_event(transport_t *transport, const transport_event_t *event)
{
    if (transport == NULL || event == NULL) {
        return;
    }
    
    if (transport->event_callback != NULL) {
        transport->event_callback(transport, event, transport->user_data);
    }
}

void transport_set_state(transport_t *transport, transport_state_t new_state)
{
    if (transport == NULL) {
        return;
    }
    
    transport_state_t old_state = transport->state;
    
    if (old_state == new_state) {
        return;
    }
    
    transport->state = new_state;
    
    transport_event_t event = {
        .type = TRANSPORT_EVENT_STATE_CHANGED,
        .state_changed = {
            .old_state = old_state,
            .new_state = new_state,
        },
    };
    
    transport_emit_event(transport, &event);
}