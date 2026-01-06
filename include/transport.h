/**
 * @file transport.h
 * @brief Transport abstraction system for ESP32
 * 
 * This module provides a unified interface for different communication
 * backends (Bluetooth Classic, BLE, WiFi, UART, etc.) using vtables
 * for polymorphism in C.
 * 
 * @author Claude
 * @version 1.0.0
 */

#ifndef TRANSPORT_H
#define TRANSPORT_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Transport system specific error codes
 */
typedef enum {
    TRANSPORT_OK = 0,
    TRANSPORT_ERR_INVALID_ARG,
    TRANSPORT_ERR_NO_MEM,
    TRANSPORT_ERR_NOT_INITIALIZED,
    TRANSPORT_ERR_ALREADY_INITIALIZED,
    TRANSPORT_ERR_NOT_CONNECTED,
    TRANSPORT_ERR_ALREADY_CONNECTED,
    TRANSPORT_ERR_TIMEOUT,
    TRANSPORT_ERR_IO,
    TRANSPORT_ERR_NOT_SUPPORTED,
    TRANSPORT_ERR_BUSY,
    TRANSPORT_ERR_INTERNAL,
} transport_err_t;

/**
 * @brief Possible transport states
 */
typedef enum {
    TRANSPORT_STATE_UNINITIALIZED = 0,
    TRANSPORT_STATE_INITIALIZED,
    TRANSPORT_STATE_CONNECTING,
    TRANSPORT_STATE_CONNECTED,
    TRANSPORT_STATE_DISCONNECTING,
    TRANSPORT_STATE_ERROR,
} transport_state_t;

/**
 * @brief Available transport types
 */
typedef enum {
    TRANSPORT_TYPE_UNKNOWN = 0,
    TRANSPORT_TYPE_BT_CLASSIC,
    TRANSPORT_TYPE_BLE,
    TRANSPORT_TYPE_WIFI_TCP,
    TRANSPORT_TYPE_WIFI_UDP,
    TRANSPORT_TYPE_UART,
    TRANSPORT_TYPE_SPI,
    TRANSPORT_TYPE_I2C,
    TRANSPORT_TYPE_CUSTOM,
} transport_type_t;

/**
 * @brief Events that can be emitted by the transport
 */
typedef enum {
    TRANSPORT_EVENT_CONNECTED = 0,
    TRANSPORT_EVENT_DISCONNECTED,
    TRANSPORT_EVENT_DATA_RECEIVED,
    TRANSPORT_EVENT_DATA_SENT,
    TRANSPORT_EVENT_ERROR,
    TRANSPORT_EVENT_STATE_CHANGED,
} transport_event_type_t;

/**
 * @brief Forward declarations
 */
typedef struct transport_s transport_t;
typedef struct transport_vtable_s transport_vtable_t;
typedef struct transport_config_s transport_config_t;
typedef struct transport_event_s transport_event_t;

/**
 * @brief Callback for transport events
 * 
 * @param transport Pointer to the transport that generated the event
 * @param event Pointer to the event data
 * @param user_data User data passed during callback registration
 */
typedef void (*transport_event_cb_t)(transport_t *transport, 
                                      const transport_event_t *event, 
                                      void *user_data);

/**
 * @brief Transport event structure
 */
struct transport_event_s {
    transport_event_type_t type;
    union {
        struct {
            const uint8_t *data;
            size_t len;
        } data_received;
        struct {
            size_t bytes_sent;
        } data_sent;
        struct {
            transport_err_t code;
            const char *message;
        } error;
        struct {
            transport_state_t old_state;
            transport_state_t new_state;
        } state_changed;
    };
};

/**
 * @brief Base configuration for transports
 */
struct transport_config_s {
    const char *name;
    uint32_t timeout_ms;
    size_t rx_buffer_size;
    size_t tx_buffer_size;
    transport_event_cb_t event_callback;
    void *user_data;
};

/**
 * @brief Vtable with transport operations
 * 
 * This structure defines the interface that all transport backends
 * must implement. Works like a vtable in C++.
 */
struct transport_vtable_s {
    /**
     * @brief Initializes the transport with specific configuration
     * @param self Pointer to the transport instance
     * @param config Backend-specific configuration
     * @return TRANSPORT_OK on success
     */
    transport_err_t (*init)(transport_t *self, const void *config);
    
    /**
     * @brief Deinitializes the transport and releases resources
     * @param self Pointer to the transport instance
     * @return TRANSPORT_OK on success
     */
    transport_err_t (*deinit)(transport_t *self);
    
    /**
     * @brief Establishes connection
     * @param self Pointer to the transport instance
     * @param address Destination address (format depends on backend)
     * @param timeout_ms Timeout in milliseconds (0 = use default)
     * @return TRANSPORT_OK on success
     */
    transport_err_t (*connect)(transport_t *self, const char *address, uint32_t timeout_ms);
    
    /**
     * @brief Terminates connection
     * @param self Pointer to the transport instance
     * @return TRANSPORT_OK on success
     */
    transport_err_t (*disconnect)(transport_t *self);
    
    /**
     * @brief Sends data
     * @param self Pointer to the transport instance
     * @param data Pointer to the data to send
     * @param len Size of data in bytes
     * @param bytes_written Pointer to store bytes written (can be NULL)
     * @param timeout_ms Timeout in milliseconds (0 = use default)
     * @return TRANSPORT_OK on success
     */
    transport_err_t (*write)(transport_t *self, const uint8_t *data, size_t len, 
                             size_t *bytes_written, uint32_t timeout_ms);
    
    /**
     * @brief Receives data
     * @param self Pointer to the transport instance
     * @param buffer Buffer to store received data
     * @param len Maximum size to receive
     * @param bytes_read Pointer to store bytes read
     * @param timeout_ms Timeout in milliseconds (0 = use default)
     * @return TRANSPORT_OK on success
     */
    transport_err_t (*read)(transport_t *self, uint8_t *buffer, size_t len, 
                            size_t *bytes_read, uint32_t timeout_ms);
    
    /**
     * @brief Checks if data is available for reading
     * @param self Pointer to the transport instance
     * @param available Pointer to store the number of available bytes
     * @return TRANSPORT_OK on success
     */
    transport_err_t (*available)(transport_t *self, size_t *available);
    
    /**
     * @brief Clears receive and transmit buffers
     * @param self Pointer to the transport instance
     * @return TRANSPORT_OK on success
     */
    transport_err_t (*flush)(transport_t *self);
    
    /**
     * @brief Gets the current transport state
     * @param self Pointer to the transport instance
     * @return Current state
     */
    transport_state_t (*get_state)(transport_t *self);
    
    /**
     * @brief Gets the transport type
     * @param self Pointer to the transport instance
     * @return Transport type
     */
    transport_type_t (*get_type)(transport_t *self);
    
    /**
     * @brief Gets information about the transport
     * @param self Pointer to the transport instance
     * @param info Buffer to store information
     * @param len Buffer size
     * @return TRANSPORT_OK on success
     */
    transport_err_t (*get_info)(transport_t *self, char *info, size_t len);
    
    /**
     * @brief Sets a transport-specific option
     * @param self Pointer to the transport instance
     * @param option Option identifier
     * @param value Pointer to the option value
     * @param len Value size
     * @return TRANSPORT_OK on success
     */
    transport_err_t (*set_option)(transport_t *self, int option, 
                                  const void *value, size_t len);
    
    /**
     * @brief Gets a transport-specific option
     * @param self Pointer to the transport instance
     * @param option Option identifier
     * @param value Buffer to store the value
     * @param len Pointer to size (input: buffer size, output: used size)
     * @return TRANSPORT_OK on success
     */
    transport_err_t (*get_option)(transport_t *self, int option, 
                                  void *value, size_t *len);
};

/**
 * @brief Base transport structure
 * 
 * All specific backends must "inherit" from this structure
 * by placing it as the first member.
 */
struct transport_s {
    const transport_vtable_t *vtable;
    transport_state_t state;
    transport_event_cb_t event_callback;
    void *user_data;
    const char *name;
    SemaphoreHandle_t mutex;
};

/**
 * @defgroup transport_api High-Level API
 * @brief Convenience functions that delegate to the vtable
 * @{
 */

/**
 * @brief Initializes a transport
 * @param transport Pointer to the transport instance
 * @param config Backend-specific configuration
 * @return TRANSPORT_OK on success
 */
static inline transport_err_t transport_init(transport_t *transport, const void *config)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->init == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->init(transport, config);
}

/**
 * @brief Deinitializes a transport
 */
static inline transport_err_t transport_deinit(transport_t *transport)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->deinit == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->deinit(transport);
}

/**
 * @brief Connects the transport
 */
static inline transport_err_t transport_connect(transport_t *transport, 
                                                 const char *address, 
                                                 uint32_t timeout_ms)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->connect == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->connect(transport, address, timeout_ms);
}

/**
 * @brief Disconnects the transport
 */
static inline transport_err_t transport_disconnect(transport_t *transport)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->disconnect == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->disconnect(transport);
}

/**
 * @brief Sends data through the transport
 * @param transport Pointer to the transport instance
 * @param data Pointer to the data to send
 * @param len Size of data in bytes
 * @param bytes_written Pointer to store bytes written (can be NULL)
 * @param timeout_ms Timeout in milliseconds (0 = use default)
 * @return TRANSPORT_OK on success
 */
static inline transport_err_t transport_write(transport_t *transport, 
                                               const uint8_t *data, 
                                               size_t len,
                                               size_t *bytes_written, 
                                               uint32_t timeout_ms)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->write == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->write(transport, data, len, bytes_written, timeout_ms);
}

/**
 * @brief Receives data from the transport
 */
static inline transport_err_t transport_read(transport_t *transport, 
                                              uint8_t *buffer, 
                                              size_t len,
                                              size_t *bytes_read, 
                                              uint32_t timeout_ms)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->read == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->read(transport, buffer, len, bytes_read, timeout_ms);
}

/**
 * @brief Checks available data
 */
static inline transport_err_t transport_available(transport_t *transport, size_t *available)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->available == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->available(transport, available);
}

/**
 * @brief Clears transport buffers
 */
static inline transport_err_t transport_flush(transport_t *transport)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->flush == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->flush(transport);
}

/**
 * @brief Gets transport state
 */
static inline transport_state_t transport_get_state(transport_t *transport)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->get_state == NULL) {
        return TRANSPORT_STATE_UNINITIALIZED;
    }
    return transport->vtable->get_state(transport);
}

/**
 * @brief Gets transport type
 */
static inline transport_type_t transport_get_type(transport_t *transport)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->get_type == NULL) {
        return TRANSPORT_TYPE_UNKNOWN;
    }
    return transport->vtable->get_type(transport);
}

/**
 * @brief Gets transport information
 */
static inline transport_err_t transport_get_info(transport_t *transport, char *info, size_t len)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->get_info == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->get_info(transport, info, len);
}

/**
 * @brief Sets transport option
 */
static inline transport_err_t transport_set_option(transport_t *transport, 
                                                    int option,
                                                    const void *value, 
                                                    size_t len)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->set_option == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->set_option(transport, option, value, len);
}

/**
 * @brief Gets transport option
 */
static inline transport_err_t transport_get_option(transport_t *transport, 
                                                    int option,
                                                    void *value, 
                                                    size_t *len)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->get_option == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->get_option(transport, option, value, len);
}

/** @} */

/**
 * @brief Helper macro to set default timeout if zero
 */
#define SET_DEFAULT_TIMEOUT(timeout, default_val) ((timeout) == 0 ? (default_val) : (timeout))

/**
 * @brief Converts error code to string
 * @param err Error code
 * @return Descriptive error string
 */
const char *transport_err_to_str(transport_err_t err);

/**
 * @brief Converts state to string
 * @param state Transport state
 * @return Descriptive state string
 */
const char *transport_state_to_str(transport_state_t state);

/**
 * @brief Converts type to string
 * @param type Transport type
 * @return Descriptive type string
 */
const char *transport_type_to_str(transport_type_t type);

/**
 * @brief Emits an event to the registered callback
 * @param transport Pointer to the transport
 * @param event Pointer to the event
 */
void transport_emit_event(transport_t *transport, const transport_event_t *event);

/**
 * @brief Changes the transport state and emits event
 * @param transport Pointer to the transport
 * @param new_state New state
 */
void transport_set_state(transport_t *transport, transport_state_t new_state);

#ifdef __cplusplus
}
#endif

#endif /* TRANSPORT_H */