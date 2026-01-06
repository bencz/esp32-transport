/**
 * @file transport_manager.h
 * @brief Centralized transport manager
 * 
 * This module provides a management layer for multiple transports,
 * allowing easy switching between backends and lifecycle management.
 */

#ifndef TRANSPORT_MANAGER_H
#define TRANSPORT_MANAGER_H

#include "transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of registered transports
 */
#define TRANSPORT_MANAGER_MAX_TRANSPORTS    (8)

/**
 * @brief Handle for a registered transport
 */
typedef int transport_handle_t;

#define TRANSPORT_HANDLE_INVALID    (-1)

/**
 * @brief Callback for automatic transport selection
 */
typedef transport_t *(*transport_selector_cb_t)(void *user_data);

/**
 * @brief Transport manager configuration
 */
typedef struct {
    bool auto_reconnect;
    uint32_t reconnect_delay_ms;
    uint32_t max_reconnect_attempts;
    transport_selector_cb_t selector;
    void *selector_user_data;
} transport_manager_config_t;

/**
 * @brief Initializes the transport manager
 * 
 * @param config Manager configuration (NULL uses defaults)
 * @return TRANSPORT_OK on success
 */
transport_err_t transport_manager_init(const transport_manager_config_t *config);

/**
 * @brief Deinitializes the transport manager
 * 
 * @return TRANSPORT_OK on success
 */
transport_err_t transport_manager_deinit(void);

/**
 * @brief Registers a transport with the manager
 * 
 * @param transport Pointer to the transport
 * @param name Unique name for identification
 * @return Transport handle, or TRANSPORT_HANDLE_INVALID on error
 */
transport_handle_t transport_manager_register(transport_t *transport, const char *name);

/**
 * @brief Removes a transport from the manager
 * 
 * @param handle Transport handle
 * @return TRANSPORT_OK on success
 */
transport_err_t transport_manager_unregister(transport_handle_t handle);

/**
 * @brief Gets a transport by handle
 * 
 * @param handle Transport handle
 * @return Pointer to the transport, or NULL if not found
 */
transport_t *transport_manager_get(transport_handle_t handle);

/**
 * @brief Gets a transport by name
 * 
 * @param name Transport name
 * @return Pointer to the transport, or NULL if not found
 */
transport_t *transport_manager_get_by_name(const char *name);

/**
 * @brief Gets a transport by type
 * 
 * @param type Transport type
 * @return Pointer to the first transport of the type, or NULL if not found
 */
transport_t *transport_manager_get_by_type(transport_type_t type);

/**
 * @brief Sets the active transport
 * 
 * @param handle Transport handle
 * @return TRANSPORT_OK on success
 */
transport_err_t transport_manager_set_active(transport_handle_t handle);

/**
 * @brief Gets the active transport
 * 
 * @return Pointer to the active transport, or NULL if none
 */
transport_t *transport_manager_get_active(void);

/**
 * @brief Connects the active transport
 * 
 * @param address Destination address
 * @param timeout_ms Timeout in milliseconds
 * @return TRANSPORT_OK on success
 */
transport_err_t transport_manager_connect(const char *address, uint32_t timeout_ms);

/**
 * @brief Disconnects the active transport
 * 
 * @return TRANSPORT_OK on success
 */
transport_err_t transport_manager_disconnect(void);

/**
 * @brief Sends data through the active transport
 * 
 * @param data Data to send
 * @param len Data size
 * @param bytes_written Pointer to bytes written (can be NULL)
 * @param timeout_ms Timeout in milliseconds
 * @return TRANSPORT_OK on success
 */
transport_err_t transport_manager_write(const uint8_t *data, size_t len, 
                                         size_t *bytes_written, uint32_t timeout_ms);

/**
 * @brief Receives data from the active transport
 * 
 * @param buffer Buffer for data
 * @param len Maximum size
 * @param bytes_read Pointer to bytes read
 * @param timeout_ms Timeout in milliseconds
 * @return TRANSPORT_OK on success
 */
transport_err_t transport_manager_read(uint8_t *buffer, size_t len, 
                                        size_t *bytes_read, uint32_t timeout_ms);

/**
 * @brief Lists all registered transports
 * 
 * @param handles Array to store handles
 * @param max_handles Maximum array size
 * @return Number of transports found
 */
size_t transport_manager_list(transport_handle_t *handles, size_t max_handles);

/**
 * @brief Gets transport statistics
 */
typedef struct {
    uint64_t bytes_sent;
    uint64_t bytes_received;
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t errors;
    uint32_t reconnects;
    uint32_t uptime_ms;
} transport_stats_t;

/**
 * @brief Gets transport statistics
 * 
 * @param handle Transport handle
 * @param stats Pointer to statistics structure
 * @return TRANSPORT_OK on success
 */
transport_err_t transport_manager_get_stats(transport_handle_t handle, 
                                             transport_stats_t *stats);

/**
 * @brief Resets transport statistics
 * 
 * @param handle Transport handle
 * @return TRANSPORT_OK on success
 */
transport_err_t transport_manager_reset_stats(transport_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* TRANSPORT_MANAGER_H */