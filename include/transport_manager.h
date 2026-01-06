/**
 * @file transport_manager.h
 * @brief Centralized transport manager
 * 
 * This module provides a management layer for multiple transports,
 * supporting simultaneous usage of all registered transports.
 * 
 * Features:
 * - Register multiple transports and use them concurrently
 * - Direct read/write to specific transports by handle
 * - Broadcast write to all connected transports
 * - Poll for data across all transports
 * - Optional "active" transport for simplified single-transport usage
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

/* ============================================================================
 * Multi-Transport Simultaneous Operations
 * ============================================================================
 * These functions allow using multiple transports at the same time.
 * Use these when you need to receive from one transport and send to another,
 * or broadcast data to all connected transports.
 */

/**
 * @brief Connects a specific transport by handle
 * 
 * @param handle Transport handle
 * @param address Destination address (format depends on transport)
 * @param timeout_ms Timeout in milliseconds
 * @return TRANSPORT_OK on success
 */
transport_err_t transport_manager_connect_handle(transport_handle_t handle,
                                                  const char *address,
                                                  uint32_t timeout_ms);

/**
 * @brief Disconnects a specific transport by handle
 * 
 * @param handle Transport handle
 * @return TRANSPORT_OK on success
 */
transport_err_t transport_manager_disconnect_handle(transport_handle_t handle);

/**
 * @brief Sends data through a specific transport by handle
 * 
 * @param handle Transport handle
 * @param data Data to send
 * @param len Data size
 * @param bytes_written Pointer to bytes written (can be NULL)
 * @param timeout_ms Timeout in milliseconds
 * @return TRANSPORT_OK on success
 */
transport_err_t transport_manager_write_handle(transport_handle_t handle,
                                                const uint8_t *data, size_t len,
                                                size_t *bytes_written, uint32_t timeout_ms);

/**
 * @brief Receives data from a specific transport by handle
 * 
 * @param handle Transport handle
 * @param buffer Buffer for data
 * @param len Maximum size
 * @param bytes_read Pointer to bytes read
 * @param timeout_ms Timeout in milliseconds
 * @return TRANSPORT_OK on success
 */
transport_err_t transport_manager_read_handle(transport_handle_t handle,
                                               uint8_t *buffer, size_t len,
                                               size_t *bytes_read, uint32_t timeout_ms);

/**
 * @brief Broadcasts data to all connected transports
 * 
 * Sends the same data to all transports that are currently connected.
 * Useful for forwarding received data to all other interfaces.
 * 
 * @param data Data to send
 * @param len Data size
 * @param exclude_handle Handle to exclude from broadcast (use TRANSPORT_HANDLE_INVALID to send to all)
 * @param timeout_ms Timeout in milliseconds per transport
 * @return Number of transports that successfully received the data
 */
size_t transport_manager_broadcast(const uint8_t *data, size_t len,
                                    transport_handle_t exclude_handle,
                                    uint32_t timeout_ms);

/**
 * @brief Result structure for poll operation
 */
typedef struct {
    transport_handle_t handle;
    size_t bytes_available;
} transport_poll_result_t;

/**
 * @brief Polls all transports for available data
 * 
 * Checks all registered transports for available data and returns
 * information about which transports have data ready to read.
 * 
 * @param results Array to store poll results
 * @param max_results Maximum number of results
 * @return Number of transports with available data
 */
size_t transport_manager_poll(transport_poll_result_t *results, size_t max_results);

/**
 * @brief Reads from the first transport that has data available
 * 
 * Polls all transports and reads from the first one with available data.
 * Returns the handle of the transport that was read from.
 * 
 * @param buffer Buffer for data
 * @param len Maximum size
 * @param bytes_read Pointer to bytes read
 * @param source_handle Pointer to store the handle of the source transport (can be NULL)
 * @param timeout_ms Timeout in milliseconds
 * @return TRANSPORT_OK on success, TRANSPORT_ERR_TIMEOUT if no data available
 */
transport_err_t transport_manager_read_any(uint8_t *buffer, size_t len,
                                            size_t *bytes_read,
                                            transport_handle_t *source_handle,
                                            uint32_t timeout_ms);

/**
 * @brief Forwards data from one transport to others
 * 
 * Reads data from the source transport and forwards it to all other
 * connected transports (or to a specific destination).
 * 
 * @param source_handle Handle of the source transport
 * @param dest_handle Destination handle (use TRANSPORT_HANDLE_INVALID to broadcast to all others)
 * @param buffer Temporary buffer for data transfer
 * @param buffer_len Size of the temporary buffer
 * @param bytes_forwarded Pointer to store total bytes forwarded (can be NULL)
 * @param timeout_ms Timeout in milliseconds
 * @return TRANSPORT_OK on success
 */
transport_err_t transport_manager_forward(transport_handle_t source_handle,
                                           transport_handle_t dest_handle,
                                           uint8_t *buffer, size_t buffer_len,
                                           size_t *bytes_forwarded,
                                           uint32_t timeout_ms);

/**
 * @brief Gets the number of currently connected transports
 * 
 * @return Number of transports in CONNECTED state
 */
size_t transport_manager_connected_count(void);

/**
 * @brief Checks if a specific transport is connected
 * 
 * @param handle Transport handle
 * @return true if connected, false otherwise
 */
bool transport_manager_is_connected(transport_handle_t handle);

/**
 * @brief Callback for data received on any transport
 */
typedef void (*transport_manager_data_cb_t)(transport_handle_t handle,
                                             const uint8_t *data, size_t len,
                                             void *user_data);

/**
 * @brief Sets a global callback for data received on any transport
 * 
 * This callback is invoked whenever data is received on any registered
 * transport, allowing centralized data handling.
 * 
 * @param callback Callback function (NULL to disable)
 * @param user_data User data passed to callback
 * @return TRANSPORT_OK on success
 */
transport_err_t transport_manager_set_data_callback(transport_manager_data_cb_t callback,
                                                     void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* TRANSPORT_MANAGER_H */