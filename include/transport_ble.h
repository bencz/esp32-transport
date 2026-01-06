/**
 * @file transport_ble.h
 * @brief Transport backend for Bluetooth Low Energy (BLE)
 * 
 * This module implements the transport interface using BLE
 * with a custom GATT service for bidirectional communication.
 */

#ifndef TRANSPORT_BLE_H
#define TRANSPORT_BLE_H

#include "transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Default UUIDs for the BLE transport service
 */
#define BLE_TRANSPORT_SERVICE_UUID      0x00FF
#define BLE_TRANSPORT_CHAR_TX_UUID      0xFF01
#define BLE_TRANSPORT_CHAR_RX_UUID      0xFF02

/**
 * @brief BLE specific options
 */
typedef enum {
    BLE_OPT_DEVICE_NAME = 0x200,
    BLE_OPT_SERVICE_UUID,
    BLE_OPT_TX_CHAR_UUID,
    BLE_OPT_RX_CHAR_UUID,
    BLE_OPT_ADV_INTERVAL,
    BLE_OPT_CONN_INTERVAL,
    BLE_OPT_MTU,
    BLE_OPT_NOTIFY_ENABLE,
} ble_option_t;

/**
 * @brief BLE device role
 */
typedef enum {
    BLE_ROLE_PERIPHERAL = 0,
    BLE_ROLE_CENTRAL,
} ble_role_t;

/**
 * @brief Advertising configuration
 */
typedef struct {
    uint16_t min_interval;
    uint16_t max_interval;
    bool connectable;
    bool scannable;
} ble_adv_config_t;

/**
 * @brief Connection configuration
 */
typedef struct {
    uint16_t min_interval;
    uint16_t max_interval;
    uint16_t latency;
    uint16_t timeout;
} ble_conn_config_t;

/**
 * @brief BLE specific configuration
 */
typedef struct {
    transport_config_t base;
    const char *device_name;
    ble_role_t role;
    uint16_t service_uuid;
    uint16_t tx_char_uuid;
    uint16_t rx_char_uuid;
    uint16_t mtu;
    ble_adv_config_t adv_config;
    ble_conn_config_t conn_config;
} ble_config_t;

/**
 * @brief BLE transport structure
 */
typedef struct {
    transport_t base;
    ble_config_t config;
    uint16_t conn_handle;
    uint16_t attr_handle_tx;
    uint16_t attr_handle_rx;
    uint8_t peer_addr[6];
    uint8_t peer_addr_type;
    uint16_t current_mtu;
    void *internal;
} ble_transport_t;

/**
 * @brief Creates a BLE transport instance
 * 
 * @return Pointer to the created transport, or NULL on error
 */
ble_transport_t *ble_transport_create(void);

/**
 * @brief Destroys a BLE transport instance
 * 
 * @param transport Pointer to the transport to destroy
 */
void ble_transport_destroy(ble_transport_t *transport);

/**
 * @brief Gets default configuration for BLE
 * 
 * @param config Pointer to configuration structure to fill
 */
void ble_config_default(ble_config_t *config);

/**
 * @brief Gets the BLE transport vtable
 * 
 * @return Pointer to the vtable
 */
const transport_vtable_t *ble_get_vtable(void);

/**
 * @brief Starts advertising (peripheral mode)
 * 
 * @param transport Pointer to the transport
 * @return TRANSPORT_OK on success
 */
transport_err_t ble_start_advertising(ble_transport_t *transport);

/**
 * @brief Stops advertising
 * 
 * @param transport Pointer to the transport
 * @return TRANSPORT_OK on success
 */
transport_err_t ble_stop_advertising(ble_transport_t *transport);

/**
 * @brief Starts scan (central mode)
 * 
 * @param transport Pointer to the transport
 * @param duration_sec Scan duration in seconds (0 = indefinite)
 * @return TRANSPORT_OK on success
 */
transport_err_t ble_start_scan(ble_transport_t *transport, uint32_t duration_sec);

/**
 * @brief Stops scan
 * 
 * @param transport Pointer to the transport
 * @return TRANSPORT_OK on success
 */
transport_err_t ble_stop_scan(ble_transport_t *transport);

/**
 * @brief Gets current connection MTU
 * 
 * @param transport Pointer to the transport
 * @return Current MTU, or 0 if not connected
 */
uint16_t ble_get_mtu(ble_transport_t *transport);

/**
 * @brief Requests MTU update
 * 
 * @param transport Pointer to the transport
 * @param mtu Desired MTU
 * @return TRANSPORT_OK on success
 */
transport_err_t ble_request_mtu(ble_transport_t *transport, uint16_t mtu);

/**
 * @brief Gets connected peer address
 * 
 * @param transport Pointer to the transport
 * @param addr Buffer to store the address (6 bytes)
 * @param addr_type Pointer to store the address type (can be NULL)
 * @return TRANSPORT_OK on success
 */
transport_err_t ble_get_peer_address(ble_transport_t *transport, uint8_t *addr, uint8_t *addr_type);

/**
 * @brief Callback for devices discovered during scan
 */
typedef void (*ble_scan_result_cb_t)(const uint8_t *addr, uint8_t addr_type, 
                                      int8_t rssi, const char *name, void *user_data);

/**
 * @brief Registers callback for scan results
 * 
 * @param transport Pointer to the transport
 * @param callback Callback function
 * @param user_data User data
 * @return TRANSPORT_OK on success
 */
transport_err_t ble_set_scan_callback(ble_transport_t *transport, 
                                       ble_scan_result_cb_t callback, 
                                       void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* TRANSPORT_BLE_H */