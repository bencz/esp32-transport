/**
 * @file transport_bt_spp.h
 * @brief Transport backend for Bluetooth Classic SPP (Serial Port Profile)
 * 
 * This module implements the transport interface using Bluetooth Classic
 * with the SPP profile for wireless serial communication.
 */

#ifndef TRANSPORT_BT_SPP_H
#define TRANSPORT_BT_SPP_H

#include "transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Bluetooth SPP specific options
 */
typedef enum {
    BT_SPP_OPT_DEVICE_NAME = 0x100,
    BT_SPP_OPT_DISCOVERABLE,
    BT_SPP_OPT_CONNECTABLE,
    BT_SPP_OPT_SECURITY_MODE,
    BT_SPP_OPT_PIN_CODE,
    BT_SPP_OPT_MTU,
} bt_spp_option_t;

/**
 * @brief Bluetooth security modes
 */
typedef enum {
    BT_SPP_SEC_NONE = 0,
    BT_SPP_SEC_AUTHENTICATE,
    BT_SPP_SEC_ENCRYPT,
    BT_SPP_SEC_AUTHENTICATE_ENCRYPT,
} bt_spp_security_t;

/**
 * @brief Device role in SPP connection
 */
typedef enum {
    BT_SPP_ROLE_MASTER = 0,
    BT_SPP_ROLE_SLAVE,
} bt_spp_role_t;

/**
 * @brief Bluetooth SPP specific configuration
 */
typedef struct {
    transport_config_t base;
    const char *device_name;
    bt_spp_role_t role;
    bt_spp_security_t security;
    const char *pin_code;
    bool discoverable;
    bool connectable;
} bt_spp_config_t;

/**
 * @brief Bluetooth SPP transport structure
 * 
 * This structure extends transport_t to include Bluetooth Classic
 * SPP specific data.
 */
typedef struct {
    transport_t base;
    bt_spp_config_t config;
    uint32_t spp_handle;
    uint8_t remote_addr[6];
    bool is_server;
    size_t rx_buffer_pos;
    size_t rx_buffer_len;
    uint8_t *rx_buffer;
    void *internal;
} bt_spp_transport_t;

/**
 * @brief Creates a Bluetooth SPP transport instance
 * 
 * @return Pointer to the created transport, or NULL on error
 */
bt_spp_transport_t *bt_spp_transport_create(void);

/**
 * @brief Destroys a Bluetooth SPP transport instance
 * 
 * @param transport Pointer to the transport to destroy
 */
void bt_spp_transport_destroy(bt_spp_transport_t *transport);

/**
 * @brief Gets default configuration for Bluetooth SPP
 * 
 * @param config Pointer to configuration structure to fill
 */
void bt_spp_config_default(bt_spp_config_t *config);

/**
 * @brief Gets the Bluetooth SPP transport vtable
 * 
 * @return Pointer to the vtable
 */
const transport_vtable_t *bt_spp_get_vtable(void);

/**
 * @brief Starts listening mode (server)
 * 
 * @param transport Pointer to the transport
 * @return TRANSPORT_OK on success
 */
transport_err_t bt_spp_listen(bt_spp_transport_t *transport);

/**
 * @brief Stops listening mode
 * 
 * @param transport Pointer to the transport
 * @return TRANSPORT_OK on success
 */
transport_err_t bt_spp_stop_listen(bt_spp_transport_t *transport);

/**
 * @brief Gets connected remote device address
 * 
 * @param transport Pointer to the transport
 * @param addr Buffer to store the address (6 bytes)
 * @return TRANSPORT_OK on success
 */
transport_err_t bt_spp_get_remote_address(bt_spp_transport_t *transport, uint8_t *addr);

/**
 * @brief Converts Bluetooth address to string
 * 
 * @param addr Bluetooth address (6 bytes)
 * @param str Buffer for string (minimum 18 bytes)
 * @param len Buffer size
 * @return Pointer to the string, or NULL on error
 */
char *bt_addr_to_str(const uint8_t *addr, char *str, size_t len);

/**
 * @brief Converts string to Bluetooth address
 * 
 * @param str String in format "XX:XX:XX:XX:XX:XX"
 * @param addr Buffer for the address (6 bytes)
 * @return TRANSPORT_OK on success
 */
transport_err_t bt_str_to_addr(const char *str, uint8_t *addr);

#ifdef __cplusplus
}
#endif

#endif /* TRANSPORT_BT_SPP_H */