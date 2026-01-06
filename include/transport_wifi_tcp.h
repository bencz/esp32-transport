/**
 * @file transport_wifi_tcp.h
 * @brief Transport backend for WiFi TCP connections
 * 
 * This module implements the transport interface using WiFi TCP
 * sockets for network communication.
 */

#ifndef TRANSPORT_WIFI_TCP_H
#define TRANSPORT_WIFI_TCP_H

#include "transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief WiFi TCP specific options
 */
typedef enum {
    WIFI_TCP_OPT_KEEP_ALIVE = 0x300,
    WIFI_TCP_OPT_KEEP_IDLE,
    WIFI_TCP_OPT_KEEP_INTERVAL,
    WIFI_TCP_OPT_KEEP_COUNT,
    WIFI_TCP_OPT_NODELAY,
    WIFI_TCP_OPT_RECV_TIMEOUT,
    WIFI_TCP_OPT_SEND_TIMEOUT,
} wifi_tcp_option_t;

/**
 * @brief WiFi TCP role (client or server)
 */
typedef enum {
    WIFI_TCP_ROLE_CLIENT = 0,
    WIFI_TCP_ROLE_SERVER,
} wifi_tcp_role_t;

/**
 * @brief WiFi TCP configuration
 */
typedef struct {
    transport_config_t base;
    wifi_tcp_role_t role;
    const char *host;
    uint16_t port;
    bool keep_alive;
    uint32_t keep_idle_sec;
    uint32_t keep_interval_sec;
    uint32_t keep_count;
    bool nodelay;
    int max_clients;
} wifi_tcp_config_t;

/**
 * @brief WiFi TCP transport structure
 */
typedef struct {
    transport_t base;
    wifi_tcp_config_t config;
    int socket;
    int client_socket;
    int server_socket;
    char remote_ip[16];
    uint16_t remote_port;
    void *internal;
} wifi_tcp_transport_t;

/**
 * @brief Creates a WiFi TCP transport instance
 * 
 * @return Pointer to the created transport, or NULL on error
 */
wifi_tcp_transport_t *wifi_tcp_transport_create(void);

/**
 * @brief Destroys a WiFi TCP transport instance
 * 
 * @param transport Pointer to the transport to destroy
 */
void wifi_tcp_transport_destroy(wifi_tcp_transport_t *transport);

/**
 * @brief Gets default configuration for WiFi TCP
 * 
 * @param config Pointer to configuration structure to fill
 */
void wifi_tcp_config_default(wifi_tcp_config_t *config);

/**
 * @brief Gets the WiFi TCP transport vtable
 * 
 * @return Pointer to the vtable
 */
const transport_vtable_t *wifi_tcp_get_vtable(void);

/**
 * @brief Starts listening for connections (server mode)
 * 
 * @param transport Pointer to the transport
 * @return TRANSPORT_OK on success
 */
transport_err_t wifi_tcp_listen(wifi_tcp_transport_t *transport);

/**
 * @brief Stops listening for connections
 * 
 * @param transport Pointer to the transport
 * @return TRANSPORT_OK on success
 */
transport_err_t wifi_tcp_stop_listen(wifi_tcp_transport_t *transport);

/**
 * @brief Accepts a pending client connection (server mode)
 * 
 * @param transport Pointer to the transport
 * @param timeout_ms Timeout in milliseconds
 * @return TRANSPORT_OK on success
 */
transport_err_t wifi_tcp_accept(wifi_tcp_transport_t *transport, uint32_t timeout_ms);

/**
 * @brief Gets the remote IP address
 * 
 * @param transport Pointer to the transport
 * @param ip Buffer to store the IP address (minimum 16 bytes)
 * @param port Pointer to store the port (can be NULL)
 * @return TRANSPORT_OK on success
 */
transport_err_t wifi_tcp_get_remote_addr(wifi_tcp_transport_t *transport, 
                                          char *ip, uint16_t *port);

/**
 * @brief Gets the local IP address
 * 
 * @param transport Pointer to the transport
 * @param ip Buffer to store the IP address (minimum 16 bytes)
 * @return TRANSPORT_OK on success
 */
transport_err_t wifi_tcp_get_local_addr(wifi_tcp_transport_t *transport, char *ip);

#ifdef __cplusplus
}
#endif

#endif /* TRANSPORT_WIFI_TCP_H */
