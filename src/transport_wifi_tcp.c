/**
 * @file transport_wifi_tcp.c
 * @brief WiFi TCP transport backend implementation
 */

#include "transport_wifi_tcp.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

static const char *TAG = "wifi_tcp_transport";

#define WIFI_TCP_DEFAULT_PORT           (3333)
#define WIFI_TCP_DEFAULT_BUFFER_SIZE    (4096)
#define WIFI_TCP_DEFAULT_TIMEOUT_MS     (5000)
#define WIFI_TCP_MAX_CLIENTS            (1)

typedef struct {
    SemaphoreHandle_t mutex;
    EventGroupHandle_t event_group;
    bool listening;
} wifi_tcp_internal_t;

#define EVENT_CONNECTED     BIT0
#define EVENT_DISCONNECTED  BIT1
#define EVENT_DATA_READY    BIT2

static transport_err_t wifi_tcp_init(transport_t *self, const void *config);
static transport_err_t wifi_tcp_deinit(transport_t *self);
static transport_err_t wifi_tcp_connect(transport_t *self, const char *address, uint32_t timeout_ms);
static transport_err_t wifi_tcp_disconnect(transport_t *self);
static transport_err_t wifi_tcp_write(transport_t *self, const uint8_t *data, size_t len, 
                                       size_t *bytes_written, uint32_t timeout_ms);
static transport_err_t wifi_tcp_read(transport_t *self, uint8_t *buffer, size_t len, 
                                      size_t *bytes_read, uint32_t timeout_ms);
static transport_err_t wifi_tcp_available(transport_t *self, size_t *available);
static transport_err_t wifi_tcp_flush(transport_t *self);
static transport_state_t wifi_tcp_get_state(transport_t *self);
static transport_type_t wifi_tcp_get_type(transport_t *self);
static transport_err_t wifi_tcp_get_info(transport_t *self, char *info, size_t len);
static transport_err_t wifi_tcp_set_option(transport_t *self, int option, 
                                            const void *value, size_t len);
static transport_err_t wifi_tcp_get_option(transport_t *self, int option, 
                                            void *value, size_t *len);

static const transport_vtable_t wifi_tcp_vtable = {
    .init       = wifi_tcp_init,
    .deinit     = wifi_tcp_deinit,
    .connect    = wifi_tcp_connect,
    .disconnect = wifi_tcp_disconnect,
    .write      = wifi_tcp_write,
    .read       = wifi_tcp_read,
    .available  = wifi_tcp_available,
    .flush      = wifi_tcp_flush,
    .get_state  = wifi_tcp_get_state,
    .get_type   = wifi_tcp_get_type,
    .get_info   = wifi_tcp_get_info,
    .set_option = wifi_tcp_set_option,
    .get_option = wifi_tcp_get_option,
};

static int get_active_socket(wifi_tcp_transport_t *transport)
{
    if (transport->config.role == WIFI_TCP_ROLE_SERVER) {
        return transport->client_socket;
    }
    return transport->socket;
}

static transport_err_t wifi_tcp_init(transport_t *self, const void *config)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    wifi_tcp_transport_t *transport = (wifi_tcp_transport_t *)self;
    
    if (self->state != TRANSPORT_STATE_UNINITIALIZED) {
        return TRANSPORT_ERR_ALREADY_INITIALIZED;
    }
    
    if (config != NULL) {
        memcpy(&transport->config, config, sizeof(wifi_tcp_config_t));
    }
    
    wifi_tcp_internal_t *internal = calloc(1, sizeof(wifi_tcp_internal_t));
    if (internal == NULL) {
        return TRANSPORT_ERR_NO_MEM;
    }
    
    internal->mutex = xSemaphoreCreateMutex();
    internal->event_group = xEventGroupCreate();
    
    if (internal->mutex == NULL || internal->event_group == NULL) {
        if (internal->mutex) vSemaphoreDelete(internal->mutex);
        if (internal->event_group) vEventGroupDelete(internal->event_group);
        free(internal);
        return TRANSPORT_ERR_NO_MEM;
    }
    
    transport->internal = internal;
    transport->socket = -1;
    transport->client_socket = -1;
    transport->server_socket = -1;
    
    transport_set_state(self, TRANSPORT_STATE_INITIALIZED);
    
    ESP_LOGI(TAG, "WiFi TCP transport initialized");
    
    return TRANSPORT_OK;
}

static transport_err_t wifi_tcp_deinit(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    wifi_tcp_transport_t *transport = (wifi_tcp_transport_t *)self;
    wifi_tcp_internal_t *internal = (wifi_tcp_internal_t *)transport->internal;
    
    if (self->state == TRANSPORT_STATE_UNINITIALIZED) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (self->state == TRANSPORT_STATE_CONNECTED) {
        wifi_tcp_disconnect(self);
    }
    
    if (internal->listening) {
        wifi_tcp_stop_listen(transport);
    }
    
    if (internal->mutex) vSemaphoreDelete(internal->mutex);
    if (internal->event_group) vEventGroupDelete(internal->event_group);
    
    free(internal);
    transport->internal = NULL;
    
    transport_set_state(self, TRANSPORT_STATE_UNINITIALIZED);
    
    ESP_LOGI(TAG, "WiFi TCP transport deinitialized");
    
    return TRANSPORT_OK;
}

static transport_err_t wifi_tcp_connect(transport_t *self, const char *address, uint32_t timeout_ms)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    wifi_tcp_transport_t *transport = (wifi_tcp_transport_t *)self;
    
    if (self->state == TRANSPORT_STATE_UNINITIALIZED) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    if (self->state == TRANSPORT_STATE_CONNECTED) {
        return TRANSPORT_ERR_ALREADY_CONNECTED;
    }
    
    if (transport->config.role == WIFI_TCP_ROLE_SERVER) {
        return wifi_tcp_accept(transport, timeout_ms);
    }
    
    const char *host = address ? address : transport->config.host;
    if (host == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    transport_set_state(self, TRANSPORT_STATE_CONNECTING);
    
    struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    
    char port_str[6];
    snprintf(port_str, sizeof(port_str), "%d", transport->config.port);
    
    int err = getaddrinfo(host, port_str, &hints, &res);
    if (err != 0 || res == NULL) {
        ESP_LOGE(TAG, "DNS lookup failed: %d", err);
        transport_set_state(self, TRANSPORT_STATE_INITIALIZED);
        return TRANSPORT_ERR_IO;
    }
    
    transport->socket = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (transport->socket < 0) {
        ESP_LOGE(TAG, "Socket creation failed: %d", errno);
        freeaddrinfo(res);
        transport_set_state(self, TRANSPORT_STATE_INITIALIZED);
        return TRANSPORT_ERR_IO;
    }
    
    if (timeout_ms > 0) {
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        setsockopt(transport->socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(transport->socket, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    }
    
    if (transport->config.keep_alive) {
        int keepalive = 1;
        int keepidle = transport->config.keep_idle_sec;
        int keepinterval = transport->config.keep_interval_sec;
        int keepcount = transport->config.keep_count;
        
        setsockopt(transport->socket, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
        setsockopt(transport->socket, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle));
        setsockopt(transport->socket, IPPROTO_TCP, TCP_KEEPINTVL, &keepinterval, sizeof(keepinterval));
        setsockopt(transport->socket, IPPROTO_TCP, TCP_KEEPCNT, &keepcount, sizeof(keepcount));
    }
    
    if (transport->config.nodelay) {
        int nodelay = 1;
        setsockopt(transport->socket, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
    }
    
    err = connect(transport->socket, res->ai_addr, res->ai_addrlen);
    if (err != 0) {
        ESP_LOGE(TAG, "Socket connect failed: %d", errno);
        close(transport->socket);
        transport->socket = -1;
        freeaddrinfo(res);
        transport_set_state(self, TRANSPORT_STATE_INITIALIZED);
        return TRANSPORT_ERR_IO;
    }
    
    struct sockaddr_in *addr_in = (struct sockaddr_in *)res->ai_addr;
    inet_ntoa_r(addr_in->sin_addr, transport->remote_ip, sizeof(transport->remote_ip));
    transport->remote_port = ntohs(addr_in->sin_port);
    
    freeaddrinfo(res);
    
    transport_set_state(self, TRANSPORT_STATE_CONNECTED);
    
    transport_event_t evt = { .type = TRANSPORT_EVENT_CONNECTED };
    transport_emit_event(self, &evt);
    
    ESP_LOGI(TAG, "Connected to %s:%d", transport->remote_ip, transport->remote_port);
    
    return TRANSPORT_OK;
}

static transport_err_t wifi_tcp_disconnect(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    wifi_tcp_transport_t *transport = (wifi_tcp_transport_t *)self;
    
    if (self->state != TRANSPORT_STATE_CONNECTED) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    transport_set_state(self, TRANSPORT_STATE_DISCONNECTING);
    
    int sock = get_active_socket(transport);
    if (sock >= 0) {
        shutdown(sock, SHUT_RDWR);
        close(sock);
    }
    
    if (transport->config.role == WIFI_TCP_ROLE_SERVER) {
        transport->client_socket = -1;
    } else {
        transport->socket = -1;
    }
    
    transport_set_state(self, TRANSPORT_STATE_INITIALIZED);
    
    transport_event_t evt = { .type = TRANSPORT_EVENT_DISCONNECTED };
    transport_emit_event(self, &evt);
    
    ESP_LOGI(TAG, "Disconnected");
    
    return TRANSPORT_OK;
}

static transport_err_t wifi_tcp_write(transport_t *self, const uint8_t *data, size_t len, 
                                       size_t *bytes_written, uint32_t timeout_ms)
{
    if (self == NULL || data == NULL || len == 0) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    wifi_tcp_transport_t *transport = (wifi_tcp_transport_t *)self;
    
    if (self->state != TRANSPORT_STATE_CONNECTED) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    int sock = get_active_socket(transport);
    if (sock < 0) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    if (timeout_ms > 0) {
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    }
    
    ssize_t sent = send(sock, data, len, 0);
    if (sent < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return TRANSPORT_ERR_TIMEOUT;
        }
        ESP_LOGE(TAG, "Send failed: %d", errno);
        return TRANSPORT_ERR_IO;
    }
    
    if (bytes_written != NULL) {
        *bytes_written = (size_t)sent;
    }
    
    return TRANSPORT_OK;
}

static transport_err_t wifi_tcp_read(transport_t *self, uint8_t *buffer, size_t len, 
                                      size_t *bytes_read, uint32_t timeout_ms)
{
    if (self == NULL || buffer == NULL || len == 0 || bytes_read == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    wifi_tcp_transport_t *transport = (wifi_tcp_transport_t *)self;
    
    if (self->state != TRANSPORT_STATE_CONNECTED) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    int sock = get_active_socket(transport);
    if (sock < 0) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    if (timeout_ms > 0) {
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }
    
    ssize_t received = recv(sock, buffer, len, 0);
    if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            *bytes_read = 0;
            return TRANSPORT_ERR_TIMEOUT;
        }
        ESP_LOGE(TAG, "Receive failed: %d", errno);
        return TRANSPORT_ERR_IO;
    }
    
    if (received == 0) {
        wifi_tcp_disconnect(self);
        *bytes_read = 0;
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    *bytes_read = (size_t)received;
    
    return TRANSPORT_OK;
}

static transport_err_t wifi_tcp_available(transport_t *self, size_t *available)
{
    if (self == NULL || available == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    wifi_tcp_transport_t *transport = (wifi_tcp_transport_t *)self;
    
    int sock = get_active_socket(transport);
    if (sock < 0) {
        *available = 0;
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    int bytes_available = 0;
    if (ioctl(sock, FIONREAD, &bytes_available) < 0) {
        *available = 0;
        return TRANSPORT_ERR_IO;
    }
    
    *available = (size_t)bytes_available;
    
    return TRANSPORT_OK;
}

static transport_err_t wifi_tcp_flush(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    wifi_tcp_transport_t *transport = (wifi_tcp_transport_t *)self;
    
    int sock = get_active_socket(transport);
    if (sock < 0) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    uint8_t discard[256];
    int bytes_available;
    
    while (ioctl(sock, FIONREAD, &bytes_available) == 0 && bytes_available > 0) {
        size_t to_read = bytes_available < (int)sizeof(discard) ? bytes_available : sizeof(discard);
        recv(sock, discard, to_read, 0);
    }
    
    return TRANSPORT_OK;
}

static transport_state_t wifi_tcp_get_state(transport_t *self)
{
    if (self == NULL) {
        return TRANSPORT_STATE_UNINITIALIZED;
    }
    return self->state;
}

static transport_type_t wifi_tcp_get_type(transport_t *self)
{
    (void)self;
    return TRANSPORT_TYPE_WIFI_TCP;
}

static transport_err_t wifi_tcp_get_info(transport_t *self, char *info, size_t len)
{
    if (self == NULL || info == NULL || len == 0) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    wifi_tcp_transport_t *transport = (wifi_tcp_transport_t *)self;
    
    if (self->state == TRANSPORT_STATE_CONNECTED) {
        snprintf(info, len, "WiFi TCP - State: %s, Remote: %s:%d",
                 transport_state_to_str(self->state), 
                 transport->remote_ip, transport->remote_port);
    } else {
        snprintf(info, len, "WiFi TCP - State: %s, Port: %d",
                 transport_state_to_str(self->state), transport->config.port);
    }
    
    return TRANSPORT_OK;
}

static transport_err_t wifi_tcp_set_option(transport_t *self, int option, 
                                            const void *value, size_t len)
{
    if (self == NULL || value == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    wifi_tcp_transport_t *transport = (wifi_tcp_transport_t *)self;
    int sock = get_active_socket(transport);
    
    switch ((wifi_tcp_option_t)option) {
        case WIFI_TCP_OPT_KEEP_ALIVE:
            if (len >= sizeof(bool)) {
                transport->config.keep_alive = *(const bool *)value;
                if (sock >= 0) {
                    int keepalive = transport->config.keep_alive ? 1 : 0;
                    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
                }
            }
            break;
            
        case WIFI_TCP_OPT_NODELAY:
            if (len >= sizeof(bool)) {
                transport->config.nodelay = *(const bool *)value;
                if (sock >= 0) {
                    int nodelay = transport->config.nodelay ? 1 : 0;
                    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
                }
            }
            break;
            
        default:
            return TRANSPORT_ERR_NOT_SUPPORTED;
    }
    
    return TRANSPORT_OK;
}

static transport_err_t wifi_tcp_get_option(transport_t *self, int option, 
                                            void *value, size_t *len)
{
    if (self == NULL || value == NULL || len == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    wifi_tcp_transport_t *transport = (wifi_tcp_transport_t *)self;
    
    switch ((wifi_tcp_option_t)option) {
        case WIFI_TCP_OPT_KEEP_ALIVE:
            if (*len >= sizeof(bool)) {
                *(bool *)value = transport->config.keep_alive;
                *len = sizeof(bool);
            }
            break;
            
        case WIFI_TCP_OPT_NODELAY:
            if (*len >= sizeof(bool)) {
                *(bool *)value = transport->config.nodelay;
                *len = sizeof(bool);
            }
            break;
            
        default:
            return TRANSPORT_ERR_NOT_SUPPORTED;
    }
    
    return TRANSPORT_OK;
}

wifi_tcp_transport_t *wifi_tcp_transport_create(void)
{
    wifi_tcp_transport_t *transport = calloc(1, sizeof(wifi_tcp_transport_t));
    if (transport == NULL) {
        return NULL;
    }
    
    transport->base.vtable = &wifi_tcp_vtable;
    transport->base.state = TRANSPORT_STATE_UNINITIALIZED;
    transport->base.name = "WiFi TCP";
    transport->socket = -1;
    transport->client_socket = -1;
    transport->server_socket = -1;
    
    wifi_tcp_config_default(&transport->config);
    
    return transport;
}

void wifi_tcp_transport_destroy(wifi_tcp_transport_t *transport)
{
    if (transport == NULL) {
        return;
    }
    
    if (transport->base.state != TRANSPORT_STATE_UNINITIALIZED) {
        wifi_tcp_deinit(&transport->base);
    }
    
    free(transport);
}

void wifi_tcp_config_default(wifi_tcp_config_t *config)
{
    if (config == NULL) {
        return;
    }
    
    memset(config, 0, sizeof(wifi_tcp_config_t));
    config->base.name = "ESP32_TCP";
    config->base.timeout_ms = WIFI_TCP_DEFAULT_TIMEOUT_MS;
    config->base.rx_buffer_size = WIFI_TCP_DEFAULT_BUFFER_SIZE;
    config->base.tx_buffer_size = WIFI_TCP_DEFAULT_BUFFER_SIZE;
    config->role = WIFI_TCP_ROLE_CLIENT;
    config->host = NULL;
    config->port = WIFI_TCP_DEFAULT_PORT;
    config->keep_alive = true;
    config->keep_idle_sec = 5;
    config->keep_interval_sec = 5;
    config->keep_count = 3;
    config->nodelay = true;
    config->max_clients = WIFI_TCP_MAX_CLIENTS;
}

const transport_vtable_t *wifi_tcp_get_vtable(void)
{
    return &wifi_tcp_vtable;
}

transport_err_t wifi_tcp_listen(wifi_tcp_transport_t *transport)
{
    if (transport == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    if (transport->base.state == TRANSPORT_STATE_UNINITIALIZED) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    wifi_tcp_internal_t *internal = (wifi_tcp_internal_t *)transport->internal;
    
    if (internal->listening) {
        return TRANSPORT_ERR_BUSY;
    }
    
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(transport->config.port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    
    transport->server_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (transport->server_socket < 0) {
        ESP_LOGE(TAG, "Server socket creation failed: %d", errno);
        return TRANSPORT_ERR_IO;
    }
    
    int opt = 1;
    setsockopt(transport->server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    int err = bind(transport->server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket bind failed: %d", errno);
        close(transport->server_socket);
        transport->server_socket = -1;
        return TRANSPORT_ERR_IO;
    }
    
    err = listen(transport->server_socket, transport->config.max_clients);
    if (err != 0) {
        ESP_LOGE(TAG, "Socket listen failed: %d", errno);
        close(transport->server_socket);
        transport->server_socket = -1;
        return TRANSPORT_ERR_IO;
    }
    
    internal->listening = true;
    
    ESP_LOGI(TAG, "Listening on port %d", transport->config.port);
    
    return TRANSPORT_OK;
}

transport_err_t wifi_tcp_stop_listen(wifi_tcp_transport_t *transport)
{
    if (transport == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    wifi_tcp_internal_t *internal = (wifi_tcp_internal_t *)transport->internal;
    
    if (!internal->listening) {
        return TRANSPORT_OK;
    }
    
    if (transport->server_socket >= 0) {
        close(transport->server_socket);
        transport->server_socket = -1;
    }
    
    internal->listening = false;
    
    ESP_LOGI(TAG, "Stopped listening");
    
    return TRANSPORT_OK;
}

transport_err_t wifi_tcp_accept(wifi_tcp_transport_t *transport, uint32_t timeout_ms)
{
    if (transport == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    wifi_tcp_internal_t *internal = (wifi_tcp_internal_t *)transport->internal;
    
    if (!internal->listening) {
        transport_err_t err = wifi_tcp_listen(transport);
        if (err != TRANSPORT_OK) {
            return err;
        }
    }
    
    if (timeout_ms > 0) {
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        setsockopt(transport->server_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }
    
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    
    transport_set_state(&transport->base, TRANSPORT_STATE_CONNECTING);
    
    transport->client_socket = accept(transport->server_socket, 
                                       (struct sockaddr *)&client_addr, &addr_len);
    
    if (transport->client_socket < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            transport_set_state(&transport->base, TRANSPORT_STATE_INITIALIZED);
            return TRANSPORT_ERR_TIMEOUT;
        }
        ESP_LOGE(TAG, "Accept failed: %d", errno);
        transport_set_state(&transport->base, TRANSPORT_STATE_INITIALIZED);
        return TRANSPORT_ERR_IO;
    }
    
    inet_ntoa_r(client_addr.sin_addr, transport->remote_ip, sizeof(transport->remote_ip));
    transport->remote_port = ntohs(client_addr.sin_port);
    
    if (transport->config.keep_alive) {
        int keepalive = 1;
        setsockopt(transport->client_socket, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
    }
    
    if (transport->config.nodelay) {
        int nodelay = 1;
        setsockopt(transport->client_socket, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
    }
    
    transport_set_state(&transport->base, TRANSPORT_STATE_CONNECTED);
    
    transport_event_t evt = { .type = TRANSPORT_EVENT_CONNECTED };
    transport_emit_event(&transport->base, &evt);
    
    ESP_LOGI(TAG, "Client connected from %s:%d", transport->remote_ip, transport->remote_port);
    
    return TRANSPORT_OK;
}

transport_err_t wifi_tcp_get_remote_addr(wifi_tcp_transport_t *transport, 
                                          char *ip, uint16_t *port)
{
    if (transport == NULL || ip == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    if (transport->base.state != TRANSPORT_STATE_CONNECTED) {
        return TRANSPORT_ERR_NOT_CONNECTED;
    }
    
    strncpy(ip, transport->remote_ip, 16);
    if (port != NULL) {
        *port = transport->remote_port;
    }
    
    return TRANSPORT_OK;
}

transport_err_t wifi_tcp_get_local_addr(wifi_tcp_transport_t *transport, char *ip)
{
    if (transport == NULL || ip == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    
    int sock = get_active_socket(transport);
    if (sock < 0 && transport->server_socket >= 0) {
        sock = transport->server_socket;
    }
    
    if (sock < 0) {
        return TRANSPORT_ERR_NOT_INITIALIZED;
    }
    
    struct sockaddr_in local_addr;
    socklen_t addr_len = sizeof(local_addr);
    
    if (getsockname(sock, (struct sockaddr *)&local_addr, &addr_len) < 0) {
        return TRANSPORT_ERR_IO;
    }
    
    inet_ntoa_r(local_addr.sin_addr, ip, 16);
    
    return TRANSPORT_OK;
}
