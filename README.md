# ESP32 Transport Abstraction Layer

Transport abstraction system for ESP32, implemented in C following best practices for embedded systems.

## Architecture

The system uses the vtable pattern to implement polymorphism in C, allowing different communication backends to be used interchangeably through a common interface.

```
                    +-------------------+
                    |   transport_t     |
                    |   (interface)     |
                    +--------+----------+
                             |
     +-----------+-----------+-----------+-----------+
     |           |           |           |           |
+----v----+ +----v----+ +----v----+ +----v----+ +----v----+
| bt_spp  | |   ble   | |wifi_tcp | |  uart   | |   usb   |
| (SPP)   | |  (BLE)  | |  (TCP)  | | (UART)  | |  (CDC)  |
+---------+ +---------+ +---------+ +---------+ +---------+
```

## Project Structure

```
esp32_transport/
├── include/
│   ├── transport.h           # Base interface and types
│   ├── transport_bt_spp.h    # Bluetooth Classic SPP backend
│   ├── transport_ble.h       # Bluetooth Low Energy backend
│   ├── transport_wifi_tcp.h  # WiFi TCP backend
│   ├── transport_uart.h      # UART backend
│   ├── transport_usb.h       # USB CDC backend
│   └── transport_manager.h   # Transport manager
├── src/
│   ├── transport.c           # Base implementation
│   ├── transport_bt_spp.c    # SPP implementation
│   ├── transport_ble.c       # BLE implementation
│   ├── transport_wifi_tcp.c  # WiFi TCP implementation
│   ├── transport_uart.c      # UART implementation
│   ├── transport_usb.c       # USB CDC implementation
│   └── transport_manager.c   # Manager implementation
├── examples/
│   ├── example_bt_spp.c      # Bluetooth SPP example
│   ├── example_ble.c         # BLE example
│   ├── example_wifi_tcp.c    # WiFi TCP example
│   ├── example_uart.c        # UART example
│   ├── example_usb.c         # USB CDC example
│   └── example_multi_transport.c  # Multi-transport gateway
├── CMakeLists.txt
└── README.md
```

## Key Concepts

### Vtable (Virtual Table)

Each backend implements a vtable with pointers to its functions:

```c
struct transport_vtable_s {
    transport_err_t (*init)(transport_t *self, const void *config);
    transport_err_t (*deinit)(transport_t *self);
    transport_err_t (*connect)(transport_t *self, const char *address, uint32_t timeout_ms);
    transport_err_t (*disconnect)(transport_t *self);
    transport_err_t (*write)(transport_t *self, const uint8_t *data, size_t len, ...);
    transport_err_t (*read)(transport_t *self, uint8_t *buffer, size_t len, ...);
    // ... other operations
};
```

### Base Structure

All backends "inherit" from the base structure:

```c
struct transport_s {
    const transport_vtable_t *vtable;  // Pointer to vtable
    transport_state_t state;
    transport_event_cb_t event_callback;
    void *user_data;
    const char *name;
};

// Specific backend "inherits" by placing base as first member
typedef struct {
    transport_t base;           // "Inheritance"
    bt_spp_config_t config;     // Specific data
    // ...
} bt_spp_transport_t;
```

### Polymorphism

Client code works with pointers to `transport_t`:

```c
void process_any_transport(transport_t *transport) {
    // Works with any backend!
    transport_connect(transport, "address", 5000);
    transport_write(transport, data, len, NULL, 1000);
    transport_disconnect(transport);
}

// Usage
bt_spp_transport_t *spp = bt_spp_transport_create();
ble_transport_t *ble = ble_transport_create();

process_any_transport(&spp->base);  // Bluetooth Classic
process_any_transport(&ble->base);  // BLE
```

## Basic Usage

### Bluetooth Classic SPP

```c
#include "transport.h"
#include "transport_bt_spp.h"

// Create transport
bt_spp_transport_t *spp = bt_spp_transport_create();

// Configure
bt_spp_config_t config;
bt_spp_config_default(&config);
config.device_name = "MyESP32";

// Initialize
transport_init(&spp->base, &config);

// Server mode (wait for connections)
bt_spp_listen(spp);

// Or client mode (connect to device)
transport_connect(&spp->base, "AA:BB:CC:DD:EE:FF", 5000);

// Send/receive data
transport_write(&spp->base, data, len, &written, 1000);
transport_read(&spp->base, buffer, sizeof(buffer), &read, 1000);

// Cleanup
transport_disconnect(&spp->base);
transport_deinit(&spp->base);
bt_spp_transport_destroy(spp);
```

### Bluetooth Low Energy

```c
#include "transport.h"
#include "transport_ble.h"

// Create transport
ble_transport_t *ble = ble_transport_create();

// Configure
ble_config_t config;
ble_config_default(&config);
config.device_name = "MyESP32_BLE";
config.mtu = 512;

// Initialize
transport_init(&ble->base, &config);

// Start advertising (peripheral)
ble_start_advertising(ble);

// Send/receive (after connection)
transport_write(&ble->base, data, len, &written, 1000);
transport_read(&ble->base, buffer, sizeof(buffer), &read, 1000);

// Cleanup
ble_stop_advertising(ble);
transport_deinit(&ble->base);
ble_transport_destroy(ble);
```

### WiFi TCP

```c
#include "transport.h"
#include "transport_wifi_tcp.h"

// Create transport
wifi_tcp_transport_t *tcp = wifi_tcp_transport_create();

// Configure as client
wifi_tcp_config_t config;
wifi_tcp_config_default(&config);
config.role = WIFI_TCP_ROLE_CLIENT;
config.host = "192.168.1.100";
config.port = 3333;

// Initialize
transport_init(&tcp->base, &config);

// Connect
transport_connect(&tcp->base, NULL, 5000);

// Send/receive data
transport_write(&tcp->base, data, len, &written, 1000);
transport_read(&tcp->base, buffer, sizeof(buffer), &read, 1000);

// Cleanup
transport_disconnect(&tcp->base);
transport_deinit(&tcp->base);
wifi_tcp_transport_destroy(tcp);
```

### WiFi TCP Server

```c
// Configure as server
wifi_tcp_config_t config;
wifi_tcp_config_default(&config);
config.role = WIFI_TCP_ROLE_SERVER;
config.port = 3333;

transport_init(&tcp->base, &config);

// Start listening
wifi_tcp_listen(tcp);

// Accept connection (blocking)
wifi_tcp_accept(tcp, 30000);

// Now connected, can read/write
```

### UART

```c
#include "transport.h"
#include "transport_uart.h"

// Create transport
uart_transport_t *uart = uart_transport_create();

// Configure
uart_transport_config_t config;
uart_transport_config_default(&config);
config.uart_num = 1;
config.baud_rate = 115200;
config.tx_pin = 17;
config.rx_pin = 16;

// Initialize
transport_init(&uart->base, &config);

// Open (connect)
transport_connect(&uart->base, NULL, 0);

// Send/receive data
transport_write(&uart->base, data, len, &written, 1000);
transport_read(&uart->base, buffer, sizeof(buffer), &read, 1000);

// Cleanup
transport_disconnect(&uart->base);
transport_deinit(&uart->base);
uart_transport_destroy(uart);
```

### USB CDC

```c
#include "transport.h"
#include "transport_usb.h"

// Create transport
usb_transport_t *usb = usb_transport_create();

// Configure
usb_transport_config_t config;
usb_transport_config_default(&config);

// Initialize
transport_init(&usb->base, &config);

// Open and wait for host connection
transport_connect(&usb->base, NULL, 10000);

// Send/receive data
transport_write(&usb->base, data, len, &written, 1000);
transport_read(&usb->base, buffer, sizeof(buffer), &read, 1000);

// Cleanup
transport_disconnect(&usb->base);
transport_deinit(&usb->base);
usb_transport_destroy(usb);
```

### Using the Manager (Single Active Transport)

```c
#include "transport_manager.h"

// Initialize manager
transport_manager_init(NULL);

// Register transports
transport_handle_t h_spp = transport_manager_register(&spp->base, "spp");
transport_handle_t h_ble = transport_manager_register(&ble->base, "ble");
transport_handle_t h_tcp = transport_manager_register(&tcp->base, "tcp");

// Set active transport
transport_manager_set_active(h_ble);

// Use via manager (uses active transport)
transport_manager_connect("address", 5000);
transport_manager_write(data, len, NULL, 1000);
transport_manager_read(buffer, len, &read, 1000);

// Switch transport at runtime
transport_manager_set_active(h_spp);

// Get statistics
transport_stats_t stats;
transport_manager_get_stats(h_ble, &stats);
printf("Bytes sent: %llu\n", stats.bytes_sent);
```

### Simultaneous Multi-Transport Usage

The manager supports using **all transports simultaneously** - perfect for gateways
that receive data from one interface and forward to others.

```c
// Read/write to specific transports by handle
transport_manager_write_handle(h_tcp, data, len, NULL, 1000);
transport_manager_read_handle(h_spp, buffer, len, &read, 1000);

// Broadcast to ALL connected transports
size_t sent = transport_manager_broadcast(data, len, TRANSPORT_HANDLE_INVALID, 1000);

// Broadcast to all EXCEPT the source (for forwarding)
transport_manager_broadcast(data, len, source_handle, 1000);

// Read from ANY transport that has data
transport_handle_t source;
transport_manager_read_any(buffer, len, &read, &source, 1000);
printf("Data from: %d\n", source);

// Forward data from one transport to another (or to all)
transport_manager_forward(h_spp, h_tcp, buffer, sizeof(buffer), &forwarded, 1000);
transport_manager_forward(h_spp, TRANSPORT_HANDLE_INVALID, buffer, sizeof(buffer), &forwarded, 1000);  // Broadcast

// Poll all transports for available data
transport_poll_result_t results[8];
size_t count = transport_manager_poll(results, 8);
for (size_t i = 0; i < count; i++) {
    printf("Transport %d has %zu bytes\n", results[i].handle, results[i].bytes_available);
}

// Check connection status
size_t connected = transport_manager_connected_count();
bool is_conn = transport_manager_is_connected(h_ble);
```

### Gateway Example (Receive from any, forward to all)

```c
void gateway_task(void *pvParameters) {
    uint8_t buffer[256];
    size_t bytes_read;
    transport_handle_t source;
    
    while (1) {
        // Read from ANY transport
        if (transport_manager_read_any(buffer, sizeof(buffer), &bytes_read, &source, 100) == TRANSPORT_OK) {
            printf("Received %zu bytes from transport %d\n", bytes_read, source);
            
            // Forward to ALL OTHER transports
            size_t sent = transport_manager_broadcast(buffer, bytes_read, source, 1000);
            printf("Forwarded to %zu transports\n", sent);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

### Global Data Callback

```c
// Called automatically when data arrives on ANY transport
void on_data(transport_handle_t handle, const uint8_t *data, size_t len, void *user_data) {
    printf("Transport %d received %zu bytes\n", handle, len);
    // Auto-forward to all others
    transport_manager_broadcast(data, len, handle, 1000);
}

// Register the callback
transport_manager_set_data_callback(on_data, NULL);
```

## Events

The system supports callbacks for events:

```c
void my_callback(transport_t *transport, const transport_event_t *event, void *user_data) {
    switch (event->type) {
        case TRANSPORT_EVENT_CONNECTED:
            printf("Connected!\n");
            break;
        case TRANSPORT_EVENT_DISCONNECTED:
            printf("Disconnected\n");
            break;
        case TRANSPORT_EVENT_DATA_RECEIVED:
            printf("Received %zu bytes\n", event->data_received.len);
            break;
        case TRANSPORT_EVENT_ERROR:
            printf("Error: %s\n", event->error.message);
            break;
    }
}

// Register callback in configuration
config.base.event_callback = my_callback;
config.base.user_data = my_context;
```

## Adding New Backends

To create a new backend (e.g., WiFi UDP):

1. Create header `transport_wifi_udp.h`:
```c
typedef struct {
    transport_t base;
    wifi_udp_config_t config;
    // specific data...
} wifi_udp_transport_t;

wifi_udp_transport_t *wifi_udp_transport_create(void);
void wifi_udp_transport_destroy(wifi_udp_transport_t *transport);
```

2. Implement the vtable in `transport_wifi_udp.c`:
```c
static const transport_vtable_t wifi_udp_vtable = {
    .init       = wifi_udp_init,
    .deinit     = wifi_udp_deinit,
    .connect    = wifi_udp_connect,
    .disconnect = wifi_udp_disconnect,
    .write      = wifi_udp_write,
    .read       = wifi_udp_read,
    .available  = wifi_udp_available,
    .flush      = wifi_udp_flush,
    .get_state  = wifi_udp_get_state,
    .get_type   = wifi_udp_get_type,
    .get_info   = wifi_udp_get_info,
    .set_option = wifi_udp_set_option,
    .get_option = wifi_udp_get_option,
};

wifi_udp_transport_t *wifi_udp_transport_create(void) {
    wifi_udp_transport_t *t = calloc(1, sizeof(wifi_udp_transport_t));
    t->base.vtable = &wifi_udp_vtable;
    t->base.state = TRANSPORT_STATE_UNINITIALIZED;
    return t;
}
```

## ESP-IDF Configuration

In `sdkconfig` or `menuconfig`:

```
# Enable Bluetooth
CONFIG_BT_ENABLED=y
CONFIG_BT_BLUEDROID_ENABLED=y

# For Bluetooth Classic
CONFIG_BT_CLASSIC_ENABLED=y
CONFIG_BT_SPP_ENABLED=y

# For BLE
CONFIG_BT_BLE_ENABLED=y
CONFIG_BT_GATTS_ENABLE=y

# For USB CDC
CONFIG_TINYUSB_CDC_ENABLED=y
```

## Supported Transports

| Transport | Type | Description |
|-----------|------|-------------|
| Bluetooth SPP | `TRANSPORT_TYPE_BT_CLASSIC` | Bluetooth Classic Serial Port Profile |
| BLE | `TRANSPORT_TYPE_BLE` | Bluetooth Low Energy with custom GATT service |
| WiFi TCP | `TRANSPORT_TYPE_WIFI_TCP` | TCP socket over WiFi (client/server) |
| UART | `TRANSPORT_TYPE_UART` | Hardware UART serial |
| USB CDC | `TRANSPORT_TYPE_CUSTOM` | USB CDC ACM (virtual serial) |

## Documentation Notice

> This README.md and the project documentation were generated with the assistance of artificial intelligence.  
> The use of AI is strictly limited to documentation; all source code was written manually.

## License

See LICENSE file for details.
