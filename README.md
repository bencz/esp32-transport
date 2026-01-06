# ESP32 Transport Abstraction Layer

Sistema de abstração para modos de transporte no ESP32, implementado em C seguindo as melhores práticas para sistemas embarcados.

## Arquitetura

O sistema utiliza o padrão de vtables para implementar polimorfismo em C, permitindo que diferentes backends de comunicação sejam utilizados de forma intercambiável através de uma interface comum.

```
                    +-------------------+
                    |   transport_t     |
                    |   (interface)     |
                    +--------+----------+
                             |
            +----------------+----------------+
            |                |                |
   +--------v-------+ +------v--------+ +----v-----------+
   | bt_spp_transport| | ble_transport | | uart_transport |
   | (Bluetooth SPP) | | (BLE)         | | (UART)         |
   +-----------------+ +---------------+ +----------------+
```

## Estrutura do Projeto

```
esp32_transport/
├── include/
│   ├── transport.h           # Interface base e tipos
│   ├── transport_bt_spp.h    # Backend Bluetooth Classic SPP
│   ├── transport_ble.h       # Backend Bluetooth Low Energy
│   └── transport_manager.h   # Gerenciador de transportes
├── src/
│   ├── transport.c           # Implementação base
│   ├── transport_bt_spp.c    # Implementação SPP
│   ├── transport_ble.c       # Implementação BLE
│   └── transport_manager.c   # Implementação do gerenciador
├── examples/
│   └── main.c                # Exemplos de uso
├── CMakeLists.txt
└── README.md
```

## Conceitos Principais

### Vtable (Virtual Table)

Cada backend implementa uma vtable com ponteiros para suas funções:

```c
struct transport_vtable_s {
    transport_err_t (*init)(transport_t *self, const void *config);
    transport_err_t (*deinit)(transport_t *self);
    transport_err_t (*connect)(transport_t *self, const char *address, uint32_t timeout_ms);
    transport_err_t (*disconnect)(transport_t *self);
    transport_err_t (*write)(transport_t *self, const uint8_t *data, size_t len, ...);
    transport_err_t (*read)(transport_t *self, uint8_t *buffer, size_t len, ...);
    // ... outras operações
};
```

### Estrutura Base

Todos os backends "herdam" da estrutura base:

```c
struct transport_s {
    const transport_vtable_t *vtable;  // Ponteiro para vtable
    transport_state_t state;
    transport_event_cb_t event_callback;
    void *user_data;
    const char *name;
};

// Backend específico "herda" colocando base como primeiro membro
typedef struct {
    transport_t base;           // "Herança"
    bt_spp_config_t config;     // Dados específicos
    // ...
} bt_spp_transport_t;
```

### Polimorfismo

O código cliente trabalha com ponteiros para `transport_t`:

```c
void process_any_transport(transport_t *transport) {
    // Funciona com qualquer backend!
    transport_connect(transport, "address", 5000);
    transport_write(transport, data, len, NULL, 1000);
    transport_disconnect(transport);
}

// Uso
bt_spp_transport_t *spp = bt_spp_transport_create();
ble_transport_t *ble = ble_transport_create();

process_any_transport(&spp->base);  // Bluetooth Classic
process_any_transport(&ble->base);  // BLE
```

## Uso Básico

### Bluetooth Classic SPP

```c
#include "transport.h"
#include "transport_bt_spp.h"

// Criar transporte
bt_spp_transport_t *spp = bt_spp_transport_create();

// Configurar
bt_spp_config_t config;
bt_spp_config_default(&config);
config.device_name = "MeuESP32";

// Inicializar
transport_init(&spp->base, &config);

// Modo servidor (aguardar conexões)
bt_spp_listen(spp);

// Ou modo cliente (conectar a dispositivo)
transport_connect(&spp->base, "AA:BB:CC:DD:EE:FF", 5000);

// Enviar/receber dados
transport_write(&spp->base, data, len, &written, 1000);
transport_read(&spp->base, buffer, sizeof(buffer), &read, 1000);

// Limpar
transport_disconnect(&spp->base);
transport_deinit(&spp->base);
bt_spp_transport_destroy(spp);
```

### Bluetooth Low Energy

```c
#include "transport.h"
#include "transport_ble.h"

// Criar transporte
ble_transport_t *ble = ble_transport_create();

// Configurar
ble_config_t config;
ble_config_default(&config);
config.device_name = "MeuESP32_BLE";
config.mtu = 512;

// Inicializar
transport_init(&ble->base, &config);

// Iniciar advertising (periférico)
ble_start_advertising(ble);

// Enviar/receber (após conexão)
transport_write(&ble->base, data, len, &written, 1000);
transport_read(&ble->base, buffer, sizeof(buffer), &read, 1000);

// Limpar
ble_stop_advertising(ble);
transport_deinit(&ble->base);
ble_transport_destroy(ble);
```

### Usando o Gerenciador

```c
#include "transport_manager.h"

// Inicializar gerenciador
transport_manager_init(NULL);

// Registrar transportes
transport_handle_t h_spp = transport_manager_register(&spp->base, "spp");
transport_handle_t h_ble = transport_manager_register(&ble->base, "ble");

// Definir transporte ativo
transport_manager_set_active(h_ble);

// Usar via gerenciador (usa transporte ativo)
transport_manager_connect("address", 5000);
transport_manager_write(data, len, NULL, 1000);
transport_manager_read(buffer, len, &read, 1000);

// Trocar transporte em runtime
transport_manager_set_active(h_spp);

// Obter estatísticas
transport_stats_t stats;
transport_manager_get_stats(h_ble, &stats);
printf("Bytes enviados: %llu\n", stats.bytes_sent);
```

## Eventos

O sistema suporta callbacks para eventos:

```c
void my_callback(transport_t *transport, const transport_event_t *event, void *user_data) {
    switch (event->type) {
        case TRANSPORT_EVENT_CONNECTED:
            printf("Conectado!\n");
            break;
        case TRANSPORT_EVENT_DISCONNECTED:
            printf("Desconectado\n");
            break;
        case TRANSPORT_EVENT_DATA_RECEIVED:
            printf("Recebidos %zu bytes\n", event->data_received.len);
            break;
        case TRANSPORT_EVENT_ERROR:
            printf("Erro: %s\n", event->error.message);
            break;
    }
}

// Registrar callback na configuração
config.base.event_callback = my_callback;
config.base.user_data = my_context;
```

## Adicionando Novos Backends

Para criar um novo backend (ex: WiFi TCP):

1. Criar header `transport_wifi_tcp.h`:
```c
typedef struct {
    transport_t base;
    wifi_tcp_config_t config;
    // dados específicos...
} wifi_tcp_transport_t;

wifi_tcp_transport_t *wifi_tcp_transport_create(void);
void wifi_tcp_transport_destroy(wifi_tcp_transport_t *transport);
```

2. Implementar a vtable em `transport_wifi_tcp.c`:
```c
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

wifi_tcp_transport_t *wifi_tcp_transport_create(void) {
    wifi_tcp_transport_t *t = calloc(1, sizeof(wifi_tcp_transport_t));
    t->base.vtable = &wifi_tcp_vtable;
    t->base.state = TRANSPORT_STATE_UNINITIALIZED;
    return t;
}
```

## Configuração ESP-IDF

No `sdkconfig` ou `menuconfig`:

```
# Habilitar Bluetooth
CONFIG_BT_ENABLED=y
CONFIG_BT_BLUEDROID_ENABLED=y

# Para Bluetooth Classic
CONFIG_BT_CLASSIC_ENABLED=y
CONFIG_BT_SPP_ENABLED=y

# Para BLE
CONFIG_BT_BLE_ENABLED=y
CONFIG_BT_GATTS_ENABLE=y
```
