/**
 * @file transport_ble.h
 * @brief Backend de transporte para Bluetooth Low Energy (BLE)
 * 
 * Este módulo implementa a interface de transporte usando BLE
 * com um serviço GATT customizado para comunicação bidirecional.
 */

#ifndef TRANSPORT_BLE_H
#define TRANSPORT_BLE_H

#include "transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief UUIDs padrão para o serviço de transporte BLE
 */
#define BLE_TRANSPORT_SERVICE_UUID      0x00FF
#define BLE_TRANSPORT_CHAR_TX_UUID      0xFF01
#define BLE_TRANSPORT_CHAR_RX_UUID      0xFF02

/**
 * @brief Opções específicas do BLE
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
 * @brief Papel do dispositivo BLE
 */
typedef enum {
    BLE_ROLE_PERIPHERAL = 0,
    BLE_ROLE_CENTRAL,
} ble_role_t;

/**
 * @brief Configuração de advertising
 */
typedef struct {
    uint16_t min_interval;
    uint16_t max_interval;
    bool connectable;
    bool scannable;
} ble_adv_config_t;

/**
 * @brief Configuração de conexão
 */
typedef struct {
    uint16_t min_interval;
    uint16_t max_interval;
    uint16_t latency;
    uint16_t timeout;
} ble_conn_config_t;

/**
 * @brief Configuração específica para BLE
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
 * @brief Estrutura do transporte BLE
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
 * @brief Cria uma instância do transporte BLE
 * 
 * @return Ponteiro para o transporte criado, ou NULL em caso de erro
 */
ble_transport_t *ble_transport_create(void);

/**
 * @brief Destrói uma instância do transporte BLE
 * 
 * @param transport Ponteiro para o transporte a ser destruído
 */
void ble_transport_destroy(ble_transport_t *transport);

/**
 * @brief Obtém configuração padrão para BLE
 * 
 * @param config Ponteiro para estrutura de configuração a ser preenchida
 */
void ble_config_default(ble_config_t *config);

/**
 * @brief Obtém a vtable do transporte BLE
 * 
 * @return Ponteiro para a vtable
 */
const transport_vtable_t *ble_get_vtable(void);

/**
 * @brief Inicia advertising (modo periférico)
 * 
 * @param transport Ponteiro para o transporte
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t ble_start_advertising(ble_transport_t *transport);

/**
 * @brief Para advertising
 * 
 * @param transport Ponteiro para o transporte
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t ble_stop_advertising(ble_transport_t *transport);

/**
 * @brief Inicia scan (modo central)
 * 
 * @param transport Ponteiro para o transporte
 * @param duration_sec Duração do scan em segundos (0 = indefinido)
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t ble_start_scan(ble_transport_t *transport, uint32_t duration_sec);

/**
 * @brief Para scan
 * 
 * @param transport Ponteiro para o transporte
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t ble_stop_scan(ble_transport_t *transport);

/**
 * @brief Obtém MTU atual da conexão
 * 
 * @param transport Ponteiro para o transporte
 * @return MTU atual, ou 0 se não conectado
 */
uint16_t ble_get_mtu(ble_transport_t *transport);

/**
 * @brief Solicita atualização de MTU
 * 
 * @param transport Ponteiro para o transporte
 * @param mtu MTU desejado
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t ble_request_mtu(ble_transport_t *transport, uint16_t mtu);

/**
 * @brief Obtém endereço do peer conectado
 * 
 * @param transport Ponteiro para o transporte
 * @param addr Buffer para armazenar o endereço (6 bytes)
 * @param addr_type Ponteiro para armazenar o tipo de endereço (pode ser NULL)
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t ble_get_peer_address(ble_transport_t *transport, uint8_t *addr, uint8_t *addr_type);

/**
 * @brief Callback para dispositivos descobertos durante scan
 */
typedef void (*ble_scan_result_cb_t)(const uint8_t *addr, uint8_t addr_type, 
                                      int8_t rssi, const char *name, void *user_data);

/**
 * @brief Registra callback para resultados de scan
 * 
 * @param transport Ponteiro para o transporte
 * @param callback Função de callback
 * @param user_data Dados do usuário
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t ble_set_scan_callback(ble_transport_t *transport, 
                                       ble_scan_result_cb_t callback, 
                                       void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* TRANSPORT_BLE_H */