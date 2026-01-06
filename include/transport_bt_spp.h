/**
 * @file transport_bt_spp.h
 * @brief Backend de transporte para Bluetooth Classic SPP (Serial Port Profile)
 * 
 * Este módulo implementa a interface de transporte usando Bluetooth Classic
 * com o perfil SPP para comunicação serial sem fio.
 */

#ifndef TRANSPORT_BT_SPP_H
#define TRANSPORT_BT_SPP_H

#include "transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opções específicas do Bluetooth SPP
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
 * @brief Modos de segurança para Bluetooth
 */
typedef enum {
    BT_SPP_SEC_NONE = 0,
    BT_SPP_SEC_AUTHENTICATE,
    BT_SPP_SEC_ENCRYPT,
    BT_SPP_SEC_AUTHENTICATE_ENCRYPT,
} bt_spp_security_t;

/**
 * @brief Papel do dispositivo na conexão SPP
 */
typedef enum {
    BT_SPP_ROLE_MASTER = 0,
    BT_SPP_ROLE_SLAVE,
} bt_spp_role_t;

/**
 * @brief Configuração específica para Bluetooth SPP
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
 * @brief Estrutura do transporte Bluetooth SPP
 * 
 * Esta estrutura estende transport_t para incluir dados específicos
 * do Bluetooth Classic SPP.
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
 * @brief Cria uma instância do transporte Bluetooth SPP
 * 
 * @return Ponteiro para o transporte criado, ou NULL em caso de erro
 */
bt_spp_transport_t *bt_spp_transport_create(void);

/**
 * @brief Destrói uma instância do transporte Bluetooth SPP
 * 
 * @param transport Ponteiro para o transporte a ser destruído
 */
void bt_spp_transport_destroy(bt_spp_transport_t *transport);

/**
 * @brief Obtém configuração padrão para Bluetooth SPP
 * 
 * @param config Ponteiro para estrutura de configuração a ser preenchida
 */
void bt_spp_config_default(bt_spp_config_t *config);

/**
 * @brief Obtém a vtable do transporte Bluetooth SPP
 * 
 * @return Ponteiro para a vtable
 */
const transport_vtable_t *bt_spp_get_vtable(void);

/**
 * @brief Inicia modo de escuta (servidor)
 * 
 * @param transport Ponteiro para o transporte
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t bt_spp_listen(bt_spp_transport_t *transport);

/**
 * @brief Para o modo de escuta
 * 
 * @param transport Ponteiro para o transporte
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t bt_spp_stop_listen(bt_spp_transport_t *transport);

/**
 * @brief Obtém endereço do dispositivo remoto conectado
 * 
 * @param transport Ponteiro para o transporte
 * @param addr Buffer para armazenar o endereço (6 bytes)
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t bt_spp_get_remote_address(bt_spp_transport_t *transport, uint8_t *addr);

/**
 * @brief Converte endereço Bluetooth para string
 * 
 * @param addr Endereço Bluetooth (6 bytes)
 * @param str Buffer para string (mínimo 18 bytes)
 * @param len Tamanho do buffer
 * @return Ponteiro para a string, ou NULL em caso de erro
 */
char *bt_addr_to_str(const uint8_t *addr, char *str, size_t len);

/**
 * @brief Converte string para endereço Bluetooth
 * 
 * @param str String no formato "XX:XX:XX:XX:XX:XX"
 * @param addr Buffer para o endereço (6 bytes)
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t bt_str_to_addr(const char *str, uint8_t *addr);

#ifdef __cplusplus
}
#endif

#endif /* TRANSPORT_BT_SPP_H */