/**
 * @file transport_manager.h
 * @brief Gerenciador centralizado de transportes
 * 
 * Este módulo fornece uma camada de gerenciamento para múltiplos
 * transportes, permitindo fácil troca entre backends e gerenciamento
 * de ciclo de vida.
 */

#ifndef TRANSPORT_MANAGER_H
#define TRANSPORT_MANAGER_H

#include "transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Número máximo de transportes registrados
 */
#define TRANSPORT_MANAGER_MAX_TRANSPORTS    (8)

/**
 * @brief Handle para um transporte registrado
 */
typedef int transport_handle_t;

#define TRANSPORT_HANDLE_INVALID    (-1)

/**
 * @brief Callback para seleção automática de transporte
 */
typedef transport_t *(*transport_selector_cb_t)(void *user_data);

/**
 * @brief Configuração do gerenciador de transportes
 */
typedef struct {
    bool auto_reconnect;
    uint32_t reconnect_delay_ms;
    uint32_t max_reconnect_attempts;
    transport_selector_cb_t selector;
    void *selector_user_data;
} transport_manager_config_t;

/**
 * @brief Inicializa o gerenciador de transportes
 * 
 * @param config Configuração do gerenciador (NULL usa padrões)
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t transport_manager_init(const transport_manager_config_t *config);

/**
 * @brief Desinicializa o gerenciador de transportes
 * 
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t transport_manager_deinit(void);

/**
 * @brief Registra um transporte no gerenciador
 * 
 * @param transport Ponteiro para o transporte
 * @param name Nome único para identificação
 * @return Handle do transporte, ou TRANSPORT_HANDLE_INVALID em caso de erro
 */
transport_handle_t transport_manager_register(transport_t *transport, const char *name);

/**
 * @brief Remove um transporte do gerenciador
 * 
 * @param handle Handle do transporte
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t transport_manager_unregister(transport_handle_t handle);

/**
 * @brief Obtém um transporte pelo handle
 * 
 * @param handle Handle do transporte
 * @return Ponteiro para o transporte, ou NULL se não encontrado
 */
transport_t *transport_manager_get(transport_handle_t handle);

/**
 * @brief Obtém um transporte pelo nome
 * 
 * @param name Nome do transporte
 * @return Ponteiro para o transporte, ou NULL se não encontrado
 */
transport_t *transport_manager_get_by_name(const char *name);

/**
 * @brief Obtém um transporte pelo tipo
 * 
 * @param type Tipo do transporte
 * @return Ponteiro para o primeiro transporte do tipo, ou NULL se não encontrado
 */
transport_t *transport_manager_get_by_type(transport_type_t type);

/**
 * @brief Define o transporte ativo
 * 
 * @param handle Handle do transporte
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t transport_manager_set_active(transport_handle_t handle);

/**
 * @brief Obtém o transporte ativo
 * 
 * @return Ponteiro para o transporte ativo, ou NULL se nenhum
 */
transport_t *transport_manager_get_active(void);

/**
 * @brief Conecta o transporte ativo
 * 
 * @param address Endereço de destino
 * @param timeout_ms Timeout em milissegundos
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t transport_manager_connect(const char *address, uint32_t timeout_ms);

/**
 * @brief Desconecta o transporte ativo
 * 
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t transport_manager_disconnect(void);

/**
 * @brief Envia dados pelo transporte ativo
 * 
 * @param data Dados a enviar
 * @param len Tamanho dos dados
 * @param bytes_written Ponteiro para bytes escritos (pode ser NULL)
 * @param timeout_ms Timeout em milissegundos
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t transport_manager_write(const uint8_t *data, size_t len, 
                                         size_t *bytes_written, uint32_t timeout_ms);

/**
 * @brief Recebe dados do transporte ativo
 * 
 * @param buffer Buffer para dados
 * @param len Tamanho máximo
 * @param bytes_read Ponteiro para bytes lidos
 * @param timeout_ms Timeout em milissegundos
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t transport_manager_read(uint8_t *buffer, size_t len, 
                                        size_t *bytes_read, uint32_t timeout_ms);

/**
 * @brief Lista todos os transportes registrados
 * 
 * @param handles Array para armazenar handles
 * @param max_handles Tamanho máximo do array
 * @return Número de transportes encontrados
 */
size_t transport_manager_list(transport_handle_t *handles, size_t max_handles);

/**
 * @brief Obtém estatísticas de um transporte
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
 * @brief Obtém estatísticas de um transporte
 * 
 * @param handle Handle do transporte
 * @param stats Ponteiro para estrutura de estatísticas
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t transport_manager_get_stats(transport_handle_t handle, 
                                             transport_stats_t *stats);

/**
 * @brief Reseta estatísticas de um transporte
 * 
 * @param handle Handle do transporte
 * @return TRANSPORT_OK em caso de sucesso
 */
transport_err_t transport_manager_reset_stats(transport_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* TRANSPORT_MANAGER_H */