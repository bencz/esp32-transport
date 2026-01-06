/**
 * @file transport.h
 * @brief Sistema de abstração para modos de transporte no ESP32
 * 
 * Este módulo fornece uma interface unificada para diferentes backends
 * de comunicação (Bluetooth Classic, BLE, WiFi, UART, etc.) usando
 * vtables para polimorfismo em C.
 * 
 * @author Claude
 * @version 1.0.0
 */

#ifndef TRANSPORT_H
#define TRANSPORT_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Códigos de erro específicos do sistema de transporte
 */
typedef enum {
    TRANSPORT_OK = 0,
    TRANSPORT_ERR_INVALID_ARG,
    TRANSPORT_ERR_NO_MEM,
    TRANSPORT_ERR_NOT_INITIALIZED,
    TRANSPORT_ERR_ALREADY_INITIALIZED,
    TRANSPORT_ERR_NOT_CONNECTED,
    TRANSPORT_ERR_ALREADY_CONNECTED,
    TRANSPORT_ERR_TIMEOUT,
    TRANSPORT_ERR_IO,
    TRANSPORT_ERR_NOT_SUPPORTED,
    TRANSPORT_ERR_BUSY,
    TRANSPORT_ERR_INTERNAL,
} transport_err_t;

/**
 * @brief Estados possíveis de um transporte
 */
typedef enum {
    TRANSPORT_STATE_UNINITIALIZED = 0,
    TRANSPORT_STATE_INITIALIZED,
    TRANSPORT_STATE_CONNECTING,
    TRANSPORT_STATE_CONNECTED,
    TRANSPORT_STATE_DISCONNECTING,
    TRANSPORT_STATE_ERROR,
} transport_state_t;

/**
 * @brief Tipos de transporte disponíveis
 */
typedef enum {
    TRANSPORT_TYPE_UNKNOWN = 0,
    TRANSPORT_TYPE_BT_CLASSIC,
    TRANSPORT_TYPE_BLE,
    TRANSPORT_TYPE_WIFI_TCP,
    TRANSPORT_TYPE_WIFI_UDP,
    TRANSPORT_TYPE_UART,
    TRANSPORT_TYPE_SPI,
    TRANSPORT_TYPE_I2C,
    TRANSPORT_TYPE_CUSTOM,
} transport_type_t;

/**
 * @brief Eventos que podem ser emitidos pelo transporte
 */
typedef enum {
    TRANSPORT_EVENT_CONNECTED = 0,
    TRANSPORT_EVENT_DISCONNECTED,
    TRANSPORT_EVENT_DATA_RECEIVED,
    TRANSPORT_EVENT_DATA_SENT,
    TRANSPORT_EVENT_ERROR,
    TRANSPORT_EVENT_STATE_CHANGED,
} transport_event_type_t;

/**
 * @brief Forward declarations
 */
typedef struct transport_s transport_t;
typedef struct transport_vtable_s transport_vtable_t;
typedef struct transport_config_s transport_config_t;
typedef struct transport_event_s transport_event_t;

/**
 * @brief Callback para eventos do transporte
 * 
 * @param transport Ponteiro para o transporte que gerou o evento
 * @param event Ponteiro para os dados do evento
 * @param user_data Dados do usuário passados no registro do callback
 */
typedef void (*transport_event_cb_t)(transport_t *transport, 
                                      const transport_event_t *event, 
                                      void *user_data);

/**
 * @brief Estrutura de evento do transporte
 */
struct transport_event_s {
    transport_event_type_t type;
    union {
        struct {
            const uint8_t *data;
            size_t len;
        } data_received;
        struct {
            size_t bytes_sent;
        } data_sent;
        struct {
            transport_err_t code;
            const char *message;
        } error;
        struct {
            transport_state_t old_state;
            transport_state_t new_state;
        } state_changed;
    };
};

/**
 * @brief Configuração base para transportes
 */
struct transport_config_s {
    const char *name;
    uint32_t timeout_ms;
    size_t rx_buffer_size;
    size_t tx_buffer_size;
    transport_event_cb_t event_callback;
    void *user_data;
};

/**
 * @brief Vtable com as operações do transporte
 * 
 * Esta estrutura define a interface que todos os backends de
 * transporte devem implementar. Funciona como uma vtable em C++.
 */
struct transport_vtable_s {
    /**
     * @brief Inicializa o transporte com configuração específica
     * @param self Ponteiro para a instância do transporte
     * @param config Configuração específica do backend
     * @return TRANSPORT_OK em caso de sucesso
     */
    transport_err_t (*init)(transport_t *self, const void *config);
    
    /**
     * @brief Desinicializa o transporte e libera recursos
     * @param self Ponteiro para a instância do transporte
     * @return TRANSPORT_OK em caso de sucesso
     */
    transport_err_t (*deinit)(transport_t *self);
    
    /**
     * @brief Estabelece conexão
     * @param self Ponteiro para a instância do transporte
     * @param address Endereço de destino (formato depende do backend)
     * @param timeout_ms Timeout em milissegundos (0 = usar padrão)
     * @return TRANSPORT_OK em caso de sucesso
     */
    transport_err_t (*connect)(transport_t *self, const char *address, uint32_t timeout_ms);
    
    /**
     * @brief Encerra conexão
     * @param self Ponteiro para a instância do transporte
     * @return TRANSPORT_OK em caso de sucesso
     */
    transport_err_t (*disconnect)(transport_t *self);
    
    /**
     * @brief Envia dados
     * @param self Ponteiro para a instância do transporte
     * @param data Ponteiro para os dados a enviar
     * @param len Tamanho dos dados em bytes
     * @param bytes_written Ponteiro para armazenar bytes escritos (pode ser NULL)
     * @param timeout_ms Timeout em milissegundos (0 = usar padrão)
     * @return TRANSPORT_OK em caso de sucesso
     */
    transport_err_t (*write)(transport_t *self, const uint8_t *data, size_t len, 
                             size_t *bytes_written, uint32_t timeout_ms);
    
    /**
     * @brief Recebe dados
     * @param self Ponteiro para a instância do transporte
     * @param buffer Buffer para armazenar dados recebidos
     * @param len Tamanho máximo a receber
     * @param bytes_read Ponteiro para armazenar bytes lidos
     * @param timeout_ms Timeout em milissegundos (0 = usar padrão)
     * @return TRANSPORT_OK em caso de sucesso
     */
    transport_err_t (*read)(transport_t *self, uint8_t *buffer, size_t len, 
                            size_t *bytes_read, uint32_t timeout_ms);
    
    /**
     * @brief Verifica se há dados disponíveis para leitura
     * @param self Ponteiro para a instância do transporte
     * @param available Ponteiro para armazenar quantidade de bytes disponíveis
     * @return TRANSPORT_OK em caso de sucesso
     */
    transport_err_t (*available)(transport_t *self, size_t *available);
    
    /**
     * @brief Limpa buffers de recepção e transmissão
     * @param self Ponteiro para a instância do transporte
     * @return TRANSPORT_OK em caso de sucesso
     */
    transport_err_t (*flush)(transport_t *self);
    
    /**
     * @brief Obtém o estado atual do transporte
     * @param self Ponteiro para a instância do transporte
     * @return Estado atual
     */
    transport_state_t (*get_state)(transport_t *self);
    
    /**
     * @brief Obtém o tipo do transporte
     * @param self Ponteiro para a instância do transporte
     * @return Tipo do transporte
     */
    transport_type_t (*get_type)(transport_t *self);
    
    /**
     * @brief Obtém informações sobre o transporte
     * @param self Ponteiro para a instância do transporte
     * @param info Buffer para armazenar informações
     * @param len Tamanho do buffer
     * @return TRANSPORT_OK em caso de sucesso
     */
    transport_err_t (*get_info)(transport_t *self, char *info, size_t len);
    
    /**
     * @brief Configura opção específica do transporte
     * @param self Ponteiro para a instância do transporte
     * @param option Identificador da opção
     * @param value Ponteiro para o valor da opção
     * @param len Tamanho do valor
     * @return TRANSPORT_OK em caso de sucesso
     */
    transport_err_t (*set_option)(transport_t *self, int option, 
                                  const void *value, size_t len);
    
    /**
     * @brief Obtém opção específica do transporte
     * @param self Ponteiro para a instância do transporte
     * @param option Identificador da opção
     * @param value Buffer para armazenar o valor
     * @param len Ponteiro para tamanho (entrada: tamanho do buffer, saída: tamanho usado)
     * @return TRANSPORT_OK em caso de sucesso
     */
    transport_err_t (*get_option)(transport_t *self, int option, 
                                  void *value, size_t *len);
};

/**
 * @brief Estrutura base do transporte
 * 
 * Todos os backends específicos devem "herdar" desta estrutura
 * colocando-a como primeiro membro.
 */
struct transport_s {
    const transport_vtable_t *vtable;
    transport_state_t state;
    transport_event_cb_t event_callback;
    void *user_data;
    const char *name;
};

/**
 * @defgroup transport_api API de Alto Nível
 * @brief Funções de conveniência que delegam para a vtable
 * @{
 */

/**
 * @brief Inicializa um transporte
 */
static inline transport_err_t transport_init(transport_t *transport, const void *config)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->init == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->init(transport, config);
}

/**
 * @brief Desinicializa um transporte
 */
static inline transport_err_t transport_deinit(transport_t *transport)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->deinit == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->deinit(transport);
}

/**
 * @brief Conecta o transporte
 */
static inline transport_err_t transport_connect(transport_t *transport, 
                                                 const char *address, 
                                                 uint32_t timeout_ms)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->connect == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->connect(transport, address, timeout_ms);
}

/**
 * @brief Desconecta o transporte
 */
static inline transport_err_t transport_disconnect(transport_t *transport)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->disconnect == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->disconnect(transport);
}

/**
 * @brief Envia dados pelo transporte
 */
static inline transport_err_t transport_write(transport_t *transport, 
                                               const uint8_t *data, 
                                               size_t len,
                                               size_t *bytes_written, 
                                               uint32_t timeout_ms)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->write == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->write(transport, data, len, bytes_written, timeout_ms);
}

/**
 * @brief Recebe dados do transporte
 */
static inline transport_err_t transport_read(transport_t *transport, 
                                              uint8_t *buffer, 
                                              size_t len,
                                              size_t *bytes_read, 
                                              uint32_t timeout_ms)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->read == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->read(transport, buffer, len, bytes_read, timeout_ms);
}

/**
 * @brief Verifica dados disponíveis
 */
static inline transport_err_t transport_available(transport_t *transport, size_t *available)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->available == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->available(transport, available);
}

/**
 * @brief Limpa buffers do transporte
 */
static inline transport_err_t transport_flush(transport_t *transport)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->flush == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->flush(transport);
}

/**
 * @brief Obtém estado do transporte
 */
static inline transport_state_t transport_get_state(transport_t *transport)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->get_state == NULL) {
        return TRANSPORT_STATE_UNINITIALIZED;
    }
    return transport->vtable->get_state(transport);
}

/**
 * @brief Obtém tipo do transporte
 */
static inline transport_type_t transport_get_type(transport_t *transport)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->get_type == NULL) {
        return TRANSPORT_TYPE_UNKNOWN;
    }
    return transport->vtable->get_type(transport);
}

/**
 * @brief Obtém informações do transporte
 */
static inline transport_err_t transport_get_info(transport_t *transport, char *info, size_t len)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->get_info == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->get_info(transport, info, len);
}

/**
 * @brief Configura opção do transporte
 */
static inline transport_err_t transport_set_option(transport_t *transport, 
                                                    int option,
                                                    const void *value, 
                                                    size_t len)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->set_option == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->set_option(transport, option, value, len);
}

/**
 * @brief Obtém opção do transporte
 */
static inline transport_err_t transport_get_option(transport_t *transport, 
                                                    int option,
                                                    void *value, 
                                                    size_t *len)
{
    if (transport == NULL || transport->vtable == NULL || transport->vtable->get_option == NULL) {
        return TRANSPORT_ERR_INVALID_ARG;
    }
    return transport->vtable->get_option(transport, option, value, len);
}

/** @} */

/**
 * @brief Converte código de erro para string
 * @param err Código de erro
 * @return String descritiva do erro
 */
const char *transport_err_to_str(transport_err_t err);

/**
 * @brief Converte estado para string
 * @param state Estado do transporte
 * @return String descritiva do estado
 */
const char *transport_state_to_str(transport_state_t state);

/**
 * @brief Converte tipo para string
 * @param type Tipo do transporte
 * @return String descritiva do tipo
 */
const char *transport_type_to_str(transport_type_t type);

/**
 * @brief Emite um evento para o callback registrado
 * @param transport Ponteiro para o transporte
 * @param event Ponteiro para o evento
 */
void transport_emit_event(transport_t *transport, const transport_event_t *event);

/**
 * @brief Altera o estado do transporte e emite evento
 * @param transport Ponteiro para o transporte
 * @param new_state Novo estado
 */
void transport_set_state(transport_t *transport, transport_state_t new_state);

#ifdef __cplusplus
}
#endif

#endif /* TRANSPORT_H */