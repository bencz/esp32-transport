/**
 * @file transport_uart.h
 * @brief Transport backend for UART serial communication
 * 
 * This module implements the transport interface using UART
 * for serial communication.
 */

#ifndef TRANSPORT_UART_H
#define TRANSPORT_UART_H

#include "transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief UART specific options
 */
typedef enum {
    UART_OPT_BAUD_RATE = 0x400,
    UART_OPT_DATA_BITS,
    UART_OPT_PARITY,
    UART_OPT_STOP_BITS,
    UART_OPT_FLOW_CTRL,
    UART_OPT_RX_FLOW_CTRL_THRESH,
} uart_transport_option_t;

/**
 * @brief UART parity modes
 */
typedef enum {
    UART_TRANSPORT_PARITY_NONE = 0,
    UART_TRANSPORT_PARITY_ODD,
    UART_TRANSPORT_PARITY_EVEN,
} uart_transport_parity_t;

/**
 * @brief UART stop bits
 */
typedef enum {
    UART_TRANSPORT_STOP_BITS_1 = 1,
    UART_TRANSPORT_STOP_BITS_1_5,
    UART_TRANSPORT_STOP_BITS_2,
} uart_transport_stop_bits_t;

/**
 * @brief UART flow control modes
 */
typedef enum {
    UART_TRANSPORT_FLOW_CTRL_NONE = 0,
    UART_TRANSPORT_FLOW_CTRL_RTS,
    UART_TRANSPORT_FLOW_CTRL_CTS,
    UART_TRANSPORT_FLOW_CTRL_RTS_CTS,
} uart_transport_flow_ctrl_t;

/**
 * @brief UART configuration
 */
typedef struct {
    transport_config_t base;
    int uart_num;
    int baud_rate;
    int data_bits;
    uart_transport_parity_t parity;
    uart_transport_stop_bits_t stop_bits;
    uart_transport_flow_ctrl_t flow_ctrl;
    int rx_flow_ctrl_thresh;
    int tx_pin;
    int rx_pin;
    int rts_pin;
    int cts_pin;
} uart_transport_config_t;

/**
 * @brief UART transport structure
 */
typedef struct {
    transport_t base;
    uart_transport_config_t config;
    bool is_open;
    void *internal;
} uart_transport_t;

/**
 * @brief Creates a UART transport instance
 * 
 * @return Pointer to the created transport, or NULL on error
 */
uart_transport_t *uart_transport_create(void);

/**
 * @brief Destroys a UART transport instance
 * 
 * @param transport Pointer to the transport to destroy
 */
void uart_transport_destroy(uart_transport_t *transport);

/**
 * @brief Gets default configuration for UART
 * 
 * @param config Pointer to configuration structure to fill
 */
void uart_transport_config_default(uart_transport_config_t *config);

/**
 * @brief Gets the UART transport vtable
 * 
 * @return Pointer to the vtable
 */
const transport_vtable_t *uart_transport_get_vtable(void);

/**
 * @brief Sets the baud rate
 * 
 * @param transport Pointer to the transport
 * @param baud_rate New baud rate
 * @return TRANSPORT_OK on success
 */
transport_err_t uart_transport_set_baudrate(uart_transport_t *transport, int baud_rate);

/**
 * @brief Gets the current baud rate
 * 
 * @param transport Pointer to the transport
 * @return Current baud rate, or -1 on error
 */
int uart_transport_get_baudrate(uart_transport_t *transport);

/**
 * @brief Sends a break signal
 * 
 * @param transport Pointer to the transport
 * @param duration_ms Duration of break in milliseconds
 * @return TRANSPORT_OK on success
 */
transport_err_t uart_transport_send_break(uart_transport_t *transport, int duration_ms);

/**
 * @brief Waits for TX FIFO to be empty
 * 
 * @param transport Pointer to the transport
 * @param timeout_ms Timeout in milliseconds
 * @return TRANSPORT_OK on success
 */
transport_err_t uart_transport_wait_tx_done(uart_transport_t *transport, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* TRANSPORT_UART_H */
