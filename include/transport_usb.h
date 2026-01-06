/**
 * @file transport_usb.h
 * @brief Transport backend for USB CDC (Communication Device Class)
 * 
 * This module implements the transport interface using USB CDC
 * for serial communication over USB.
 */

#ifndef TRANSPORT_USB_H
#define TRANSPORT_USB_H

#include "transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief USB CDC specific options
 */
typedef enum {
    USB_OPT_LINE_CODING = 0x500,
    USB_OPT_DTR,
    USB_OPT_RTS,
} usb_transport_option_t;

/**
 * @brief USB CDC line coding structure
 */
typedef struct {
    uint32_t baud_rate;
    uint8_t stop_bits;
    uint8_t parity;
    uint8_t data_bits;
} usb_line_coding_t;

/**
 * @brief USB CDC configuration
 */
typedef struct {
    transport_config_t base;
    usb_line_coding_t line_coding;
} usb_transport_config_t;

/**
 * @brief USB CDC transport structure
 */
typedef struct {
    transport_t base;
    usb_transport_config_t config;
    bool is_open;
    bool dtr;
    bool rts;
    void *internal;
} usb_transport_t;

/**
 * @brief Creates a USB CDC transport instance
 * 
 * @return Pointer to the created transport, or NULL on error
 */
usb_transport_t *usb_transport_create(void);

/**
 * @brief Destroys a USB CDC transport instance
 * 
 * @param transport Pointer to the transport to destroy
 */
void usb_transport_destroy(usb_transport_t *transport);

/**
 * @brief Gets default configuration for USB CDC
 * 
 * @param config Pointer to configuration structure to fill
 */
void usb_transport_config_default(usb_transport_config_t *config);

/**
 * @brief Gets the USB CDC transport vtable
 * 
 * @return Pointer to the vtable
 */
const transport_vtable_t *usb_transport_get_vtable(void);

/**
 * @brief Checks if USB host is connected (DTR signal)
 * 
 * @param transport Pointer to the transport
 * @return true if connected, false otherwise
 */
bool usb_transport_is_host_connected(usb_transport_t *transport);

/**
 * @brief Gets the current line coding
 * 
 * @param transport Pointer to the transport
 * @param line_coding Pointer to store the line coding
 * @return TRANSPORT_OK on success
 */
transport_err_t usb_transport_get_line_coding(usb_transport_t *transport, 
                                               usb_line_coding_t *line_coding);

/**
 * @brief Waits for USB host connection
 * 
 * @param transport Pointer to the transport
 * @param timeout_ms Timeout in milliseconds (0 = wait forever)
 * @return TRANSPORT_OK on success, TRANSPORT_ERR_TIMEOUT on timeout
 */
transport_err_t usb_transport_wait_for_host(usb_transport_t *transport, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* TRANSPORT_USB_H */
