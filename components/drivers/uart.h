#include "driver/uart.h"
#include "app_context.h"

#define UART_PORT            UART_NUM_1
#define BUF_SIZE             1024
#define UART_RX_PIN          16
#define UART_TX_PIN          17

/**
 * @brief       initialise UART for receiving commands.
 *
 * This function configures the UART peripheral with the specified
 * parameters (baud rate, data bits, parity, stop bits, and flow control),
 * sets the TX and RX pins, and installs the UART driver with a queue
 * for incoming data events. The driver will post events into the
 * provided command queue in the application context.
 *
 * @param[in,out] ctx  Pointer to the application context containing the
 *                      commandQueue handle where UART events will be posted.
 *
 * @note  The UART port, pins, buffer size, and queue length are defined
 *        by the macros UART_PORT, UART_TX_PIN, UART_RX_PIN, and BUF_SIZE.
 */
void init_uart(app_ctx_t *ctx);