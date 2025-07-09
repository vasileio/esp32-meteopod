#include "uart.h"

void init_uart(app_ctx_t *ctx)
{
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_PORT, &cfg);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    /* commandQueue lives in ctx */
    uart_driver_install(UART_PORT,
                        BUF_SIZE * 2,
                        BUF_SIZE * 2,
                        20,
                        &ctx->commandQueue,
                        0);
}