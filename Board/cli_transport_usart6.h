#ifndef CLI_TRANSPORT_USART6_H
#define CLI_TRANSPORT_USART6_H

// cli_transport_usart6.h — USART6 / RS232 transport for TimbreOS CLI

// Call once at board init (after MX_DMA_Init + MX_USART6_UART_Init) to
// configure the DMA ring buffer and make USART6 the default CLI transport.
void usart6_transport_init(void);

// Wire into DMA2_Stream1_IRQHandler — drains the DMA ring buffer on HT/TC.
void usart6_dma_irq(void);

// Wire into USART6_IRQHandler — handles IDLE drain + TX (TXE + TC flags).
void usart6_irq(void);

#endif // CLI_TRANSPORT_USART6_H
