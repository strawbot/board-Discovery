#ifndef CLI_TRANSPORT_USART6_H
#define CLI_TRANSPORT_USART6_H

// cli_transport_usart6.h — USART6 / RS232 transport for TimbreOS CLI

// Call once at board init to enable RXNE interrupt and make USART6 the
// default CLI transport.
void usart6_transport_init(void);

// Call from USART6_IRQHandler in stm32f4xx_it.c:
//   void USART6_IRQHandler(void) { usart6_rx_irq(); }
void usart6_rx_irq(void);

#endif // CLI_TRANSPORT_USART6_H
