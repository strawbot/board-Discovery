#ifndef CLI_TRANSPORT_USART6_H
#define CLI_TRANSPORT_USART6_H

// cli_transport_usart6.h — USART6 / RS232 transport for TimbreOS CLI

// Call once at board init to enable RXNE interrupt and make USART6 the
// default CLI transport.
void usart6_transport_init(void);

// Call from USART6_IRQHandler — guard each call on the matching flag + IT:
//   if (LL_USART_IsActiveFlag_RXNE(USART6) && LL_USART_IsEnabledIT_RXNE(USART6))
//       usart6_rx_irq();
//   if (LL_USART_IsActiveFlag_TXE(USART6)  && LL_USART_IsEnabledIT_TXE(USART6))
//       usart6_tx_irq();
//   if (LL_USART_IsActiveFlag_TC(USART6)   && LL_USART_IsEnabledIT_TC(USART6))
//       usart6_tc_irq();
void usart6_rx_irq(void);
void usart6_tx_irq(void);
void usart6_tc_irq(void);

#endif // CLI_TRANSPORT_USART6_H
