#ifndef BUTTON_H
#define BUTTON_H

// button_init — configure EXTI0 for PA0 in IT mode and enable EXTI0_IRQn.
//
// PA0 is already input / no-pull and EXTI0 is already mapped to PORTA by
// MX_GPIO_Init().  button_init() promotes EXTI0 from EVENT to IT mode so
// that B1 presses generate EXTI0_IRQHandler interrupts.
//
// Call once after Tea is initialised.
void button_init(void);

// button_exti0_isr — ISR trampoline for B1 (PA0 / EXTI0 rising edge).
// Must be called from EXTI0_IRQHandler after the EXTI pending flag is cleared,
// only when EXTI0 is currently routed to PA0 (see SYSCFG EXTICR dispatch in
// stm32f4xx_it.c).  Debounces in ISR context; defers work to the event loop.
void button_exti0_isr(void);

#endif // BUTTON_H
