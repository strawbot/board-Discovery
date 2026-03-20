#ifndef USB_INIT_H_
#define USB_INIT_H_

// cdc_transport_init — call once from main() after all MX peripheral inits.
//
// Configures PA11/PA12 for OTG_FS, enables the USB peripheral clock,
// initialises TinyUSB, and starts the usb_action pump in the tea.c queue.
// After this call the USB device is visible to the host and the CDC serial
// port accepts CLI input just like USART6 and Telnet.

void cdc_transport_init(void);

// usb_action — tea.c action that drives tud_task().
// Declared here so stm32f4xx_it.c or other modules can reference it if needed.
void usb_action(void);

#endif // USB_INIT_H_
