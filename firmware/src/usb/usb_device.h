#ifndef USB_DEVICE_H
#define USB_DEVICE_H

#include <stdbool.h>
#include <stdint.h>
extern volatile uint8_t USB_configured, USB_suspended;
extern volatile uint8_t USB_sleepPending;

void USB_Init(void);

void USB_Poll(void);

void USB_SuspendPoll(void);

void USB_ResumeData(void);

/* Returns false while the USB SIE or a completion still owns old DMA. */

bool USB_Flush(void);
#endif
