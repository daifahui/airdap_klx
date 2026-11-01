#ifndef USB_DESCRIPTORS_H
#define USB_DESCRIPTORS_H

#include <stdint.h>
#include "dap/dap_config.h"
#include "version.h"
#include "usb/usb_requests.h"

#define USB_STRING_MANUFACTURER    1U
#define USB_STRING_PRODUCT         2U
#define USB_STRING_SERIAL          3U
#define USB_STRING_DAP_BULK        4U
#define USB_STRING_DAP_HID         5U
#define USB_CDC_NOTIFY_PACKET_SIZE 16U
#define USB_CDC_NOTIFY_INTERVAL_MS 16U
#define CDC_CONTROL_IF             (USB_HAS_DAP ? 1 : 0)
#define CDC_OUT_EP                 0x05U
#define CDC_IN_EP                  0x86U
#define CDC_NOTIFY_EP              0x84U
#define USB_HAS_RF                 APP_HAS_RF_USB
#define RF_USB_IF                  (USB_HAS_DAP + 2 * 1)
#define RF_USB_OUT_EP              0x01
#define RF_USB_IN_EP               0x81
#define CDC_DATA_IF                (CDC_CONTROL_IF + 1)
#define DAP_HID_IF                 (USB_HAS_DAP + 0 + 2 * 1)
#define DAP_BULK_OUT_EP            0x02U
#define DAP_BULK_IN_EP             0x83U
#define DAP_HID_OUT_EP             0x01U
#define DAP_HID_IN_EP              0x81U
#define DAP_BULK_IN_EP_NUMBER      (DAP_BULK_IN_EP & USB_EP_NUMBER_MASK)
#define DAP_HID_EP_NUMBER          (DAP_HID_OUT_EP & USB_EP_NUMBER_MASK)
#define CDC_IN_EP_NUMBER           (CDC_IN_EP & USB_EP_NUMBER_MASK)
#define CDC_NOTIFY_EP_NUMBER       (CDC_NOTIFY_EP & USB_EP_NUMBER_MASK)
#define DAP_HID_REPORT_LENGTH      27U
#define USB_BASE_INTERFACE_COUNT   (2 * USB_HAS_DAP + 2)
#define USB_BASE_CONFIG_LENGTH     (9 + 55 * USB_HAS_DAP + 66)
#define USB_DFU_RUNTIME_INTERFACE  USB_BASE_INTERFACE_COUNT
#define USB_INTERFACE_COUNT        (USB_BASE_INTERFACE_COUNT + USB_DFU_ENABLE)
#define USB_CONFIG_LENGTH          (USB_BASE_CONFIG_LENGTH + 18 * USB_DFU_ENABLE)

#if BSP_USB_DONGLE
#define USB_PRODUCT_NAME "AirDAP (Dongle)"
#else
#define USB_PRODUCT_NAME "AirDAP (Full)"
#endif

/* One EP0 SETUP owns this descriptor scratch. The next
 * SETUP cancels the preceding control transfer; neither buffer is used by
 * bulk/HID/CDC DMA or by the foreground. Keep DMA storage USB_ep0 separate. */
typedef union {
    uint32_t reply[8];
    uint8_t  string[40];
} usb_control_scratch_t;

extern usb_control_scratch_t USB_controlScratch;
extern const uint8_t         USB_deviceDescriptor[USB_DEVICE_DESC_SIZE];
extern const uint8_t         USB_configDescriptor[USB_CONFIG_LENGTH];

const uint8_t *USB_Descriptor(unsigned type, unsigned index, unsigned language, uint16_t *length);

const uint8_t *USB_OsDescriptor(unsigned index, unsigned interface, uint16_t *length);

const uint8_t *USB_WebusbDescriptor(unsigned urlIndex, uint16_t *length);

const uint8_t *USB_HidDescriptor(unsigned type, unsigned index, unsigned interface,
                                 uint16_t *length);
#endif
