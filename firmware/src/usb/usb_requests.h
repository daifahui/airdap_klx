#ifndef USB_REQUESTS_H
#define USB_REQUESTS_H

#include "usb/usb_protocol.h"

/* bRequest values are scoped by bmRequestType, never dispatched alone. */
#define USB_REQ_GET_STATUS        0x00U
#define USB_REQ_CLEAR_FEATURE     0x01U
#define USB_REQ_SET_FEATURE       0x03U
#define USB_REQ_SET_ADDRESS       0x05U
#define USB_REQ_GET_DESCRIPTOR    0x06U
#define USB_REQ_GET_CONFIGURATION 0x08U
#define USB_REQ_SET_CONFIGURATION 0x09U
#define USB_REQ_GET_INTERFACE     0x0AU
#define USB_REQ_SET_INTERFACE     0x0BU

#define USB_HID_GET_REPORT       0x01U
#define USB_HID_GET_IDLE         0x02U
#define USB_HID_GET_PROTOCOL     0x03U
#define USB_HID_SET_REPORT       0x09U
#define USB_HID_SET_IDLE         0x0AU
#define USB_HID_SET_PROTOCOL     0x0BU
#define USB_CDC_SET_LINE_CODING  0x20U
#define USB_CDC_GET_LINE_CODING  0x21U
#define USB_CDC_SET_CONTROL_LINE 0x22U
#define USB_CDC_SEND_BREAK       0x23U

#define USB_DFU_DETACH    0x00U
#define USB_DFU_DNLOAD    0x01U
#define USB_DFU_UPLOAD    0x02U
#define USB_DFU_GETSTATUS 0x03U
#define USB_DFU_CLRSTATUS 0x04U
#define USB_DFU_GETSTATE  0x05U
#define USB_DFU_ABORT     0x06U

#define USB_MODE_REQUEST       0x01U
#define USB_POWER_REQUEST      0x02U
#define USB_BIND_REQUEST       0x03U
#define USB_PEERS_REQUEST      0x04U
#define USB_OS_VENDOR_CODE     0x20U
#define USB_WEBUSB_VENDOR_CODE 0x21U
#define USB_BOOT_SCAN_REQUEST  0x6DU
#define USB_MODE_AUTO          0U
#define USB_MODE_LOCAL         1U
#define USB_MODE_RF_HOST       2U
#define USB_MODE_RF_TARGET     3U
#endif
