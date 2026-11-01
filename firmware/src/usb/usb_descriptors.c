/* USB descriptors, CMSIS-DAP HID and DAP-v2 WinUSB/WebUSB discovery. */

#include "usb/usb_descriptors.h"
#include "ch572/attributes.h"
#include "bsp/board.h"

#if USB_DFU_ENABLE
#include "dfu/dfu_runtime.h"
#endif

#include <string.h>
#include "usb/usb_ms_functions.h"

/* Each row describes one logical field group or one complete descriptor. */
// clang-format off
const uint8_t USB_deviceDescriptor[USB_DEVICE_DESC_SIZE] = {
    USB_DEVICE_DESC_SIZE, USB_DESC_DEVICE,
    U16_BYTES((USB_HAS_DAP || USB_DFU_ENABLE) ? USB_BCD_2_1 : USB_BCD_2_0),
    USB_CLASS_MISCELLANEOUS, USB_SUBCLASS_COMMON, USB_PROTOCOL_IAD, USB_FS_PACKET_SIZE,
    U16_BYTES(USB_VID), U16_BYTES(USB_PID), U16_BYTES(FIRMWARE_VERSION),
    USB_STRING_MANUFACTURER, USB_STRING_PRODUCT, USB_STRING_SERIAL, USB_CONFIG_DEFAULT
};

const uint8_t USB_configDescriptor[USB_CONFIG_LENGTH] = {
    USB_CONFIG_DESC_SIZE, USB_DESC_CONFIGURATION, U16_BYTES(USB_CONFIG_LENGTH),
    USB_INTERFACE_COUNT, USB_CONFIG_DEFAULT, 0, USB_CONFIG_BUS_POWERED,
    USB_MAX_POWER_MA / USB_MAX_POWER_UNIT_MA,

#if USB_HAS_DAP
    // Interface 0: CMSIS-DAP v2. Endpoint order is OUT, IN.
    USB_INTERFACE_DESC_SIZE, USB_DESC_INTERFACE, 0, USB_ALT_DEFAULT, 2, USB_CLASS_VENDOR, 0, 0, USB_STRING_DAP_BULK,
    USB_ENDPOINT_DESC_SIZE, USB_DESC_ENDPOINT, DAP_BULK_OUT_EP, USB_EP_BULK,
    U16_BYTES(USB_FS_PACKET_SIZE), 0,
    USB_ENDPOINT_DESC_SIZE, USB_DESC_ENDPOINT, DAP_BULK_IN_EP, USB_EP_BULK,
    U16_BYTES(USB_FS_PACKET_SIZE), 0,
#endif

    // CDC ACM: IAD, control interface, functional descriptors, notification EP.
    USB_IAD_DESC_SIZE, USB_DESC_IAD, CDC_CONTROL_IF, 2,
    USB_CLASS_CDC_CONTROL, USB_CDC_SUBCLASS_ACM, USB_CDC_PROTOCOL_AT, 0,
    USB_INTERFACE_DESC_SIZE, USB_DESC_INTERFACE, CDC_CONTROL_IF, USB_ALT_DEFAULT, 1,
    USB_CLASS_CDC_CONTROL, USB_CDC_SUBCLASS_ACM, USB_CDC_PROTOCOL_AT, 0,
    5, USB_DESC_CS_INTERFACE, USB_CDC_FUNC_HEADER, U16_BYTES(USB_CDC_BCD_1_10),
    5, USB_DESC_CS_INTERFACE, USB_CDC_FUNC_CALL_MANAGEMENT, 0, CDC_DATA_IF,
    4, USB_DESC_CS_INTERFACE, USB_CDC_FUNC_ACM, USB_CDC_CAP_LINE_CONTROL | USB_CDC_CAP_SEND_BREAK,
    5, USB_DESC_CS_INTERFACE, USB_CDC_FUNC_UNION, CDC_CONTROL_IF, CDC_DATA_IF,
    USB_ENDPOINT_DESC_SIZE, USB_DESC_ENDPOINT, CDC_NOTIFY_EP, USB_EP_INTERRUPT,
    U16_BYTES(USB_CDC_NOTIFY_PACKET_SIZE), USB_CDC_NOTIFY_INTERVAL_MS,
    // CDC data interface and bulk OUT/IN endpoints.
    USB_INTERFACE_DESC_SIZE, USB_DESC_INTERFACE, CDC_DATA_IF, USB_ALT_DEFAULT, 2,
    USB_CLASS_CDC_DATA, 0, 0, 0,
    USB_ENDPOINT_DESC_SIZE, USB_DESC_ENDPOINT, CDC_OUT_EP, USB_EP_BULK,
    U16_BYTES(USB_FS_PACKET_SIZE), 0,
    USB_ENDPOINT_DESC_SIZE, USB_DESC_ENDPOINT, CDC_IN_EP, USB_EP_BULK,
    U16_BYTES(USB_FS_PACKET_SIZE), 0,

#if USB_HAS_DAP
    // CMSIS-DAP v1: 64-byte vendor HID reports, no report ID or boot protocol.
    USB_INTERFACE_DESC_SIZE, USB_DESC_INTERFACE, DAP_HID_IF, USB_ALT_DEFAULT, 2,
    USB_CLASS_HID, 0, 0, USB_STRING_DAP_HID,
    USB_HID_DESC_SIZE, USB_DESC_HID, U16_BYTES(USB_HID_BCD_1_11), 0, 1,
    USB_DESC_HID_REPORT, U16_BYTES(DAP_HID_REPORT_LENGTH),
    USB_ENDPOINT_DESC_SIZE, USB_DESC_ENDPOINT, DAP_HID_IN_EP, USB_EP_INTERRUPT,
    U16_BYTES(USB_HID_REPORT_SIZE), USB_HID_POLL_INTERVAL_MS,
    USB_ENDPOINT_DESC_SIZE, USB_DESC_ENDPOINT, DAP_HID_OUT_EP, USB_EP_INTERRUPT,
    U16_BYTES(USB_HID_REPORT_SIZE), USB_HID_POLL_INTERVAL_MS,
#endif

#if USB_DFU_ENABLE
    USB_DFU_RUNTIME_DESCRIPTORS,
#endif
};
// clang-format on

static const uint8_t USB_languageString[] = {USB_STRING_HEADER_SIZE + USB_STRING_CHAR_SIZE,
                                             USB_DESC_STRING, U16_BYTES(USB_LANGID_EN_US)};

#if (USB_HAS_DAP || USB_DFU_ENABLE)
static const uint8_t USB_bosDescriptor[] = {
    USB_BOS_DESC_SIZE, USB_DESC_BOS,
    U16_BYTES(USB_BOS_DESC_SIZE + USB_MS_OS20_CAP_SIZE + USB_WEBUSB_CAP_SIZE * USB_HAS_DAP),
    1 + USB_HAS_DAP,

#if USB_HAS_DAP
    // WebUSB platform capability, version 1.0 and AirDAP HTTPS landing URL 1.
    USB_WEBUSB_CAP_SIZE, USB_DESC_CAPABILITY, USB_CAP_PLATFORM, 0, 0x38, 0xB6, 0x08, 0x34, 0xA9,
    0x09, 0xA0, 0x47, 0x8B, 0xFD, 0xA0, 0x76, 0x88, 0x15, 0xB6, 0x65, U16_BYTES(USB_WEBUSB_BCD_1_0),
    USB_WEBUSB_VENDOR_CODE, USB_WEBUSB_LANDING_PAGE,
#endif

    // Microsoft OS 2.0 capability, Windows 8.1+ descriptor set.
    USB_MS_OS20_CAP_SIZE, USB_DESC_CAPABILITY, USB_CAP_PLATFORM, 0, 0xDF, 0x60, 0xDD, 0xD8, 0x89,
    0x45, 0xC7, 0x4C, 0x9C, 0xD2, 0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F, U32_BYTES(USB_MS_WINDOWS_8_1),
    U16_BYTES(USB_MS_OS20_BYTES), USB_OS_VENDOR_CODE, 0};
#endif

#if USB_HAS_DAP
// clang-format off
static const uint8_t USB_webusbUrl[37] = {
    sizeof(USB_webusbUrl), USB_WEBUSB_URL_TYPE, USB_WEBUSB_SCHEME_HTTPS,
    'o', 's', 'h', 'w', 'h', 'u', 'b', '.', 'c', 'o', 'm', '/', 'm', 'e', 't', 'r', 'o', '/',
    'p', 'r', 'o', 'j', 'e', 'c', 't', '_', 'o', 'i', 'a', 'd', 'r', 'n', 's', 'o'
};
// clang-format on

static const uint8_t USB_hidReportDescriptor[DAP_HID_REPORT_LENGTH] = {
    USB_HID_ITEM_USAGE_PAGE16,  U16_BYTES(USB_HID_USAGE_PAGE_VENDOR),
    USB_HID_ITEM_USAGE,         1,
    USB_HID_ITEM_COLLECTION,    USB_HID_COLLECTION_APPLICATION,
    USB_HID_ITEM_LOGICAL_MIN8,  0,
    USB_HID_ITEM_LOGICAL_MAX16, U16_BYTES(UINT8_MAX),
    USB_HID_ITEM_REPORT_SIZE,   8,
    USB_HID_ITEM_REPORT_COUNT,  USB_HID_REPORT_SIZE,
    USB_HID_ITEM_USAGE,         1,
    USB_HID_ITEM_INPUT,         USB_HID_DATA_VARIABLE_ABSOLUTE,
    USB_HID_ITEM_REPORT_COUNT,  USB_HID_REPORT_SIZE,
    USB_HID_ITEM_USAGE,         1,
    USB_HID_ITEM_OUTPUT,        USB_HID_DATA_VARIABLE_ABSOLUTE,
    USB_HID_ITEM_END_COLLECTION};
#endif

#if (USB_HAS_DAP || USB_DFU_ENABLE)
// clang-format off
static const uint8_t USB_osString[] = {
    USB_MS_OS_STRING_SIZE, USB_DESC_STRING,
    'M', 0, 'S', 0, 'F', 0, 'T', 0, '1', 0, '0', 0, '0', 0, USB_OS_VENDOR_CODE, 0
};
// clang-format on
#endif

/* Longest fixed string is 17 ASCII characters; serial is 16. */

/* CPU-only staging; the separate USB_ep0 is the actual DMA buffer. */
usb_control_scratch_t USB_controlScratch __ALIGNED(4);

const uint8_t *USB_Descriptor(unsigned type, unsigned index, unsigned language, uint16_t *length) {
    if (type == USB_DESC_DEVICE && index == 0) {
        *length = sizeof(USB_deviceDescriptor);
        return USB_deviceDescriptor;
    }
    if (type == USB_DESC_CONFIGURATION && index == 0) {
        *length = sizeof(USB_configDescriptor);
        return USB_configDescriptor;
    }

#if (USB_HAS_DAP || USB_DFU_ENABLE)
    if (type == USB_DESC_BOS && index == 0 && language == 0) {
        *length = sizeof(USB_bosDescriptor);
        return USB_bosDescriptor;
    }
#endif

    if (type != USB_DESC_STRING) {
        return 0;
    }
    if (index == 0) {
        *length = sizeof(USB_languageString);
        return USB_languageString;
    }

#if (USB_HAS_DAP || USB_DFU_ENABLE)
    if (index == USB_MS_OS_STRING_INDEX) {
        *length = sizeof(USB_osString);
        return USB_osString;
    }
#endif

    if (language != USB_LANGID_EN_US) {
        return 0;
    }
    const char *s;
    switch (index) {
        case USB_STRING_MANUFACTURER:
            s = "WCH probe project";
            break;
        case USB_STRING_PRODUCT:
            s = USB_PRODUCT_NAME;
            break;
        case USB_STRING_SERIAL:
            s = BSP_serial;
            break;
        case USB_STRING_DAP_BULK:
            if (!USB_HAS_DAP) {
                return 0;
            }
            s = "CMSIS-DAP v2";
            break;
        case USB_STRING_DAP_HID:
            if (!USB_HAS_DAP) {
                return 0;
            }
            s = "CMSIS-DAP v1";
            break;
        default:
            return 0;
    }
    unsigned n = strlen(s);

    if (n > (sizeof(USB_controlScratch.string) - USB_STRING_HEADER_SIZE) / USB_STRING_CHAR_SIZE) {
        n = (sizeof(USB_controlScratch.string) - USB_STRING_HEADER_SIZE) / USB_STRING_CHAR_SIZE;
    }
    USB_controlScratch.string[0] = USB_STRING_HEADER_SIZE + USB_STRING_CHAR_SIZE * n;
    USB_controlScratch.string[1] = USB_DESC_STRING;
    for (unsigned i = 0; i < n; i++) {
        USB_controlScratch.string[USB_STRING_HEADER_SIZE + USB_STRING_CHAR_SIZE * i]     = s[i];
        USB_controlScratch.string[USB_STRING_HEADER_SIZE + 1 + USB_STRING_CHAR_SIZE * i] = 0;
    }
    *length = USB_controlScratch.string[0];
    return USB_controlScratch.string;
}

const uint8_t *USB_HidDescriptor(unsigned type, unsigned index, unsigned interface,
                                 uint16_t *length) {
#if USB_DFU_ENABLE
    if (!index && interface == USB_DFU_RUNTIME_INTERFACE && type == USB_DESC_DFU_FUNCTIONAL) {
        *length = USB_DFU_DESC_SIZE;
        return USB_configDescriptor + USB_CONFIG_LENGTH - USB_DFU_DESC_SIZE;
    }
#endif

#if USB_HAS_DAP
    if (index || interface != DAP_HID_IF) {
        return 0;
    }
    if (type == USB_DESC_HID_REPORT) {
        *length = sizeof(USB_hidReportDescriptor);
        return USB_hidReportDescriptor;
    }
    if (type == USB_DESC_HID) {
        *length = USB_HID_DESC_SIZE;
        return USB_configDescriptor + USB_BASE_CONFIG_LENGTH -
               (USB_HID_DESC_SIZE + 2 * USB_ENDPOINT_DESC_SIZE);
    }
#else
    (void)type;
    (void)index;
    (void)interface;
    (void)length;
#endif

    return 0;
}

const uint8_t *USB_OsDescriptor(unsigned index, unsigned interface, uint16_t *length) {
#if (USB_HAS_DAP || USB_DFU_ENABLE)
    if (!interface && index == USB_MS_OS10_COMPAT_INDEX) {
        *length = sizeof(USB_msCompat);
        return USB_msCompat;
    }
    if (!interface && index == USB_MS_OS20_SET_INDEX) {
        *length = sizeof(USB_msOs20);
        return USB_msOs20;
    }
    if (index == USB_MS_OS10_PROPERTIES_INDEX) {
#if USB_HAS_DAP
        if (!interface) {
            *length = sizeof(USB_msProperties);
            return USB_msProperties;
        }
#endif

#if USB_DFU_ENABLE
        if (interface == USB_DFU_RUNTIME_INTERFACE) {
            *length = sizeof(USB_dfuProperties);
            return USB_dfuProperties;
        }
#endif
    }
#else
    (void)index;
    (void)interface;
    (void)length;
#endif

    return 0;
}

const uint8_t *USB_WebusbDescriptor(unsigned urlIndex, uint16_t *length) {
#if USB_HAS_DAP
    if (urlIndex == USB_WEBUSB_LANDING_PAGE) {
        *length = sizeof(USB_webusbUrl);
        return USB_webusbUrl;
    }
#else
    (void)urlIndex;
    (void)length;
#endif

    return 0;
}
