#ifndef USB_PROTOCOL_H
#define USB_PROTOCOL_H

/* USB 2.0 Chapter 9: complete bmRequestType values. */
#define USB_RT_OUT_STANDARD_DEVICE    0x00U
#define USB_RT_IN_STANDARD_DEVICE     0x80U
#define USB_RT_OUT_STANDARD_INTERFACE 0x01U
#define USB_RT_IN_STANDARD_INTERFACE  0x81U
#define USB_RT_OUT_STANDARD_ENDPOINT  0x02U
#define USB_RT_IN_STANDARD_ENDPOINT   0x82U
#define USB_RT_OUT_CLASS_INTERFACE    0x21U
#define USB_RT_IN_CLASS_INTERFACE     0xA1U
#define USB_RT_OUT_VENDOR_DEVICE      0x40U
#define USB_RT_IN_VENDOR_DEVICE       0xC0U
#define USB_RT_IN_VENDOR_INTERFACE    0xC1U

#define USB_SETUP_PACKET_SIZE      8U
#define USB_SETUP_TYPE_OFFSET      0U
#define USB_SETUP_REQUEST_OFFSET   1U
#define USB_SETUP_VALUE_OFFSET     2U
#define USB_SETUP_INDEX_OFFSET     4U
#define USB_SETUP_LENGTH_OFFSET    6U
#define USB_ADDRESS_LIMIT          128U
#define USB_CONFIG_DEFAULT         1U
#define USB_ALT_DEFAULT            0U
#define USB_STATUS_SIZE            2U
#define USB_CONTROL_EP             0U
#define USB_EP_IN                  0x80U
#define USB_EP_NUMBER_MASK         0x0FU
#define USB_EP_INDEX_RESERVED_MASK 0xFF00U
#define USB_EP_BULK                2U
#define USB_EP_INTERRUPT           3U
#define USB_FS_PACKET_SIZE         64U
#define USB_CONFIG_BUS_POWERED     0x80U
#define USB_MAX_POWER_UNIT_MA      2U

#define USB_DESC_DEVICE         0x01U
#define USB_DESC_CONFIGURATION  0x02U
#define USB_DESC_STRING         0x03U
#define USB_DESC_INTERFACE      0x04U
#define USB_DESC_ENDPOINT       0x05U
#define USB_DESC_IAD            0x0BU
#define USB_DESC_BOS            0x0FU
#define USB_DESC_CAPABILITY     0x10U
#define USB_DESC_HID            0x21U
#define USB_DESC_HID_REPORT     0x22U
#define USB_DESC_DFU_FUNCTIONAL 0x21U
#define USB_DESC_CS_INTERFACE   0x24U
#define USB_DEVICE_DESC_SIZE    18U
#define USB_CONFIG_DESC_SIZE    9U
#define USB_INTERFACE_DESC_SIZE 9U
#define USB_ENDPOINT_DESC_SIZE  7U
#define USB_IAD_DESC_SIZE       8U
#define USB_BOS_DESC_SIZE       5U
#define USB_HID_DESC_SIZE       9U
#define USB_DFU_DESC_SIZE       9U
#define USB_STRING_HEADER_SIZE  2U
#define USB_STRING_CHAR_SIZE    2U
#define USB_LANGID_EN_US        0x0409U
#define USB_BCD_2_0             0x0200U
#define USB_BCD_2_1             0x0210U

#define USB_CLASS_MISCELLANEOUS  0xEFU
#define USB_SUBCLASS_COMMON      0x02U
#define USB_PROTOCOL_IAD         0x01U
#define USB_CLASS_VENDOR         0xFFU
#define USB_CLASS_HID            0x03U
#define USB_CLASS_CDC_CONTROL    0x02U
#define USB_CLASS_CDC_DATA       0x0AU
#define USB_CDC_SUBCLASS_ACM     0x02U
#define USB_CDC_PROTOCOL_AT      0x01U
#define USB_CLASS_APPLICATION    0xFEU
#define USB_DFU_SUBCLASS         0x01U
#define USB_DFU_PROTOCOL_RUNTIME 0x01U
#define USB_DFU_PROTOCOL_MODE    0x02U

/* HID 1.11 and CMSIS-DAP's fixed 64-byte, unnumbered vendor report. */
#define USB_HID_BCD_1_11               0x0111U
#define USB_HID_REPORT_INPUT           0x0100U
#define USB_HID_REPORT_OUTPUT          0x0200U
#define USB_HID_PROTOCOL_REPORT        1U
#define USB_HID_REPORT_SIZE            64U
#define USB_HID_POLL_INTERVAL_MS       1U
#define USB_HID_ITEM_USAGE_PAGE16      0x06U
#define USB_HID_ITEM_USAGE             0x09U
#define USB_HID_ITEM_COLLECTION        0xA1U
#define USB_HID_ITEM_END_COLLECTION    0xC0U
#define USB_HID_ITEM_LOGICAL_MIN8      0x15U
#define USB_HID_ITEM_LOGICAL_MAX16     0x26U
#define USB_HID_ITEM_REPORT_SIZE       0x75U
#define USB_HID_ITEM_REPORT_COUNT      0x95U
#define USB_HID_ITEM_INPUT             0x81U
#define USB_HID_ITEM_OUTPUT            0x91U
#define USB_HID_USAGE_PAGE_VENDOR      0xFF00U
#define USB_HID_COLLECTION_APPLICATION 1U
#define USB_HID_DATA_VARIABLE_ABSOLUTE 0x02U

/* CDC ACM 1.10 line coding and SERIAL_STATE notification. */
#define USB_CDC_BCD_1_10                 0x0110U
#define USB_CDC_FUNC_HEADER              0U
#define USB_CDC_FUNC_CALL_MANAGEMENT     1U
#define USB_CDC_FUNC_ACM                 2U
#define USB_CDC_FUNC_UNION               6U
#define USB_CDC_CAP_LINE_CONTROL         0x02U
#define USB_CDC_CAP_SEND_BREAK           0x04U
#define USB_CDC_LINE_CODING_SIZE         7U
#define USB_CDC_STOP_OFFSET              4U
#define USB_CDC_PARITY_OFFSET            5U
#define USB_CDC_DATA_BITS_OFFSET         6U
#define USB_CDC_STOP_1_5                 1U
#define USB_CDC_STOP_2                   2U
#define USB_CDC_PARITY_SPACE             4U
#define USB_CDC_CONTROL_DTR              0x01U
#define USB_CDC_CONTROL_RTS              0x02U
#define USB_CDC_CONTROL_MASK             (USB_CDC_CONTROL_DTR | USB_CDC_CONTROL_RTS)
#define USB_CDC_NOTIFY_SERIAL_STATE      0x20U
#define USB_CDC_NOTIFICATION_HEADER_SIZE 8U
#define USB_CDC_SERIAL_STATE_SIZE        2U
#define USB_CDC_STATE_BREAK              (1U << 2)
#define USB_CDC_STATE_FRAMING            (1U << 4)
#define USB_CDC_STATE_PARITY             (1U << 5)
#define USB_CDC_STATE_DCD                (1 << 0)
#define USB_CDC_STATE_DSR                (1 << 1)
#define USB_CDC_STATE_OVERRUN            (1U << 6)

/* DFU 1.1, common to runtime and the separate DFU application. */
#define USB_DFU_BCD_1_1              0x0110U
#define USB_DFU_ATTR_DOWNLOAD        0x01U
#define USB_DFU_ATTR_UPLOAD          0x02U
#define USB_DFU_ATTR_WILL_DETACH     0x08U
#define USB_DFU_TRANSFER_SIZE        4096U
#define USB_DFU_DETACH_TIMEOUT_MS    1000U
#define USB_DFU_STATUS_SIZE          6U
#define USB_DFU_STATE_SIZE           1U
#define USB_DFU_STATUS_OFFSET        0U
#define USB_DFU_POLL_TIMEOUT_OFFSET  1U
#define USB_DFU_STATE_OFFSET         4U
#define USB_DFU_STATUS_STRING_OFFSET 5U
#define USB_DFU_APP_IDLE             0U
#define USB_DFU_APP_DETACH           1U

/* Microsoft OS descriptors and WebUSB descriptor request indices. */
#define USB_MS_OS_STRING_SIZE              18U
#define USB_MS_OS10_PROPERTIES_HEADER_SIZE 10U
#define USB_MS_OS10_GUID_SZ_PROPERTY_SIZE  132U
#define USB_MS_OS10_GUID_PROPERTY_SIZE     136U
#define USB_MS_OS_STRING_INDEX             0xEEU
#define USB_MS_OS10_COMPAT_INDEX           4U
#define USB_MS_OS10_PROPERTIES_INDEX       5U
#define USB_MS_OS20_SET_INDEX              7U
#define USB_MS_OS10_BCD_1_0                0x0100U
#define USB_MS_OS10_HEADER_SIZE            16U
#define USB_MS_OS10_FUNCTION_SIZE          24U
#define USB_MS_OS20_HEADER_SIZE            10U
#define USB_MS_OS20_CONFIG_SIZE            8U
#define USB_MS_OS20_FUNCTION_SIZE          8U
#define USB_MS_OS20_COMPAT_SIZE            20U
#define USB_MS_OS20_PROPERTY_SIZE          132U
#define USB_MS_OS20_TYPE_SET               0U
#define USB_MS_OS20_TYPE_CONFIG            1U
#define USB_MS_OS20_TYPE_FUNCTION          2U
#define USB_MS_OS20_TYPE_COMPAT            3U
#define USB_MS_OS20_TYPE_PROPERTY          4U
#define USB_MS_WINDOWS_8_1                 0x06030000UL
#define USB_MS_REG_SZ                      1U
#define USB_MS_REG_MULTI_SZ                7U
#define USB_CAP_PLATFORM                   5U
#define USB_WEBUSB_CAP_SIZE                24U
#define USB_MS_OS20_CAP_SIZE               28U
#define USB_WEBUSB_BCD_1_0                 0x0100U
#define USB_WEBUSB_GET_URL_INDEX           2U
#define USB_WEBUSB_URL_TYPE                3U
#define USB_WEBUSB_SCHEME_HTTPS            1U
#define USB_WEBUSB_LANDING_PAGE            1U
#endif
