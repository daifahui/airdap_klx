#include "usb/usb_requests.h"
#include "ch572/attributes.h"
#include "version.h"
#include "app_config.h"
#include "ch572/ch572sfr.h"
#include "dfu/dfu.h"

#define IDLE         0
#define DATA_IN      1
#define DATA_OUT     2
#define STATUS_IN    3
#define STATUS_OUT   4
#define U16_BYTES(v) ((v) & 255), (((v) >> 8) & 255)
#define U32_BYTES(v) U16_BYTES(v), U16_BYTES((v) >> 16)

#define DFU_USB_CONFIG_SIZE                                                                        \
    (USB_CONFIG_DESC_SIZE + 2 * (USB_INTERFACE_DESC_SIZE + USB_DFU_DESC_SIZE))

#define DFU_USB_FUNCTIONAL                                                                         \
    USB_DFU_DESC_SIZE, USB_DESC_DFU_FUNCTIONAL,                                                    \
        USB_DFU_ATTR_DOWNLOAD | USB_DFU_ATTR_UPLOAD | USB_DFU_ATTR_WILL_DETACH,                    \
        U16_BYTES(USB_DFU_DETACH_TIMEOUT_MS), U16_BYTES(DFU_TRANSFER), U16_BYTES(USB_DFU_BCD_1_1)

#define MEMORY_MAP u"@CH572 /0x00000000/060*004Kg"

uint8_t               USB_ep0Dma[USB_FS_PACKET_SIZE] __ALIGNED(4);
uint8_t               USB_ep0Control, USB_ep0Length, USB_address, USB_configuration;
static uint16_t       USB_remaining, USB_offset, USB_requested;
static const uint8_t *USB_source;
static uint8_t        USB_phase, USB_pendingAddress;
static bool           USB_zlp, USB_classTransfer;
static uint8_t        USB_alternate;

// clang-format off

/* Keep each USB descriptor on its own logical row. */
static const uint8_t USB_deviceDescriptor[] = {
    USB_DEVICE_DESC_SIZE, USB_DESC_DEVICE, U16_BYTES(USB_BCD_2_0), 0, 0, 0, USB_FS_PACKET_SIZE,
    U16_BYTES(USB_VID), U16_BYTES(USB_PID), U16_BYTES(FIRMWARE_VERSION), 0, 0, 0, USB_CONFIG_DEFAULT
};

/* Both alternates retain the same base DFU1.1 functional descriptor. */
static const uint8_t USB_configurationDescriptor[] = {
    USB_CONFIG_DESC_SIZE, USB_DESC_CONFIGURATION, U16_BYTES(DFU_USB_CONFIG_SIZE),
    1, USB_CONFIG_DEFAULT, 0, USB_CONFIG_BUS_POWERED, 100 / USB_MAX_POWER_UNIT_MA,
    USB_INTERFACE_DESC_SIZE, USB_DESC_INTERFACE, 0, 0, 0,
    USB_CLASS_APPLICATION, USB_DFU_SUBCLASS, USB_DFU_PROTOCOL_MODE, DFU_DFUSE_ALT == 0 ? 1 : 0,
    DFU_USB_FUNCTIONAL,
    USB_INTERFACE_DESC_SIZE, USB_DESC_INTERFACE, 0, 1, 0,
    USB_CLASS_APPLICATION, USB_DFU_SUBCLASS, USB_DFU_PROTOCOL_MODE, DFU_DFUSE_ALT == 1 ? 1 : 0,
    DFU_USB_FUNCTIONAL
};
static const uint8_t USB_languageDescriptor[] = {
    USB_STRING_HEADER_SIZE + USB_STRING_CHAR_SIZE, USB_DESC_STRING, U16_BYTES(USB_LANGID_EN_US)
};
static const uint8_t USB_osString[] = {
    USB_MS_OS_STRING_SIZE, USB_DESC_STRING,
    'M', 0, 'S', 0, 'F', 0, 'T', 0, '1', 0, '0', 0, '0', 0, USB_OS_VENDOR_CODE, 0
};
static const uint8_t USB_compatibleId[] = {
    U32_BYTES(USB_MS_OS10_HEADER_SIZE + USB_MS_OS10_FUNCTION_SIZE),
    U16_BYTES(USB_MS_OS10_BCD_1_0), U16_BYTES(USB_MS_OS10_COMPAT_INDEX), 1, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 'W', 'I', 'N', 'U', 'S', 'B', 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0
};
// clang-format on

/* Microsoft OS1 extended properties: REG_SZ DeviceInterfaceGUID. */
static const struct __PACKED {
    uint32_t length;
    uint16_t version, index, count;
    uint32_t size, type;
    uint16_t name_bytes;
    uint16_t name[20];
    uint32_t data_bytes;
    uint16_t guid[39];
} USB_osProperties = {142,
                      USB_MS_OS10_BCD_1_0,
                      USB_MS_OS10_PROPERTIES_INDEX,
                      1,
                      USB_MS_OS20_PROPERTY_SIZE,
                      USB_MS_REG_SZ,
                      40,
                      u"DeviceInterfaceGUID",
                      78,
                      u"{4DD39A36-78AE-4D4B-8CDB-431B50417257}"};

static const struct {
    uint8_t  length, type;
    uint16_t text[sizeof(MEMORY_MAP) / 2];
} USB_memoryMap = {sizeof(MEMORY_MAP), USB_DESC_STRING, MEMORY_MAP};

static const uint8_t USB_zeros[USB_SETUP_VALUE_OFFSET] = {0, 0};

static void USB_Stall(void) {
    USB_phase      = IDLE;
    USB_ep0Control = (RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL);
}

static void USB_Send(void) {
    uint8_t n = USB_remaining > USB_FS_PACKET_SIZE ? USB_FS_PACKET_SIZE : USB_remaining;

    for (unsigned i = 0; i < n; i++)
        USB_ep0Dma[i] = USB_source[USB_offset + i];
    USB_offset    += n;
    USB_remaining -= n;
    USB_ep0Length  = n;
    /* OUT ACK receives the control status stage. */
    USB_ep0Control &= RB_UEP_R_TOG | RB_UEP_T_TOG;
}

static void USB_ReadReply(const uint8_t *data, uint16_t size) {
    USB_source     = data;
    USB_remaining  = size < USB_requested ? size : USB_requested;
    USB_offset     = 0;
    USB_zlp        = (USB_remaining < USB_requested && !(USB_remaining & (USB_FS_PACKET_SIZE - 1)));
    USB_phase      = DATA_IN;
    USB_ep0Control = (RB_UEP_R_TOG | RB_UEP_T_TOG);
    USB_Send();
}

void USB_ControlReset(void) {
    USB_phase   = IDLE;
    USB_address = USB_configuration = USB_alternate = 0;
    USB_pendingAddress                              = UINT8_MAX;
    USB_ep0Control                                  = UEP_T_RES_NAK;
    USB_ep0Length                                   = 0;
    DFU_BusReset();
    DFU_protocol = DFU_PROTOCOL_FOR_ALT(0);
}

void USB_ControlSetup(const uint8_t *s) {
    uint8_t  type = s[USB_SETUP_TYPE_OFFSET], req = s[USB_SETUP_REQUEST_OFFSET];
    uint16_t value = s[USB_SETUP_VALUE_OFFSET] | (uint16_t)s[USB_SETUP_VALUE_OFFSET + 1] << 8,
             index = s[USB_SETUP_INDEX_OFFSET] | (uint16_t)s[USB_SETUP_INDEX_OFFSET + 1] << 8;
    USB_requested  = s[USB_SETUP_LENGTH_OFFSET] | (uint16_t)s[USB_SETUP_LENGTH_OFFSET + 1] << 8;
    DFU_CancelControl();
    USB_pendingAddress = UINT8_MAX;
    USB_classTransfer  = false;
    USB_ep0Control     = (RB_UEP_R_TOG | RB_UEP_T_TOG);
    USB_ep0Length      = 0;
    USB_offset         = 0;
    if ((type == USB_RT_IN_STANDARD_DEVICE || type == USB_RT_IN_STANDARD_INTERFACE) &&
        req == USB_REQ_GET_DESCRIPTOR) {
        if (value == (USB_DESC_DEVICE << 8))
            USB_ReadReply(USB_deviceDescriptor, sizeof(USB_deviceDescriptor));
        else if (value == (USB_DESC_CONFIGURATION << 8))
            USB_ReadReply(USB_configurationDescriptor, sizeof(USB_configurationDescriptor));
        else if (value == (USB_DESC_DFU_FUNCTIONAL << 8) && !index)
            USB_ReadReply(USB_configurationDescriptor + USB_CONFIG_DESC_SIZE +
                              USB_INTERFACE_DESC_SIZE +
                              (USB_alternate ? USB_INTERFACE_DESC_SIZE + USB_DFU_DESC_SIZE : 0),
                          USB_DFU_DESC_SIZE);
        else if (value == (USB_DESC_STRING << 8))
            USB_ReadReply(USB_languageDescriptor, 4);
        else if (value == ((USB_DESC_STRING << 8) | 1))
            USB_ReadReply((const uint8_t *)&USB_memoryMap, sizeof(USB_memoryMap) - 2);
        else if (value == ((USB_DESC_STRING << 8) | USB_MS_OS_STRING_INDEX))
            USB_ReadReply(USB_osString, sizeof(USB_osString));
        else
            USB_Stall();
        return;
    }
    if (type == USB_RT_OUT_STANDARD_DEVICE && req == USB_REQ_SET_ADDRESS && !index &&
        !USB_requested && value < USB_ADDRESS_LIMIT)
        USB_pendingAddress = value;
    else if (type == USB_RT_OUT_STANDARD_DEVICE && req == USB_REQ_SET_CONFIGURATION && !index &&
             !USB_requested && value <= USB_CONFIG_DEFAULT) {
        USB_configuration = value;
        USB_alternate     = 0;
        DFU_Configure(DFU_PROTOCOL_FOR_ALT(0));
    } else if (type == USB_RT_IN_STANDARD_DEVICE && req == USB_REQ_GET_CONFIGURATION && !value &&
               !index && USB_requested == 1) {
        USB_ReadReply(&USB_configuration, 1);
        return;
    } else if (type == USB_RT_IN_STANDARD_INTERFACE && req == USB_REQ_GET_INTERFACE && !value &&
               !index && USB_requested == 1) {
        USB_ReadReply(&USB_alternate, 1);
        return;
    } else if (type == USB_RT_OUT_STANDARD_INTERFACE && req == USB_REQ_SET_INTERFACE &&
               value <= 1 && !index && !USB_requested) {
        USB_alternate = value;
        DFU_Configure(DFU_PROTOCOL_FOR_ALT(value));
    } else if ((type == USB_RT_IN_STANDARD_DEVICE || type == USB_RT_IN_STANDARD_INTERFACE ||
                type == USB_RT_IN_STANDARD_ENDPOINT) &&
               req == USB_REQ_GET_STATUS && !value && USB_requested == USB_STATUS_SIZE) {
        USB_ReadReply(USB_zeros, USB_STATUS_SIZE);
        return;
    } else if (req == USB_OS_VENDOR_CODE && !value &&
               (type == USB_RT_IN_VENDOR_DEVICE || type == USB_RT_IN_VENDOR_INTERFACE) &&
               (index == USB_MS_OS10_COMPAT_INDEX || index == USB_MS_OS10_PROPERTIES_INDEX)) {
        USB_ReadReply(index == USB_MS_OS10_COMPAT_INDEX ? USB_compatibleId
                                                        : (const uint8_t *)&USB_osProperties,
                      index == USB_MS_OS10_COMPAT_INDEX ? sizeof(USB_compatibleId)
                                                        : sizeof(USB_osProperties));
        return;
    } else {
        int count = DFU_Setup(type, req, value, index, USB_requested);

        USB_classTransfer = true;
        if (count == DFU_SETUP_STALL) {
            USB_Stall();
            return;
        }
        if (count == DFU_SETUP_DATA_OUT) {
            USB_phase      = DATA_OUT;
            USB_remaining  = USB_requested;
            USB_ep0Control = (RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_T_RES_NAK);
            return;
        }
        if (count >= 0) {
            USB_ReadReply(req == USB_DFU_UPLOAD ? DFU_buffer : DFU_response, count);
            return;
        }
    }
    USB_phase      = STATUS_IN;
    USB_ep0Control = (RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK);
    USB_ep0Length  = 0;
}

void USB_ControlOut(const uint8_t *data, uint8_t n, bool toggleOk) {
    if (!toggleOk)
        return;
    if (USB_phase == STATUS_OUT && !n) {
        USB_phase      = IDLE;
        USB_ep0Control = UEP_T_RES_NAK;
        if (USB_classTransfer)
            DFU_StatusComplete();
        return;
    }
    if (USB_phase != DATA_OUT || n > USB_remaining || (!n && USB_remaining)) {
        USB_Stall();
        return;
    }
    for (unsigned i = 0; i < n; i++)
        DFU_buffer[USB_offset + i] = data[i];
    USB_offset     += n;
    USB_remaining  -= n;
    USB_ep0Control ^= RB_UEP_R_TOG;
    if (!USB_remaining) {
        DFU_DataComplete(USB_offset);
        USB_phase      = STATUS_IN;
        USB_ep0Length  = 0;
        USB_ep0Control = (RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK);
    } else if (n < USB_FS_PACKET_SIZE) {
        USB_Stall();
        DFU_CancelControl();
    }
}

void USB_ControlIn(void) {
    if (USB_phase == STATUS_IN) {
        USB_phase      = IDLE;
        USB_ep0Control = UEP_T_RES_NAK;
        if (USB_pendingAddress != UINT8_MAX) {
            USB_address        = USB_pendingAddress;
            USB_pendingAddress = UINT8_MAX;
        }
        if (USB_classTransfer)
            DFU_StatusComplete();
    } else if (USB_phase == DATA_IN) {
        USB_ep0Control ^= RB_UEP_T_TOG;
        if (USB_remaining)
            USB_Send();
        else if (USB_zlp && USB_ep0Length) {
            USB_zlp = false;
            USB_Send();
        } else {
            USB_phase      = STATUS_OUT;
            USB_ep0Control = (RB_UEP_R_TOG | UEP_T_RES_NAK);
        }
    }
}
