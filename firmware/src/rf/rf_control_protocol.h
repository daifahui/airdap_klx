#ifndef RF_CONTROL_PROTOCOL_H
#define RF_CONTROL_PROTOCOL_H

/* Common prefix for RD discovery, RC channel and RP peer-power records. */
#define RF_CONTROL_MAGIC_0        'R'
#define RF_DISCOVERY_MAGIC_1      'D'
#define RF_CHANNEL_MAGIC_1        'C'
#define RF_SLEEP_MAGIC_1          'P'
#define RF_CONTROL_VERSION_OFFSET 2U
#define RF_CONTROL_ROLE_OFFSET    3U
#define RF_CONTROL_SESSION_OFFSET 4U
#define RF_CONTROL_SESSION_SIZE   8U
#define RF_CONTROL_BODY_OFFSET    12U
#define RF_CONTROL_VERSION        1U

#define RF_DISCOVERY_NONCE_OFFSET    12U
#define RF_DISCOVERY_KIND_OFFSET     20U
#define RF_DISCOVERY_RESERVED_OFFSET 21U
#define RF_DISCOVERY_UID_OFFSET      24U
#define RF_DISCOVERY_UID_SIZE        8U
#define RF_DISCOVERY_AA_OFFSET       32U
#define RF_DISCOVERY_VERSION         0x48U
#define RF_DISCOVERY_SIZE            36U

#define RF_CHANNEL_PHASE_OFFSET         RF_CONTROL_BODY_OFFSET
#define RF_CHANNEL_NUMBER_OFFSET        13U
#define RF_CHANNEL_EPOCH_OFFSET         14U
#define RF_CHANNEL_RESERVED_OFFSET      15U
#define RF_CHANNEL_RESERVED_TAIL_OFFSET 16U
#define RF_CHANNEL_SIZE                 24U
#define RF_CHANNEL_FIRST                34U
#define RF_CHANNEL_LAST                 36U

#define RF_SLEEP_SIZE            16U
#define RF_SLEEP_KIND_OFFSET     RF_CONTROL_BODY_OFFSET
#define RF_SLEEP_RESERVED_OFFSET 13U
#define RF_SLEEP_KIND_REQUEST    1U
#define RF_SLEEP_KIND_ACK        2U

/* RF Basic2M DMA prefix and CRC configuration; not BLE link-layer PDUs. */
#define RF_PHY_TAG_OFFSET    0U
#define RF_PHY_LENGTH_OFFSET 1U
#define RF_PHY_DATA_OFFSET   2U
#define RF_PHY_TX_TAG        0x55U
#define RF_BASIC2M_CRC_INIT  0x555555U
#define RF_BASIC2M_CRC_POLY  0x80032DU
#endif
