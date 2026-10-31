/**
 * \author Mr.Nobody
 * \defgroup Can Can
 * \brief Can module
 */

/**
 * \author Mr.Nobody
 * \file Can_Types.h
 * \ingroup Can
 * \brief Can module global types definition
 *
 * This file contains the types definitions used across the module and are
 * available for other modules through the Port file. STM32H5 has FDCAN1 on
 * every device line, FDCAN2 on every line except STM32H503 and STM32H562, and
 * FDCAN3 on the STM32H5E4 / STM32H5E5 / STM32H5F4 / STM32H5F5 lines only
 * (confirmed from the CMSIS device headers, see the module README).
 *
 */

#ifndef CAN_CAN_TYPES_H
#define CAN_CAN_TYPES_H
/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
#include "Gpio_Port.h"                      /* GPIO Port module definitions   */
#include "Nvic_Port.h"                      /* NVIC Port module definitions   */
#include "Rcc_Port.h"                       /* RCC Port module definitions    */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define CAN_NULL_PTR                           ( ( void* ) 0u )

/** Maximum data length of a CAN FD frame (byte) */
#define CAN_DATA_LEN_FD_MAX                     ( 64u )

/** Maximum data length of a classic CAN frame (byte) */
#define CAN_DATA_LEN_CLASSIC_MAX                ( 8u )

/** Number of standard (11-bit) acceptance filter elements per instance (fixed message RAM layout, RM) */
#define CAN_STD_FILTER_ELEMENT_CNT               ( 28u )

/** Number of extended (29-bit) acceptance filter elements per instance (fixed message RAM layout, RM) */
#define CAN_EXT_FILTER_ELEMENT_CNT                ( 8u )

/** Number of elements of every Rx FIFO (fixed message RAM layout, RM) */
#define CAN_RX_FIFO_ELEMENT_CNT                   ( 3u )

/** Number of Tx buffer elements (FIFO or queue, fixed message RAM layout, RM) */
#define CAN_TX_BUFFER_ELEMENT_CNT                  ( 3u )

/** Number of Tx event FIFO elements (fixed message RAM layout, RM) */
#define CAN_TX_EVENT_ELEMENT_CNT                   ( 3u )

/** Invalid Tx handle (no buffer taken) */
#define CAN_TX_HANDLE_INVALID                      ( 0xFFu )

/** Sample point / tolerance unit: 1000 = 100.0 % */
#define CAN_SAMPLE_POINT_SCALE                     ( 1000u )

/** Peripheral identification bit offset in an encoded pin value */
#define CAN_BIT_MASK_PERIPH_BIT_OFFSET             ( 15u )

/** Port identification bit offset in an encoded pin value */
#define CAN_BIT_MASK_PORT_BIT_OFFSET               ( 10u )

/** Pin identification bit offset in an encoded pin value */
#define CAN_BIT_MASK_PIN_BIT_OFFSET                ( 5u )

/** Alternate function identification bit offset in an encoded pin value */
#define CAN_BIT_MASK_AF_BIT_OFFSET                  ( 0u )

/* ========================== EXPORTED MACROS =============================== */

/** Encode a TX / RX pin configuration into a single bit-mask */
#define CAN_PIN_BIT_MASK_ENCODE( PERIPH_ID, PORT_ID, PIN_ID, AF_ID ) \
            ( ( (uint32_t)(PERIPH_ID) << CAN_BIT_MASK_PERIPH_BIT_OFFSET ) | \
              ( (uint32_t)(PORT_ID)   << CAN_BIT_MASK_PORT_BIT_OFFSET   ) | \
              ( (uint32_t)(PIN_ID)    << CAN_BIT_MASK_PIN_BIT_OFFSET    ) | \
              ( (uint32_t)(AF_ID)     << CAN_BIT_MASK_AF_BIT_OFFSET     )   )

/** Extract the peripheral identification from an encoded pin value */
#define CAN_BIT_MASK_DECODE_PERIPH( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> CAN_BIT_MASK_PERIPH_BIT_OFFSET ) & 0x1Fu )

/** Extract the port identification from an encoded pin value */
#define CAN_BIT_MASK_DECODE_PORT( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> CAN_BIT_MASK_PORT_BIT_OFFSET ) & 0x1Fu )

/** Extract the pin identification from an encoded pin value */
#define CAN_BIT_MASK_DECODE_PIN( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> CAN_BIT_MASK_PIN_BIT_OFFSET ) & 0x1Fu )

/** Extract the alternate function identification from an encoded pin value */
#define CAN_BIT_MASK_DECODE_AF( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> CAN_BIT_MASK_AF_BIT_OFFSET ) & 0x1Fu )

/* ============================== TYPEDEFS ================================== */

/** Module version */
typedef struct
{
    uint8_t     Major;                      /**< Major version number */
    uint8_t     Minor;                      /**< Minor version number */
    uint8_t     Patch;                      /**< Patch version number */
}   can_ModuleVersion_t;

/** Result of a module request */
typedef enum
{
    CAN_REQUEST_OK    = 0u,                 /**< The request succeeded */
    CAN_REQUEST_ERROR                       /**< The request failed (bad parameter, bad state, hardware timeout) */
}   can_RequestState_t;

/** Generic active / inactive function state of the module */
typedef enum
{
    CAN_FUNCTION_INACTIVE = 0u,             /**< The function is inactive */
    CAN_FUNCTION_ACTIVE                     /**< The function is active */
}   can_FunctionState_t;

/** Generic active / inactive flag state of the module */
typedef enum
{
    CAN_FLAG_INACTIVE = 0u, /**< The flag is not set */
    CAN_FLAG_ACTIVE         /**< The flag is set     */
}   can_FlagState_t;

/** List of the CAN (FDCAN) instances of the device line */
typedef enum
{
    CAN_PERIPH_1 = 0u, /**< FDCAN1 instance (every STM32H5 line)                  */
#if defined(FDCAN2)
    CAN_PERIPH_2,      /**< FDCAN2 instance                                       */
#endif
#if defined(FDCAN3)
    CAN_PERIPH_3,      /**< FDCAN3 instance (STM32H5E4 / H5E5 / H5F4 / H5F5 only) */
#endif
    CAN_PERIPH_CNT     /**< Number of CAN instances of the device line            */
}   can_PeriphId_t;

/* ========================= PIN ENUMERATIONS, EARLY ========================= */
/* Can_Pins.h uses CAN_PERIPH_1 / CAN_PERIPH_2 / CAN_PERIPH_3 / CAN_PERIPH_CNT,
 * defined just above, and its items (can_TxPin_t, can_RxPin_t) are used by
 * can_Config_t below, so it is included here rather than at the end of this
 * file. */
#include "Can_Pins.h"                       /* Generated pin enumerations     */

/** Controller state (README concept table) */
typedef enum
{
    CAN_STATE_STOPPED = 0u, /**< Configured, not on the bus (initialization mode) */
    CAN_STATE_STARTED,      /**< Takes part in the bus                            */
    CAN_STATE_SLEEP,        /**< Power-down / clock-stop requested                */
    CAN_STATE_CNT           /**< Number of state items                            */
}   can_State_t;

/** Operating mode of the controller */
typedef enum
{
    CAN_MODE_NORMAL = 0u,       /**< Normal operation                                                     */
    CAN_MODE_RESTRICTED,        /**< Restricted operation (ASM): receives / acknowledges, never transmits */
    CAN_MODE_MONITORING,        /**< Bus monitoring (listen only, silent)                                 */
    CAN_MODE_LOOPBACK_INTERNAL, /**< Internal loop-back (no pin activity)                                 */
    CAN_MODE_LOOPBACK_EXTERNAL, /**< External loop-back (pin activity, self-reception)                    */
    CAN_MODE_CNT                /**< Number of mode items                                                 */
}   can_Mode_t;

/** Identifier type of a frame or a filter */
typedef enum
{
    CAN_ID_STANDARD = 0u, /**< 11-bit standard identifier      */
    CAN_ID_EXTENDED,      /**< 29-bit extended identifier      */
    CAN_ID_TYPE_CNT       /**< Number of identifier type items */
}   can_IdType_t;

/** Frame type */
typedef enum
{
    CAN_FRAME_DATA = 0u, /**< Data frame                 */
    CAN_FRAME_REMOTE,    /**< Remote frame               */
    CAN_FRAME_TYPE_CNT   /**< Number of frame type items */
}   can_FrameType_t;

/** Receive FIFO identification */
typedef enum
{
    CAN_RX_FIFO_0 = 0u, /**< Receive FIFO 0               */
    CAN_RX_FIFO_1,      /**< Receive FIFO 1               */
    CAN_RX_FIFO_CNT     /**< Number of receive FIFO items */
}   can_RxFifoId_t;

/** Transmit buffer operation mode */
typedef enum
{
    CAN_TX_FIFO = 0u, /**< Transmit in the order of the requests */
    CAN_TX_QUEUE,     /**< Transmit by the identifier priority   */
    CAN_TX_MODE_CNT   /**< Number of transmit mode items         */
}   can_TxMode_t;

/** Acceptance filter element type */
typedef enum
{
    CAN_FILTER_MASK = 0u, /**< Classic bit mask filter (Id1 = filter, Id2 = mask) */
    CAN_FILTER_RANGE,     /**< Range filter (from Id1 to Id2)                     */
    CAN_FILTER_DUAL_ID,   /**< Dual ID filter (Id1 or Id2)                        */
    CAN_FILTER_TYPE_CNT   /**< Number of filter type items                        */
}   can_FilterType_t;

/** Acceptance filter / global filter action on a match */
typedef enum
{
    CAN_FILTER_TO_RX_FIFO_0 = 0u, /**< Store the frame in Rx FIFO 0                           */
    CAN_FILTER_TO_RX_FIFO_1,      /**< Store the frame in Rx FIFO 1                           */
    CAN_FILTER_REJECT,            /**< Reject the frame                                       */
    CAN_FILTER_PRIORITY,          /**< Mark with the high priority message flag, do not store */
    CAN_FILTER_ACTION_CNT         /**< Number of filter action items                          */
}   can_FilterAction_t;

/** Bus-off recovery mode */
typedef enum
{
    CAN_BUS_OFF_RECOVERY_AUTO = 0u, /**< Recovers by itself after the bus-off sequence */
    CAN_BUS_OFF_RECOVERY_MANUAL,    /**< Waits for Can_Set_BusOffRestart               */
    CAN_BUS_OFF_RECOVERY_CNT        /**< Number of bus-off recovery mode items         */
}   can_BusOffRecovery_t;

/** Bus state (error counter thresholds of ISO11898-1) */
typedef enum
{
    CAN_BUS_ACTIVE = 0u, /**< Error-active, below the warning limit */
    CAN_BUS_WARNING,     /**< Error counter 96 (warning)            */
    CAN_BUS_PASSIVE,     /**< Error counter 128 (error passive)     */
    CAN_BUS_OFF,         /**< Error counter 256 (bus-off)           */
    CAN_BUS_STATE_CNT    /**< Number of bus state items             */
}   can_BusState_t;

/** Last protocol error code (FDCAN_PSR.LEC / DLEC coding) */
typedef enum
{
    CAN_ERROR_NONE = 0u, /**< No error since the last clear                   */
    CAN_ERROR_STUFF,     /**< Stuff error                                     */
    CAN_ERROR_FORM,      /**< Form error                                      */
    CAN_ERROR_ACK,       /**< Acknowledge error                               */
    CAN_ERROR_BIT1,      /**< Bit1 (recessive wanted, dominant sampled) error */
    CAN_ERROR_BIT0,      /**< Bit0 (dominant wanted, recessive sampled) error */
    CAN_ERROR_CRC,       /**< CRC error                                       */
    CAN_ERROR_NO_CHANGE, /**< No CAN bus event since the last read            */
    CAN_ERROR_CNT        /**< Number of last error items                      */
}   can_LastError_t;

/** Tx buffer handle returned by Can_Set_TxRequest */
typedef uint8_t can_TxHandle_t;

/** Frame / element counter */
typedef uint8_t can_FrameCnt_t;

/** Filter counter */
typedef uint8_t can_FilterCnt_t;

/** Bit rate (bit/s) */
typedef uint32_t can_Bitrate_t;

/** Sample point or bit rate tolerance (CAN_SAMPLE_POINT_SCALE = 100.0 %) */
typedef uint16_t can_SamplePoint_t;

/** Kernel clock frequency (Hz) */
typedef uint32_t can_Frequency_Hz_t;

/** Identifier of a frame or of an acceptance filter (11 or 29 bit) */
typedef uint32_t can_Identifier_t;

/** Data length of a frame (byte) */
typedef uint8_t can_DataLength_t;

/** Application marker copied into the Tx event / Tx callback */
typedef uint8_t can_Marker_t;

/** FDCAN internal timestamp counter value */
typedef uint32_t can_Timestamp_t;

/** Transmit / receive error counter value */
typedef uint8_t can_ErrorCnt_t;

/** Bit timing (nominal and, for CAN FD, data phase) */
typedef struct
{
    can_Bitrate_t     Bitrate;                 /**< Nominal bit rate (bit/s)                                      */
    can_SamplePoint_t SamplePoint;             /**< Nominal sample point (CAN_SAMPLE_POINT_SCALE = 100.0 %)       */
    can_SamplePoint_t BitrateTolerance;        /**< Allowed bit rate deviation (CAN_SAMPLE_POINT_SCALE = 100.0 %) */
    can_Bitrate_t     DataBitrate;             /**< CAN FD data phase bit rate (bit/s), 0 when FdEnable inactive  */
    can_SamplePoint_t DataSamplePoint;         /**< CAN FD data phase sample point (CAN_SAMPLE_POINT_SCALE scale) */
    can_Bitrate_t     AchievedBitrate;         /**< Nominal bit rate actually programmed (output)                 */
    can_SamplePoint_t AchievedSamplePoint;     /**< Nominal sample point actually programmed (output)             */
    can_Bitrate_t     AchievedDataBitrate;     /**< Data phase bit rate actually programmed (output)              */
    can_SamplePoint_t AchievedDataSamplePoint; /**< Data phase sample point actually programmed (output)          */
}   can_BitTiming_t;

/** CAN frame (classic or FD, up to 64 data byte) */
typedef struct
{
    can_Identifier_t Id;                          /**< Identifier (11 or 29 bit, depending on IdType)             */
    can_IdType_t     IdType;                      /**< Standard or extended identifier                            */
    can_FrameType_t  FrameType;                   /**< Data or remote frame                                       */
    can_FlagState_t  FdFrame;                     /**< CAN FD frame format                                        */
    can_FlagState_t  BitRateSwitch;               /**< Data phase transmitted at the data bit rate                */
    can_FlagState_t  ErrorStateIndicator;         /**< Received: ESI bit of the frame                             */
    can_DataLength_t Length;                      /**< Data length (byte): 0 .. 8 classic, 0 .. 64 CAN FD         */
    uint8_t          Data[ CAN_DATA_LEN_FD_MAX ]; /**< Frame data                                                 */
    can_Marker_t     Marker;                      /**< Application marker, copied into the Tx event / Tx callback */
    can_Timestamp_t  Timestamp;                   /**< Received: FDCAN internal timestamp counter value           */
}   can_Frame_t;

/** Acceptance filter element */
typedef struct
{
    can_FilterType_t   Type;   /**< Mask, range or dual ID                      */
    can_IdType_t       IdType; /**< Standard or extended identifier             */
    can_Identifier_t   Id1;    /**< First ID (filter / range start / dual ID 1) */
    can_Identifier_t   Id2;    /**< Second ID (mask / range end / dual ID 2)    */
    can_FilterAction_t Action; /**< Action on a match                           */
}   can_Filter_t;

/** Behaviour for a frame that matches no filter / a remote frame */
typedef struct
{
    can_FilterAction_t       NonMatchingStd;         /**< Standard frame matching no filter (FIFO 0 / 1 / reject) */
    can_FilterAction_t       NonMatchingExt;         /**< Extended frame matching no filter (FIFO 0 / 1 / reject) */
    can_FlagState_t          RejectRemoteStd;        /**< Reject every standard remote frame */
    can_FlagState_t          RejectRemoteExt;        /**< Reject every extended remote frame */
}   can_GlobalFilter_t;

/** Transmit / receive error counters */
typedef struct
{
    can_ErrorCnt_t  TxErrorCnt;     /**< Transmit error counter (0 .. 255)                     */
    can_ErrorCnt_t  RxErrorCnt;     /**< Receive error counter (0 .. 127)                      */
    can_FlagState_t RxErrorPassive; /**< Receive error counter reached the error passive level */
}   can_ErrorCounters_t;

/** Transmit complete / aborted callback */
typedef void ( *can_TxCallback_t )( can_PeriphId_t periphId, can_Marker_t marker, can_RequestState_t result );

/** Frame(s) received callback (one of the two Rx FIFOs) */
typedef void ( *can_RxCallback_t )( can_PeriphId_t periphId, can_RxFifoId_t fifoId, can_FrameCnt_t frameCnt );

/** Receive FIFO overflow (frame lost) callback */
typedef void ( *can_RxOverflowCallback_t )( can_PeriphId_t periphId, can_RxFifoId_t fifoId );

/** Bus state change callback */
typedef void ( *can_BusStateCallback_t )( can_PeriphId_t periphId, can_BusState_t busState );

/** Protocol error callback (last error code changed) */
typedef void ( *can_ErrorCallback_t )( can_PeriphId_t periphId, can_LastError_t lastError );

/** Wake-up (bus activity during sleep / power-down) callback */
typedef void ( *can_WakeUpCallback_t )( can_PeriphId_t periphId );

/** Instance configuration */
typedef struct
{
    can_PeriphId_t           PeriphId;           /**< Instance to configure                                    */
    rcc_PeriphId_t           ClkSource;          /**< FDCAN kernel clock source (PLL1Q / PLL2Q / HSE, see Rcc) */
    can_BitTiming_t          BitTiming;          /**< Nominal (and, if FdEnable, data phase) bit timing        */
    can_Mode_t               Mode;               /**< Operating mode                                           */
    can_FunctionState_t      FdEnable;           /**< CAN FD frame format / bit rate switching enable          */
    can_FunctionState_t      AutoRetransmission; /**< Automatic retransmission of a frame not acknowledged     */
    can_TxMode_t             TxMode;             /**< Tx buffers operate as FIFO or as queue                   */
    can_BusOffRecovery_t     BusOffRecovery;     /**< Automatic or application-controlled bus-off recovery     */
    can_TxPin_t              TxPin;              /**< FDCAN_TX pin                                             */
    can_RxPin_t              RxPin;              /**< FDCAN_RX pin                                             */
    gpio_PortId_t            StandbyPort;        /**< Transceiver standby pin port, GPIO_PORT_CNT when unused  */
    gpio_PinId_t             StandbyPin;         /**< Transceiver standby pin, GPIO_PIN_ID_CNT when unused     */
    nvic_IrqPrio_t           IrqPriority;        /**< Priority of the instance interrupt line                  */
    can_TxCallback_t         TxCallback;         /**< Transmit callback, CAN_NULL_PTR disables it              */
    can_RxCallback_t         RxCallback;         /**< Frame(s) received callback, CAN_NULL_PTR to disable      */
    can_RxOverflowCallback_t RxOverflowCallback; /**< FIFO overflow callback, CAN_NULL_PTR to disable          */
    can_BusStateCallback_t   BusStateCallback;   /**< Bus state change callback, CAN_NULL_PTR to disable       */
    can_ErrorCallback_t      ErrorCallback;      /**< Protocol error callback, CAN_NULL_PTR to disable         */
    can_WakeUpCallback_t     WakeUpCallback;     /**< Wake-up callback, CAN_NULL_PTR to disable                */
}   can_Config_t;

#endif /* CAN_CAN_TYPES_H */
