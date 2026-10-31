/**
 * \author Mr.Nobody
 * \file Can.c
 * \ingroup Can
 * \brief Can module source file (STM32H5 FDCAN implementation)
 *
 * Bit timing is searched from the kernel clock reported by Rcc for the
 * requested bit rate / sample point, exactly per the NBTP / DBTP field
 * formulas of RM0481/RM0492/RM0517 (nominal bit time = [NTSEG1+NTSEG2+3] tq,
 * data bit time = [DTSEG1+DTSEG2+3] tq, tq = (BRP+1) kernel clock periods).
 * The message RAM layout (acceptance filters, Rx FIFOs, Tx buffers, Tx event
 * FIFO) is fixed per instance (see Can_Reg.h/.c) - STM32H5 has no RAM size
 * configuration registers.
 *
 */

/* ============================= INCLUDES =================================== */
#include "Can.h"                            /* Module private header          */
#include "Can_Reg.h"                        /* Module Reg component header    */
#include "Can_Port.h"                       /* Module public interface        */
#include "Stm32.h"                          /* Device registers definition    */
#include "Rcc_Port.h"                       /* RCC Port module definitions     */
#include "Nvic_Port.h"                      /* NVIC Port module definitions    */
#include "Gpio_Port.h"                      /* GPIO Port module definitions    */

#if defined(FDCAN1)

/* ========================== PRIVATE DECLARATIONS =========================== */

/** \brief Module version, major */
#define CAN_VERSION_MAJOR           ( 1u )
/** \brief Module version, minor */
#define CAN_VERSION_MINOR           ( 0u )
/** \brief Module version, patch */
#define CAN_VERSION_PATCH           ( 0u )

/** \brief Lowest practical number of time quanta per bit accepted by the bit timing search */
#define CAN_TQ_PER_BIT_MIN            ( 5u )
/** \brief Highest number of time quanta per bit accepted by the nominal bit timing search */
#define CAN_NOMINAL_TQ_PER_BIT_MAX     ( 100u )
/** \brief Highest number of time quanta per bit accepted by the data phase bit timing search */
#define CAN_DATA_TQ_PER_BIT_MAX         ( 48u )

/** \brief Highest prescaler value accepted by the nominal bit timing search (NBRP field, 9 bit) */
#define CAN_NOMINAL_PRESCALER_MAX        ( 512u )
/** \brief Highest prescaler value accepted by the data phase bit timing search (DBRP field, 5 bit) */
#define CAN_DATA_PRESCALER_MAX            ( 32u )

/** \brief Highest tseg1 (sync excluded) value of the nominal bit timing (NTSEG1 field, 8 bit) */
#define CAN_NOMINAL_TSEG1_MAX              ( 256u )
/** \brief Highest tseg2 value of the nominal bit timing (NTSEG2 field, 7 bit) */
#define CAN_NOMINAL_TSEG2_MAX               ( 128u )
/** \brief Highest tseg1 (sync excluded) value of the data phase bit timing (DTSEG1 field, 5 bit) */
#define CAN_DATA_TSEG1_MAX                   ( 32u )
/** \brief Highest tseg2 value of the data phase bit timing (DTSEG2 field, 4 bit) */
#define CAN_DATA_TSEG2_MAX                    ( 16u )

/** \brief Sentinel "no candidate found yet" value of the bit timing search (never a valid ppm deviation) */
#define CAN_BITTIMING_DEV_INVALID               ( 0xFFFFFFFFu )

/** \brief Count of the FDCAN DLC codes (0 .. 15, RM Table 676) */
#define CAN_DLC_CODE_CNT                          ( 16u )

/** \brief First DLC code with an FD-only (not 1:1 byte) data length (RM Table 676) */
#define CAN_DLC_CODE_FD_FIRST                       ( 9u )

/** \brief Count of the FD-only DLC codes (CAN_DLC_CODE_FD_FIRST .. CAN_DLC_CODE_CNT - 1) */
#define CAN_FD_DLC_CNT                                ( CAN_DLC_CODE_CNT - CAN_DLC_CODE_FD_FIRST )

/** \brief Mask covering every bit of a 32-bit register */
#define CAN_REG_MASK_ALL                              ( 0xFFFFFFFFu )

/** \brief Default configuration: nominal bit rate (bit/s), 500 kbit/s */
#define CAN_DEFAULT_BITRATE                             ( 500000u )
/** \brief Default configuration: nominal sample point (CAN_SAMPLE_POINT_SCALE = 87.5 %) */
#define CAN_DEFAULT_SAMPLE_POINT                          ( 875u )
/** \brief Default configuration: bit rate tolerance (CAN_SAMPLE_POINT_SCALE = 0.5 %) */
#define CAN_DEFAULT_BITRATE_TOLERANCE                       ( 5u )
/** \brief Default configuration: CAN FD data phase sample point (CAN_SAMPLE_POINT_SCALE = 80 %) */
#define CAN_DEFAULT_DATA_SAMPLE_POINT                         ( 800u )

/** \brief FDCAN_RXGFC: LSS (standard filter list size) field shift */
#define CAN_RXGFC_LSS_SHIFT                                     ( 16u )
/** \brief FDCAN_RXGFC: LSE (extended filter list size) field shift */
#define CAN_RXGFC_LSE_SHIFT                                      ( 24u )

/** \brief FDCAN_RXGFC: ANFS (accept non-matching standard) field shift */
#define CAN_RXGFC_ANFS_SHIFT                                      ( 4u )
/** \brief FDCAN_RXGFC: ANFE (accept non-matching extended) field shift */
#define CAN_RXGFC_ANFE_SHIFT                                       ( 2u )
/** \brief FDCAN_RXGFC: ANFS / ANFE value "reject" */
#define CAN_RXGFC_ANF_REJECT                                        ( 2u )

/** \brief Count of data byte packed into one message RAM word */
#define CAN_WORD_BYTE_CNT                                          ( 4u )
/** \brief Bit width of one data byte inside a word (shift unit between two consecutive bytes) */
#define CAN_BYTE_SHIFT_UNIT                                         ( 8u )
/** \brief Mask of one data byte */
#define CAN_BYTE_MASK                                                ( 0xFFu )

/** \brief FDCAN_RXF0S / FDCAN_RXF1S: get index field mask */
#define CAN_RXFS_GETIDX_MASK                                            ( 0x300u )
/** \brief FDCAN_RXF0S / FDCAN_RXF1S: get index field shift */
#define CAN_RXFS_GETIDX_SHIFT                                            ( 8u )
/** \brief FDCAN_RXF0A / FDCAN_RXF1A: acknowledge index field mask */
#define CAN_RXFA_ACKIDX_MASK                                              ( 0x7u )
/** \brief FDCAN_RXF0S / FDCAN_RXF1S: fill level field mask */
#define CAN_RXFS_FILLLVL_MASK                                              ( 0xFu )

/** \brief T0 word of a message RAM element / standard filter: error state indicator bit */
#define CAN_RAM_T0_ESI                          ( 0x80000000u )
/** \brief T0 word of a message RAM element: extended identifier bit */
#define CAN_RAM_T0_XTD                           ( 0x40000000u )
/** \brief T0 word of a message RAM element: remote frame bit */
#define CAN_RAM_T0_RTR                            ( 0x20000000u )
/** \brief T0 word of a message RAM element: identifier mask */
#define CAN_RAM_T0_ID_MASK                         ( 0x1FFFFFFFu )
/** \brief Bit position of the standard identifier inside the 29-bit ID field */
#define CAN_RAM_STD_ID_SHIFT                        ( 18u )

/** \brief T1 word of a message RAM element: message marker shift */
#define CAN_RAM_T1_MM_SHIFT                          ( 24u )
/** \brief T1 word of a message RAM element: message marker mask (shifted) */
#define CAN_RAM_T1_MM_MASK                            ( 0xFF000000u )
/** \brief T1 word of a message RAM element: event FIFO control bit */
#define CAN_RAM_T1_EFC                                 ( 0x00800000u )
/** \brief T1 word of a message RAM element: FD format bit */
#define CAN_RAM_T1_FDF                                  ( 0x00200000u )
/** \brief T1 word of a message RAM element: bit rate switch bit */
#define CAN_RAM_T1_BRS                                   ( 0x00100000u )
/** \brief T1 word of a message RAM element: DLC shift */
#define CAN_RAM_T1_DLC_SHIFT                              ( 16u )
/** \brief T1 word of a message RAM element: DLC mask (shifted) */
#define CAN_RAM_T1_DLC_MASK                                ( 0x000F0000u )
/** \brief R1 word of an Rx FIFO element: received timestamp mask */
#define CAN_RAM_R1_RXTS_MASK                                ( 0x0000FFFFu )

/** \brief Standard filter element: SFT (filter type) shift */
#define CAN_RAM_SF_SFT_SHIFT                                 ( 30u )
/** \brief Standard filter element: SFEC (filter action) shift */
#define CAN_RAM_SF_SFEC_SHIFT                                 ( 27u )
/** \brief Standard filter element: SFID1 shift */
#define CAN_RAM_SF_SFID1_SHIFT                                 ( 16u )
/** \brief Standard filter element: 11-bit identifier mask */
#define CAN_RAM_SF_ID_MASK                                     ( 0x7FFu )

/** \brief Extended filter element: EFEC (filter action) shift */
#define CAN_RAM_EF_EFEC_SHIFT                                   ( 29u )
/** \brief Extended filter element: EFT (filter type) shift */
#define CAN_RAM_EF_EFT_SHIFT                                     ( 30u )
/** \brief Extended filter element: 29-bit identifier mask */
#define CAN_RAM_EF_ID_MASK                                       ( 0x1FFFFFFFu )

/** \brief Filter element configuration: disabled */
#define CAN_RAM_FEC_DISABLE                                       ( 0u )
/** \brief Filter element configuration: store in Rx FIFO 0 */
#define CAN_RAM_FEC_FIFO0                                          ( 1u )
/** \brief Filter element configuration: store in Rx FIFO 1 */
#define CAN_RAM_FEC_FIFO1                                           ( 2u )
/** \brief Filter element configuration: reject */
#define CAN_RAM_FEC_REJECT                                          ( 3u )
/** \brief Filter element configuration: set priority */
#define CAN_RAM_FEC_PRIORITY                                         ( 4u )

/** \brief Standard filter element: filter type mask */
#define CAN_RAM_SFT_MASK                                              ( 2u )
/** \brief Standard / extended filter element: filter type range */
#define CAN_RAM_FT_RANGE                                               ( 0u )
/** \brief Standard / extended filter element: filter type dual ID */
#define CAN_RAM_FT_DUAL_ID                                              ( 1u )

/** \brief Byte value used to pad unused data bytes of a frame */
#define CAN_DATA_PAD_BYTE                                                ( 0u )

/* =========================== LOCAL VARIABLES ============================== */

/** \brief Per-instance run-time state */
static can_InstanceState_t can_State[ CAN_PERIPH_CNT ];

/* ========================== PRIVATE FUNCTIONS ============================== */

/**
 * \brief Converts a data length (byte) into the FDCAN DLC code (RM Table 676).
 */
static uint8_t Can_Len_To_Dlc( uint8_t length )
{
    /* Data length (byte) of every FD-only DLC code 9 .. 15, indexed from 0 (RM Table 676) */
    static const uint8_t canFdLenTable[ CAN_FD_DLC_CNT ] = { 12u, 16u, 20u, 24u, 32u, 48u, CAN_DATA_LEN_FD_MAX };
    uint8_t dlc   = 0u;
    uint8_t found = 0u;

    if( CAN_DATA_LEN_CLASSIC_MAX >= length )
    {
        dlc = length;
    }
    else
    {
        dlc = CAN_DLC_CODE_CNT - 1u;

        for( uint8_t idx = 0u;
             ( CAN_FD_DLC_CNT > idx    ) &&
             ( 0u             == found )    ;
             idx++ )
        {
            if( canFdLenTable[ idx ] >= length )
            {
                dlc   = CAN_DLC_CODE_FD_FIRST + idx;
                found = 1u;
            }
            else
            {
                /* Not yet the smallest FD length covering the requested length */
            }
        }
    }

    return ( dlc );
}


/**
 * \brief Converts an FDCAN DLC code into a data length (byte) (RM Table 676).
 */
static uint8_t Can_Dlc_To_Len( uint8_t dlc )
{
    /* Data length (byte) of every DLC code 0 .. 15 (RM Table 676) */
    static const uint8_t canDlcLen[ CAN_DLC_CODE_CNT ] = { 0u,1u,2u,3u,4u,5u,6u,7u,8u,12u,16u,20u,24u,32u,48u,64u };
    uint8_t length = 0u;

    if( CAN_DLC_CODE_CNT > dlc )
    {
        length = canDlcLen[ dlc ];
    }
    else
    {
        /* Invalid DLC, no data byte reported */
        length = 0u;
    }

    return ( length );
}


/**
 * \brief Searches the prescaler / tseg1 / tseg2 combination closest to the requested
 *        bit rate and sample point for the given kernel clock.
 *
 * \param kernelClkHz   [in]  : Kernel clock frequency (Hz).
 * \param bitrate       [in]  : Requested bit rate (bit/s).
 * \param samplePoint   [in]  : Requested sample point (CAN_SAMPLE_POINT_SCALE = 100 %).
 * \param tolerance     [in]  : Allowed bit rate deviation (CAN_SAMPLE_POINT_SCALE = 100 %).
 * \param prescalerMax  [in]  : Highest prescaler value accepted (field range).
 * \param tseg1Max      [in]  : Highest tseg1 (sync excluded) value accepted (field range).
 * \param tseg2Max      [in]  : Highest tseg2 value accepted (field range).
 * \param tqPerBitMax   [in]  : Highest total time quanta per bit searched.
 * \param prescaler     [out] : Prescaler found (1 .. prescalerMax).
 * \param tseg1         [out] : tseg1 found (sync excluded, 1 .. tseg1Max).
 * \param tseg2         [out] : tseg2 found (1 .. tseg2Max).
 * \param achievedRate  [out] : Bit rate actually achieved with the found values (bit/s).
 * \param achievedPoint [out] : Sample point actually achieved with the found values.
 *
 * \return CAN_REQUEST_OK when a combination within the tolerance was found, CAN_REQUEST_ERROR otherwise.
 */
static can_RequestState_t Can_Search_BitTiming( can_Frequency_Hz_t kernelClkHz,
                                                can_Bitrate_t bitrate,
                                                can_SamplePoint_t samplePoint,
                                                can_SamplePoint_t tolerance,
                                                uint32_t prescalerMax,
                                                uint32_t tseg1Max,
                                                uint32_t tseg2Max,
                                                uint32_t tqPerBitMax,
                                                uint32_t * const prescaler,
                                                uint32_t * const tseg1,
                                                uint32_t * const tseg2,
                                                can_Bitrate_t * const achievedRate,
                                                can_SamplePoint_t * const achievedPoint )
{
    can_RequestState_t retState    = CAN_REQUEST_ERROR;
    uint32_t              bestDevPpm = CAN_BITTIMING_DEV_INVALID;
    uint32_t              candidateRate   = 0u;
    uint32_t              candidateDevPpm = 0u;
    uint32_t              tSeg1Phys       = 0u;
    uint32_t              tSeg2Phys       = 0u;

    for( uint32_t presc = 1u;
         ( prescalerMax >= presc    ) &&
         ( 0u            != bitrate     ) &&
         ( 0u            != kernelClkHz )    ;
         presc++ )
    {
        for( uint32_t tqPerBit = CAN_TQ_PER_BIT_MIN; tqPerBitMax >= tqPerBit; tqPerBit++ )
        {
            candidateRate = kernelClkHz / ( presc * tqPerBit );

            if( candidateRate >= bitrate )
            {
                candidateDevPpm = ( ( candidateRate - bitrate ) * CAN_SAMPLE_POINT_SCALE ) / bitrate;
            }
            else
            {
                candidateDevPpm = ( ( bitrate - candidateRate ) * CAN_SAMPLE_POINT_SCALE ) / bitrate;
            }

            /* RM0481 S60.4.7: functional bit time = SYNC_SEG(1) + tseg1Phys + tseg2Phys = tqPerBit,
             * the sync segment is excluded from tseg1Phys before the sample point split. */
            tSeg1Phys = ( ( (uint32_t)samplePoint * tqPerBit ) / CAN_SAMPLE_POINT_SCALE );

            if( 1u < tSeg1Phys )
            {
                tSeg1Phys = tSeg1Phys - 1u;
            }
            else
            {
                tSeg1Phys = 1u;
            }

            if( tqPerBit > ( tSeg1Phys + 1u ) )
            {
                tSeg2Phys = tqPerBit - 1u - tSeg1Phys;
            }
            else
            {
                tSeg2Phys = 1u;
            }

            if( ( tolerance        >= candidateDevPpm ) &&
                ( bestDevPpm        > candidateDevPpm ) &&
                ( tseg1Max          >= tSeg1Phys       ) &&
                ( tseg2Max          >= tSeg2Phys       ) &&
                ( 1u                <= tSeg1Phys       ) &&
                ( 1u                <= tSeg2Phys       )    )
            {
                bestDevPpm       = candidateDevPpm;
                *prescaler        = presc;
                *tseg1            = tSeg1Phys;
                *tseg2            = tSeg2Phys;
                *achievedRate     = candidateRate;
                *achievedPoint    = (uint16_t)( ( ( 1u + tSeg1Phys ) * CAN_SAMPLE_POINT_SCALE ) / tqPerBit );
            }
            else
            {
                /* Candidate outside the tolerance or the field range, not kept */
            }
        }
    }

    if( CAN_BITTIMING_DEV_INVALID != bestDevPpm )
    {
        retState = CAN_REQUEST_OK;
    }
    else
    {
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes one standard (11-bit) acceptance filter element to the message RAM.
 */
static void Can_Write_StdFilter( can_PeriphId_t periphId, can_FilterCnt_t idx, const can_Filter_t * const filter )
{
    uint32_t fec       = CAN_RAM_FEC_DISABLE;
    uint32_t sft        = CAN_RAM_FT_RANGE;
    uint32_t word        = 0u;

    if( CAN_FILTER_TO_RX_FIFO_0 == filter->Action )
    {
        fec = CAN_RAM_FEC_FIFO0;
    }
    else if( CAN_FILTER_TO_RX_FIFO_1 == filter->Action )
    {
        fec = CAN_RAM_FEC_FIFO1;
    }
    else if( CAN_FILTER_REJECT == filter->Action )
    {
        fec = CAN_RAM_FEC_REJECT;
    }
    else
    {
        fec = CAN_RAM_FEC_PRIORITY;
    }

    if( CAN_FILTER_RANGE == filter->Type )
    {
        sft = CAN_RAM_FT_RANGE;
    }
    else if( CAN_FILTER_DUAL_ID == filter->Type )
    {
        sft = CAN_RAM_FT_DUAL_ID;
    }
    else
    {
        sft = CAN_RAM_SFT_MASK;
    }

    word = ( sft  << CAN_RAM_SF_SFT_SHIFT  ) |
           ( fec  << CAN_RAM_SF_SFEC_SHIFT ) |
           ( ( filter->Id1 & CAN_RAM_SF_ID_MASK ) << CAN_RAM_SF_SFID1_SHIFT ) |
           (   filter->Id2 & CAN_RAM_SF_ID_MASK                            );

    Can_Ram_Set_Word( periphId, CAN_RAM_WORD_OFFSET_STD_FILTER + idx, word );
}


/**
 * \brief Writes one extended (29-bit) acceptance filter element to the message RAM.
 */
static void Can_Write_ExtFilter( can_PeriphId_t periphId, can_FilterCnt_t idx, const can_Filter_t * const filter )
{
    uint32_t fec       = CAN_RAM_FEC_DISABLE;
    uint32_t eft        = CAN_RAM_FT_RANGE;
    uint32_t wordOffset  = 0u;

    if( CAN_FILTER_TO_RX_FIFO_0 == filter->Action )
    {
        fec = CAN_RAM_FEC_FIFO0;
    }
    else if( CAN_FILTER_TO_RX_FIFO_1 == filter->Action )
    {
        fec = CAN_RAM_FEC_FIFO1;
    }
    else if( CAN_FILTER_REJECT == filter->Action )
    {
        fec = CAN_RAM_FEC_REJECT;
    }
    else
    {
        fec = CAN_RAM_FEC_PRIORITY;
    }

    if( CAN_FILTER_RANGE == filter->Type )
    {
        eft = CAN_RAM_FT_RANGE;
    }
    else if( CAN_FILTER_DUAL_ID == filter->Type )
    {
        eft = CAN_RAM_FT_DUAL_ID;
    }
    else
    {
        eft = CAN_RAM_SFT_MASK;
    }

    wordOffset = CAN_RAM_WORD_OFFSET_EXT_FILTER + ( idx * CAN_RAM_WORD_CNT );

    Can_Ram_Set_Word( periphId,
                      wordOffset,
                      ( fec << CAN_RAM_EF_EFEC_SHIFT ) |
                      ( filter->Id1 & CAN_RAM_EF_ID_MASK ) );
    Can_Ram_Set_Word( periphId,
                      wordOffset + 1u,
                      ( eft << CAN_RAM_EF_EFT_SHIFT  ) |
                      ( filter->Id2 & CAN_RAM_EF_ID_MASK ) );
}


/**
 * \brief Disables every filter element of both lists (all elements set to SFT/EFT = disabled).
 */
static void Can_Clear_Filters( can_PeriphId_t periphId )
{
    uint32_t disabledStd = ( CAN_RAM_SFT_MASK + 1u ) << CAN_RAM_SF_SFT_SHIFT; /* SFT = 11b = disabled */
    uint32_t disabledExt = ( CAN_RAM_SFT_MASK + 1u ) << CAN_RAM_EF_EFT_SHIFT; /* EFT = 11b, XIDAM not applied */

    for( uint8_t idx = 0u; CAN_STD_FILTER_ELEMENT_CNT > idx; idx++ )
    {
        Can_Ram_Set_Word( periphId, CAN_RAM_WORD_OFFSET_STD_FILTER + idx, disabledStd );
    }

    for( uint8_t idx = 0u; CAN_EXT_FILTER_ELEMENT_CNT > idx; idx++ )
    {
        Can_Ram_Set_Word( periphId, CAN_RAM_WORD_OFFSET_EXT_FILTER + ( idx * CAN_RAM_WORD_CNT ) + 1u, disabledExt );
    }
}


/**
 * \brief Configures one GPIO pin as an FDCAN alternate function pin (TX or RX, same electrical setup).
 */
static can_RequestState_t Can_Init_AltFuncGpio( uint32_t pinCode )
{
    can_RequestState_t retState      = CAN_REQUEST_ERROR;
    gpio_RequestState_t gpioInitState = GPIO_REQUEST_ERROR;
    gpio_Config_t         pinConfig     = { 0u };

    pinConfig.PortId         = (gpio_PortId_t)CAN_BIT_MASK_DECODE_PORT( pinCode );
    pinConfig.PinId          = (gpio_PinId_t)CAN_BIT_MASK_DECODE_PIN( pinCode );
    pinConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
    pinConfig.PinPull        = GPIO_PIN_PULL_NONE;
    pinConfig.PinSpeed       = GPIO_PIN_SPEED_HIGH;
    pinConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    pinConfig.PinAltFunction = (gpio_AltFunction_t)CAN_BIT_MASK_DECODE_AF( pinCode );

    gpioInitState = Gpio_Init( &pinConfig );

    if( GPIO_REQUEST_ERROR != gpioInitState )
    {
        retState = CAN_REQUEST_OK;
    }
    else
    {
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the FDCAN_PSR register and derives the module bus state enum from it.
 */
static can_BusState_t Can_Read_BusState( can_PeriphId_t periphId )
{
    uint32_t         psrVal  = Can_Get_RegVal( periphId, CAN_REG_PSR, FDCAN_PSR_BO | FDCAN_PSR_EP | FDCAN_PSR_EW );
    can_BusState_t    busState = CAN_BUS_ACTIVE;

    if( 0u != ( psrVal & FDCAN_PSR_BO ) )
    {
        busState = CAN_BUS_OFF;
    }
    else if( 0u != ( psrVal & FDCAN_PSR_EP ) )
    {
        busState = CAN_BUS_PASSIVE;
    }
    else if( 0u != ( psrVal & FDCAN_PSR_EW ) )
    {
        busState = CAN_BUS_WARNING;
    }
    else
    {
        busState = CAN_BUS_ACTIVE;
    }

    return ( busState );
}


/**
 * \brief Services one instance: dispatches the configured callbacks from the current hardware
 *        state. Called both from Can_Task (poll mode) and from the instance ISR trampoline.
 */
static void Can_Service( can_PeriphId_t periphId )
{
    can_InstanceState_t * const instState = &can_State[ periphId ];
    can_BusState_t                 busState  = CAN_BUS_ACTIVE;
    uint32_t                        lecVal   = Can_Get_RegVal( periphId, CAN_REG_PSR, FDCAN_PSR_LEC );
    can_LastError_t                  lastError = (can_LastError_t)lecVal;
    uint32_t                          rx0Cnt   = Can_Get_RegVal( periphId, CAN_REG_RXF0S, FDCAN_RXF0S_F0FL );
    uint32_t                           rx1Cnt   = Can_Get_RegVal( periphId, CAN_REG_RXF1S, FDCAN_RXF1S_F1FL );

    if( CAN_FUNCTION_ACTIVE != instState->Initialized )
    {
        /* Instance not initialised, nothing to service */
    }
    else
    {
        busState = Can_Read_BusState( periphId );

        if( busState != instState->LastBusState )
        {
            instState->LastBusState = busState;

            if( CAN_NULL_PTR != instState->Config.BusStateCallback )
            {
                instState->Config.BusStateCallback( periphId, busState );
            }
            else
            {
                /* No bus state callback configured */
            }
        }
        else
        {
            /* Bus state unchanged, no callback */
        }

        if( lastError != instState->LastError )
        {
            instState->LastError = lastError;

            if( ( CAN_NULL_PTR       != instState->Config.ErrorCallback ) &&
                ( CAN_ERROR_NO_CHANGE != lastError )                         )
            {
                instState->Config.ErrorCallback( periphId, lastError );
            }
            else
            {
                /* No error callback configured, or no new error to report */
            }
        }
        else
        {
            /* Last error unchanged, no callback */
        }

        if( ( 0u          != rx0Cnt ) &&
            ( CAN_NULL_PTR != instState->Config.RxCallback )    )
        {
            instState->Config.RxCallback( periphId, CAN_RX_FIFO_0, (can_FrameCnt_t)rx0Cnt );
        }
        else
        {
            /* Rx FIFO 0 empty, or no receive callback configured */
        }

        if( ( 0u          != rx1Cnt ) &&
            ( CAN_NULL_PTR != instState->Config.RxCallback )    )
        {
            instState->Config.RxCallback( periphId, CAN_RX_FIFO_1, (can_FrameCnt_t)rx1Cnt );
        }
        else
        {
            /* Rx FIFO 1 empty, or no receive callback configured */
        }

        if( ( CAN_BUS_OFF                == busState ) &&
            ( CAN_BUS_OFF_RECOVERY_AUTO  == instState->Config.BusOffRecovery )    )
        {
            Can_Reset_RegBit( periphId, CAN_REG_CCCR, FDCAN_CCCR_INIT );
        }
        else
        {
            /* Not in bus-off, or manual recovery requested: INIT left as is */
        }
    }
}


#if defined(FDCAN1)
/** \brief ISR trampoline for FDCAN1 interrupt line 0 (registered through Nvic) */
static void Can_Isr_Fdcan1_It0( void )
{
    Can_Service( CAN_PERIPH_1 );
}
#endif

#if defined(FDCAN2)
/** \brief ISR trampoline for FDCAN2 interrupt line 0 (registered through Nvic) */
static void Can_Isr_Fdcan2_It0( void )
{
    Can_Service( CAN_PERIPH_2 );
}
#endif

#if defined(FDCAN3)
/** \brief ISR trampoline for FDCAN3 interrupt line 0 (registered through Nvic) */
static void Can_Isr_Fdcan3_It0( void )
{
    Can_Service( CAN_PERIPH_3 );
}
#endif


/**
 * \brief Returns the NVIC interrupt line 0 identification of the given instance.
 */
static nvic_PeriphIrqList_t Can_Get_IrqLine( can_PeriphId_t periphId )
{
    nvic_PeriphIrqList_t irqLine = NVIC_PERIPH_IRQ_FDCAN1_IT0;

#if !defined(FDCAN2) && \
    !defined(FDCAN3)
    (void)periphId; /* Single instance line: parameter not evaluated */
#endif
#if defined(FDCAN2)
    if( CAN_PERIPH_2 == periphId )
    {
        irqLine = NVIC_PERIPH_IRQ_FDCAN2_IT0;
    }
    else
#endif
#if defined(FDCAN3)
    if( CAN_PERIPH_3 == periphId )
    {
        irqLine = NVIC_PERIPH_IRQ_FDCAN3_IT0;
    }
    else
#endif
    {
        irqLine = NVIC_PERIPH_IRQ_FDCAN1_IT0;
    }

    return ( irqLine );
}


/**
 * \brief Returns the ISR trampoline function of the given instance.
 */
static nvic_IsrCallback_t Can_Get_IsrHandler( can_PeriphId_t periphId )
{
    nvic_IsrCallback_t isrHandler = Can_Isr_Fdcan1_It0;

#if !defined(FDCAN2) && \
    !defined(FDCAN3)
    (void)periphId; /* Single instance line: parameter not evaluated */
#endif
#if defined(FDCAN2)
    if( CAN_PERIPH_2 == periphId )
    {
        isrHandler = Can_Isr_Fdcan2_It0;
    }
    else
#endif
#if defined(FDCAN3)
    if( CAN_PERIPH_3 == periphId )
    {
        isrHandler = Can_Isr_Fdcan3_It0;
    }
    else
#endif
    {
        isrHandler = Can_Isr_Fdcan1_It0;
    }

    return ( isrHandler );
}

/* ========================= EXPORTED FUNCTIONS =============================== */

/**
 * \brief Gets the version of the module.
 *
 * \return Module version.
 */
can_ModuleVersion_t Can_Get_ModuleVersion( void )
{
    can_ModuleVersion_t moduleVersion = { 0u };

    moduleVersion.Major = CAN_VERSION_MAJOR;
    moduleVersion.Minor = CAN_VERSION_MINOR;
    moduleVersion.Patch = CAN_VERSION_PATCH;

    return ( moduleVersion );
}


/**
 * \brief Fills a configuration structure with safe default values (500 kbit/s nominal,
 *        normal mode, Tx FIFO, automatic retransmission, automatic bus-off recovery).
 *
 * \param canConfig [out] : Configuration structure to fill.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a null pointer.
 */
can_RequestState_t Can_Get_DefaultConfig( can_Config_t * const canConfig )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( CAN_NULL_PTR != canConfig )
    {
        canConfig->PeriphId                    = CAN_PERIPH_1;
        canConfig->ClkSource                   = RCC_PERIPH_FDCAN_PLL1Q;
        canConfig->BitTiming.Bitrate            = CAN_DEFAULT_BITRATE;
        canConfig->BitTiming.SamplePoint        = CAN_DEFAULT_SAMPLE_POINT;
        canConfig->BitTiming.BitrateTolerance   = CAN_DEFAULT_BITRATE_TOLERANCE;
        canConfig->BitTiming.DataBitrate        = 0u;
        canConfig->BitTiming.DataSamplePoint    = CAN_DEFAULT_DATA_SAMPLE_POINT;
        canConfig->Mode                         = CAN_MODE_NORMAL;
        canConfig->FdEnable                     = CAN_FUNCTION_INACTIVE;
        canConfig->AutoRetransmission           = CAN_FUNCTION_ACTIVE;
        canConfig->TxMode                       = CAN_TX_FIFO;
        canConfig->BusOffRecovery                = CAN_BUS_OFF_RECOVERY_AUTO;
        canConfig->TxPin                         = CAN_TX_PIN_UNUSED;
        canConfig->RxPin                         = CAN_RX_PIN_UNUSED;
        canConfig->StandbyPort                   = GPIO_PORT_CNT;
        canConfig->StandbyPin                    = GPIO_PIN_ID_CNT;
        canConfig->IrqPriority                   = 0u;
        canConfig->TxCallback                    = CAN_NULL_PTR;
        canConfig->RxCallback                    = CAN_NULL_PTR;
        canConfig->RxOverflowCallback            = CAN_NULL_PTR;
        canConfig->BusStateCallback              = CAN_NULL_PTR;
        canConfig->ErrorCallback                 = CAN_NULL_PTR;
        canConfig->WakeUpCallback                = CAN_NULL_PTR;

        retState = CAN_REQUEST_OK;
    }
    else
    {
        /* Null pointer, nothing filled */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initialises one FDCAN instance: clock, bit timing, mode, pins, interrupt line.
 *        Leaves the instance in CAN_STATE_STOPPED (Can_Set_Start brings it onto the bus).
 *
 * \param canConfig [in] : Configuration of the instance.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter or a hardware timeout
 *         (the instance is deinitialised in this case).
 */
can_RequestState_t Can_Init( const can_Config_t * const canConfig )
{
    can_RequestState_t     retState  = CAN_REQUEST_ERROR;
    can_RequestState_t      cceState  = CAN_REQUEST_ERROR;
    rcc_RequestState_t       rccState  = RCC_REQUEST_ERROR;
    can_InstanceState_t *     instState = CAN_NULL_PTR;

    if( ( CAN_NULL_PTR    != canConfig                 ) &&
        ( CAN_PERIPH_CNT  > canConfig->PeriphId )              )
    {
        instState = &can_State[ canConfig->PeriphId ];

        rccState = Rcc_Set_PeriphActive( canConfig->ClkSource );
        (void)Rcc_Set_ResetActive( canConfig->ClkSource );
        (void)Rcc_Set_ResetInactive( canConfig->ClkSource );

        Can_Set_RegBit( canConfig->PeriphId, CAN_REG_CCCR, FDCAN_CCCR_INIT );
        cceState = Can_Reg_Wait_RegBitSet( canConfig->PeriphId, CAN_REG_CCCR, FDCAN_CCCR_INIT );
        Can_Set_RegBit( canConfig->PeriphId, CAN_REG_CCCR, FDCAN_CCCR_CCE );

        instState->Config      = *canConfig;
        instState->State        = CAN_STATE_STOPPED;
        instState->LastBusState = CAN_BUS_ACTIVE;
        instState->LastError    = CAN_ERROR_NO_CHANGE;

        if( ( RCC_REQUEST_OK != rccState ) ||
            ( CAN_REQUEST_OK != cceState )    )
        {
            instState->Initialized = CAN_FUNCTION_INACTIVE;
            (void)Can_Deinit( canConfig->PeriphId );
            retState = CAN_REQUEST_ERROR;
        }
        else
        {
            instState->Initialized = CAN_FUNCTION_ACTIVE;

            Can_Clear_Filters( canConfig->PeriphId );
            (void)Can_Set_BitTiming( canConfig->PeriphId, &canConfig->BitTiming );
            (void)Can_Set_Mode( canConfig->PeriphId, canConfig->Mode );
            (void)Can_Set_AutoRetransmission( canConfig->PeriphId, canConfig->AutoRetransmission );

            if( CAN_FUNCTION_ACTIVE == canConfig->FdEnable )
            {
                Can_Set_RegBit( canConfig->PeriphId, CAN_REG_CCCR, FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE );
            }
            else
            {
                Can_Reset_RegBit( canConfig->PeriphId, CAN_REG_CCCR, FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE );
            }

            if( CAN_TX_QUEUE == canConfig->TxMode )
            {
                Can_Set_RegBit( canConfig->PeriphId, CAN_REG_TXBC, FDCAN_TXBC_TFQM );
            }
            else
            {
                Can_Reset_RegBit( canConfig->PeriphId, CAN_REG_TXBC, FDCAN_TXBC_TFQM );
            }

            Can_Set_RegVal( canConfig->PeriphId,
                            CAN_REG_RXGFC,
                            FDCAN_RXGFC_LSS | FDCAN_RXGFC_LSE,
                            ( (uint32_t)CAN_STD_FILTER_ELEMENT_CNT << CAN_RXGFC_LSS_SHIFT ) |
                            ( (uint32_t)CAN_EXT_FILTER_ELEMENT_CNT << CAN_RXGFC_LSE_SHIFT )   );

            Can_Set_RegBit( canConfig->PeriphId, CAN_REG_ILE, 1u );

            if( GPIO_PORT_CNT != canConfig->StandbyPort )
            {
                (void)Can_InitStandbyGpio( canConfig->StandbyPort, canConfig->StandbyPin );
            }
            else
            {
                /* No transceiver standby pin configured */
            }

            if( CAN_TX_PIN_UNUSED != canConfig->TxPin )
            {
                (void)Can_InitTxGpio( canConfig->TxPin );
            }
            else
            {
                /* No transmit pin configured */
            }

            if( CAN_RX_PIN_UNUSED != canConfig->RxPin )
            {
                (void)Can_InitRxGpio( canConfig->RxPin );
            }
            else
            {
                /* No receive pin configured */
            }

            (void)Nvic_Set_PeriphIrq_Handler( Can_Get_IrqLine( canConfig->PeriphId ),
                                               Can_Get_IsrHandler( canConfig->PeriphId ) );
            (void)Can_Set_IrqPriority( canConfig->PeriphId, canConfig->IrqPriority );
            (void)Nvic_Set_PeriphIrq_Active( Can_Get_IrqLine( canConfig->PeriphId ) );

            retState = CAN_REQUEST_OK;
        }
    }
    else
    {
        /* Bad parameter (null pointer or invalid instance), nothing configured */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief De-initialises one FDCAN instance (interrupt disabled, clock disabled, state reset).
 *
 * \param periphId [in] : Instance to de-initialise.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for an invalid instance.
 */
can_RequestState_t Can_Deinit( can_PeriphId_t periphId )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( CAN_PERIPH_CNT > periphId )
    {
        (void)Nvic_Set_PeriphIrq_Inactive( Can_Get_IrqLine( periphId ) );
        Can_Set_RegBit( periphId, CAN_REG_CCCR, FDCAN_CCCR_INIT );
        (void)Rcc_Set_PeriphInactive( can_State[ periphId ].Config.ClkSource );

        can_State[ periphId ].State        = CAN_STATE_STOPPED;
        can_State[ periphId ].Initialized  = CAN_FUNCTION_INACTIVE;
        can_State[ periphId ].LastBusState = CAN_BUS_ACTIVE;
        can_State[ periphId ].LastError    = CAN_ERROR_NO_CHANGE;

        retState = CAN_REQUEST_OK;
    }
    else
    {
        /* Invalid instance, nothing de-initialised */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Services every initialised instance (poll mode fallback of the interrupt
 *        servicing - the same Can_Service function the instance ISR calls).
 */
void Can_Task( void )
{
    for( uint8_t periphId = 0u; CAN_PERIPH_CNT > periphId; periphId++ )
    {
        Can_Service( (can_PeriphId_t)periphId );
    }
}


/**
 * \brief Brings the instance onto the bus (leaves the initialization mode).
 *
 * \param periphId [in] : Instance to start.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for an invalid / uninitialised instance.
 */
can_RequestState_t Can_Set_Start( can_PeriphId_t periphId )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( ( CAN_PERIPH_CNT      > periphId ) &&
        ( CAN_FUNCTION_ACTIVE == can_State[ periphId ].Initialized )    )
    {
        Can_Reset_RegBit( periphId, CAN_REG_CCCR, FDCAN_CCCR_CCE | FDCAN_CCCR_INIT );
        can_State[ periphId ].State = CAN_STATE_STARTED;
        retState = CAN_REQUEST_OK;
    }
    else
    {
        /* Invalid or uninitialised instance, nothing started */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Takes the instance off the bus (pending transmissions are aborted by the INIT mode).
 *
 * \param periphId [in] : Instance to stop.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for an invalid / uninitialised instance.
 */
can_RequestState_t Can_Set_Stop( can_PeriphId_t periphId )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;
    can_RequestState_t cceState = CAN_REQUEST_ERROR;

    if( ( CAN_PERIPH_CNT      > periphId ) &&
        ( CAN_FUNCTION_ACTIVE == can_State[ periphId ].Initialized )    )
    {
        Can_Set_RegBit( periphId, CAN_REG_CCCR, FDCAN_CCCR_INIT );
        cceState = Can_Reg_Wait_RegBitSet( periphId, CAN_REG_CCCR, FDCAN_CCCR_INIT );
        Can_Set_RegBit( periphId, CAN_REG_CCCR, FDCAN_CCCR_CCE );
        can_State[ periphId ].State = CAN_STATE_STOPPED;
        retState = cceState;
    }
    else
    {
        /* Invalid or uninitialised instance, nothing stopped */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Gets the current state of the instance.
 *
 * \param periphId [in]  : Instance to query.
 * \param canState [out] : Current state.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter.
 */
can_RequestState_t Can_Get_State( can_PeriphId_t periphId, can_State_t * const canState )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( ( CAN_PERIPH_CNT      > periphId ) &&
        ( CAN_NULL_PTR        != canState ) &&
        ( CAN_FUNCTION_ACTIVE == can_State[ periphId ].Initialized )    )
    {
        *canState = can_State[ periphId ].State;
        retState  = CAN_REQUEST_OK;
    }
    else
    {
        /* Bad parameter or uninitialised instance, no state reported */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Requests the clock-stop (power-down) sequence: INIT is set, pending transfers finish,
 *        the bus is left idle, then the hardware acknowledges CSA. The instance moves to
 *        CAN_STATE_SLEEP. Rcc kernel / peripheral clock gating is not part of this call.
 *
 * \param periphId [in] : Instance to power down.
 *
 * \return CAN_REQUEST_OK once CSA is acknowledged, CAN_REQUEST_ERROR for a bad parameter / timeout.
 */
can_RequestState_t Can_Set_PowerDown( can_PeriphId_t periphId )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( ( CAN_PERIPH_CNT      > periphId ) &&
        ( CAN_FUNCTION_ACTIVE == can_State[ periphId ].Initialized )    )
    {
        Can_Set_RegBit( periphId, CAN_REG_CCCR, FDCAN_CCCR_CSR );
        retState = Can_Reg_Wait_RegBitSet( periphId, CAN_REG_CCCR, FDCAN_CCCR_CSA );

        if( CAN_REQUEST_OK == retState )
        {
            can_State[ periphId ].State = CAN_STATE_SLEEP;
        }
        else
        {
            /* Clock stop not acknowledged within the timeout, state unchanged */
        }
    }
    else
    {
        /* Invalid or uninitialised instance, nothing requested */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Clears the clock-stop request and waits for CSA to clear (wake-up).
 *
 * \param periphId [in] : Instance to power up.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter / timeout.
 */
can_RequestState_t Can_Set_PowerUp( can_PeriphId_t periphId )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( ( CAN_PERIPH_CNT      > periphId ) &&
        ( CAN_FUNCTION_ACTIVE == can_State[ periphId ].Initialized )    )
    {
        Can_Reset_RegBit( periphId, CAN_REG_CCCR, FDCAN_CCCR_CSR );
        retState = Can_Reg_Wait_RegBitClr( periphId, CAN_REG_CCCR, FDCAN_CCCR_CSA );

        if( CAN_REQUEST_OK == retState )
        {
            can_State[ periphId ].State = CAN_STATE_STOPPED;

            if( CAN_NULL_PTR != can_State[ periphId ].Config.WakeUpCallback )
            {
                can_State[ periphId ].Config.WakeUpCallback( periphId );
            }
            else
            {
                /* No wake-up callback configured */
            }
        }
        else
        {
            /* Clock stop acknowledge not cleared within the timeout, state unchanged */
        }
    }
    else
    {
        /* Invalid or uninitialised instance, nothing requested */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures the nominal (and, if FdEnable is active, the data phase) bit timing.
 *        Only allowed in CAN_STATE_STOPPED (the README rule 2).
 *
 * \param periphId  [in] : Instance to configure.
 * \param bitTiming [in] : Requested bit timing; the achieved values are written back.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter, a bad state, or a
 *         bit rate that the kernel clock cannot reach within the tolerance.
 */
can_RequestState_t Can_Set_BitTiming( can_PeriphId_t periphId, const can_BitTiming_t * const bitTiming )
{
    can_RequestState_t retState   = CAN_REQUEST_ERROR;
    can_RequestState_t searchState = CAN_REQUEST_ERROR;
    uint32_t              kernelClkHz = 0u;
    uint32_t               presc     = 0u;
    uint32_t                tseg1     = 0u;
    uint32_t                tseg2     = 0u;
    uint32_t                 achRate  = 0u;
    uint16_t                  achPoint = 0u;
    can_BitTiming_t             localTiming = { 0u };

    if( ( CAN_PERIPH_CNT > periphId )                               &&
        ( CAN_NULL_PTR != bitTiming )                               &&
        ( CAN_STATE_STOPPED == can_State[ periphId ].State )         &&
        ( CAN_FUNCTION_ACTIVE == can_State[ periphId ].Initialized )    )
    {
        localTiming = *bitTiming;
        (void)Can_Get_ClkFrequency( periphId, &kernelClkHz );

        searchState = Can_Search_BitTiming( kernelClkHz,
                                            localTiming.Bitrate,
                                            localTiming.SamplePoint,
                                            localTiming.BitrateTolerance,
                                            CAN_NOMINAL_PRESCALER_MAX,
                                            CAN_NOMINAL_TSEG1_MAX,
                                            CAN_NOMINAL_TSEG2_MAX,
                                            CAN_NOMINAL_TQ_PER_BIT_MAX,
                                            &presc,
                                            &tseg1,
                                            &tseg2,
                                            &achRate,
                                            &achPoint );

        if( CAN_REQUEST_OK == searchState )
        {
            Can_Set_RegVal( periphId,
                            CAN_REG_NBTP,
                            CAN_REG_MASK_ALL,
                            ( ( presc - 1u ) << FDCAN_NBTP_NBRP_Pos   ) |
                            ( ( tseg1 - 1u ) << FDCAN_NBTP_NTSEG1_Pos ) |
                            ( ( tseg2 - 1u ) << FDCAN_NBTP_NTSEG2_Pos ) |
                            ( ( tseg2 - 1u ) << FDCAN_NBTP_NSJW_Pos   )   );
            localTiming.AchievedBitrate     = achRate;
            localTiming.AchievedSamplePoint = achPoint;

            if( ( CAN_FUNCTION_ACTIVE == can_State[ periphId ].Config.FdEnable ) &&
                ( 0u != localTiming.DataBitrate )                                   )
            {
                searchState = Can_Search_BitTiming( kernelClkHz,
                                                    localTiming.DataBitrate,
                                                    localTiming.DataSamplePoint,
                                                    localTiming.BitrateTolerance,
                                                    CAN_DATA_PRESCALER_MAX,
                                                    CAN_DATA_TSEG1_MAX,
                                                    CAN_DATA_TSEG2_MAX,
                                                    CAN_DATA_TQ_PER_BIT_MAX,
                                                    &presc,
                                                    &tseg1,
                                                    &tseg2,
                                                    &achRate,
                                                    &achPoint );

                if( CAN_REQUEST_OK == searchState )
                {
                    Can_Set_RegVal( periphId,
                                    CAN_REG_DBTP,
                                    CAN_REG_MASK_ALL,
                                    ( ( presc - 1u ) << FDCAN_DBTP_DBRP_Pos   ) |
                                    ( ( tseg1 - 1u ) << FDCAN_DBTP_DTSEG1_Pos ) |
                                    ( ( tseg2 - 1u ) << FDCAN_DBTP_DTSEG2_Pos ) |
                                    ( ( tseg2 - 1u ) << FDCAN_DBTP_DSJW_Pos   )   );
                    localTiming.AchievedDataBitrate     = achRate;
                    localTiming.AchievedDataSamplePoint = achPoint;
                }
                else
                {
                    /* Data phase bit rate unreachable within the tolerance, searchState propagated */
                }
            }
            else
            {
                /* CAN FD data phase not requested, nominal timing already applied */
            }

            can_State[ periphId ].Config.BitTiming = localTiming;
            retState = searchState;
        }
        else
        {
            /* Nominal bit rate unreachable within the tolerance, nothing applied */
            retState = CAN_REQUEST_ERROR;
        }
    }
    else
    {
        /* Bad parameter or bad state, nothing applied */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Gets the bit timing (including the achieved values) of the instance.
 *
 * \param periphId  [in]  : Instance to query.
 * \param bitTiming [out] : Current bit timing.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter.
 */
can_RequestState_t Can_Get_BitTiming( can_PeriphId_t periphId, can_BitTiming_t * const bitTiming )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( ( CAN_PERIPH_CNT      > periphId   ) &&
        ( CAN_NULL_PTR        != bitTiming ) &&
        ( CAN_FUNCTION_ACTIVE == can_State[ periphId ].Initialized )    )
    {
        *bitTiming = can_State[ periphId ].Config.BitTiming;
        retState   = CAN_REQUEST_OK;
    }
    else
    {
        /* Bad parameter or uninitialised instance, nothing reported */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures the operating mode (CAN_STATE_STOPPED only).
 *
 * \param periphId [in] : Instance to configure.
 * \param mode     [in] : Requested mode.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter or a bad state.
 */
can_RequestState_t Can_Set_Mode( can_PeriphId_t periphId, can_Mode_t mode )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( ( CAN_PERIPH_CNT > periphId )                            &&
        ( CAN_MODE_CNT   > mode     )                            &&
        ( CAN_STATE_STOPPED == can_State[ periphId ].State )        )
    {
        Can_Reset_RegBit( periphId, CAN_REG_CCCR, FDCAN_CCCR_ASM | FDCAN_CCCR_MON | FDCAN_CCCR_TEST );
        Can_Reset_RegBit( periphId, CAN_REG_TEST, FDCAN_TEST_LBCK );

        if( CAN_MODE_RESTRICTED == mode )
        {
            Can_Set_RegBit( periphId, CAN_REG_CCCR, FDCAN_CCCR_ASM );
        }
        else if( CAN_MODE_MONITORING == mode )
        {
            Can_Set_RegBit( periphId, CAN_REG_CCCR, FDCAN_CCCR_MON );
        }
        else if( CAN_MODE_LOOPBACK_INTERNAL == mode )
        {
            Can_Set_RegBit( periphId, CAN_REG_CCCR, FDCAN_CCCR_TEST | FDCAN_CCCR_MON );
            Can_Set_RegBit( periphId, CAN_REG_TEST, FDCAN_TEST_LBCK );
        }
        else if( CAN_MODE_LOOPBACK_EXTERNAL == mode )
        {
            Can_Set_RegBit( periphId, CAN_REG_CCCR, FDCAN_CCCR_TEST );
            Can_Set_RegBit( periphId, CAN_REG_TEST, FDCAN_TEST_LBCK );
        }
        else
        {
            /* CAN_MODE_NORMAL: every test / restricted / monitoring bit already cleared above */
        }

        can_State[ periphId ].Config.Mode = mode;
        retState = CAN_REQUEST_OK;
    }
    else
    {
        /* Bad parameter or bad state, nothing applied */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Gets the configured operating mode.
 *
 * \param periphId [in]  : Instance to query.
 * \param mode     [out] : Current mode.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter.
 */
can_RequestState_t Can_Get_Mode( can_PeriphId_t periphId, can_Mode_t * const mode )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( ( CAN_PERIPH_CNT > periphId ) &&
        ( CAN_NULL_PTR    != mode )    )
    {
        *mode    = can_State[ periphId ].Config.Mode;
        retState = CAN_REQUEST_OK;
    }
    else
    {
        /* Bad parameter, nothing reported */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables / disables the automatic retransmission of a frame not acknowledged.
 *
 * \param periphId  [in] : Instance to configure.
 * \param funcState [in] : CAN_FUNCTION_ACTIVE to enable, CAN_FUNCTION_INACTIVE to disable.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter.
 */
can_RequestState_t Can_Set_AutoRetransmission( can_PeriphId_t periphId, can_FunctionState_t funcState )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( CAN_PERIPH_CNT > periphId )
    {
        if( CAN_FUNCTION_ACTIVE == funcState )
        {
            Can_Reset_RegBit( periphId, CAN_REG_CCCR, FDCAN_CCCR_DAR );
        }
        else
        {
            Can_Set_RegBit( periphId, CAN_REG_CCCR, FDCAN_CCCR_DAR );
        }

        can_State[ periphId ].Config.AutoRetransmission = funcState;
        retState = CAN_REQUEST_OK;
    }
    else
    {
        /* Invalid instance, nothing applied */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Gets the automatic retransmission setting.
 *
 * \param periphId  [in]  : Instance to query.
 * \param funcState [out] : Current setting.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter.
 */
can_RequestState_t Can_Get_AutoRetransmission( can_PeriphId_t periphId, can_FunctionState_t * const funcState )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( ( CAN_PERIPH_CNT > periphId ) &&
        ( CAN_NULL_PTR    != funcState )    )
    {
        *funcState = can_State[ periphId ].Config.AutoRetransmission;
        retState   = CAN_REQUEST_OK;
    }
    else
    {
        /* Bad parameter, nothing reported */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Gets the kernel clock frequency of the instance (via Rcc).
 *
 * \param periphId  [in]  : Instance to query.
 * \param clkFreqHz [out] : Kernel clock frequency (Hz).
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter / uninitialised instance.
 */
can_RequestState_t Can_Get_ClkFrequency( can_PeriphId_t periphId, can_Frequency_Hz_t * const clkFreqHz )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;
    rcc_RequestState_t  rccState = RCC_REQUEST_ERROR;
    rcc_FreqHz_t          rccFreq = 0u;

    if( ( CAN_PERIPH_CNT > periphId ) &&
        ( CAN_NULL_PTR    != clkFreqHz )    )
    {
        rccState = Rcc_Get_PeriphClk( can_State[ periphId ].Config.ClkSource, &rccFreq );

        if( RCC_REQUEST_OK == rccState )
        {
            *clkFreqHz = (uint32_t)rccFreq;
            retState   = CAN_REQUEST_OK;
        }
        else
        {
            *clkFreqHz = 0u;
            retState   = CAN_REQUEST_ERROR;
        }
    }
    else
    {
        /* Bad parameter, nothing reported */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures the acceptance filter lists (standard and extended). The lists are
 *        rejected as a whole when they do not fit the fixed message RAM capacity
 *        (CAN_STD_FILTER_ELEMENT_CNT, CAN_EXT_FILTER_ELEMENT_CNT) - nothing is half-applied.
 *        Only allowed in CAN_STATE_STOPPED.
 *
 * \param periphId  [in] : Instance to configure.
 * \param filters   [in] : Filter list (standard and extended items in any order).
 * \param filterCnt [in] : Number of filter elements.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter, a bad state, or a
 *         list that exceeds the available capacity.
 */
can_RequestState_t Can_Set_Filters( can_PeriphId_t periphId,
                                    const can_Filter_t * const filters,
                                    can_FilterCnt_t filterCnt )
{
    can_RequestState_t retState  = CAN_REQUEST_ERROR;
    can_FilterCnt_t      stdCnt    = 0u;
    can_FilterCnt_t       extCnt    = 0u;

    if( ( CAN_PERIPH_CNT > periphId )                            &&
        ( CAN_NULL_PTR != filters )                              &&
        ( CAN_STATE_STOPPED == can_State[ periphId ].State )        )
    {
        for( can_FilterCnt_t idx = 0u; filterCnt > idx; idx++ )
        {
            if( CAN_ID_STANDARD == filters[ idx ].IdType )
            {
                stdCnt++;
            }
            else
            {
                extCnt++;
            }
        }

        if( ( CAN_STD_FILTER_ELEMENT_CNT >= stdCnt ) &&
            ( CAN_EXT_FILTER_ELEMENT_CNT >= extCnt )    )
        {
            Can_Clear_Filters( periphId );
            stdCnt = 0u;
            extCnt = 0u;

            for( can_FilterCnt_t idx = 0u; filterCnt > idx; idx++ )
            {
                if( CAN_ID_STANDARD == filters[ idx ].IdType )
                {
                    Can_Write_StdFilter( periphId, stdCnt, &filters[ idx ] );
                    stdCnt++;
                }
                else
                {
                    Can_Write_ExtFilter( periphId, extCnt, &filters[ idx ] );
                    extCnt++;
                }
            }

            retState = CAN_REQUEST_OK;
        }
        else
        {
            /* The list does not fit the fixed message RAM capacity, nothing applied */
            retState = CAN_REQUEST_ERROR;
        }
    }
    else
    {
        /* Bad parameter or bad state, nothing applied */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures the behaviour for a frame that matches no filter / a remote frame.
 *        Only allowed in CAN_STATE_STOPPED.
 *
 * \param periphId     [in] : Instance to configure.
 * \param globalFilter [in] : Global filter configuration.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter or a bad state.
 */
can_RequestState_t Can_Set_GlobalFilter( can_PeriphId_t periphId, const can_GlobalFilter_t * const globalFilter )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;
    uint32_t              anfs    = 0u;
    uint32_t               anfe    = 0u;
    uint32_t                regVal  = 0u;

    if( ( CAN_PERIPH_CNT > periphId )                            &&
        ( CAN_NULL_PTR != globalFilter )                         &&
        ( CAN_STATE_STOPPED == can_State[ periphId ].State )        )
    {
        if( CAN_FILTER_TO_RX_FIFO_1 == globalFilter->NonMatchingStd )
        {
            anfs = 1u;
        }
        else if( CAN_FILTER_TO_RX_FIFO_0 == globalFilter->NonMatchingStd )
        {
            anfs = 0u;
        }
        else
        {
            anfs = CAN_RXGFC_ANF_REJECT;
        }

        if( CAN_FILTER_TO_RX_FIFO_1 == globalFilter->NonMatchingExt )
        {
            anfe = 1u;
        }
        else if( CAN_FILTER_TO_RX_FIFO_0 == globalFilter->NonMatchingExt )
        {
            anfe = 0u;
        }
        else
        {
            anfe = CAN_RXGFC_ANF_REJECT;
        }

        regVal = ( anfs << CAN_RXGFC_ANFS_SHIFT ) | ( anfe << CAN_RXGFC_ANFE_SHIFT );

        if( CAN_FLAG_ACTIVE == globalFilter->RejectRemoteStd )
        {
            regVal |= FDCAN_RXGFC_RRFS;
        }
        else
        {
            /* Standard remote frames filtered normally */
        }

        if( CAN_FLAG_ACTIVE == globalFilter->RejectRemoteExt )
        {
            regVal |= FDCAN_RXGFC_RRFE;
        }
        else
        {
            /* Extended remote frames filtered normally */
        }

        Can_Set_RegVal( periphId,
                        CAN_REG_RXGFC,
                        FDCAN_RXGFC_ANFS | FDCAN_RXGFC_ANFE | FDCAN_RXGFC_RRFS | FDCAN_RXGFC_RRFE,
                        regVal );
        retState = CAN_REQUEST_OK;
    }
    else
    {
        /* Bad parameter or bad state, nothing applied */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Requests the transmission of a frame: writes it to the next free Tx buffer
 *        and sets the corresponding FDCAN_TXBAR bit.
 *
 * \param periphId [in]  : Instance to use.
 * \param frame    [in]  : Frame to transmit.
 * \param txHandle [out] : Handle of the Tx buffer taken (for Can_Set_TxAbort / Can_Get_TxState).
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter, an invalid frame
 *         (identifier range, length, FD flags on a non-FD configuration) or a full Tx FIFO/queue.
 */
can_RequestState_t Can_Set_TxRequest( can_PeriphId_t periphId,
                                      const can_Frame_t * const frame,
                                      can_TxHandle_t * const txHandle )
{
    can_RequestState_t retState  = CAN_REQUEST_ERROR;
    uint32_t              txfqs    = 0u;
    uint32_t               putIdx   = 0u;
    uint32_t                wordOffset = 0u;
    uint32_t                 t0Word  = 0u;
    uint32_t                  t1Word  = 0u;
    uint8_t                    dlc     = 0u;

    if( ( CAN_PERIPH_CNT > periphId )                                  &&
        ( CAN_NULL_PTR != frame )                                      &&
        ( CAN_NULL_PTR != txHandle )                                   &&
        ( CAN_STATE_STARTED == can_State[ periphId ].State )           &&
        ( ( CAN_FUNCTION_ACTIVE == can_State[ periphId ].Config.FdEnable ) ||
          ( CAN_FLAG_INACTIVE   == frame->FdFrame ) )                    &&
        ( ( CAN_FUNCTION_ACTIVE == can_State[ periphId ].Config.FdEnable ) ||
          ( CAN_DATA_LEN_CLASSIC_MAX >= frame->Length ) )                   )
    {
        txfqs  = Can_Get_RegVal( periphId, CAN_REG_TXFQS, FDCAN_TXFQS_TFQF | FDCAN_TXFQS_TFQPI );
        putIdx = ( txfqs & FDCAN_TXFQS_TFQPI ) >> FDCAN_TXFQS_TFQPI_Pos;

        if( 0u == ( txfqs & FDCAN_TXFQS_TFQF ) )
        {
            dlc        = Can_Len_To_Dlc( frame->Length );
            wordOffset = CAN_RAM_WORD_OFFSET_TX_BUFFER + ( putIdx * CAN_RAM_ELEMENT_WORD_CNT_FRAME );

            t0Word = 0u;

            if( CAN_FRAME_REMOTE == frame->FrameType )
            {
                t0Word |= CAN_RAM_T0_RTR;
            }
            else
            {
                /* Data frame, RTR left clear */
            }

            if( CAN_ID_EXTENDED == frame->IdType )
            {
                t0Word |= CAN_RAM_T0_XTD | ( frame->Id & CAN_RAM_T0_ID_MASK );
            }
            else
            {
                t0Word |= ( ( frame->Id & CAN_RAM_SF_ID_MASK ) << CAN_RAM_STD_ID_SHIFT );
            }

            if( CAN_FLAG_ACTIVE == frame->ErrorStateIndicator )
            {
                t0Word |= CAN_RAM_T0_ESI;
            }
            else
            {
                /* ESI left to the hardware (error passive flag) */
            }

            t1Word = ( (uint32_t)frame->Marker << CAN_RAM_T1_MM_SHIFT ) | CAN_RAM_T1_EFC |
                     ( (uint32_t)dlc << CAN_RAM_T1_DLC_SHIFT );

            if( CAN_FUNCTION_ACTIVE == can_State[ periphId ].Config.FdEnable )
            {
                if( CAN_FLAG_ACTIVE == frame->FdFrame )
                {
                    t1Word |= CAN_RAM_T1_FDF;
                }
                else
                {
                    /* Classic CAN frame inside an FD-enabled instance */
                }

                if( CAN_FLAG_ACTIVE == frame->BitRateSwitch )
                {
                    t1Word |= CAN_RAM_T1_BRS;
                }
                else
                {
                    /* No bit rate switch requested */
                }
            }
            else
            {
                /* Instance not FD-enabled, FDF / BRS left clear */
            }

            Can_Ram_Set_Word( periphId, wordOffset,     t0Word );
            Can_Ram_Set_Word( periphId, wordOffset + 1u, t1Word );

            for( uint8_t byteIdx = 0u; CAN_DATA_LEN_FD_MAX > byteIdx; byteIdx += CAN_WORD_BYTE_CNT )
            {
                uint32_t dataWord = 0u;

                for( uint8_t subIdx = 0u; CAN_WORD_BYTE_CNT > subIdx; subIdx++ )
                {
                    dataWord |= ( (uint32_t)frame->Data[ byteIdx + subIdx ] <<
                                  ( subIdx * CAN_BYTE_SHIFT_UNIT )              );
                }

                Can_Ram_Set_Word( periphId, wordOffset + CAN_RAM_WORD_CNT + ( byteIdx / CAN_WORD_BYTE_CNT ), dataWord );
            }

            Can_Set_RegVal( periphId, CAN_REG_TXBAR, ( 1u << putIdx ), ( 1u << putIdx ) );
            *txHandle = (can_TxHandle_t)putIdx;
            retState  = CAN_REQUEST_OK;
        }
        else
        {
            /* Tx FIFO / queue full, the frame is not queued */
            *txHandle = CAN_TX_HANDLE_INVALID;
            retState  = CAN_REQUEST_ERROR;
        }
    }
    else
    {
        /* Bad parameter, bad state, or a frame invalid for the current configuration */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Cancels a pending transmission request.
 *
 * \param periphId [in] : Instance to use.
 * \param txHandle [in] : Handle returned by Can_Set_TxRequest.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter.
 */
can_RequestState_t Can_Set_TxAbort( can_PeriphId_t periphId, can_TxHandle_t txHandle )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( ( CAN_PERIPH_CNT          > periphId ) &&
        ( CAN_TX_BUFFER_ELEMENT_CNT > txHandle )    )
    {
        Can_Set_RegVal( periphId, CAN_REG_TXBCR, ( 1u << txHandle ), ( 1u << txHandle ) );
        retState = CAN_REQUEST_OK;
    }
    else
    {
        /* Bad parameter, nothing cancelled */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reports the current state of a transmission request.
 *
 * \param periphId [in]  : Instance to use.
 * \param txHandle [in]  : Handle returned by Can_Set_TxRequest.
 * \param txState  [out] : CAN_REQUEST_OK once transmitted, CAN_REQUEST_ERROR while pending or cancelled.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter.
 */
can_RequestState_t Can_Get_TxState( can_PeriphId_t periphId,
                                    can_TxHandle_t txHandle,
                                    can_RequestState_t * const txState )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;
    uint32_t              txbtoVal = 0u;

    if( ( CAN_PERIPH_CNT          > periphId ) &&
        ( CAN_TX_BUFFER_ELEMENT_CNT > txHandle ) &&
        ( CAN_NULL_PTR              != txState )    )
    {
        txbtoVal = Can_Get_RegVal( periphId, CAN_REG_TXBTO, ( 1u << txHandle ) );

        if( 0u != txbtoVal )
        {
            *txState = CAN_REQUEST_OK;
        }
        else
        {
            *txState = CAN_REQUEST_ERROR;
        }

        retState = CAN_REQUEST_OK;
    }
    else
    {
        /* Bad parameter, nothing reported */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Gets the number of free Tx buffer elements.
 *
 * \param periphId [in]  : Instance to query.
 * \param freeCnt  [out] : Number of free elements.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter.
 */
can_RequestState_t Can_Get_TxFreeCount( can_PeriphId_t periphId, can_FrameCnt_t * const freeCnt )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( ( CAN_PERIPH_CNT      > periphId ) &&
        ( CAN_NULL_PTR        != freeCnt ) &&
        ( CAN_FUNCTION_ACTIVE == can_State[ periphId ].Initialized )    )
    {
        *freeCnt = (can_FrameCnt_t)Can_Get_RegVal( periphId, CAN_REG_TXFQS, FDCAN_TXFQS_TFFL );
        retState = CAN_REQUEST_OK;
    }
    else
    {
        /* Bad parameter or uninitialised instance, nothing reported */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Gets the number of frames waiting in a receive FIFO.
 *
 * \param periphId [in]  : Instance to query.
 * \param fifoId   [in]  : Receive FIFO to query.
 * \param rxCnt    [out] : Number of frames waiting.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter.
 */
can_RequestState_t Can_Get_RxCount( can_PeriphId_t periphId, can_RxFifoId_t fifoId, can_FrameCnt_t * const rxCnt )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( ( CAN_PERIPH_CNT      > periphId ) &&
        ( CAN_RX_FIFO_CNT     > fifoId   ) &&
        ( CAN_NULL_PTR        != rxCnt   ) &&
        ( CAN_FUNCTION_ACTIVE == can_State[ periphId ].Initialized )    )
    {
        if( CAN_RX_FIFO_0 == fifoId )
        {
            *rxCnt = (can_FrameCnt_t)Can_Get_RegVal( periphId, CAN_REG_RXF0S, FDCAN_RXF0S_F0FL );
        }
        else
        {
            *rxCnt = (can_FrameCnt_t)Can_Get_RegVal( periphId, CAN_REG_RXF1S, FDCAN_RXF1S_F1FL );
        }

        retState = CAN_REQUEST_OK;
    }
    else
    {
        /* Bad parameter, nothing reported */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads one frame out of a receive FIFO and acknowledges it (get index advanced).
 *
 * \param periphId [in]  : Instance to use.
 * \param fifoId   [in]  : Receive FIFO to read from.
 * \param frame    [out] : Frame read.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter or an empty FIFO.
 */
can_RequestState_t Can_Get_RxFrame( can_PeriphId_t periphId, can_RxFifoId_t fifoId, can_Frame_t * const frame )
{
    can_RequestState_t retState     = CAN_REQUEST_ERROR;
    can_RegId_t           statusReg    = CAN_REG_RXF0S;
    can_RegId_t            ackReg       = CAN_REG_RXF0A;
    uint32_t                fillLevel    = 0u;
    uint32_t                 getIdx      = 0u;
    uint32_t                  wordOffset  = 0u;
    uint32_t                   r0Word     = 0u;
    uint32_t                    r1Word     = 0u;

    if( CAN_RX_FIFO_1 == fifoId )
    {
        statusReg = CAN_REG_RXF1S;
        ackReg    = CAN_REG_RXF1A;
        wordOffset = CAN_RAM_WORD_OFFSET_RX_FIFO_1;
    }
    else
    {
        statusReg = CAN_REG_RXF0S;
        ackReg    = CAN_REG_RXF0A;
        wordOffset = CAN_RAM_WORD_OFFSET_RX_FIFO_0;
    }

    if( ( CAN_PERIPH_CNT  > periphId ) &&
        ( CAN_RX_FIFO_CNT > fifoId   ) &&
        ( CAN_NULL_PTR    != frame   )    )
    {
        fillLevel = Can_Get_RegVal( periphId, statusReg, CAN_RXFS_FILLLVL_MASK );
        getIdx    = Can_Get_RegVal( periphId, statusReg, CAN_RXFS_GETIDX_MASK ) >> CAN_RXFS_GETIDX_SHIFT;

        if( 0u != fillLevel )
        {
            wordOffset += ( getIdx * CAN_RAM_ELEMENT_WORD_CNT_FRAME );
            r0Word = Can_Ram_Get_Word( periphId, wordOffset );
            r1Word = Can_Ram_Get_Word( periphId, wordOffset + 1u );

            if( 0u != ( r0Word & CAN_RAM_T0_XTD ) )
            {
                frame->IdType = CAN_ID_EXTENDED;
                frame->Id     = r0Word & CAN_RAM_T0_ID_MASK;
            }
            else
            {
                frame->IdType = CAN_ID_STANDARD;
                frame->Id     = ( r0Word >> CAN_RAM_STD_ID_SHIFT ) & CAN_RAM_SF_ID_MASK;
            }

            if( 0u != ( r0Word & CAN_RAM_T0_RTR ) )
            {
                frame->FrameType = CAN_FRAME_REMOTE;
            }
            else
            {
                frame->FrameType = CAN_FRAME_DATA;
            }

            if( 0u != ( r0Word & CAN_RAM_T0_ESI ) )
            {
                frame->ErrorStateIndicator = CAN_FLAG_ACTIVE;
            }
            else
            {
                frame->ErrorStateIndicator = CAN_FLAG_INACTIVE;
            }

            if( 0u != ( r1Word & CAN_RAM_T1_FDF ) )
            {
                frame->FdFrame = CAN_FLAG_ACTIVE;
            }
            else
            {
                frame->FdFrame = CAN_FLAG_INACTIVE;
            }

            if( 0u != ( r1Word & CAN_RAM_T1_BRS ) )
            {
                frame->BitRateSwitch = CAN_FLAG_ACTIVE;
            }
            else
            {
                frame->BitRateSwitch = CAN_FLAG_INACTIVE;
            }

            frame->Length    = Can_Dlc_To_Len( (uint8_t)( ( r1Word & CAN_RAM_T1_DLC_MASK ) >> CAN_RAM_T1_DLC_SHIFT ) );
            frame->Timestamp = r1Word & CAN_RAM_R1_RXTS_MASK;
            frame->Marker    = 0u;

            for( uint8_t byteIdx = 0u; CAN_DATA_LEN_FD_MAX > byteIdx; byteIdx += CAN_WORD_BYTE_CNT )
            {
                uint32_t dataWord = Can_Ram_Get_Word( periphId,
                                        wordOffset + CAN_RAM_WORD_CNT + ( byteIdx / CAN_WORD_BYTE_CNT ) );

                for( uint8_t subIdx = 0u; CAN_WORD_BYTE_CNT > subIdx; subIdx++ )
                {
                    frame->Data[ byteIdx + subIdx ] =
                        (uint8_t)( ( dataWord >> ( subIdx * CAN_BYTE_SHIFT_UNIT ) ) & CAN_BYTE_MASK );
                }
            }

            Can_Set_RegVal( periphId, ackReg, CAN_RXFA_ACKIDX_MASK, getIdx );
            retState = CAN_REQUEST_OK;
        }
        else
        {
            /* FIFO empty, nothing read */
            retState = CAN_REQUEST_ERROR;
        }
    }
    else
    {
        /* Bad parameter, nothing read */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Gets the current bus state (error counter thresholds, README table).
 *
 * \param periphId [in]  : Instance to query.
 * \param busState [out] : Current bus state.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter.
 */
can_RequestState_t Can_Get_BusState( can_PeriphId_t periphId, can_BusState_t * const busState )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( ( CAN_PERIPH_CNT      > periphId  ) &&
        ( CAN_NULL_PTR        != busState ) &&
        ( CAN_FUNCTION_ACTIVE == can_State[ periphId ].Initialized )    )
    {
        *busState = Can_Read_BusState( periphId );
        retState  = CAN_REQUEST_OK;
    }
    else
    {
        /* Bad parameter or uninitialised instance, nothing reported */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Gets the transmit / receive error counters.
 *
 * \param periphId      [in]  : Instance to query.
 * \param errorCounters [out] : Current error counters.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter.
 */
can_RequestState_t Can_Get_ErrorCounters( can_PeriphId_t periphId, can_ErrorCounters_t * const errorCounters )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;
    uint32_t              ecrVal  = 0u;

    if( ( CAN_PERIPH_CNT > periphId ) &&
        ( CAN_NULL_PTR    != errorCounters )    )
    {
        ecrVal = Can_Get_RegVal( periphId, CAN_REG_ECR, FDCAN_ECR_TEC | FDCAN_ECR_REC | FDCAN_ECR_RP );

        errorCounters->TxErrorCnt = (uint8_t)( ecrVal & FDCAN_ECR_TEC );
        errorCounters->RxErrorCnt = (uint8_t)( ( ecrVal & FDCAN_ECR_REC ) >> FDCAN_ECR_REC_Pos );

        if( 0u != ( ecrVal & FDCAN_ECR_RP ) )
        {
            errorCounters->RxErrorPassive = CAN_FLAG_ACTIVE;
        }
        else
        {
            errorCounters->RxErrorPassive = CAN_FLAG_INACTIVE;
        }

        retState = CAN_REQUEST_OK;
    }
    else
    {
        /* Bad parameter, nothing reported */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Gets the last protocol error code (FDCAN_PSR.LEC).
 *
 * \param periphId  [in]  : Instance to query.
 * \param lastError [out] : Last error code.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter.
 */
can_RequestState_t Can_Get_LastError( can_PeriphId_t periphId, can_LastError_t * const lastError )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( ( CAN_PERIPH_CNT > periphId ) &&
        ( CAN_NULL_PTR    != lastError )    )
    {
        *lastError = (can_LastError_t)Can_Get_RegVal( periphId, CAN_REG_PSR, FDCAN_PSR_LEC );
        retState   = CAN_REQUEST_OK;
    }
    else
    {
        /* Bad parameter, nothing reported */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures the bus-off recovery mode.
 *
 * \param periphId       [in] : Instance to configure.
 * \param busOffRecovery [in] : CAN_BUS_OFF_RECOVERY_AUTO or CAN_BUS_OFF_RECOVERY_MANUAL.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter.
 */
can_RequestState_t Can_Set_BusOffRecovery( can_PeriphId_t periphId, can_BusOffRecovery_t busOffRecovery )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( ( CAN_PERIPH_CNT             > periphId      ) &&
        ( CAN_BUS_OFF_RECOVERY_CNT   > busOffRecovery )    )
    {
        can_State[ periphId ].Config.BusOffRecovery = busOffRecovery;
        retState = CAN_REQUEST_OK;
    }
    else
    {
        /* Bad parameter, nothing applied */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts the bus-off recovery sequence by hand (CAN_BUS_OFF_RECOVERY_MANUAL):
 *        clears INIT so the hardware can start waiting for 129 occurrences of bus-idle.
 *
 * \param periphId [in] : Instance to recover.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter.
 */
can_RequestState_t Can_Set_BusOffRestart( can_PeriphId_t periphId )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( CAN_PERIPH_CNT > periphId )
    {
        Can_Reset_RegBit( periphId, CAN_REG_CCCR, FDCAN_CCCR_INIT );
        retState = CAN_REQUEST_OK;
    }
    else
    {
        /* Invalid instance, nothing restarted */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Drives the transceiver standby pin (active state = standby requested = pin high).
 *
 * \param periphId  [in] : Instance to use.
 * \param funcState [in] : CAN_FUNCTION_ACTIVE requests standby, CAN_FUNCTION_INACTIVE leaves normal operation.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter or no standby pin configured.
 */
can_RequestState_t Can_Set_TransceiverState( can_PeriphId_t periphId, can_FunctionState_t funcState )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;
    gpio_RequestState_t gpioState = GPIO_REQUEST_ERROR;
    gpio_PortId_t         portId    = GPIO_PORT_CNT;
    gpio_PinId_t           pinId     = GPIO_PIN_ID_CNT;

    if( CAN_PERIPH_CNT > periphId )
    {
        portId = can_State[ periphId ].Config.StandbyPort;
        pinId  = can_State[ periphId ].Config.StandbyPin;

        if( GPIO_PORT_CNT != portId )
        {
            if( CAN_FUNCTION_ACTIVE == funcState )
            {
                gpioState = Gpio_Set_PinLevel( portId, pinId, GPIO_PIN_LEVEL_HIGH );
            }
            else
            {
                gpioState = Gpio_Set_PinLevel( portId, pinId, GPIO_PIN_LEVEL_LOW );
            }

            if( GPIO_REQUEST_OK == gpioState )
            {
                retState = CAN_REQUEST_OK;
            }
            else
            {
                retState = CAN_REQUEST_ERROR;
            }
        }
        else
        {
            /* No standby pin configured for this instance */
            retState = CAN_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid instance, nothing driven */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures the priority of the instance interrupt line (IT0, used for every event).
 *
 * \param periphId [in] : Instance to configure.
 * \param irqPrio  [in] : Requested priority.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter.
 */
can_RequestState_t Can_Set_IrqPriority( can_PeriphId_t periphId, nvic_IrqPrio_t irqPrio )
{
    can_RequestState_t retState  = CAN_REQUEST_ERROR;
    nvic_RequestState_t nvicState = NVIC_REQUEST_ERROR;

    if( CAN_PERIPH_CNT > periphId )
    {
        nvicState = Nvic_Set_PeriphIrq_Prio( Can_Get_IrqLine( periphId ), irqPrio );
        can_State[ periphId ].Config.IrqPriority = irqPrio;

        if( NVIC_REQUEST_OK == nvicState )
        {
            retState = CAN_REQUEST_OK;
        }
        else
        {
            retState = CAN_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid instance, nothing applied */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Gets the configured priority of the instance interrupt line.
 *
 * \param periphId [in]  : Instance to query.
 * \param irqPrio  [out] : Current priority.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter.
 */
can_RequestState_t Can_Get_IrqPriority( can_PeriphId_t periphId, nvic_IrqPrio_t * const irqPrio )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( ( CAN_PERIPH_CNT > periphId ) &&
        ( CAN_NULL_PTR    != irqPrio )    )
    {
        *irqPrio = can_State[ periphId ].Config.IrqPriority;
        retState = CAN_REQUEST_OK;
    }
    else
    {
        /* Bad parameter, nothing reported */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures the FDCAN_TX pin of an instance.
 *
 * \param pinId [in] : One of the device's can_TxPin_t items.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for CAN_TX_PIN_UNUSED or a Gpio error.
 */
can_RequestState_t Can_InitTxGpio( can_TxPin_t pinId )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( CAN_TX_PIN_UNUSED != pinId )
    {
        retState = Can_Init_AltFuncGpio( (uint32_t)pinId );
    }
    else
    {
        /* Unused pin, nothing configured */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures the FDCAN_RX pin of an instance.
 *
 * \param pinId [in] : One of the device's can_RxPin_t items.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for CAN_RX_PIN_UNUSED or a Gpio error.
 */
can_RequestState_t Can_InitRxGpio( can_RxPin_t pinId )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;

    if( CAN_RX_PIN_UNUSED != pinId )
    {
        retState = Can_Init_AltFuncGpio( (uint32_t)pinId );
    }
    else
    {
        /* Unused pin, nothing configured */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures the transceiver standby pin as a push-pull output (board option,
 *        plain GPIO - no FDCAN alternate function exists for it).
 *
 * \param portId [in] : Port of the standby pin.
 * \param pinId  [in] : Standby pin.
 *
 * \return CAN_REQUEST_OK, CAN_REQUEST_ERROR for a bad parameter or a Gpio error.
 */
can_RequestState_t Can_InitStandbyGpio( gpio_PortId_t portId, gpio_PinId_t pinId )
{
    can_RequestState_t retState      = CAN_REQUEST_ERROR;
    gpio_RequestState_t gpioInitState = GPIO_REQUEST_ERROR;
    gpio_Config_t         pinConfig     = { 0u };

    if( ( GPIO_PORT_CNT   > portId ) &&
        ( GPIO_PIN_ID_CNT > pinId  )    )
    {
        pinConfig.PortId         = portId;
        pinConfig.PinId          = pinId;
        pinConfig.PinMode        = GPIO_PIN_MODE_OUTPUT;
        pinConfig.PinPull        = GPIO_PIN_PULL_NONE;
        pinConfig.PinSpeed       = GPIO_PIN_SPEED_LOW;
        pinConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
        pinConfig.PinAltFunction = GPIO_ALT_FUNC_CNT;

        gpioInitState = Gpio_Init( &pinConfig );

        if( GPIO_REQUEST_ERROR != gpioInitState )
        {
            retState = CAN_REQUEST_OK;
        }
        else
        {
            retState = CAN_REQUEST_ERROR;
        }
    }
    else
    {
        /* Bad parameter, nothing configured */
        retState = CAN_REQUEST_ERROR;
    }

    return ( retState );
}

#endif /* FDCAN1 */
