/**
 * \author Mr.Nobody
 * \file Can_Reg.c
 * \ingroup Can
 * \brief Can module Reg component source file
 *
 * This component contains the functionality for register operations of the
 * multi-instance FDCAN peripheral: every accessor takes the instance
 * identification in addition to the register identification, the address is
 * built from the instance base address and the byte offset of the register
 * inside the CMSIS FDCAN_GlobalTypeDef structure.
 *
 * Message RAM access works on a fixed, non-configurable layout: the STM32H5
 * FDCAN (RM0481/RM0492/RM0517 chapter 60/44) has no SIDFC/XIDFC/RXF0C/RXF1C/
 * RXESC/TXESC/TXEFC configuration registers at all (confirmed against the
 * CMSIS FDCAN_GlobalTypeDef / FDCAN_Config_TypeDef, which list only CKDIV,
 * OPTR, HWCFG, VERR, IPIDR, SIDR in the configuration block). Every instance
 * gets a fixed 0x350-byte (212-word) slice of the shared message RAM: 28
 * standard filter elements (1 word), 8 extended filter elements (2 words), a
 * 3-element Rx FIFO 0 and Rx FIFO 1 (18 words each), a 3-element Tx event
 * FIFO (2 words) and 3 Tx buffer elements (18 words, FIFO or queue mode).
 * This was derived from RM0481 Rev 5 section 60.3.6 "Message RAM" (Figure
 * 875, the FDCANn start/end address formula) and sections 60.3.9 .. 60.3.12
 * (Tx buffer / Tx event / standard and extended filter element layout).
 * The instance RAM base address is SRAMCAN_BASE + instance index *
 * CAN_MSG_RAM_INSTANCE_BYTE_SIZE.
 *
 */

/* ============================= INCLUDES =================================== */
#include "Can_Reg.h"                        /* Module Reg component header    */
#include "Stm32.h"                          /* Device registers definition    */
#include "stddef.h"                         /* offsetof() definition          */

#if defined(FDCAN1)

/* ========================== PRIVATE DECLARATIONS =========================== */

/** \brief Count of read-back iterations of a wait on a register bit */
#define CAN_REG_WAIT_TIMEOUT_RAW     ( 10000u )

/** \brief Register value without any bit set */
#define CAN_REG_BITS_CLEAR           ( 0u )

/* =========================== LOCAL VARIABLES ============================== */

/** \brief Base address of every available FDCAN instance, indexed by can_PeriphId_t */
static FDCAN_GlobalTypeDef * const can_InstanceBase[ CAN_PERIPH_CNT ] =
{
    FDCAN1, /**< CAN_PERIPH_1 base address */
#if defined(FDCAN2)
    FDCAN2, /**< CAN_PERIPH_2 base address */
#endif
#if defined(FDCAN3)
    FDCAN3, /**< CAN_PERIPH_3 base address */
#endif
};

_Static_assert( ( sizeof( can_InstanceBase ) / sizeof( FDCAN_GlobalTypeDef * ) ) == CAN_PERIPH_CNT,
                 "Can_Reg: can_InstanceBase has incorrect size." );


/** \brief Byte offset of every used FDCAN register inside FDCAN_GlobalTypeDef, indexed by can_RegId_t */
static const uint32_t can_RegOffset[ CAN_REG_CNT ] =
{
    offsetof( FDCAN_GlobalTypeDef, DBTP  ), /**< CAN_REG_DBTP  */
    offsetof( FDCAN_GlobalTypeDef, TEST  ), /**< CAN_REG_TEST  */
    offsetof( FDCAN_GlobalTypeDef, CCCR  ), /**< CAN_REG_CCCR  */
    offsetof( FDCAN_GlobalTypeDef, NBTP  ), /**< CAN_REG_NBTP  */
    offsetof( FDCAN_GlobalTypeDef, ECR   ), /**< CAN_REG_ECR   */
    offsetof( FDCAN_GlobalTypeDef, PSR   ), /**< CAN_REG_PSR   */
    offsetof( FDCAN_GlobalTypeDef, IR    ), /**< CAN_REG_IR    */
    offsetof( FDCAN_GlobalTypeDef, IE    ), /**< CAN_REG_IE    */
    offsetof( FDCAN_GlobalTypeDef, ILE   ), /**< CAN_REG_ILE   */
    offsetof( FDCAN_GlobalTypeDef, RXGFC ), /**< CAN_REG_RXGFC */
    offsetof( FDCAN_GlobalTypeDef, XIDAM ), /**< CAN_REG_XIDAM */
    offsetof( FDCAN_GlobalTypeDef, RXF0S ), /**< CAN_REG_RXF0S */
    offsetof( FDCAN_GlobalTypeDef, RXF0A ), /**< CAN_REG_RXF0A */
    offsetof( FDCAN_GlobalTypeDef, RXF1S ), /**< CAN_REG_RXF1S */
    offsetof( FDCAN_GlobalTypeDef, RXF1A ), /**< CAN_REG_RXF1A */
    offsetof( FDCAN_GlobalTypeDef, TXBC  ), /**< CAN_REG_TXBC  */
    offsetof( FDCAN_GlobalTypeDef, TXFQS ), /**< CAN_REG_TXFQS */
    offsetof( FDCAN_GlobalTypeDef, TXBRP ), /**< CAN_REG_TXBRP */
    offsetof( FDCAN_GlobalTypeDef, TXBAR ), /**< CAN_REG_TXBAR */
    offsetof( FDCAN_GlobalTypeDef, TXBCR ), /**< CAN_REG_TXBCR */
    offsetof( FDCAN_GlobalTypeDef, TXBTO ), /**< CAN_REG_TXBTO */
    offsetof( FDCAN_GlobalTypeDef, TXBCF ), /**< CAN_REG_TXBCF */
    offsetof( FDCAN_GlobalTypeDef, TXEFS ), /**< CAN_REG_TXEFS */
    offsetof( FDCAN_GlobalTypeDef, TXEFA ), /**< CAN_REG_TXEFA */
};

_Static_assert( ( sizeof( can_RegOffset ) / sizeof( uint32_t ) ) == CAN_REG_CNT,
                 "Can_Reg: can_RegOffset has incorrect size." );

/* ========================== PRIVATE FUNCTIONS ============================== */

/**
 * \brief Builds the register address of an instance / register pair
 *
 * \param periphId [in] : Instance identification.
 * \param regId    [in] : Register identification.
 *
 * \return Address of the register, or CAN_NULL_PTR for an invalid parameter.
 */
static volatile uint32_t * Can_Reg_GetAddr( can_PeriphId_t periphId, can_RegId_t regId )
{
    volatile uint32_t *regAddr = CAN_NULL_PTR;

    if( ( CAN_PERIPH_CNT > periphId ) &&
        ( CAN_REG_CNT    > regId    )    )
    {
        regAddr = (volatile uint32_t *)( (uint8_t *)can_InstanceBase[periphId] + can_RegOffset[regId] );
    }
    else
    {
        /* Invalid instance or register ID, no address is built */
        regAddr = CAN_NULL_PTR;
    }

    return ( regAddr );
}


/**
 * \brief Builds the message RAM word address of an instance / word-offset pair
 *
 * \param periphId   [in] : Instance identification.
 * \param wordOffset [in] : Word offset inside the instance message RAM slice.
 *
 * \return Address of the word, or CAN_NULL_PTR for an invalid parameter.
 */
static volatile uint32_t * Can_Ram_GetAddr( can_PeriphId_t periphId, uint32_t wordOffset )
{
    volatile uint32_t *wordAddr      = CAN_NULL_PTR;
    uint32_t             byteOffset  = CAN_REG_BITS_CLEAR;

    byteOffset = ( (uint32_t)periphId * CAN_MSG_RAM_INSTANCE_BYTE_SIZE ) + ( wordOffset * sizeof( uint32_t ) );

    if( ( CAN_PERIPH_CNT                      > periphId   ) &&
        ( CAN_MSG_RAM_INSTANCE_BYTE_SIZE      > ( wordOffset * sizeof( uint32_t ) ) )    )
    {
        wordAddr = (volatile uint32_t *)( (uint8_t *)SRAMCAN_BASE + byteOffset );
    }
    else
    {
        /* Invalid instance or out-of-range word offset, no address is built */
        wordAddr = CAN_NULL_PTR;
    }

    return ( wordAddr );
}

/* ========================= EXPORTED FUNCTIONS =============================== */

/**
 * \brief Sets the register bit(s) with the given mask.
 *
 * \param periphId [in] : CAN instance ID
 * \param regId    [in] : CAN register ID
 * \param bitMask  [in] : Mask to apply to the register
 */
void Can_Set_RegBit( can_PeriphId_t periphId, can_RegId_t regId, uint32_t bitMask )
{
    volatile uint32_t *regAddr = Can_Reg_GetAddr( periphId, regId );

    if( CAN_NULL_PTR != regAddr )
    {
        *regAddr |= bitMask;
    }
    else
    {
        /* Invalid instance or register ID, no action required */
    }
}


/**
 * \brief Resets the register bit(s) with the given mask.
 *
 * \param periphId [in] : CAN instance ID
 * \param regId    [in] : CAN register ID
 * \param bitMask  [in] : Mask to apply to the register
 */
void Can_Reset_RegBit( can_PeriphId_t periphId, can_RegId_t regId, uint32_t bitMask )
{
    volatile uint32_t *regAddr = Can_Reg_GetAddr( periphId, regId );

    if( CAN_NULL_PTR != regAddr )
    {
        *regAddr &= ~bitMask;
    }
    else
    {
        /* Invalid instance or register ID, no action required */
    }
}


/**
 * \brief Gets the register bit(s) value with the given mask.
 *
 * \param periphId [in] : CAN instance ID
 * \param regId    [in] : CAN register ID
 * \param bitMask  [in] : Mask to apply to the register
 *
 * \return Register bit value with applied mask, CAN_REG_BITS_CLEAR for an invalid parameter.
 */
uint32_t Can_Get_RegBit( can_PeriphId_t periphId, can_RegId_t regId, uint32_t bitMask )
{
    volatile uint32_t *regAddr  = Can_Reg_GetAddr( periphId, regId );
    uint32_t            retValue = CAN_REG_BITS_CLEAR;

    if( CAN_NULL_PTR != regAddr )
    {
        retValue = ( *regAddr & bitMask );
    }
    else
    {
        /* Invalid instance or register ID, no bit is set */
        retValue = CAN_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Updates the register value with the given mask and value.
 *
 * \param periphId [in] : CAN instance ID
 * \param regId    [in] : CAN register ID
 * \param regMask  [in] : Mask to apply to the register
 * \param regValue [in] : Value to set in the register
 */
void Can_Set_RegVal( can_PeriphId_t periphId, can_RegId_t regId, uint32_t regMask, uint32_t regValue )
{
    volatile uint32_t *regAddr = Can_Reg_GetAddr( periphId, regId );

    if( CAN_NULL_PTR != regAddr )
    {
        *regAddr = ( ( *regAddr & ~regMask ) | ( regValue & regMask ) );
    }
    else
    {
        /* Invalid instance or register ID, no action required */
    }
}


/**
 * \brief Gets the register value with the given mask.
 *
 * \param periphId [in] : CAN instance ID
 * \param regId    [in] : CAN register ID
 * \param regMask  [in] : Mask to apply to the register
 *
 * \return Register value with applied mask, CAN_REG_BITS_CLEAR for an invalid parameter.
 */
uint32_t Can_Get_RegVal( can_PeriphId_t periphId, can_RegId_t regId, uint32_t regMask )
{
    volatile uint32_t *regAddr  = Can_Reg_GetAddr( periphId, regId );
    uint32_t            retValue = CAN_REG_BITS_CLEAR;

    if( CAN_NULL_PTR != regAddr )
    {
        retValue = ( *regAddr & regMask );
    }
    else
    {
        /* Invalid instance or register ID, no bit is set */
        retValue = CAN_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Waits (bounded) until every bit of the mask is set in the register.
 *
 * \param periphId [in] : CAN instance ID
 * \param regId    [in] : CAN register ID
 * \param bitMask  [in] : Mask of the bit(s) to wait for
 *
 * \return CAN_REQUEST_OK once every bit of the mask is set, CAN_REQUEST_ERROR on timeout.
 */
can_RequestState_t Can_Reg_Wait_RegBitSet( can_PeriphId_t periphId, can_RegId_t regId, uint32_t bitMask )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;
    uint32_t              bitValue = CAN_REG_BITS_CLEAR;

    for( uint32_t idx = 0u;
         ( CAN_REG_WAIT_TIMEOUT_RAW > idx ) &&
         ( bitMask                  != bitValue ) ;
         idx++ )
    {
        bitValue = Can_Get_RegBit( periphId, regId, bitMask );
    }

    if( bitMask == bitValue )
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
 * \brief Waits (bounded) until every bit of the mask is cleared in the register.
 *
 * \param periphId [in] : CAN instance ID
 * \param regId    [in] : CAN register ID
 * \param bitMask  [in] : Mask of the bit(s) to wait for
 *
 * \return CAN_REQUEST_OK once every bit of the mask is cleared, CAN_REQUEST_ERROR on timeout.
 */
can_RequestState_t Can_Reg_Wait_RegBitClr( can_PeriphId_t periphId, can_RegId_t regId, uint32_t bitMask )
{
    can_RequestState_t retState = CAN_REQUEST_ERROR;
    uint32_t              bitValue = bitMask;

    for( uint32_t idx = 0u;
         ( CAN_REG_WAIT_TIMEOUT_RAW > idx ) &&
         ( CAN_REG_BITS_CLEAR       != bitValue ) ;
         idx++ )
    {
        bitValue = Can_Get_RegBit( periphId, regId, bitMask );
    }

    if( CAN_REG_BITS_CLEAR == bitValue )
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
 * \brief Writes one 32-bit word of the message RAM of an instance.
 *
 * \param periphId   [in] : CAN instance ID
 * \param wordOffset [in] : Word offset inside the instance message RAM slice
 * \param value      [in] : Value to write
 */
void Can_Ram_Set_Word( can_PeriphId_t periphId, uint32_t wordOffset, uint32_t value )
{
    volatile uint32_t *wordAddr = Can_Ram_GetAddr( periphId, wordOffset );

    if( CAN_NULL_PTR != wordAddr )
    {
        *wordAddr = value;
    }
    else
    {
        /* Invalid instance or out-of-range word offset, no action required */
    }
}


/**
 * \brief Reads one 32-bit word of the message RAM of an instance.
 *
 * \param periphId   [in] : CAN instance ID
 * \param wordOffset [in] : Word offset inside the instance message RAM slice
 *
 * \return Value of the word, CAN_REG_BITS_CLEAR for an invalid parameter.
 */
uint32_t Can_Ram_Get_Word( can_PeriphId_t periphId, uint32_t wordOffset )
{
    volatile uint32_t *wordAddr  = Can_Ram_GetAddr( periphId, wordOffset );
    uint32_t             retValue = CAN_REG_BITS_CLEAR;

    if( CAN_NULL_PTR != wordAddr )
    {
        retValue = *wordAddr;
    }
    else
    {
        /* Invalid instance or out-of-range word offset, no value is read */
        retValue = CAN_REG_BITS_CLEAR;
    }

    return ( retValue );
}

#endif /* FDCAN1 */
