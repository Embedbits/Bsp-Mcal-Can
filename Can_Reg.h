/**
 * \author Mr.Nobody
 * \file Can_Reg.h
 * \ingroup Can
 * \brief Can module Reg component header file
 *
 * This component contains the functionality for register operations of the
 * multi-instance FDCAN peripheral: every accessor takes the instance
 * identification in addition to the register identification, the address is
 * built from the instance base address and the byte offset of the register
 * inside the CMSIS FDCAN_GlobalTypeDef structure. Message RAM access
 * (acceptance filters, Rx FIFOs, Tx buffers, Tx event FIFO) is provided by a
 * second set of accessors working on the fixed, non-configurable layout
 * described in the module README (one 0x350-byte / 212-word slice per
 * instance of the shared message RAM, RM0481/RM0492/RM0517 FDCAN chapter).
 *
 */

#ifndef CAN_CAN_REG_H
#define CAN_CAN_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Can_Types.h"                      /* Module types definition        */
/* ============================= TYPEDEFS =================================== */

/** \brief List of FDCAN registers used by the module */
typedef enum
{
    CAN_REG_DBTP = 0u, /**< Data bit timing and prescaler register    */
    CAN_REG_TEST,      /**< Test register                             */
    CAN_REG_CCCR,      /**< CC control register                       */
    CAN_REG_NBTP,      /**< Nominal bit timing and prescaler register */
    CAN_REG_ECR,       /**< Error counter register                    */
    CAN_REG_PSR,       /**< Protocol status register                  */
    CAN_REG_IR,        /**< Interrupt register                        */
    CAN_REG_IE,        /**< Interrupt enable register                 */
    CAN_REG_ILE,       /**< Interrupt line enable register            */
    CAN_REG_RXGFC,     /**< Global filter configuration register      */
    CAN_REG_XIDAM,     /**< Extended ID AND mask register             */
    CAN_REG_RXF0S,     /**< Rx FIFO 0 status register                 */
    CAN_REG_RXF0A,     /**< Rx FIFO 0 acknowledge register            */
    CAN_REG_RXF1S,     /**< Rx FIFO 1 status register                 */
    CAN_REG_RXF1A,     /**< Rx FIFO 1 acknowledge register            */
    CAN_REG_TXBC,      /**< Tx buffer configuration register          */
    CAN_REG_TXFQS,     /**< Tx FIFO / queue status register           */
    CAN_REG_TXBRP,     /**< Tx buffer request pending register        */
    CAN_REG_TXBAR,     /**< Tx buffer add request register            */
    CAN_REG_TXBCR,     /**< Tx buffer cancellation request register   */
    CAN_REG_TXBTO,     /**< Tx buffer transmission occurred register  */
    CAN_REG_TXBCF,     /**< Tx buffer cancellation finished register  */
    CAN_REG_TXEFS,     /**< Tx event FIFO status register             */
    CAN_REG_TXEFA,     /**< Tx event FIFO acknowledge register        */
    CAN_REG_CNT        /**< Count of available FDCAN registers        */
}   can_RegId_t;

/** \brief List of 32-bit words inside one message RAM element */
typedef enum
{
    CAN_RAM_WORD_0 = 0u, /**< Element word 0 (header: ID / ESI / XTD / RTR, or filter word 0) */
    CAN_RAM_WORD_1     , /**< Element word 1 (header: MM / EFC / FDF / BRS / DLC / RXTS, or filter word 1) */
    CAN_RAM_WORD_CNT     /**< Count of header words of an element */
}   can_RamWordId_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** \brief Byte size of one message RAM slice of one instance (RM: FDCANn end - FDCANn start + 4) */
#define CAN_MSG_RAM_INSTANCE_BYTE_SIZE         ( 0x350u )

/** \brief Word offset of the standard filter list inside the instance message RAM slice (FLSSA) */
#define CAN_RAM_WORD_OFFSET_STD_FILTER          ( 0u )

/** \brief Word offset of the extended filter list inside the instance message RAM slice (FLESA) */
#define CAN_RAM_WORD_OFFSET_EXT_FILTER          ( CAN_STD_FILTER_ELEMENT_CNT )

/** \brief Word offset of Rx FIFO 0 inside the instance message RAM slice */
#define CAN_RAM_WORD_OFFSET_RX_FIFO_0            ( CAN_RAM_WORD_OFFSET_EXT_FILTER + \
                                                    ( CAN_EXT_FILTER_ELEMENT_CNT * CAN_RAM_WORD_CNT ) )

/** \brief Word size of one Rx FIFO / Tx buffer element (18 32-bit words, up to 64 data byte) */
#define CAN_RAM_ELEMENT_WORD_CNT_FRAME            ( 18u )

/** \brief Word size of one Tx event FIFO element */
#define CAN_RAM_ELEMENT_WORD_CNT_TX_EVENT          ( 2u )

/** \brief Word offset of Rx FIFO 1 inside the instance message RAM slice */
#define CAN_RAM_WORD_OFFSET_RX_FIFO_1                ( CAN_RAM_WORD_OFFSET_RX_FIFO_0 + \
                                                        ( CAN_RX_FIFO_ELEMENT_CNT * CAN_RAM_ELEMENT_WORD_CNT_FRAME ) )

/** \brief Word offset of the Tx event FIFO inside the instance message RAM slice (EFSA) */
#define CAN_RAM_WORD_OFFSET_TX_EVENT                  ( CAN_RAM_WORD_OFFSET_RX_FIFO_1 + \
                                                          ( CAN_RX_FIFO_ELEMENT_CNT * CAN_RAM_ELEMENT_WORD_CNT_FRAME ) )

/** \brief Word offset of the Tx buffers inside the instance message RAM slice (TBSA) */
#define CAN_RAM_WORD_OFFSET_TX_BUFFER                  ( CAN_RAM_WORD_OFFSET_TX_EVENT + \
                                                      ( CAN_TX_EVENT_ELEMENT_CNT * CAN_RAM_ELEMENT_WORD_CNT_TX_EVENT ) )

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* ---------------------------- Generic accessors --------------------------- */

void                 Can_Set_RegBit         ( can_PeriphId_t periphId, can_RegId_t regId, uint32_t bitMask );
void                 Can_Reset_RegBit       ( can_PeriphId_t periphId, can_RegId_t regId, uint32_t bitMask );
uint32_t             Can_Get_RegBit         ( can_PeriphId_t periphId, can_RegId_t regId, uint32_t bitMask );

void                 Can_Set_RegVal         ( can_PeriphId_t periphId,
                                              can_RegId_t regId,
                                              uint32_t regMask,
                                              uint32_t regValue );
uint32_t             Can_Get_RegVal         ( can_PeriphId_t periphId, can_RegId_t regId, uint32_t regMask );

can_RequestState_t   Can_Reg_Wait_RegBitSet ( can_PeriphId_t periphId, can_RegId_t regId, uint32_t bitMask );
can_RequestState_t   Can_Reg_Wait_RegBitClr ( can_PeriphId_t periphId, can_RegId_t regId, uint32_t bitMask );

/* ----------------------------- Message RAM access -------------------------- */

void                 Can_Ram_Set_Word       ( can_PeriphId_t periphId, uint32_t wordOffset, uint32_t value );
uint32_t             Can_Ram_Get_Word       ( can_PeriphId_t periphId, uint32_t wordOffset );

#ifdef __cplusplus
}
#endif

#endif /* CAN_CAN_REG_H */
