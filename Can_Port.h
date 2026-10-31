/**
 * \author Mr.Nobody
 * \file Can_Port.h
 * \ingroup Can
 * \brief Can module public interface
 *
 * Public functions of the module, identical for every supported family (see
 * the master README). The implementation is specific to the family of the
 * branch (STM32H5: FDCAN1 .. FDCAN3, see the family README).
 *
 */

#ifndef CAN_CAN_PORT_H
#define CAN_CAN_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Can_Types.h"                      /* Module types definition        */
/* ======================== EXPORTED FUNCTIONS ============================== */

/* ------------------------------ Module management --------------------------- */

can_ModuleVersion_t  Can_Get_ModuleVersion   ( void );

can_RequestState_t   Can_Init                ( const can_Config_t * const canConfig );
can_RequestState_t   Can_Deinit              ( can_PeriphId_t periphId );
void                 Can_Task                ( void );
can_RequestState_t   Can_Get_DefaultConfig   ( can_Config_t * const canConfig );

/* ---------------------------------- Control ---------------------------------- */

can_RequestState_t   Can_Set_Start           ( can_PeriphId_t periphId );
can_RequestState_t   Can_Set_Stop            ( can_PeriphId_t periphId );
can_RequestState_t   Can_Get_State           ( can_PeriphId_t periphId, can_State_t * const canState );
can_RequestState_t   Can_Set_PowerDown       ( can_PeriphId_t periphId );
can_RequestState_t   Can_Set_PowerUp         ( can_PeriphId_t periphId );

/* ------------------------------ Bit timing and mode --------------------------- */

can_RequestState_t   Can_Set_BitTiming       ( can_PeriphId_t periphId, const can_BitTiming_t * const bitTiming );
can_RequestState_t   Can_Get_BitTiming       ( can_PeriphId_t periphId, can_BitTiming_t * const bitTiming );
can_RequestState_t   Can_Set_Mode            ( can_PeriphId_t periphId, can_Mode_t mode );
can_RequestState_t   Can_Get_Mode            ( can_PeriphId_t periphId, can_Mode_t * const mode );
can_RequestState_t   Can_Set_AutoRetransmission( can_PeriphId_t periphId, can_FunctionState_t funcState );
can_RequestState_t   Can_Get_AutoRetransmission( can_PeriphId_t periphId, can_FunctionState_t * const funcState );
can_RequestState_t   Can_Get_ClkFrequency     ( can_PeriphId_t periphId, can_Frequency_Hz_t * const clkFreqHz );

/* ---------------------------------- Filters ----------------------------------- */

can_RequestState_t   Can_Set_Filters          ( can_PeriphId_t periphId,
                                                const can_Filter_t * const filters,
                                                can_FilterCnt_t filterCnt );
can_RequestState_t   Can_Set_GlobalFilter     ( can_PeriphId_t periphId,
                                                const can_GlobalFilter_t * const globalFilter );

/* -------------------------------- Transmission --------------------------------- */

can_RequestState_t   Can_Set_TxRequest        ( can_PeriphId_t periphId,
                                                const can_Frame_t * const frame,
                                                can_TxHandle_t * const txHandle );
can_RequestState_t   Can_Set_TxAbort          ( can_PeriphId_t periphId, can_TxHandle_t txHandle );
can_RequestState_t   Can_Get_TxState          ( can_PeriphId_t periphId,
                                                can_TxHandle_t txHandle,
                                                can_RequestState_t * const txState );
can_RequestState_t   Can_Get_TxFreeCount      ( can_PeriphId_t periphId, can_FrameCnt_t * const freeCnt );

/* --------------------------------- Reception ------------------------------------ */

can_RequestState_t   Can_Get_RxCount          ( can_PeriphId_t periphId,
                                                can_RxFifoId_t fifoId,
                                                can_FrameCnt_t * const rxCnt );
can_RequestState_t   Can_Get_RxFrame          ( can_PeriphId_t periphId,
                                                can_RxFifoId_t fifoId,
                                                can_Frame_t * const frame );

/* ---------------------------------- Bus state ------------------------------------ */

can_RequestState_t   Can_Get_BusState         ( can_PeriphId_t periphId, can_BusState_t * const busState );
can_RequestState_t   Can_Get_ErrorCounters    ( can_PeriphId_t periphId, can_ErrorCounters_t * const errorCounters );
can_RequestState_t   Can_Get_LastError        ( can_PeriphId_t periphId, can_LastError_t * const lastError );
can_RequestState_t   Can_Set_BusOffRecovery   ( can_PeriphId_t periphId, can_BusOffRecovery_t busOffRecovery );
can_RequestState_t   Can_Set_BusOffRestart    ( can_PeriphId_t periphId );

/* --------------------------------- Transceiver ------------------------------------ */

can_RequestState_t   Can_Set_TransceiverState ( can_PeriphId_t periphId, can_FunctionState_t funcState );

/* --------------------------------- Interrupts ------------------------------------- */

can_RequestState_t   Can_Set_IrqPriority      ( can_PeriphId_t periphId, nvic_IrqPrio_t irqPrio );
can_RequestState_t   Can_Get_IrqPriority      ( can_PeriphId_t periphId, nvic_IrqPrio_t * const irqPrio );

/* ------------------------------------ Pins ----------------------------------------- */

can_RequestState_t   Can_InitTxGpio           ( can_TxPin_t pinId );
can_RequestState_t   Can_InitRxGpio           ( can_RxPin_t pinId );
can_RequestState_t   Can_InitStandbyGpio      ( gpio_PortId_t portId, gpio_PinId_t pinId );

#ifdef __cplusplus
}
#endif

#endif /* CAN_CAN_PORT_H */
