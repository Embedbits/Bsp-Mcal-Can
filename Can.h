/**
 * \author Mr.Nobody
 * \file Can.h
 * \ingroup Can
 * \brief Can module private header file
 *
 * Internal state of the module, private to Can.c; not part of the public
 * interface (Can_Port.h).
 *
 */

#ifndef CAN_CAN_H
#define CAN_CAN_H
/* ============================= INCLUDES =================================== */
#include "Can_Types.h"                      /* Module types definition        */
/* ============================= TYPEDEFS =================================== */

/** \brief Per-instance run-time state, private to Can.c */
typedef struct
{
    can_State_t         State;        /**< Current controller state                                 */
    can_FunctionState_t Initialized;  /**< CAN_FUNCTION_ACTIVE once Can_Init succeeded              */
    can_Config_t        Config;       /**< Copy of the last applied configuration                   */
    can_BusState_t      LastBusState; /**< Bus state reported by the last Can_Task / ISR pass       */
    can_LastError_t     LastError;    /**< Last error code reported by the last Can_Task / ISR pass */
}   can_InstanceState_t;

#endif /* CAN_CAN_H */
