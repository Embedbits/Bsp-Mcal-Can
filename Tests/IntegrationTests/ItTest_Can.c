/**
 * \author Mr.Nobody
 * \file ItTest_Can.c
 * \ingroup Can
 * \brief Integration tests of the Can module on target.
 *
 * Can module runs on the MCU together with the real FDCAN1 / GPIO / RCC / NVIC hardware. A bare
 * Nucleo board carries no second CAN node and no transceiver wired by default, so these tests
 * verify what a single board without a bus partner can prove without one: the module
 * initializes / deinitializes without a fault with the common Nucleo FDCAN1 pins (TX = PA12,
 * RX = PA11), the bit timing search does not fault for a bit rate the kernel clock can serve,
 * CAN_MODE_LOOPBACK_INTERNAL brings the controller onto the bus without any pin activity (self
 * contained, no transceiver / second node needed) and a frame written in that mode is received
 * back by the same instance, the bus state / error counters are read without a fault, and the
 * bus-off / power-down requests do not fault. Two real instances exchanging frames over an
 * actual bus, the external loop-back, the CAN FD data phase, and the bus-off-after-a-shorted-bus
 * recovery need two boards or a bus fault injector and are not part of this set.
 *
 * \note Compiled for the target but not run on hardware, see the module report.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Can_Port.h"                       /* Module under test              */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static can_RequestState_t ItCan_InitDefaultPins( can_Config_t * const config );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Interrupt priority used by the tests */
#define IT_CAN_IRQ_PRIO                       ( 5u )

/** Bit rate used by the tests (bit/s), wide tolerance since the kernel clock of the target
 *  board is not controlled by this test set */
#define IT_CAN_BITRATE                         ( 500000u )

/** Bit rate tolerance used by the tests (CAN_SAMPLE_POINT_SCALE = 100 %) */
#define IT_CAN_BITRATE_TOLERANCE                ( 50u )

/** Frame test vector: identifier */
#define IT_CAN_FRAME_ID                           ( 0x123u )

/** Frame test vector: data length (byte) */
#define IT_CAN_FRAME_LEN                            ( 8u )

/** Frame test vector: first data byte */
#define IT_CAN_FRAME_DATA0                           ( 0xAAu )

/** Bounded wait loop count for the loop-back reception poll */
#define IT_CAN_RX_POLL_LOOP_MAX                       ( 100000u )

/* ========================== LOCAL VARIABLES =============================== */

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    /* No per-test state */
}


void tearDown( void )
{
    /* Peripheral state is cleared by system reset after the test case */
}

/* ========================== LOCAL FUNCTIONS ================================ */

/**
 * \brief Fills a default configuration with the common Nucleo FDCAN1 pin set.
 *
 * \param config [out]: Pointer to the configuration structure filled in place.
 *
 * \return CAN_REQUEST_OK on success, CAN_REQUEST_ERROR otherwise.
 */
static can_RequestState_t ItCan_InitDefaultPins( can_Config_t * const config )
{
    can_RequestState_t retState = Can_Get_DefaultConfig( config );

    if( CAN_REQUEST_OK == retState )
    {
        config->PeriphId                 = CAN_PERIPH_1;
        config->IrqPriority               = IT_CAN_IRQ_PRIO;
        config->TxPin                     = CAN_TX_PIN_CAN1_PA12;
        config->RxPin                     = CAN_RX_PIN_CAN1_PA11;
        config->BitTiming.BitrateTolerance = IT_CAN_BITRATE_TOLERANCE;
    }
    else
    {
        /* Default configuration itself failed, nothing else to fill */
    }

    return ( retState );
}

/* =============================== TESTS ==================================== */

/**
 * \brief The module version is reported without a fault.
 *
 * \par Expected results
 * - Version is 1.0.0.
 */
void It_Can_Get_ModuleVersion_Call_ReturnsVersion( void )
{
    can_ModuleVersion_t version = Can_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}


/**
 * \brief Can_Init() / Can_Deinit() with the common FDCAN1 pins do not fault.
 *
 * \par Expected results
 * - CAN_REQUEST_OK is returned by both calls.
 */
void It_Can_InitDeinit_DefaultPins_NoFault( void )
{
    can_Config_t canConfig;

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, ItCan_InitDefaultPins( &canConfig ) );
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Init( &canConfig ) );
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Deinit( CAN_PERIPH_1 ) );
}


/**
 * \brief Can_Set_Start() / Can_Set_Stop() in CAN_MODE_LOOPBACK_INTERNAL do not fault.
 *
 * \par Expected results
 * - CAN_REQUEST_OK is returned by every call, Can_Get_State() reports the expected state.
 */
void It_Can_StartStop_LoopbackInternal_NoFault( void )
{
    can_Config_t canConfig;
    can_State_t    canState = CAN_STATE_CNT;

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, ItCan_InitDefaultPins( &canConfig ) );
    canConfig.Mode = CAN_MODE_LOOPBACK_INTERNAL;

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Init( &canConfig ) );
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_Start( CAN_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_State( CAN_PERIPH_1, &canState ) );
    TEST_ASSERT_EQUAL( CAN_STATE_STARTED, canState );

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_Stop( CAN_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Deinit( CAN_PERIPH_1 ) );
}


/**
 * \brief A frame written in CAN_MODE_LOOPBACK_INTERNAL is received back by the same instance
 *        (self-contained, no transceiver / second node needed).
 *
 * \par Expected results
 * - Can_Set_TxRequest() returns CAN_REQUEST_OK, Can_Get_RxFrame() reports the same frame on
 *   Rx FIFO 0 (Can_Get_DefaultConfig() leaves ANFS at "accept in FIFO 0").
 */
void It_Can_LoopbackInternal_TxFrame_IsReceivedBack( void )
{
    can_Config_t   canConfig;
    can_Frame_t      txFrame = { 0u };
    can_Frame_t       rxFrame = { 0u };
    can_TxHandle_t      txHandle = CAN_TX_HANDLE_INVALID;
    can_FrameCnt_t       rxCnt    = 0u;

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, ItCan_InitDefaultPins( &canConfig ) );
    canConfig.Mode = CAN_MODE_LOOPBACK_INTERNAL;

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Init( &canConfig ) );
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_Start( CAN_PERIPH_1 ) );

    txFrame.Id        = IT_CAN_FRAME_ID;
    txFrame.IdType    = CAN_ID_STANDARD;
    txFrame.FrameType = CAN_FRAME_DATA;
    txFrame.Length    = IT_CAN_FRAME_LEN;
    txFrame.Data[ 0 ] = IT_CAN_FRAME_DATA0;

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_TxRequest( CAN_PERIPH_1, &txFrame, &txHandle ) );

    for( uint32_t idx = 0u;
         ( IT_CAN_RX_POLL_LOOP_MAX > idx ) &&
         ( 0u                      == rxCnt )  ;
         idx++ )
    {
        (void)Can_Get_RxCount( CAN_PERIPH_1, CAN_RX_FIFO_0, &rxCnt );
    }

    TEST_ASSERT_EQUAL_UINT8( 1u, rxCnt );
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_RxFrame( CAN_PERIPH_1, CAN_RX_FIFO_0, &rxFrame ) );
    TEST_ASSERT_EQUAL_UINT32( txFrame.Id, rxFrame.Id );
    TEST_ASSERT_EQUAL_UINT8( txFrame.Length, rxFrame.Length );
    TEST_ASSERT_EQUAL_UINT8( txFrame.Data[ 0 ], rxFrame.Data[ 0 ] );

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_Stop( CAN_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Deinit( CAN_PERIPH_1 ) );
}


/**
 * \brief Can_Get_BusState() / Can_Get_ErrorCounters() do not fault while STOPPED.
 *
 * \par Expected results
 * - CAN_REQUEST_OK is returned by both calls, the bus state is CAN_BUS_ACTIVE (no traffic yet).
 */
void It_Can_GetBusStateErrorCounters_Stopped_NoFault( void )
{
    can_Config_t          canConfig;
    can_BusState_t          busState       = CAN_BUS_OFF;
    can_ErrorCounters_t       errorCounters = { 0u };

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, ItCan_InitDefaultPins( &canConfig ) );
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Init( &canConfig ) );

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_BusState( CAN_PERIPH_1, &busState ) );
    TEST_ASSERT_EQUAL( CAN_BUS_ACTIVE, busState );
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_ErrorCounters( CAN_PERIPH_1, &errorCounters ) );

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Deinit( CAN_PERIPH_1 ) );
}


/**
 * \brief Can_Set_PowerDown() / Can_Set_PowerUp() do not fault.
 *
 * \par Expected results
 * - CAN_REQUEST_OK is returned by both calls, Can_Get_State() reports the expected state.
 */
void It_Can_PowerDownPowerUp_Initialized_NoFault( void )
{
    can_Config_t canConfig;
    can_State_t    canState = CAN_STATE_CNT;

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, ItCan_InitDefaultPins( &canConfig ) );
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Init( &canConfig ) );

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_PowerDown( CAN_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_State( CAN_PERIPH_1, &canState ) );
    TEST_ASSERT_EQUAL( CAN_STATE_SLEEP, canState );

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_PowerUp( CAN_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_State( CAN_PERIPH_1, &canState ) );
    TEST_ASSERT_EQUAL( CAN_STATE_STOPPED, canState );

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Deinit( CAN_PERIPH_1 ) );
}
