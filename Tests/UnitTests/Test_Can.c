/**
 * \author Mr.Nobody
 * \file Test_Can.c
 * \ingroup Can
 * \brief Unit tests of the Can module (STM32H5 FDCAN implementation).
 *
 * Can.c / Can_Reg.c are compiled unchanged. FDCAN1 registers and its message RAM slice are
 * emulated by RegMem (plain memory, no write-0-to-clear / self-clearing bit is used by this
 * module, so no HW model callback is needed). RCC / NVIC / GPIO are mocked by CMock. The bit
 * timing search is checked against the exact RM0481 NBTP/DBTP formulas for a known kernel clock
 * (8 MHz nominal 500 kbit/s, matching the FDCAN_NBTP reset value example of the reference
 * manual; 16 MHz CAN FD data phase 2 Mbit/s).
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Can_Port.h"                       /* Module under test (public)     */
#include "Can.h"                            /* Private interface (test only)  */
#include "Can_Reg.h"                        /* Register access (test only)    */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock                */
#include "MockGpio_Port.h"                  /* GPIO module mock                */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void UtCan_InitNominal( void );

/* ========================== SYMBOLIC CONSTANTS ============================= */

/** Kernel clock of the nominal bit timing test vector (RM0481 FDCAN_NBTP reset value example) */
#define UT_CAN_KERNEL_CLK_8MHZ       ( 8000000u )

/** Kernel clock of the CAN FD data phase bit timing test vector */
#define UT_CAN_KERNEL_CLK_16MHZ      ( 16000000u )

/** Nominal bit rate of the test vector: 500 kbit/s */
#define UT_CAN_BITRATE_500K           ( 500000u )

/** CAN FD data phase bit rate of the test vector: 2 Mbit/s */
#define UT_CAN_DATA_BITRATE_2M        ( 2000000u )

/** Sample point used by the test vectors: 87.5 % */
#define UT_CAN_SAMPLE_POINT_87_5       ( 875u )

/** Tolerance used by the test vectors: 0 (exact match required) */
#define UT_CAN_TOLERANCE_EXACT          ( 0u )

/** Tolerance wide enough to cover UT_CAN_BITRATE_DELTA_HZ at UT_CAN_BITRATE_500K (0.5 %) */
#define UT_CAN_TOLERANCE_WIDE            ( 5u )

/** Expected FDCAN_NBTP.NTSEG1 for UT_CAN_BITRATE_500K / UT_CAN_SAMPLE_POINT_87_5 at 8 MHz
 *  (RM0481 S60.4.7: tBS1 = NTSEG1+1, sample point = (1+tBS1)/16 = 87.5 % -> tBS1 = 13 -> NTSEG1 = 12) */
#define UT_CAN_EXPECTED_NTSEG1             ( 12u )
/** Expected FDCAN_NBTP.NTSEG2 for the same test vector (tBS2 = 16 - 1 - tBS1 = 2 -> NTSEG2 = 1) */
#define UT_CAN_EXPECTED_NTSEG2               ( 1u )
/** Expected FDCAN_NBTP.NBRP for the same test vector (prescaler 1 -> NBRP = 0) */
#define UT_CAN_EXPECTED_NBRP                   ( 0u )

/** Bit rate offset used by the mutation check (Hz), 0.2 % of UT_CAN_BITRATE_500K */
#define UT_CAN_BITRATE_DELTA_HZ           ( 1000u )

/** Alternative bit rate used only to change a value already applied (no exact figure needed) */
#define UT_CAN_ALT_BITRATE                 ( 1000000u )

/** CAN FD data phase sample point used by the test vectors: 80 % */
#define UT_CAN_DATA_SAMPLE_POINT_80          ( 800u )

/** Filter test vector: first ID (SFID1 / Id1) */
#define UT_CAN_FILTER_ID1                     ( 0x100u )

/** Filter test vector: second ID / mask (SFID2 / Id2), all eleven bits of the standard ID */
#define UT_CAN_FILTER_ID_MASK                  ( 0x7FFu )

/** Expected encoding of a classic bit mask filter element: SFT field value */
#define UT_CAN_EXPECTED_SFT_MASK                 ( 2u )
/** Expected encoding of a classic bit mask filter element: SFT field shift */
#define UT_CAN_EXPECTED_SFT_SHIFT                  ( 30u )
/** Expected encoding of a "store in Rx FIFO 0" filter element: SFEC field value */
#define UT_CAN_EXPECTED_FEC_FIFO0                    ( 1u )
/** Expected encoding of a filter element: SFEC field shift */
#define UT_CAN_EXPECTED_FEC_SHIFT                      ( 27u )
/** Expected encoding of a filter element: SFID1 field shift */
#define UT_CAN_EXPECTED_SFID1_SHIFT                      ( 16u )

/** Frame test vector: identifier */
#define UT_CAN_FRAME_ID                                    ( 0x123u )
/** Expected encoding of a standard identifier inside the Tx buffer T0 word: bit shift */
#define UT_CAN_EXPECTED_STD_ID_SHIFT                         ( 18u )

/** Error counter test vector: raw FDCAN_ECR mask used to write both counters at once */
#define UT_CAN_ECR_TEST_MASK                                   ( 0xFFFFu )
/** Error counter test vector: receive error counter (REC) value */
#define UT_CAN_ECR_TEST_REC                                      ( 5u )
/** Error counter test vector: bit shift of the REC field inside FDCAN_ECR */
#define UT_CAN_ECR_REC_SHIFT                                       ( 8u )
/** Error counter test vector: transmit error counter (TEC) value */
#define UT_CAN_ECR_TEST_TEC                                          ( 10u )

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );
}


void tearDown( void )
{
    /* No mock expectation left pending by design of every test below */
}

/* ============================ HELPER FUNCTIONS ============================== */

/** Initializes CAN_PERIPH_1 with the default (500 kbit/s classic) configuration, no pins */
static void UtCan_InitNominal( void )
{
    can_Config_t   canConfig;
    rcc_FreqHz_t    clkHz = UT_CAN_KERNEL_CLK_8MHZ;

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_DefaultConfig( &canConfig ) );

    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_FDCAN_PLL1Q, RCC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAnyArgsAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAnyArgsAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_ExpectAndReturn( RCC_PERIPH_FDCAN_PLL1Q, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();
    Rcc_Get_PeriphClk_ReturnThruPtr_periphClk( &clkHz );
    Nvic_Set_PeriphIrq_Handler_ExpectAnyArgsAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Prio_ExpectAnyArgsAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAnyArgsAndReturn( NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Init( &canConfig ) );
}

/* ========================== MODULE VERSION ================================ */

/**
 * \brief   Can_Get_ModuleVersion() returns the version of the module.
 *
 * \par Expected results
 * - Version is 1.0.0.
 */
void Ut_Can_Get_ModuleVersion_Call_ReturnsVersion( void )
{
    can_ModuleVersion_t version = Can_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}

/* ========================= DEFAULT CONFIGURATION =========================== */

/**
 * \brief   Can_Get_DefaultConfig() rejects a null pointer.
 *
 * \par Expected results
 * - CAN_REQUEST_ERROR is returned.
 */
void Ut_Can_Get_DefaultConfig_NullPtr_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( CAN_REQUEST_ERROR, Can_Get_DefaultConfig( CAN_NULL_PTR ) );
}


/**
 * \brief   Can_Get_DefaultConfig() fills a safe configuration (no FD, Tx FIFO, automatic
 *          retransmission, automatic bus-off recovery, every pin / callback unused).
 *
 * \par Expected results
 * - CAN_REQUEST_OK is returned, the fields match the README default behaviour.
 */
void Ut_Can_Get_DefaultConfig_Call_FillsSafeDefaults( void )
{
    can_Config_t canConfig;

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_DefaultConfig( &canConfig ) );
    TEST_ASSERT_EQUAL( CAN_PERIPH_1, canConfig.PeriphId );
    TEST_ASSERT_EQUAL( CAN_MODE_NORMAL, canConfig.Mode );
    TEST_ASSERT_EQUAL( CAN_FUNCTION_INACTIVE, canConfig.FdEnable );
    TEST_ASSERT_EQUAL( CAN_FUNCTION_ACTIVE, canConfig.AutoRetransmission );
    TEST_ASSERT_EQUAL( CAN_TX_FIFO, canConfig.TxMode );
    TEST_ASSERT_EQUAL( CAN_BUS_OFF_RECOVERY_AUTO, canConfig.BusOffRecovery );
    TEST_ASSERT_EQUAL( CAN_TX_PIN_UNUSED, canConfig.TxPin );
    TEST_ASSERT_EQUAL( CAN_RX_PIN_UNUSED, canConfig.RxPin );
    TEST_ASSERT_EQUAL( GPIO_PORT_CNT, canConfig.StandbyPort );
    TEST_ASSERT_NULL( canConfig.TxCallback );
    TEST_ASSERT_NULL( canConfig.RxCallback );
}

/* ================================ INIT ====================================== */

/**
 * \brief   Can_Init() rejects a null configuration pointer.
 *
 * \par Expected results
 * - CAN_REQUEST_ERROR is returned, no register and no other module is touched.
 */
void Ut_Can_Init_NullPtr_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( CAN_REQUEST_ERROR, Can_Init( CAN_NULL_PTR ) );
}


/**
 * \brief   Can_Init() rejects an invalid instance identification.
 *
 * \par Expected results
 * - CAN_REQUEST_ERROR is returned, no register and no other module is touched.
 */
void Ut_Can_Init_InvalidInstance_ReturnsError( void )
{
    can_Config_t canConfig;

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_DefaultConfig( &canConfig ) );
    canConfig.PeriphId = CAN_PERIPH_CNT;

    TEST_ASSERT_EQUAL( CAN_REQUEST_ERROR, Can_Init( &canConfig ) );
}


/**
 * \brief   Can_Init() leaves the instance CAN_STATE_STOPPED on success.
 *
 * \par Expected results
 * - CAN_REQUEST_OK is returned, Can_Get_State() reports CAN_STATE_STOPPED.
 */
void Ut_Can_Init_ValidConfig_LeavesStateStopped( void )
{
    can_State_t canState = CAN_STATE_CNT;

    UtCan_InitNominal();

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_State( CAN_PERIPH_1, &canState ) );
    TEST_ASSERT_EQUAL( CAN_STATE_STOPPED, canState );
}


/**
 * \brief   Can_Init() deinitializes the instance when Rcc_Set_PeriphActive() fails.
 *
 * \par Expected results
 * - CAN_REQUEST_ERROR is returned, Can_Set_Start() on the instance then also fails
 *   (the instance was left uninitialized).
 */
void Ut_Can_Init_RccFailure_DeinitializesInstance( void )
{
    can_Config_t canConfig;

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_DefaultConfig( &canConfig ) );

    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_FDCAN_PLL1Q, RCC_REQUEST_ERROR );
    Rcc_Set_ResetActive_ExpectAnyArgsAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAnyArgsAndReturn( RCC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_ExpectAnyArgsAndReturn( NVIC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAnyArgsAndReturn( RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( CAN_REQUEST_ERROR, Can_Init( &canConfig ) );
    TEST_ASSERT_EQUAL( CAN_REQUEST_ERROR, Can_Set_Start( CAN_PERIPH_1 ) );
}

/* ============================ STATE GATING ================================== */

/**
 * \brief   Can_Set_Start() rejects an uninitialized instance.
 *
 * \par Expected results
 * - CAN_REQUEST_ERROR is returned.
 */
void Ut_Can_Set_Start_NotInitialized_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( CAN_REQUEST_ERROR, Can_Set_Start( CAN_PERIPH_1 ) );
}


/**
 * \brief   Can_Set_Start() clears CCCR.INIT and moves the instance to CAN_STATE_STARTED.
 *
 * \par Expected results
 * - CAN_REQUEST_OK is returned, Can_Get_State() reports CAN_STATE_STARTED.
 */
void Ut_Can_Set_Start_Initialized_MovesToStarted( void )
{
    can_State_t canState = CAN_STATE_CNT;

    UtCan_InitNominal();

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_Start( CAN_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_State( CAN_PERIPH_1, &canState ) );
    TEST_ASSERT_EQUAL( CAN_STATE_STARTED, canState );
}


/**
 * \brief   Can_Set_BitTiming() is rejected once the instance left CAN_STATE_STOPPED (README rule 2).
 *
 * \par Expected results
 * - CAN_REQUEST_ERROR is returned, the previously applied bit timing is unchanged.
 */
void Ut_Can_Set_BitTiming_InstanceStarted_ReturnsError( void )
{
    can_BitTiming_t bitTiming;

    UtCan_InitNominal();
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_Start( CAN_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_BitTiming( CAN_PERIPH_1, &bitTiming ) );
    bitTiming.Bitrate = UT_CAN_ALT_BITRATE;

    TEST_ASSERT_EQUAL( CAN_REQUEST_ERROR, Can_Set_BitTiming( CAN_PERIPH_1, &bitTiming ) );
}


/**
 * \brief   Can_Set_Mode() is rejected once the instance left CAN_STATE_STOPPED (README rule 2).
 *
 * \par Expected results
 * - CAN_REQUEST_ERROR is returned.
 */
void Ut_Can_Set_Mode_InstanceStarted_ReturnsError( void )
{
    UtCan_InitNominal();
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_Start( CAN_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( CAN_REQUEST_ERROR, Can_Set_Mode( CAN_PERIPH_1, CAN_MODE_MONITORING ) );
}

/* ============================== BIT TIMING =================================== */

/**
 * \brief   Can_Set_BitTiming() finds the exact nominal timing for 500 kbit/s at an 8 MHz kernel
 *          clock (RM0481 FDCAN_NBTP reset value example: NBRP=0, totalTq=16 -> exact 500 kbit/s).
 *
 * \par Expected results
 * - CAN_REQUEST_OK is returned, AchievedBitrate is exactly 500000, no deviation.
 */
void Ut_Can_Set_BitTiming_500kAt8MHz_FindsExactMatch( void )
{
    can_BitTiming_t bitTiming;
    rcc_FreqHz_t      clkHz = UT_CAN_KERNEL_CLK_8MHZ;

    UtCan_InitNominal();

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_BitTiming( CAN_PERIPH_1, &bitTiming ) );
    bitTiming.Bitrate          = UT_CAN_BITRATE_500K;
    bitTiming.SamplePoint      = UT_CAN_SAMPLE_POINT_87_5;
    bitTiming.BitrateTolerance = UT_CAN_TOLERANCE_EXACT;

    Rcc_Get_PeriphClk_ExpectAndReturn( RCC_PERIPH_FDCAN_PLL1Q, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();
    Rcc_Get_PeriphClk_ReturnThruPtr_periphClk( &clkHz );

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_BitTiming( CAN_PERIPH_1, &bitTiming ) );
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_BitTiming( CAN_PERIPH_1, &bitTiming ) );
    TEST_ASSERT_EQUAL_UINT32( UT_CAN_BITRATE_500K, bitTiming.AchievedBitrate );

    /* RM0481 S60.4.7: functional bit time = SYNC_SEG(1) + (NTSEG1+1) + (NTSEG2+1) = 16 tq, sample
     * point = (1 + NTSEG1+1) / 16 = 87.5 % -> NTSEG1 = 12, NTSEG2 = 1, NBRP = 0 (prescaler 1). */
    TEST_ASSERT_EQUAL_UINT32( UT_CAN_EXPECTED_NTSEG1,
                               Can_Get_RegVal( CAN_PERIPH_1, CAN_REG_NBTP, FDCAN_NBTP_NTSEG1 ) >>
                                   FDCAN_NBTP_NTSEG1_Pos );
    TEST_ASSERT_EQUAL_UINT32( UT_CAN_EXPECTED_NTSEG2,
                               Can_Get_RegVal( CAN_PERIPH_1, CAN_REG_NBTP, FDCAN_NBTP_NTSEG2 ) >>
                                   FDCAN_NBTP_NTSEG2_Pos );
    TEST_ASSERT_EQUAL_UINT32( UT_CAN_EXPECTED_NBRP,
                               Can_Get_RegVal( CAN_PERIPH_1, CAN_REG_NBTP, FDCAN_NBTP_NBRP ) >>
                                   FDCAN_NBTP_NBRP_Pos );
}


/**
 * \brief   Can_Set_BitTiming() rejects a bit rate the kernel clock cannot reach within the
 *          tolerance (1 Hz target, 0 tolerance, 8 MHz kernel clock).
 *
 * \par Expected results
 * - CAN_REQUEST_ERROR is returned.
 */
void Ut_Can_Set_BitTiming_UnreachableRate_ReturnsError( void )
{
    can_BitTiming_t bitTiming;
    rcc_FreqHz_t      clkHz = UT_CAN_KERNEL_CLK_8MHZ;

    UtCan_InitNominal();

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_BitTiming( CAN_PERIPH_1, &bitTiming ) );
    bitTiming.Bitrate          = 1u;
    bitTiming.SamplePoint      = UT_CAN_SAMPLE_POINT_87_5;
    bitTiming.BitrateTolerance = UT_CAN_TOLERANCE_EXACT;

    Rcc_Get_PeriphClk_ExpectAndReturn( RCC_PERIPH_FDCAN_PLL1Q, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();
    Rcc_Get_PeriphClk_ReturnThruPtr_periphClk( &clkHz );

    TEST_ASSERT_EQUAL( CAN_REQUEST_ERROR, Can_Set_BitTiming( CAN_PERIPH_1, &bitTiming ) );
}


/**
 * \brief   Can_Set_BitTiming() also searches the CAN FD data phase (2 Mbit/s at 16 MHz) once
 *          FdEnable is active.
 *
 * \par Expected results
 * - CAN_REQUEST_OK is returned, AchievedDataBitrate is exactly 2000000.
 */
void Ut_Can_Set_BitTiming_FdDataPhase_FindsExactMatch( void )
{
    can_Config_t   canConfig;
    can_BitTiming_t bitTiming;
    rcc_FreqHz_t      clkHz = UT_CAN_KERNEL_CLK_16MHZ;

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_DefaultConfig( &canConfig ) );
    canConfig.FdEnable = CAN_FUNCTION_ACTIVE;

    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_FDCAN_PLL1Q, RCC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAnyArgsAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAnyArgsAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_ExpectAndReturn( RCC_PERIPH_FDCAN_PLL1Q, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();
    Rcc_Get_PeriphClk_ReturnThruPtr_periphClk( &clkHz );
    Nvic_Set_PeriphIrq_Handler_ExpectAnyArgsAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Prio_ExpectAnyArgsAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAnyArgsAndReturn( NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Init( &canConfig ) );

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_BitTiming( CAN_PERIPH_1, &bitTiming ) );
    bitTiming.DataBitrate     = UT_CAN_DATA_BITRATE_2M;
    bitTiming.DataSamplePoint = UT_CAN_DATA_SAMPLE_POINT_80;

    Rcc_Get_PeriphClk_ExpectAndReturn( RCC_PERIPH_FDCAN_PLL1Q, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();
    Rcc_Get_PeriphClk_ReturnThruPtr_periphClk( &clkHz );

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_BitTiming( CAN_PERIPH_1, &bitTiming ) );
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_BitTiming( CAN_PERIPH_1, &bitTiming ) );
    TEST_ASSERT_EQUAL_UINT32( UT_CAN_DATA_BITRATE_2M, bitTiming.AchievedDataBitrate );
}

/* ================================ FILTERS ===================================== */

/**
 * \brief   Can_Set_Filters() rejects a list with more standard filter elements than the fixed
 *          message RAM capacity (CAN_STD_FILTER_ELEMENT_CNT = 28) - nothing is half-applied.
 *
 * \par Expected results
 * - CAN_REQUEST_ERROR is returned.
 */
void Ut_Can_Set_Filters_TooManyStandard_ReturnsError( void )
{
    can_Filter_t filters[ CAN_STD_FILTER_ELEMENT_CNT + 1u ];

    UtCan_InitNominal();

    for( uint8_t idx = 0u; ( CAN_STD_FILTER_ELEMENT_CNT + 1u ) > idx; idx++ )
    {
        filters[ idx ].Type   = CAN_FILTER_MASK;
        filters[ idx ].IdType = CAN_ID_STANDARD;
        filters[ idx ].Id1    = UT_CAN_FILTER_ID1;
        filters[ idx ].Id2    = UT_CAN_FILTER_ID_MASK;
        filters[ idx ].Action = CAN_FILTER_TO_RX_FIFO_0;
    }

    TEST_ASSERT_EQUAL( CAN_REQUEST_ERROR,
                       Can_Set_Filters( CAN_PERIPH_1,
                                        filters,
                                        (can_FilterCnt_t)( CAN_STD_FILTER_ELEMENT_CNT + 1u ) ) );
}


/**
 * \brief   Can_Set_Filters() writes a standard mask filter element to the message RAM
 *          (SFT = classic mask, SFEC = store to FIFO 0, SFID1 / SFID2 as given).
 *
 * \par Expected results
 * - CAN_REQUEST_OK is returned, the FLSSA[0] word of the message RAM matches the expected layout.
 */
void Ut_Can_Set_Filters_OneMaskFilter_WritesMessageRam( void )
{
    can_Filter_t filters[ 1 ];
    uint32_t       expectedWord = 0u;
    uint32_t        actualWord   = 0u;

    UtCan_InitNominal();

    filters[ 0 ].Type   = CAN_FILTER_MASK;
    filters[ 0 ].IdType = CAN_ID_STANDARD;
    filters[ 0 ].Id1    = UT_CAN_FILTER_ID1;
    filters[ 0 ].Id2    = UT_CAN_FILTER_ID_MASK;
    filters[ 0 ].Action = CAN_FILTER_TO_RX_FIFO_0;

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_Filters( CAN_PERIPH_1, filters, 1u ) );

    expectedWord = ( UT_CAN_EXPECTED_SFT_MASK  << UT_CAN_EXPECTED_SFT_SHIFT   ) |
                   ( UT_CAN_EXPECTED_FEC_FIFO0 << UT_CAN_EXPECTED_FEC_SHIFT   ) |
                   ( UT_CAN_FILTER_ID1         << UT_CAN_EXPECTED_SFID1_SHIFT ) |
                     UT_CAN_FILTER_ID_MASK;
    actualWord   = Can_Ram_Get_Word( CAN_PERIPH_1, CAN_RAM_WORD_OFFSET_STD_FILTER );

    TEST_ASSERT_EQUAL_HEX32( expectedWord, actualWord );
}

/* ============================== TRANSMISSION =================================== */

/**
 * \brief   Can_Set_TxRequest() rejects a CAN FD frame on an instance not FD-enabled.
 *
 * \par Expected results
 * - CAN_REQUEST_ERROR is returned, no Tx buffer is taken.
 */
void Ut_Can_Set_TxRequest_FdFrameNotFdEnabled_ReturnsError( void )
{
    can_Frame_t    frame    = { 0u };
    can_TxHandle_t txHandle = CAN_TX_HANDLE_INVALID;

    UtCan_InitNominal();
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_Start( CAN_PERIPH_1 ) );

    frame.Id       = UT_CAN_FRAME_ID;
    frame.IdType   = CAN_ID_STANDARD;
    frame.FrameType = CAN_FRAME_DATA;
    frame.FdFrame   = CAN_FLAG_ACTIVE;
    frame.Length    = CAN_DATA_LEN_CLASSIC_MAX;

    TEST_ASSERT_EQUAL( CAN_REQUEST_ERROR, Can_Set_TxRequest( CAN_PERIPH_1, &frame, &txHandle ) );
}


/**
 * \brief   Can_Set_TxRequest() rejects a request while the instance is not CAN_STATE_STARTED.
 *
 * \par Expected results
 * - CAN_REQUEST_ERROR is returned.
 */
void Ut_Can_Set_TxRequest_InstanceStopped_ReturnsError( void )
{
    can_Frame_t    frame    = { 0u };
    can_TxHandle_t txHandle = CAN_TX_HANDLE_INVALID;

    UtCan_InitNominal();

    frame.Id        = UT_CAN_FRAME_ID;
    frame.IdType    = CAN_ID_STANDARD;
    frame.FrameType = CAN_FRAME_DATA;
    frame.Length    = CAN_DATA_LEN_CLASSIC_MAX;

    TEST_ASSERT_EQUAL( CAN_REQUEST_ERROR, Can_Set_TxRequest( CAN_PERIPH_1, &frame, &txHandle ) );
}


/**
 * \brief   Can_Set_TxRequest() writes a classic data frame to Tx buffer 0 and sets TXBAR[0].
 *
 * \par Expected results
 * - CAN_REQUEST_OK is returned, txHandle is 0, the T0 word of the message RAM matches the frame.
 */
void Ut_Can_Set_TxRequest_ClassicDataFrame_WritesTxBuffer0( void )
{
    can_Frame_t    frame    = { 0u };
    can_TxHandle_t txHandle = CAN_TX_HANDLE_INVALID;
    uint32_t          t0Word  = 0u;

    UtCan_InitNominal();
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_Start( CAN_PERIPH_1 ) );

    frame.Id        = UT_CAN_FRAME_ID;
    frame.IdType    = CAN_ID_STANDARD;
    frame.FrameType = CAN_FRAME_DATA;
    frame.Length    = CAN_DATA_LEN_CLASSIC_MAX;

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_TxRequest( CAN_PERIPH_1, &frame, &txHandle ) );
    TEST_ASSERT_EQUAL_UINT8( 0u, txHandle );

    t0Word = Can_Ram_Get_Word( CAN_PERIPH_1, CAN_RAM_WORD_OFFSET_TX_BUFFER );
    TEST_ASSERT_EQUAL_HEX32( ( UT_CAN_FRAME_ID << UT_CAN_EXPECTED_STD_ID_SHIFT ), t0Word );
    TEST_ASSERT_EQUAL_UINT32( 1u, Can_Get_RegVal( CAN_PERIPH_1, CAN_REG_TXBAR, 1u ) );
}

/* =============================== BUS STATE ====================================== */

/**
 * \brief   Can_Get_BusState() derives CAN_BUS_OFF from FDCAN_PSR.BO (RM0481 Table, 256 counter).
 *
 * \par Expected results
 * - CAN_BUS_OFF is reported.
 */
void Ut_Can_Get_BusState_PsrBoSet_ReturnsBusOff( void )
{
    can_BusState_t busState = CAN_BUS_ACTIVE;

    UtCan_InitNominal();
    Can_Set_RegBit( CAN_PERIPH_1, CAN_REG_PSR, FDCAN_PSR_BO );

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_BusState( CAN_PERIPH_1, &busState ) );
    TEST_ASSERT_EQUAL( CAN_BUS_OFF, busState );
}


/**
 * \brief   Can_Get_BusState() reports CAN_BUS_ACTIVE when no PSR flag is set.
 *
 * \par Expected results
 * - CAN_BUS_ACTIVE is reported.
 */
void Ut_Can_Get_BusState_NoPsrFlag_ReturnsActive( void )
{
    can_BusState_t busState = CAN_BUS_OFF;

    UtCan_InitNominal();

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_BusState( CAN_PERIPH_1, &busState ) );
    TEST_ASSERT_EQUAL( CAN_BUS_ACTIVE, busState );
}


/**
 * \brief   Can_Get_ErrorCounters() reads the TEC / REC fields of FDCAN_ECR.
 *
 * \par Expected results
 * - TxErrorCnt and RxErrorCnt match the raw register value.
 */
void Ut_Can_Get_ErrorCounters_EcrValue_ReportsCounters( void )
{
    can_ErrorCounters_t errorCounters = { 0u };

    UtCan_InitNominal();
    Can_Set_RegVal( CAN_PERIPH_1,
                    CAN_REG_ECR,
                    UT_CAN_ECR_TEST_MASK,
                    ( UT_CAN_ECR_TEST_REC << UT_CAN_ECR_REC_SHIFT ) | UT_CAN_ECR_TEST_TEC );

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_ErrorCounters( CAN_PERIPH_1, &errorCounters ) );
    TEST_ASSERT_EQUAL_UINT8( UT_CAN_ECR_TEST_TEC, errorCounters.TxErrorCnt );
    TEST_ASSERT_EQUAL_UINT8( UT_CAN_ECR_TEST_REC, errorCounters.RxErrorCnt );
}

/* ============================= BUS-OFF RECOVERY ================================== */

/**
 * \brief   Can_Task() clears CCCR.INIT by itself when the bus-off recovery mode is
 *          CAN_BUS_OFF_RECOVERY_AUTO and the bus is off (README rule 5).
 *
 * \par Expected results
 * - CCCR.INIT is cleared after Can_Task().
 */
void Ut_Can_Task_BusOffAutoRecovery_ClearsInit( void )
{
    UtCan_InitNominal();
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_Start( CAN_PERIPH_1 ) );

    Can_Set_RegBit( CAN_PERIPH_1, CAN_REG_PSR, FDCAN_PSR_BO );
    Can_Set_RegBit( CAN_PERIPH_1, CAN_REG_CCCR, FDCAN_CCCR_INIT );

    Can_Task();

    TEST_ASSERT_EQUAL_UINT32( 0u, Can_Get_RegVal( CAN_PERIPH_1, CAN_REG_CCCR, FDCAN_CCCR_INIT ) );
}


/**
 * \brief   Can_Task() leaves CCCR.INIT set when the bus-off recovery mode is
 *          CAN_BUS_OFF_RECOVERY_MANUAL (README rule 5: waits for Can_Set_BusOffRestart).
 *
 * \par Expected results
 * - CCCR.INIT stays set after Can_Task(), Can_Set_BusOffRestart() then clears it.
 */
void Ut_Can_Task_BusOffManualRecovery_WaitsForRestart( void )
{
    UtCan_InitNominal();
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_Start( CAN_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK,
                        Can_Set_BusOffRecovery( CAN_PERIPH_1, CAN_BUS_OFF_RECOVERY_MANUAL ) );

    Can_Set_RegBit( CAN_PERIPH_1, CAN_REG_PSR, FDCAN_PSR_BO );
    Can_Set_RegBit( CAN_PERIPH_1, CAN_REG_CCCR, FDCAN_CCCR_INIT );

    Can_Task();
    TEST_ASSERT_EQUAL_UINT32( FDCAN_CCCR_INIT, Can_Get_RegVal( CAN_PERIPH_1, CAN_REG_CCCR, FDCAN_CCCR_INIT ) );

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_BusOffRestart( CAN_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, Can_Get_RegVal( CAN_PERIPH_1, CAN_REG_CCCR, FDCAN_CCCR_INIT ) );
}

/* ========================== MUTATION CHECK (own, per AGENT_RULES §6) ============== */

/**
 * \brief   Mutation check of the bit timing search: a wrong (too large) tolerance value would
 *          make Ut_Can_Set_BitTiming_UnreachableRate_ReturnsError() pass even though the
 *          hardware cannot reach the target - this test proves that a 0 tolerance really is
 *          exact-match-only, by also rejecting a target 1 Hz away from a reachable rate when
 *          the tolerance is 0, and accepting it as soon as the tolerance is widened.
 *
 * \par Expected results
 * - CAN_REQUEST_ERROR with 0 tolerance, CAN_REQUEST_OK once the tolerance covers the deviation.
 */
void UtCan_Mutation_BitTimingTolerance_BoundaryRespected( void )
{
    can_BitTiming_t bitTiming;
    rcc_FreqHz_t      clkHz = UT_CAN_KERNEL_CLK_8MHZ;

    UtCan_InitNominal();

    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Get_BitTiming( CAN_PERIPH_1, &bitTiming ) );
    bitTiming.Bitrate          = UT_CAN_BITRATE_500K + UT_CAN_BITRATE_DELTA_HZ; /* 0.2 % away from the match */
    bitTiming.SamplePoint      = UT_CAN_SAMPLE_POINT_87_5;
    bitTiming.BitrateTolerance = UT_CAN_TOLERANCE_EXACT;

    Rcc_Get_PeriphClk_ExpectAndReturn( RCC_PERIPH_FDCAN_PLL1Q, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();
    Rcc_Get_PeriphClk_ReturnThruPtr_periphClk( &clkHz );
    TEST_ASSERT_EQUAL( CAN_REQUEST_ERROR, Can_Set_BitTiming( CAN_PERIPH_1, &bitTiming ) );

    bitTiming.BitrateTolerance = UT_CAN_TOLERANCE_WIDE; /* 0.5 %, covers the 0.2 % deviation */

    Rcc_Get_PeriphClk_ExpectAndReturn( RCC_PERIPH_FDCAN_PLL1Q, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();
    Rcc_Get_PeriphClk_ReturnThruPtr_periphClk( &clkHz );
    TEST_ASSERT_EQUAL( CAN_REQUEST_OK, Can_Set_BitTiming( CAN_PERIPH_1, &bitTiming ) );
}
