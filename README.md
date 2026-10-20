# CAN Peripheral Driver

This module provides an abstraction layer for the **CAN controllers** of STM32 MCUs - the classic `bxCAN` (CAN 2.0A / B) and the
`FDCAN` (CAN 2.0 and **CAN FD** according to ISO 11898-1): the bit timing calculated from the bit rates, the operating modes,
the acceptance filters, the transmission and the reception of the frames, the bus state with the error counters and the
bus-off recovery, the low-power modes and the pins of the transceiver. It is a low-level MCAL module: it owns the CAN
hardware (and the message RAM), the application and the other modules never touch the registers.

The public interface (`Can_Port.h`, `Can_Types.h`) is **identical for all supported families**. The implementation
for a family lives in its own branch (`Dev/<Family>` for development, `Releases/<Family>` for the released versions), so
a module or an application written against the interface does not change when the MCU family changes.

This README is the landing page and the **specification of the interface** (master). The family branches contain their
own README with everything that is specific to the family: the instances of the device, their settings and the status of
the implementation.

> **Status: design.** No implementation exists yet, the interface below is a proposal. Stories and branches follow.

---

## Intent and purpose

- CAN is the bus of the vehicles, the drives and the machines. The protocols above it (CANopen, J1939, UDS / ISO-TP,
  DroneCAN, the proprietary ones) are **middlewares** that work with *frames*: an identifier, a length, the data. The module
  gives them one interface for every STM32 family - the same frame, the same filters, the same callbacks - whether the
  device has the classic bxCAN or the FDCAN.
- The controllers differ a lot inside: bxCAN has three mailboxes, two FIFOs of three frames and filter banks shared between the
  two controllers; FDCAN has a configurable **message RAM** with the filter lists, two receive FIFOs, a transmit FIFO / queue and a
  transmit event FIFO, the data up to 64 bytes and a second bit rate for the data phase. The module hides it: the application
  describes the frames and the filters, the module lays out the message RAM or the banks.
- The user thinks in **bit rate and sample point**, not in prescalers and segments: the module searches the prescaler and the
  time segments for the kernel clock, reports the achieved bit rate and sample point and rejects a configuration that the
  clock cannot make within the tolerance. CAN FD gets the second pair (the data phase) and the transmitter delay
  compensation.
- The bus state is a part of the interface: the error counters, the error warning / passive / bus-off states, the last error
  code and the **bus-off recovery** (automatic after the 128 x 11 recessive bits, or started by the application) are
  reported and controlled the same way on all families.
- The module hides the differences between the families: bxCAN against FDCAN, the number of instances, the message RAM,
  the kernel clock selection, the interrupt lines, the sleep and the wake-up differ, the interface stays the same.

What the module is **not**: it does not implement a protocol on the top of the frames (addressing, segmentation, the object
dictionary), it does not drive the transceiver beyond the optional standby pin and it does not guarantee the real-time
behaviour of the application that serves the callbacks. The time-triggered CAN (TTCAN) is not part of it.

---

## Concept

An instance is identified by `can_PeriphId_t`. The controller has three states (`can_State_t`):

| State | Meaning |
|---|---|
| `CAN_STATE_STOPPED` | The controller is configured but **not on the bus** (initialization mode). Everything can be configured. |
| `CAN_STATE_STARTED` | The controller takes part in the bus (it synchronizes to 11 recessive bits first). The configuration of the bit timing, the mode and the filters is frozen. |
| `CAN_STATE_SLEEP` | Power-down / sleep: the clock of the CAN core is stopped; a request or a bus activity brings it back (where the family allows the wake-up). |

The data model:

| Item | Meaning |
|---|---|
| **Frame** (`can_Frame_t`) | Identifier (11 or 29 bit, `CAN_ID_STANDARD` / `CAN_ID_EXTENDED`), data frame or remote frame, the length in bytes (0 ... 8, for CAN FD 0 ... 64 - the DLC coding is done by the module), the data, the FD flags (`FdFrame`, `BitRateSwitch`), `ErrorStateIndicator` (received), `Marker` (an application number that comes back with the transmit event) and the `Timestamp` (received). |
| **Bit timing** (`can_BitTiming_t`) | Nominal bit rate (bit/s) and sample point (0.1 %); for the FD frames the data bit rate and its sample point; the module returns the achieved values. |
| **Mode** (`can_Mode_t`) | `CAN_MODE_NORMAL`, `CAN_MODE_RESTRICTED` (receives and acknowledges, does not transmit), `CAN_MODE_MONITORING` (listen only, silent), `CAN_MODE_LOOPBACK_INTERNAL`, `CAN_MODE_LOOPBACK_EXTERNAL`. |
| **Filter** (`can_Filter_t`) | Type (`MASK`, `RANGE`, `DUAL_ID`), identifier type, two values, the action (accept to `CAN_RX_FIFO_0` / `CAN_RX_FIFO_1`, reject, mark with the high priority). The first matching filter wins. |
| **Global filter** | What happens with a frame that matches no filter (accept to a FIFO or reject, separately for standard and extended identifiers) and with the remote frames. |
| **Transmission** | `CAN_TX_FIFO` (the order of the requests) or `CAN_TX_QUEUE` (by the identifier priority); a request returns a handle, `Can_Set_TxAbort` aborts it, the transmit callback reports the result with the marker. |
| **Reception** | The FIFOs of the controller; the receive callback reports a new frame (or a threshold), `Can_Get_RxFrame` takes it out. |
| **Bus state** (`can_BusState_t`) | `CAN_BUS_ACTIVE`, `CAN_BUS_WARNING` (error counter 96), `CAN_BUS_PASSIVE` (128), `CAN_BUS_OFF` (256), and the counters TEC / REC. |

Rules:

1. `Can_Init` configures the instance (the clock, the bit timing, the mode, the message RAM, the filters, the pins, the interrupts)
   and leaves it `STOPPED`; `Can_Set_Start` brings it onto the bus, `Can_Set_Stop` takes it out (pending transmissions are aborted).
2. The bit timing, the mode, the filters, the FD enable and the message RAM layout are changed only in the `STOPPED` state; otherwise
   the call returns an error and nothing changes.
3. The frame is checked before it is written: the identifier range, the length for the frame type, the FD flags on a
   controller or a configuration without the FD (`CAN_REQUEST_ERROR`), a full transmit FIFO (`CAN_REQUEST_ERROR` and
   the state `CAN_XFER_TX_FULL`, the frame is not queued).
4. The callbacks are called from the interrupt of the instance: transmitted (with the marker), received (FIFO and the number of
   frames), FIFO overflow (frames lost), bus state change, error (the last error code), wake-up. The flags are cleared by the module.
5. After the `BUS_OFF` the controller is silent. With `CAN_BUS_OFF_RECOVERY_AUTO` it recovers by itself after the bus-off
   sequence; with `CAN_BUS_OFF_RECOVERY_MANUAL` it waits for `Can_Set_BusOffRestart`. The state change is reported by the callback.
6. Several instances of the device are independent; the filter banks / the message RAM shared between them are managed by
   the module (an instance cannot take the resources of another one).

---

## Supported families

Every family gets its own story, its own tasks and its own branches `Dev/<Family>` and `Releases/<Family>` cut from this
master.

| Family | Branch | Controller | Status |
|---|---|---|---|
| STM32H5 | `Dev/STM32H5` | `FDCAN1` (STM32H503), `FDCAN1`, `FDCAN2` (most lines), `FDCAN1` ... `FDCAN3` (H5E / H5F lines): CAN FD with 64-byte payloads, two receive FIFOs, transmit FIFO / queue, transmit event FIFO, the kernel clock selected in the RCC | planned (first) |
| STM32G4 | `Dev/STM32G4` | `FDCAN1` ... `FDCAN3` | planned |
| STM32H7 | `Dev/STM32H7` | `FDCAN1` ... `FDCAN3` (shared message RAM) | planned |
| STM32U5 | `Dev/STM32U5` | `FDCAN1` | planned |
| STM32L4 | `Dev/STM32L4` | `CAN1` (bxCAN) | planned |
| STM32F4, STM32F7 | `Dev/STM32F4`, `Dev/STM32F7` | `CAN1`, `CAN2`, and `CAN3` where the device has it (bxCAN, the second controller uses the filter banks of the first) | planned |
| STM32U3 | `Dev/STM32U3` | device dependent | planned |

A function without a meaning in a family stays in the interface, returns `CAN_REQUEST_ERROR` and suppresses the unused
parameters (for example the CAN FD frames and the data bit rate on bxCAN, the restricted mode where the hardware does not have it,
the transmit event FIFO of bxCAN).

---

## Public interface in short (proposal)

| Group | Functions |
|---|---|
| Module management | `Can_Get_ModuleVersion`, `Can_Init`, `Can_Deinit`, `Can_Task`, `Can_Get_DefaultConfig` |
| Control | `Can_Set_Start`, `Can_Set_Stop`, `Can_Get_State`, `Can_Set_PowerDown`, `Can_Set_PowerUp` |
| Bit timing and mode | `Can_Set_BitTiming` / `Can_Get_BitTiming`, `Can_Set_Mode` / `Get`, `Can_Set_AutoRetransmission` / `Get`, `Can_Get_ClkFrequency` |
| Filters | `Can_Set_Filters`, `Can_Set_GlobalFilter` |
| Transmission | `Can_Set_TxRequest`, `Can_Set_TxAbort`, `Can_Get_TxState`, `Can_Get_TxFreeCount` |
| Reception | `Can_Get_RxCount`, `Can_Get_RxFrame` |
| Bus state | `Can_Get_BusState`, `Can_Get_ErrorCounters`, `Can_Get_LastError`, `Can_Set_BusOffRecovery`, `Can_Set_BusOffRestart` |
| Transceiver | `Can_Set_TransceiverState` |
| Interrupts | `Can_Set_IrqPriority` / `Get`, the callbacks are a part of the configuration |
| Pins | `Can_InitTxGpio`, `Can_InitRxGpio`, `Can_InitStandbyGpio` |

```c
can_ModuleVersion_t Can_Get_ModuleVersion  ( void );

can_RequestState_t  Can_Init               ( const can_Config_t * const canConfig );
can_RequestState_t  Can_Deinit             ( can_PeriphId_t periphId );
void                Can_Task               ( void );
can_RequestState_t  Can_Get_DefaultConfig  ( can_Config_t * const canConfig );

can_RequestState_t  Can_Set_Start          ( can_PeriphId_t periphId );
can_RequestState_t  Can_Set_Stop           ( can_PeriphId_t periphId );
can_RequestState_t  Can_Get_State          ( can_PeriphId_t periphId, can_State_t * const canState );

can_RequestState_t  Can_Set_BitTiming      ( can_PeriphId_t periphId, const can_BitTiming_t * const bitTiming );
can_RequestState_t  Can_Get_BitTiming      ( can_PeriphId_t periphId, can_BitTiming_t * const bitTiming );
can_RequestState_t  Can_Set_Mode           ( can_PeriphId_t periphId, can_Mode_t mode );

can_RequestState_t  Can_Set_Filters        ( can_PeriphId_t periphId, const can_Filter_t * const filters, can_FilterCnt_t filterCnt );
can_RequestState_t  Can_Set_GlobalFilter   ( can_PeriphId_t periphId, const can_GlobalFilter_t * const globalFilter );

can_RequestState_t  Can_Set_TxRequest      ( can_PeriphId_t periphId, const can_Frame_t * const frame, can_TxHandle_t * const txHandle );
can_RequestState_t  Can_Set_TxAbort        ( can_PeriphId_t periphId, can_TxHandle_t txHandle );
can_RequestState_t  Can_Get_TxFreeCount    ( can_PeriphId_t periphId, can_FrameCnt_t * const freeCnt );

can_RequestState_t  Can_Get_RxCount        ( can_PeriphId_t periphId, can_RxFifoId_t fifoId, can_FrameCnt_t * const rxCnt );
can_RequestState_t  Can_Get_RxFrame        ( can_PeriphId_t periphId, can_RxFifoId_t fifoId, can_Frame_t * const frame );

can_RequestState_t  Can_Get_BusState       ( can_PeriphId_t periphId, can_BusState_t * const busState );
can_RequestState_t  Can_Get_ErrorCounters  ( can_PeriphId_t periphId, can_ErrorCounters_t * const errorCounters );
can_RequestState_t  Can_Set_BusOffRestart  ( can_PeriphId_t periphId );
```

Every function returns `CAN_REQUEST_OK` / `CAN_REQUEST_ERROR` and checks its parameters and the state of the module
before it touches a register.

| Type | Meaning |
|---|---|
| `can_Config_t` | Instance, kernel clock source, bit timing, mode, FD enable, the transmit mode (FIFO / queue), the message RAM / buffer sizes, filters, global filter, bus-off recovery, pins, callbacks, interrupt priority |
| `can_Frame_t` | Identifier, identifier type, frame type, FD flags, length, data (64 bytes), marker, timestamp, error state indicator |
| `can_Filter_t` | Type, identifier type, `Id1`, `Id2` / mask, action |
| `can_ErrorCounters_t` | Transmit and receive error counters |
| `can_LastError_t` | `CAN_ERROR_NONE`, `STUFF`, `FORM`, `ACK`, `BIT1`, `BIT0`, `CRC`, `NO_CHANGE` |
| Callbacks | `TxCallback( periphId, marker, result )`, `RxCallback( periphId, fifoId, frameCnt )`, `RxOverflowCallback`, `BusStateCallback`, `ErrorCallback`, `WakeUpCallback` |

---

## Usage

### 500 kbit/s CAN with two filters and the receive callback

```c
#include "Can_Port.h"

static void App_CanRx( can_PeriphId_t periphId, can_RxFifoId_t fifoId, can_FrameCnt_t frameCnt )
{
    can_Frame_t        frame    = { 0u };
    can_RequestState_t reqState = CAN_REQUEST_ERROR;

    for( can_FrameCnt_t frameIdx = 0u; frameCnt > frameIdx; frameIdx++ )
    {
        reqState = Can_Get_RxFrame( periphId, fifoId, &frame );

        if( CAN_REQUEST_OK == reqState )
        {
            /* hand the frame to the protocol layer */
        }
        else
        {
            /* The FIFO is empty */
        }
    }
}

can_Config_t canConfig;

(void)Can_Get_DefaultConfig( &canConfig );

canConfig.PeriphId              = CAN_PERIPH_1;
canConfig.BitTiming.Bitrate     = 500000u;
canConfig.BitTiming.SamplePoint = 875u;                      /* 87.5 % */
canConfig.RxCallback            = App_CanRx;
canConfig.TxPin                 = CAN_TX_PIN_CAN1_PA11;
canConfig.RxPin                 = CAN_RX_PIN_CAN1_PA10;

(void)Can_Init( &canConfig );

const can_Filter_t filters[] =
{
    { .Type = CAN_FILTER_DUAL_ID, .IdType = CAN_ID_STANDARD, .Id1 = 0x100u, .Id2 = 0x101u, .Action = CAN_FILTER_TO_RX_FIFO_0 },
    { .Type = CAN_FILTER_MASK,    .IdType = CAN_ID_EXTENDED, .Id1 = 0x18FF0000u, .Id2 = 0x1FFF0000u, .Action = CAN_FILTER_TO_RX_FIFO_1 },
};

(void)Can_Set_Filters( CAN_PERIPH_1, filters, 2u );
(void)Can_Set_Start( CAN_PERIPH_1 );
```

### CAN FD frame with the bit rate switch

```c
canConfig.FdEnable                  = CAN_FUNCTION_ACTIVE;
canConfig.BitTiming.DataBitrate     = 2000000u;
canConfig.BitTiming.DataSamplePoint = 750u;

can_Frame_t frame = { .Id = 0x123u, .IdType = CAN_ID_STANDARD, .FrameType = CAN_FRAME_DATA,
                      .FdFrame = CAN_FLAG_ACTIVE, .BitRateSwitch = CAN_FLAG_ACTIVE, .Length = 24u, .Marker = 7u };
/* ... fill frame.Data ... */
(void)Can_Set_TxRequest( CAN_PERIPH_1, &frame, &txHandle );     /* the TX callback reports the marker 7 */
```

### Bus-off handling

```c
canConfig.BusOffRecovery = CAN_BUS_OFF_RECOVERY_MANUAL;
canConfig.BusStateCallback = App_BusState;                       /* on CAN_BUS_OFF: wait, then: */
(void)Can_Set_BusOffRestart( CAN_PERIPH_1 );
```

---

## Behaviour and limits

- **Bit timing.** The result depends on the kernel clock: an exact bit rate and a sample point close to the required one is
  searched; the tolerance (`BitrateTolerance`) is in the configuration. For CAN FD the data phase uses its own
  segments and the transmitter delay compensation, which the module enables above the bit rate where the hardware requires it.
- **Start.** `Can_Set_Start` returns after the controller left the initialization mode; being on the bus needs 11 recessive
  bits and, on a bus without another node, there is nobody to acknowledge - the transmission then repeats according to the
  auto-retransmission setting.
- **FIFO overflow.** A receive FIFO that is not read in time loses frames (the overflow callback or the lost-frame flag); the
  module does not buffer them in software beyond the FIFOs of the hardware.
- **Callbacks** run in the interrupt; read the frames out and hand them to a task, do not process the protocol there.
- **Filters.** The capacity of the lists is limited by the family (bxCAN: the banks, shared with the second controller;
  FDCAN: the message RAM elements). A list that does not fit is rejected as a whole; nothing is half-applied.
- **Low power.** The sleep / power-down and the wake-up on the bus activity differ in the families (and the wake-up from
  Stop may need the pin on the EXTI); the README of the family describes what works.
- **Not part of the first version:** the time-triggered operation, the ISO-TP and other transport protocols, the
  sleep-aware partial networking, the DMA of the frames (the message RAM is the buffer), the arbitrary message RAM
  layout beyond the sizes in the configuration.

---

## Dependencies

- **Rcc** - peripheral clock, kernel clock source selection and frequency, reset
- **Nvic** - interrupt lines of the instance
- **Gpio** - TX, RX and the optional transceiver standby pin
- **RAL (Register Abstraction Layer)** - used internally to access the registers

All mandatory RCC, NVIC and GPIO configurations are handled internally.

---

## Testing

- **Unit tests** (host, Unity + CMock + RegMem): the bit timing search for the kernel clocks and the bit rates (the exact
  ones, the ones within the tolerance, the unreachable ones), the DLC coding for the lengths up to 64 bytes, the validation
  of the frames and of the filters, the layout of the message RAM / the banks, the state rules (changes only in `STOPPED`), the
  bus-off recovery modes, the receive and the transmit flow with the callbacks.
- **Integration tests** (target, probe-rs): the internal and the external loop-back of every frame type and length, two
  instances of one device (or two boards) exchanging frames at several bit rates, the filters against a generated
  traffic, the bus-off after a shorted bus and the recovery, the CAN FD data phase.

---

## Interface conventions

The module follows the common rules of the MCAL layer, so that it behaves like the other modules:

- **Lists instead of free members.** Everything that the hardware restricts (the pins, the kernel clock sources, the DMA
  channels, the triggers and the other selectable sources) is a `typedef enum` list with exactly the items that are valid
  for the device line; a pin is an item `CAN_<SIGNAL>_PIN_<INSTANCE>_P<port><pin>` and there is an `*_UNUSED` item. A choice
  that is not in the list cannot be written.
- **Own peripheral only.** The module calls only the registers / LL functions of its own peripheral. The clocks, the
  interrupts, the pins, the DMA and the EXTI lines go through the API of the Rcc, Nvic, Gpio, Gpdma and Exti modules; a
  missing function is added to that module, it is not bypassed.
- **Init verifies itself.** `Can_Init` checks the result at the end; when it is not as required, the module is
  deinitialized and an error is returned. The waiting for the hardware is a loop on the register state with a timeout,
  never a blind delay.
- **Transfer modes.** Where the module moves data, the POLL (served by `Can_Task`), ISR and DMA modes have the same
  configuration structure, the same request and the same callbacks (error, half transfer, transfer complete).
- **Flags and states are enums of the module** (`can_FunctionState_t`, `can_FlagState_t`), the limits and the constants are
  named, there are no magic numbers in the interface.
- **Unsupported functions stay.** A function without a meaning in a family keeps its signature, suppresses the unused
  parameters and returns `CAN_REQUEST_ERROR`.
- **Documentation.** Every function and type has a Doxygen description, the main header has the `\defgroup`.

---

## Repository layout and branches

The master holds this README and the license. The family branches (`Dev/<Family>`, `Releases/<Family>`) hold:

| Path | Content |
|---|---|
| `README.md` | README of the family: the instances of the device, specifics, errata, status |
| `Can_Port.h`, `Can_Types.h` | Public interface (identical in all families) |
| `Can*.c`, `Can.h` | Implementation of the family |
| `CMakeLists.txt` | Library `Can_Lib` of the EmBi platform |
| `Tests/UnitTests` | Unit tests (Unity + CMock + RegMem) |
| `Tests/IntegrationTests` | Integration tests on the target (probe-rs) |

A change of the public interface is made in all families.

---

## CMake integration

```cmake
target_link_libraries( User_Lib PRIVATE Can_Lib )
```

1. Link `Can_Lib` to your CMake library.
2. Include `Can_Port.h` in your project.
3. Configure the module as needed for your hardware.

---

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to use, modify, and share this work for **non-commercial purposes**, provided appropriate credit is given.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** - [embedbits.com](https://embedbits.com)

Contributions are welcome! Please open a pull request.
