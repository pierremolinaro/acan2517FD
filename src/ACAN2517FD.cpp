//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// A CAN driver for MCP2517FD, CANFD mode
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2517FD
//
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#include <ACAN2517FD.h>

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// Note about ESP32
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

// It appears that Arduino ESP32 interrupts are managed in a completely different way from "usual" Arduino:
//   - SPI.usingInterrupt is not implemented;
//   - noInterrupts() and interrupts() are NOPs;
//   - interrupt service routines should be fast, otherwise you get an "Guru Meditation Error: Core 1 panic'ed
//     (Interrupt wdt timeout on CPU1)".

// So we handle the ESP32 interrupt in the following way:
//   - interrupt service routine performs a xSemaphoreGive on mISRSemaphore of can driver
//   - this activates the myESP32Task task that performs "isr_core" that is done by interrupt service routine
//     in "usual" Arduino;
//   - as this task runs in parallel with setup / loop routines, SPI access is protected by a mutual access exclusion
//     semaphore named gMutualExclusion.

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#ifdef ARDUINO_ARCH_ESP32
  static SemaphoreHandle_t gMutualExclusion = xSemaphoreCreateCounting (1, 1) ;
#endif

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#ifdef ARDUINO_ARCH_ESP32
  static void myESP32Task (void * pData) {
    ACAN2517FD * canDriver = (ACAN2517FD *) pData ;
    while (1) {
      xSemaphoreTake (canDriver->mISRSemaphore, portMAX_DELAY) ;
      bool loop = true ;
      while (loop) {
        xSemaphoreTake (gMutualExclusion, portMAX_DELAY) ;
          loop = canDriver->isr_core () ;
        xSemaphoreGive (gMutualExclusion) ;
	    }
    }
  }
#endif

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// ACAN2517FD register addresses
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

static const uint16_t C1CON_REGISTER      = 0x000 ;
static const uint16_t C1NBTCFG_REGISTER   = 0x004 ;
static const uint16_t C1DBTCFG_REGISTER   = 0x008 ;
static const uint16_t C1TDC_REGISTER      = 0x00C ;

static const uint16_t C1TREC_REGISTER     = 0x034 ;
static const uint16_t C1BDIAG0_REGISTER   = 0x038 ;
static const uint16_t C1BDIAG1_REGISTER   = 0x03C ;

//······················································································································
//   TXQ REGISTERS
//······················································································································

static const uint16_t C1TXQCON_REGISTER   = 0x050 ;
static const uint16_t C1TXQSTA_REGISTER   = 0x054 ;
static const uint16_t C1TXQUA_REGISTER    = 0x058 ;

//······················································································································
//   INTERRUPT REGISTERS
//······················································································································

static const uint16_t C1INT_REGISTER = 0x01C ;

//······················································································································
//   FIFO REGISTERS
//······················································································································

static uint16_t C1FIFOCON_REGISTER (const uint16_t inFIFOIndex) { // 1 ... 31
  return 0x05C + 12 * (inFIFOIndex - 1) ;
}

//······················································································································

static uint16_t C1FIFOSTA_REGISTER (const uint16_t inFIFOIndex) { // 1 ... 31
  return 0x060 + 12 * (inFIFOIndex - 1) ;
}

//······················································································································

static uint16_t C1FIFOUA_REGISTER (const uint16_t inFIFOIndex) { // 1 ... 31
  return 0x064 + 12 * (inFIFOIndex - 1) ;
}

//······················································································································
//   FILTER REGISTERS
//······················································································································

static uint16_t C1FLTCON_REGISTER (const uint16_t inFilterIndex) { // 0 ... 31 (DS20005688B, page 58)
  return 0x1D0 + inFilterIndex ;
}

//······················································································································

static uint16_t C1FLTOBJ_REGISTER (const uint16_t inFilterIndex) { // 0 ... 31 (DS20005688B, page 60)
  return 0x1F0 + 8 * inFilterIndex ;
}

//······················································································································

static uint16_t C1MASK_REGISTER (const uint16_t inFilterIndex) { // 0 ... 31 (DS20005688B, page 61)
  return 0x1F4 + 8 * inFilterIndex ;
}

//······················································································································
//   OSCILLATOR REGISTER
//······················································································································

static const uint16_t OSC_REGISTER   = 0xE00 ;

//······················································································································
//   INPUT / OUPUT CONTROL REGISTER
//······················································································································

static const uint16_t IOCON_REGISTER = 0xE04 ;

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//    RECEIVE FIFO INDEX
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

static const uint8_t receiveFIFOIndex = 1 ;

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

ACAN2517FD::ACAN2517FD (const uint8_t inCS, // CS input of MCP2517FD
                        SPIClass & inSPI, // Hardware SPI object
                        const uint8_t inINT) : // INT output of MCP2517FD
mSPISettings (),
mSPI (inSPI),
mCS (inCS),
mINT (inINT),
mUsesTXQ (false),
mControllerTxFIFOFull (false),
mHasDataBitRate (false),
mTransmitFIFOPayload (0),
mTXQBufferPayload (0),
mReceiveFIFOPayload (0),
mDriverReceiveBuffer (),
mDriverTransmitBuffer ()
#ifdef ARDUINO_ARCH_ESP32
  , mISRSemaphore (xSemaphoreCreateCounting (10, 0))
#endif
{
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

uint32_t ACAN2517FD::begin (const ACAN2517FDSettings & inSettings,
                            void (* inInterruptServiceRoutine) (void)) {
//--- Add pass-all filter
  ACAN2517FDFilters filters ;
  filters.appendPassAllFilter (NULL) ;
//---
  return begin (inSettings, inInterruptServiceRoutine, filters) ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

uint32_t ACAN2517FD::begin (const ACAN2517FDSettings & inSettings,
                            void (* inInterruptServiceRoutine) (void),
                            const ACAN2517FDFilters & inFilters) {
  uint32_t errorCode = 0 ; // Means no error
//----------------------------------- If ok, check if settings are correct
  if (!inSettings.mArbitrationBitRateClosedToDesiredRate) {
    errorCode |= kTooFarFromDesiredBitRate ;
  }
  if (inSettings.CANBitSettingConsistency () != 0) {
    errorCode |= kInconsistentBitRateSettings ;
  }
//----------------------------------- Check mINT has interrupt capability
  const int8_t itPin = digitalPinToInterrupt (mINT) ;
  if ((mINT != 255) && (itPin == NOT_AN_INTERRUPT)) {
    errorCode = kINTPinIsNotAnInterrupt ;
  }
//----------------------------------- Check interrupt service routine is not null
  if ((mINT != 255) && (inInterruptServiceRoutine == NULL)) {
    errorCode |= kISRIsNull ;
  }
//----------------------------------- Check consistency between ISR and INT pin
  if ((mINT == 255) && (inInterruptServiceRoutine != NULL)) {
    errorCode |= kISRNotNullAndNoIntPin ;
  }
//----------------------------------- Check TXQ size is <= 32
  if (inSettings.mControllerTXQSize > 32) {
    errorCode |= kControllerTXQSizeGreaterThan32 ;
  }
//----------------------------------- Check TXQ priority is <= 31
  if (inSettings.mControllerTXQBufferPriority > 31) {
    errorCode |= kControllerTXQPriorityGreaterThan31 ;
  }
//----------------------------------- Check controller receive FIFO size is 1 ... 32
  if (inSettings.mControllerReceiveFIFOSize == 0) {
    errorCode |= kControllerReceiveFIFOSizeIsZero ;
  }else if (inSettings.mControllerReceiveFIFOSize > 32) {
    errorCode |= kControllerReceiveFIFOSizeGreaterThan32 ;
  }
//----------------------------------- Check controller transmit FIFO size is 1 ... 32
  if (inSettings.mControllerTransmitFIFOSize == 0) {
    errorCode |= kControllerTransmitFIFOSizeIsZero ;
  }else if (inSettings.mControllerTransmitFIFOSize > 32) {
    errorCode |= kControllerTransmitFIFOSizeGreaterThan32 ;
  }
//----------------------------------- Check Transmit FIFO priority is <= 31
  if (inSettings.mControllerTransmitFIFOPriority > 31) {
    errorCode |= kControllerTransmitFIFOPriorityGreaterThan31 ;
  }
//----------------------------------- Check MCP2517FD controller RAM usage is <= 2048 bytes
  if (inSettings.ramUsage () > 2048) {
    errorCode |= kControllerRamUsageGreaterThan2048 ;
  }
//----------------------------------- Check Filter definition
  if (inFilters.filterCount () > 32) {
    errorCode |= kMoreThan32Filters ;
  }
  if (inFilters.filterStatus () != ACAN2517FDFilters::kFiltersOk) {
    errorCode |= kFilterDefinitionError ;
  }
//----------------------------------- CS pin
  if (errorCode == 0) {
    pinMode (mINT, INPUT_PULLUP) ;
    pinMode (mCS, OUTPUT) ;
    deassertCS () ;
  //----------------------------------- Set SPI clock to 1 MHz
    mSPISettings = SPISettings (1 * 1000 * 1000, MSBFIRST, SPI_MODE0) ;
  //----------------------------------- Request configuration
    writeByteRegister (C1CON_REGISTER + 3, 0x04 | (1 << 3)) ; // Request configuration mode, abort all transmissions
  //----------------------------------- Wait (2 ms max) until requested mode is reached
    bool wait = true ;
    const uint32_t deadline = millis () + 2 ;
    while (wait) {
      const uint32_t actualMode = (readByteRegister (C1CON_REGISTER + 2) >> 5) & 0x07 ;
      wait = actualMode != 0x04 ;
      if (wait && (millis () >= deadline)) {
        errorCode |= kRequestedConfigurationModeTimeOut ;
        wait = false ;
      }
    }
  //----------------------------------- Reset MCP2517FD (allways use a 1 MHz clock)
    reset2517FD () ;
  }
//----------------------------------- Check SPI connection is on (with a 1 MHz clock)
// We write and the read back 2517 RAM at address 0x400
  for (uint32_t i=1 ; (i != 0) && (errorCode == 0) ; i <<= 1) {
    writeRegister (0x400, i) ;
    const uint32_t readBackValue = readRegister (0x400) ;
    if (readBackValue != i) {
      errorCode = kReadBackErrorWith1MHzSPIClock ;
    }
  }
//----------------------------------- Now, set internal clock with OSC register
//     Bit 0: (rw) 1 --> 10xPLL
//     Bit 4: (rw) 0 --> SCLK is divided by 1, 1 --> SCLK is divided by 2
//     Bits 5-6: Clovk Output Divisor
  if (errorCode == 0) {
    uint8_t pll = 0 ; // No PLL
    uint8_t osc = 0 ; // Divide by 1
    switch (inSettings.oscillator ()) {
    case ACAN2517FDSettings::OSC_4MHz:
    case ACAN2517FDSettings::OSC_20MHz:
    case ACAN2517FDSettings::OSC_40MHz:
      break ;
    case ACAN2517FDSettings::OSC_4MHz_DIVIDED_BY_2:
    case ACAN2517FDSettings::OSC_20MHz_DIVIDED_BY_2:
    case ACAN2517FDSettings::OSC_40MHz_DIVIDED_BY_2:
      osc =  1 << 4 ; // Divide by 2
      break ;
    case ACAN2517FDSettings::OSC_4MHz10xPLL_DIVIDED_BY_2 :
      pll = 1 ; // Enable 10x PLL
      osc =  1 << 4 ; // Divide by 2
      break ;
    case ACAN2517FDSettings::OSC_4MHz10xPLL :
      pll = 1 ; // Enable 10x PLL
      break ;
    }
    osc |= pll ;
    if (inSettings.mCLKOPin != ACAN2517FDSettings::SOF) {
      osc |= ((uint8_t) inSettings.mCLKOPin) << 5 ;
    }
    writeByteRegister (OSC_REGISTER, osc) ; // DS20005688B, page 16
  //--- Wait for PLL is ready (wait max 2 ms)
    if (pll != 0) {
      bool wait = true ;
      const uint32_t deadline = millis () + 2 ;
      while (wait) {
        wait = (readByteRegister (OSC_REGISTER + 1) & 0x4) == 0 ;  // DS20005688B, page 16
        if (wait && (millis () >= deadline)) {
          errorCode = kX10PLLNotReadyWithin1MS ;
          wait = false ;
        }
      }
    }
  }
//----------------------------------- Set full speed clock
  mSPISettings = SPISettings (inSettings.sysClock () / 2, MSBFIRST, SPI_MODE0) ;
//----------------------------------- Checking SPI connection is on (with a full speed clock)
//    We write and the read back 2517 RAM at address 0x400
  for (uint32_t i=1 ; (i != 0) && (errorCode == 0) ; i <<= 1) {
    writeRegister (0x400, i) ;
    const uint32_t readBackValue = readRegister (0x400) ;
    if (readBackValue != i) {
      errorCode = kReadBackErrorWithFullSpeedSPIClock ;
    }
  }
//----------------------------------- Install interrupt, configure external interrupt
  if (errorCode == 0) {
  //----------------------------------- Configure transmit and receive buffers
    mDriverTransmitBuffer.initWithSize (inSettings.mDriverTransmitFIFOSize) ;
    mDriverReceiveBuffer.initWithSize (inSettings.mDriverReceiveFIFOSize) ;
  //----------------------------------- Reset RAM
    for (uint16_t address = 0x400 ; address < 0xC00 ; address += 4) {
      writeRegister (address, 0) ;
    }
  //----------------------------------- Configure CLKO pin
    uint8_t d = 0x03 ; // Respect PM1-PM0 default values
    if (inSettings.mCLKOPin == ACAN2517FDSettings::SOF) {
      d |= 1 << 5 ; // SOF
    }
    if (inSettings.mTXCANIsOpenDrain) {
      d |= 1 << 4 ; // TXCANOD
    }
    writeByteRegister (IOCON_REGISTER + 3, d); // DS20005688B, page 24
  //----------------------------------- Configure ISO CRC Enable bit
    d = 1 << 6 ; // PXEDIS <-- 1
    if (inSettings.mISOCRCEnabled) {
      d |= 1 << 5 ; //  Enable ISO CRC in CAN FD Frames bit
    }
    writeByteRegister (IOCON_REGISTER, d); // DS20005688B, page 24
  //----------------------------------- Configure C1DTC (DS20005688B, page 29)
//     data = 1 << 25 ; // Enable Edge Filtering during Bus Integration state bit
//     data |= 1 << 17 ; // Auto TDC
//     writeRegister (C1TDC_REGISTER, data);
  //----------------------------------- Configure TXQ
    d = inSettings.mControllerTXQBufferRetransmissionAttempts ;
    d <<= 5 ;
    d |= inSettings.mControllerTXQBufferPriority ;
    writeByteRegister (C1TXQCON_REGISTER + 2, d); // DS20005688B, page 48
  // Bit 5-7: Payload Size bits
  // Bit 4-0: TXQ size
    mUsesTXQ = inSettings.mControllerTXQSize > 0 ;
    d = inSettings.mControllerTXQSize - 1 ;
    d |= inSettings.mControllerTXQBufferPayload << 5 ; // Payload
    writeByteRegister (C1TXQCON_REGISTER + 3, d); // DS20005688B, page 48
    mTXQBufferPayload = ACAN2517FDSettings::objectSizeForPayload (inSettings.mControllerTXQBufferPayload) ;
  //----------------------------------- Configure TXQ and TEF
  // Bit 4: Enable Transmit Queue bit ---> 1: Enable TXQ and reserves space in RAM
  // Bit 3: Store in Transmit Event FIFO bit ---> 0: Don’t save transmitted messages in TEF
    d = mUsesTXQ ? 0x04 : 0x00 ;
    writeByteRegister (C1CON_REGISTER + 2, d); // DS20005688B, page 24
  //----------------------------------- Configure RX FIFO (C1FIFOCON, DS20005688B, page 52)
    d = inSettings.mControllerReceiveFIFOSize - 1 ; // Set receive FIFO size
    d |= inSettings.mControllerReceiveFIFOPayload << 5 ; // Payload
    writeByteRegister (C1FIFOCON_REGISTER (1) + 3, d) ;
    d = 1 ; // Interrupt Enabled for FIFO not Empty (TFNRFNIE)
    writeByteRegister (C1FIFOCON_REGISTER (1), d) ;
    mReceiveFIFOPayload = ACAN2517FDSettings::objectSizeForPayload (inSettings.mControllerReceiveFIFOPayload) ;
  //----------------------------------- Configure TX FIFO (C1FIFOCON, DS20005688B, page 52)
    d = inSettings.mControllerTransmitFIFORetransmissionAttempts ;
    d <<= 5 ;
    d |= inSettings.mControllerTransmitFIFOPriority ;
    writeByteRegister (C1FIFOCON_REGISTER (2) + 2, d) ;
    d = inSettings.mControllerTransmitFIFOSize - 1 ; // Set transmit FIFO size
    d |= inSettings.mControllerTransmitFIFOPayload << 5 ; // Payload
    writeByteRegister (C1FIFOCON_REGISTER (2) + 3, d) ;
    d = 1 << 7 ; // FIFO 2 is a Tx FIFO
    writeByteRegister (C1FIFOCON_REGISTER (2), d) ;
    mTransmitFIFOPayload = ACAN2517FDSettings::objectSizeForPayload (inSettings.mControllerTransmitFIFOPayload) ;
  //----------------------------------- Configure receive filters
    uint8_t filterIndex = 0 ;
    ACAN2517FDFilters::Filter * filter = inFilters.mFirstFilter ;
    mCallBackFunctionArray = new ACANFDCallBackRoutine [inFilters.filterCount ()] ;
    while (NULL != filter) {
      mCallBackFunctionArray [filterIndex] = filter->mCallBackRoutine ;
      writeRegister (C1MASK_REGISTER (filterIndex), filter->mFilterMask) ; // DS20005688B, page 61
      writeRegister (C1FLTOBJ_REGISTER (filterIndex), filter->mAcceptanceFilter) ; // DS20005688B, page 60
      d = 1 << 7 ; // Filter is enabled
      d |= 1 ; // Message matching filter is stored in FIFO1
      writeByteRegister (C1FLTCON_REGISTER (filterIndex), d) ; // DS20005688B, page 58
      filter = filter->mNextFilter ;
      filterIndex += 1 ;
    }
  //----------------------------------- Activate interrupts (C1INT, DS20005688B page 34)
    d  = (1 << 1) ; // Receive FIFO Interrupt Enable
    d |= (1 << 0) ; // Transmit FIFO Interrupt Enable
    writeByteRegister (C1INT_REGISTER + 2, d) ;
    writeByteRegister (C1INT_REGISTER + 3, 0) ;
  //----------------------------------- Program nominal bit rate (C1NBTCFG register)
  //  bits 31-24: BRP - 1
  //  bits 23-16: TSEG1 - 1
  //  bit 15: unused
  //  bits 14-8: TSEG2 - 1
  //  bit 7: unused
  //  bits 6-0: SJW - 1
    uint32_t data = inSettings.mBitRatePrescaler - 1 ;
    data <<= 8 ;
    data |= inSettings.mArbitrationPhaseSegment1 - 1 ;
    data <<= 8 ;
    data |= inSettings.mArbitrationPhaseSegment2 - 1 ;
    data <<= 8 ;
    data |= inSettings.mArbitrationSJW - 1 ;
    writeRegister (C1NBTCFG_REGISTER, data);
  //----------------------------------- Program data bit rate (C1DBTCFG register)
  //  bits 31-24: BRP - 1
  //  bits 23-21: unused
  //  bits 20-16: TSEG1 - 1
  //  bits 15-12: unused
  //  bits 11-8: TSEG2 - 1
  //  bits 7-4: unused
  //  bits 3-0: SJW - 1
    mHasDataBitRate = inSettings.mDataBitRateFactor != 1 ;
    if (mHasDataBitRate) {
      data = inSettings.mBitRatePrescaler - 1 ;
      data <<= 8 ;
      data |= inSettings.mDataPhaseSegment1 - 1 ;
      data <<= 8 ;
      data |= inSettings.mDataPhaseSegment2 - 1 ;
      data <<= 8 ;
      data |= inSettings.mDataSJW - 1 ;
      writeRegister (C1DBTCFG_REGISTER, data) ;
    }
  //----------------------------------- Request mode (C1CON_REGISTER + 3)
  //  bits 7-4: Transmit Bandwith Sharing Bits ---> 0
  //  bit 3: Abort All Pending Transmissions bit --> 0
    writeByteRegister (C1CON_REGISTER + 3, inSettings.mRequestedMode);
  //----------------------------------- Wait (2 ms max) until requested mode is reached
    bool wait = true ;
    const uint32_t deadline = millis () + 2 ;
    while (wait) {
      const uint8_t actualMode = (readByteRegister (C1CON_REGISTER + 2) >> 5) & 0x07 ;
      wait = actualMode != inSettings.mRequestedMode ;
      if (wait && (millis () >= deadline)) {
        errorCode |= kRequestedModeTimeOut ;
        wait = false ;
      }
    }
    #ifdef ARDUINO_ARCH_ESP32
      xTaskCreate (myESP32Task, "ACAN2517Handler", 1024, this, 256, NULL) ;
    #endif
    if (mINT != 255) { // 255 means interrupt is not used
      #ifdef ARDUINO_ARCH_ESP32
        attachInterrupt (itPin, inInterruptServiceRoutine, FALLING) ;
      #else
        attachInterrupt (itPin, inInterruptServiceRoutine, LOW) ;
        mSPI.usingInterrupt (itPin) ; // usingInterrupt is not implemented in Arduino ESP32
      #endif
    }
  }
//---
  return errorCode ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//    SEND FRAME
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool ACAN2517FD::tryToSend (const CANMessage & inMessage) {
  CANFDMessage message ;
  message.id = inMessage.id ;
  message.rtr = inMessage.rtr ;
  message.ext = inMessage.ext ;
  message.len = inMessage.len ;
  message.data64 [0] = inMessage.data64 ;
//---
  return tryToSend (message) ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool ACAN2517FD::tryToSend (const CANFDMessage & inMessage) {
//--- Workaround: the Teensy 3.5 / 3.6 "SPI.usingInterrupt" bug
//    https://github.com/PaulStoffregen/SPI/issues/35
  bool ok = inMessage.isValid () ;
  if (ok) {
    #if (defined (__MK64FX512__) || defined (__MK66FX1M0__))
      noInterrupts () ;
    #elif defined (ARDUINO_ARCH_ESP32)
      xSemaphoreTake (gMutualExclusion, portMAX_DELAY) ;
    #endif
      mSPI.beginTransaction (mSPISettings) ;
        if (inMessage.idx == 0) {
          ok = inMessage.len <= mTransmitFIFOPayload ;
          if (ok) {
            ok = enterInTransmitBuffer (inMessage) ;
          }
        }else if (inMessage.idx == 255) {
          ok = inMessage.len <= mTXQBufferPayload ;
          if (ok) {
            ok = sendViaTXQ (inMessage) ;
          }
        }
      mSPI.endTransaction () ;
    #if (defined (__MK64FX512__) || defined (__MK66FX1M0__))
      interrupts () ;
    #elif defined (ARDUINO_ARCH_ESP32)
      xSemaphoreGive (gMutualExclusion) ;
    #endif
  }
  return ok ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool ACAN2517FD::enterInTransmitBuffer (const CANFDMessage & inMessage) {
  bool result ;
  if (mControllerTxFIFOFull) {
    result = mDriverTransmitBuffer.append (inMessage) ;
  }else{
    result = true ;
    appendInControllerTxFIFO (inMessage) ;
  //--- If controller FIFO is full, enable "FIFO not full" interrupt
    const uint8_t status = readByteRegisterSPI (C1FIFOSTA_REGISTER (2)) ;
    if ((status & 1) == 0) { // FIFO is full
      uint8_t d = 1 << 7 ;  // FIFO is a transmit FIFO
      d |= 1 ; // Enable "FIFO not full" interrupt
      writeByteRegisterSPI (C1FIFOCON_REGISTER (2), d) ;
      mControllerTxFIFOFull = true ;
    }
  }
  return result ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

static uint32_t lengthCodeForLength (const uint8_t inLength) {
  uint32_t result = inLength & 0x0F ;
  switch (inLength) {
    case 12 : result =  9 ; break ;
    case 16 : result = 10 ; break ;
    case 20 : result = 11 ; break ;
    case 24 : result = 12 ; break ;
    case 32 : result = 13 ; break ;
    case 48 : result = 14 ; break ;
    case 64 : result = 15 ; break ;
  }
  return result ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void ACAN2517FD::appendInControllerTxFIFO (const CANFDMessage & inMessage) {
  const uint16_t ramAddress = (uint16_t) (0x400 + readRegisterSPI (C1FIFOUA_REGISTER (2))) ;
  assertCS () ;
    writeCommandSPI (ramAddress) ;
    //--- Write identifier: if an extended frame is sent, identifier bits sould be reordered (see DS20005678B, page 27)
      uint32_t idf = inMessage.id ;
      if (inMessage.ext) {
        idf = ((inMessage.id >> 18) & 0x7FF) | ((inMessage.id & 0x3FFFF) << 11) ;
      }
      writeWordSPI (idf) ;
  //--- Write DLC, RTR, IDE bits
    uint32_t data = lengthCodeForLength (inMessage.len) ;
    if (inMessage.rtr) {
      data |= 1 << 5 ; // Set RTR bit
    }
    if (inMessage.ext) {
      data |= 1 << 4 ; // Set EXT bit
    }
    if (inMessage.len > 8) { // Send a CANFD frame
      data |= 1 << 7 ; // Set FDF bit
    }
    writeWordSPI (data) ;
  //--- Write data (Swap data if processor is big endian)
    const uint32_t wordCount = (inMessage.len + 3) / 4 ;
    for (uint32_t i=0 ; i < wordCount ; i++) {
      writeWordSPI (inMessage.data32 [i]) ;
    }
  deassertCS () ;
//--- Increment FIFO, send message (see DS20005688B, page 48)
  const uint8_t d = (1 << 0) | (1 << 1) ; // Set UINC bit, TXREQ bit
  writeByteRegisterSPI (C1FIFOCON_REGISTER (2) + 1, d);
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool ACAN2517FD::sendViaTXQ (const CANFDMessage & inMessage) {
//--- Enter message only if TXQ FIFO is not full (see DS20005688B, page 50)
  const bool TXQNotFull = mUsesTXQ && (readByteRegisterSPI (C1TXQSTA_REGISTER) & 1) != 0 ;
  if (TXQNotFull) {
    const uint16_t ramAddress = (uint16_t) (0x400 + readRegisterSPI (C1TXQUA_REGISTER)) ;
    assertCS () ;
      writeCommandSPI (ramAddress) ;
    //--- Write identifier: if an extended frame is sent, identifier bits sould be reordered (see DS20005678B, page 27)
      uint32_t idf = inMessage.id ;
      if (inMessage.ext) {
        idf = ((inMessage.id >> 18) & 0x7FF) | ((inMessage.id & 0x3FFFF) << 11) ;
      }
      writeWordSPI (idf) ;
    //--- Write DLC, RTR, IDE bits
      uint32_t data = lengthCodeForLength (inMessage.len) ;
      if (inMessage.rtr) {
        data |= 1 << 5 ; // Set RTR bit
      }
      if (inMessage.ext) {
        data |= 1 << 4 ; // Set EXT bit
      }
      if (mHasDataBitRate) {
        data |= 1 << 6 ; // Set BRS bit
      }
      writeWordSPI (data) ;
    //--- Write data (Swap data if processor is big endian)
      const uint32_t wordCount = (inMessage.len + 3) / 4 ;
      for (uint32_t i=0 ; i < wordCount ; i++) {
        writeWordSPI (inMessage.data32 [i]) ;
      }
    deassertCS () ;
  //--- Increment FIFO, send message (see DS20005688B, page 48)
    const uint8_t d = (1 << 0) | (1 << 1) ; // Set UINC bit, TXREQ bit
    writeByteRegisterSPI (C1TXQCON_REGISTER + 1, d);
  }
  return TXQNotFull ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//    RECEIVE FRAME
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool ACAN2517FD::available (void) {
  #ifdef ARDUINO_ARCH_ESP32
    xSemaphoreTake (gMutualExclusion, portMAX_DELAY) ;
  #else
    noInterrupts () ;
  #endif
    const bool hasReceivedMessage = mDriverReceiveBuffer.count () > 0 ;
  #ifdef ARDUINO_ARCH_ESP32
    xSemaphoreGive (gMutualExclusion) ;
  #else
    interrupts () ;
  #endif
  return hasReceivedMessage ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool ACAN2517FD::receive (CANFDMessage & outMessage) {
  #ifdef ARDUINO_ARCH_ESP32
    xSemaphoreTake (gMutualExclusion, portMAX_DELAY) ;
  #else
    noInterrupts () ;
  #endif
    const bool hasReceivedMessage = mDriverReceiveBuffer.remove (outMessage) ;
    if (hasReceivedMessage) { // Receive FIFO is not full, enable "FIFO  not empty" interrupt
      writeByteRegisterSPI (C1FIFOCON_REGISTER (receiveFIFOIndex), 1) ;
    }
  #ifdef ARDUINO_ARCH_ESP32
    xSemaphoreGive (gMutualExclusion) ;
  #else
    interrupts () ;
  #endif
//---
  return hasReceivedMessage ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool ACAN2517FD::dispatchReceivedMessage (const tFilterMatchCallBack inFilterMatchCallBack) {
  CANFDMessage receivedMessage ;
  const bool hasReceived = receive (receivedMessage) ;
  if (hasReceived) {
    const uint32_t filterIndex = receivedMessage.idx ;
    if (NULL != inFilterMatchCallBack) {
      inFilterMatchCallBack (filterIndex) ;
    }
    ACANFDCallBackRoutine callBackFunction = (mCallBackFunctionArray == NULL) ? NULL : mCallBackFunctionArray [filterIndex] ;
    if (NULL != callBackFunction) {
      callBackFunction (receivedMessage) ;
    }
  }
  return hasReceived ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//    POLLING (ESP32)
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#ifdef ARDUINO_ARCH_ESP32
  void ACAN2517FD::poll (void) {
    xSemaphoreGive (mISRSemaphore) ;
  }
#endif

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//    POLLING (other than ESP32)
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#ifndef ARDUINO_ARCH_ESP32
  void ACAN2517FD::poll (void) {
    noInterrupts () ;
      while (isr_core ()) {}
    interrupts () ;
  }
#endif

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//   INTERRUPT SERVICE ROUTINE (ESP32)
// https://stackoverflow.com/questions/51750377/how-to-disable-interrupt-watchdog-in-esp32-or-increase-isr-time-limit
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#ifdef ARDUINO_ARCH_ESP32
  void ACAN2517FD::isr (void) {
    xSemaphoreGive (mISRSemaphore) ;
  }
#endif

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//   INTERRUPT SERVICE ROUTINE (other than ESP32)
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#ifndef ARDUINO_ARCH_ESP32
  void ACAN2517FD::isr (void) {
    isr_core () ;
  }
#endif

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//   INTERRUPT SERVICE ROUTINES (common)
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

bool ACAN2517FD::isr_core (void) {
  bool handled = false ;
  mSPI.beginTransaction (mSPISettings) ;
  const uint32_t it = readRegisterSPI (C1INT_REGISTER) ; // DS20005688B, page 34
  if ((it & (1 << 1)) != 0) { // Receive FIFO interrupt
    receiveInterrupt () ;
    handled = true ;
  }
  if ((it & (1 << 0)) != 0) { // Transmit FIFO interrupt
    transmitInterrupt () ;
    handled = true ;
  }
  if ((it & (1 << 2)) != 0) { // TBCIF interrupt
    writeByteRegisterSPI (C1INT_REGISTER, 1 << 2) ;
  }
  if ((it & (1 << 3)) != 0) { // MODIF interrupt
    writeByteRegisterSPI (C1INT_REGISTER, 1 << 3) ;
  }
  if ((it & (1 << 12)) != 0) { // SERRIF interrupt
    writeByteRegisterSPI (C1INT_REGISTER + 1, 1 << 4) ;
  }
  mSPI.endTransaction () ;
  return handled ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void ACAN2517FD::transmitInterrupt (void) {
  CANFDMessage message ;
  mDriverTransmitBuffer.remove (message) ;
  appendInControllerTxFIFO (message) ;
//--- If driver transmit buffer is empty, disable "FIFO not full" interrupt
  if (mDriverTransmitBuffer.count () == 0) {
    const uint8_t d = 1 << 7 ;  // FIFO is a transmit FIFO
    writeByteRegisterSPI (C1FIFOCON_REGISTER (2), d) ;
    mControllerTxFIFOFull = false ;
  }
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void ACAN2517FD::receiveInterrupt (void) {
  static const uint8_t kActualLength [16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64} ;
  readByteRegisterSPI (C1FIFOSTA_REGISTER (receiveFIFOIndex)) ;
  const uint16_t ramAddress = (uint16_t) (0x400 + readRegisterSPI (C1FIFOUA_REGISTER (receiveFIFOIndex))) ;
  CANFDMessage message ;
  assertCS () ;
    readCommandSPI (ramAddress) ;
  //--- Read identifier (see DS20005678A, page 42)
    message.id = readWordSPI () ;
  //--- Read DLC, RTR, IDE bits, and math filter index
    const uint32_t data = readWordSPI () ;
    message.rtr = (data & (1 << 5)) != 0 ;
    message.ext = (data & (1 << 4)) != 0 ;
    message.len = kActualLength [data & 0x0F] ;
    message.idx = (uint8_t) ((data >> 11) & 0x1F) ;
  //--- Write data (Swap data if processor is big endian)
    const uint32_t wordCount = (message.len + 3) / 4 ;
    for (uint32_t i=0 ; i < wordCount ; i++) {
      message.data32 [i] = readWordSPI () ;
    }
  deassertCS () ;
//--- If an extended frame is received, identifier bits sould be reordered (see DS20005678B, page 42)
  if (message.ext) {
    const uint32_t tempID = message.id ;
    message.id = ((tempID >> 11) & 0x3FFFF) | ((tempID & 0x7FF) << 18) ;
  }
//--- Append message to driver receive FIFO
  mDriverReceiveBuffer.append (message) ;
//--- Increment FIFO
  const uint8_t d = 1 << 0 ; // Set UINC bit (DS20005688B, page 52)
  writeByteRegisterSPI (C1FIFOCON_REGISTER (receiveFIFOIndex) + 1, d) ;
//--- If driver receive FIFO is full, disable "FIFO not empty" interrupt
  if (mDriverReceiveBuffer.count () == mDriverReceiveBuffer.size ()) {
    writeByteRegisterSPI (C1FIFOCON_REGISTER (receiveFIFOIndex), 0) ;
  }
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//   MCP2517FD REGISTER ACCESS, FIRST LEVEL FUNCTIONS
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void ACAN2517FD::readCommandSPI (const uint16_t inRegisterAddress) {
  const uint16_t readCommand = (inRegisterAddress & 0x0FFF) | (0b0011 << 12) ;
  mSPI.transfer16 (readCommand) ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void ACAN2517FD::writeCommandSPI (const uint16_t inRegisterAddress) {
  const uint16_t readCommand = (inRegisterAddress & 0x0FFF) | (0b0010 << 12) ;
  mSPI.transfer16 (readCommand) ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

uint32_t ACAN2517FD::readWordSPI (void) {
  uint32_t result = mSPI.transfer (0) ;
  result |= ((uint32_t) mSPI.transfer (0)) <<  8 ;
  result |= ((uint32_t) mSPI.transfer (0)) << 16 ;
  result |= ((uint32_t) mSPI.transfer (0)) << 24 ;
  return result ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void ACAN2517FD::writeWordSPI (const uint32_t inValue) {
  mSPI.transfer ((uint8_t) inValue) ;
  mSPI.transfer ((uint8_t) (inValue >>  8)) ;
  mSPI.transfer ((uint8_t) (inValue >> 16)) ;
  mSPI.transfer ((uint8_t) (inValue >> 24)) ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//   MCP2517FD REGISTER ACCESS, SECOND LEVEL FUNCTIONS
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void ACAN2517FD::assertCS (void) {
  digitalWrite (mCS, LOW) ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void ACAN2517FD::deassertCS (void) {
  digitalWrite (mCS, HIGH) ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void ACAN2517FD::writeRegisterSPI (const uint16_t inRegisterAddress, const uint32_t inValue) {
  assertCS () ;
    writeCommandSPI (inRegisterAddress) ; // Command
    writeWordSPI (inValue) ; // Data
  deassertCS () ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

uint32_t ACAN2517FD::readRegisterSPI (const uint16_t inRegisterAddress) {
  assertCS () ;
    readCommandSPI (inRegisterAddress) ; // Command
    const uint32_t result = readWordSPI () ; // Data
  deassertCS () ;
  return result ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void ACAN2517FD::writeByteRegisterSPI (const uint16_t inRegisterAddress, const uint8_t inValue) {
  assertCS () ;
    writeCommandSPI (inRegisterAddress) ; // Command
    mSPI.transfer (inValue) ; // Data
  deassertCS () ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

uint8_t ACAN2517FD::readByteRegisterSPI (const uint16_t inRegisterAddress) {
  assertCS () ;
    readCommandSPI (inRegisterAddress) ; // Command
    const uint8_t result = mSPI.transfer (0) ; // Data
  deassertCS () ;
  return result ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//   MCP2517FD REGISTER ACCESS, THIRD LEVEL FUNCTIONS
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void ACAN2517FD::writeByteRegister (const uint16_t inRegisterAddress, const uint8_t inValue) {
  mSPI.beginTransaction (mSPISettings) ;
    writeByteRegisterSPI (inRegisterAddress, inValue) ;
  mSPI.endTransaction () ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

uint8_t ACAN2517FD::readByteRegister (const uint16_t inRegisterAddress) {
  mSPI.beginTransaction (mSPISettings) ;
    const uint8_t result = readByteRegisterSPI (inRegisterAddress) ;
  mSPI.endTransaction () ;
  return result ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void ACAN2517FD::writeRegister (const uint16_t inRegisterAddress, const uint32_t inValue) {
  mSPI.beginTransaction (mSPISettings) ;
    writeRegisterSPI (inRegisterAddress, inValue) ;
  mSPI.endTransaction () ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

uint32_t ACAN2517FD::readRegister (const uint16_t inRegisterAddress) {
  mSPI.beginTransaction (mSPISettings) ;
    const uint32_t result = readRegisterSPI (inRegisterAddress) ;
  mSPI.endTransaction () ;
  return result ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

uint32_t ACAN2517FD::readErrorCounters (void) {
  mSPI.beginTransaction (mSPISettings) ;
    const uint32_t result = readRegisterSPI (C1BDIAG0_REGISTER) ;
  mSPI.endTransaction () ;
  return result ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void ACAN2517FD::reset2517FD (void) {
  mSPI.beginTransaction (mSPISettings) ; // Check RESET is performed with 1 MHz clock
    assertCS () ;
      mSPI.transfer16 (0x00) ; // Reset instruction: 0x0000
    deassertCS () ;
  mSPI.endTransaction () ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

