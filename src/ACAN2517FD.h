//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// A CAN driver for MCP2517FD, CANFD mode
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2517FD
//
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#pragma once

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#include <ACAN2517FDSettings.h>
#include <ACANFDBuffer.h>
#include <CANMessage.h>
#include <ACAN2517FDFilters.h>
#include <SPI.h>

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//   ACAN2517FD class
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

class ACAN2517FD {

//······················································································································
//   CONSTRUCTOR
//······················································································································

  public: ACAN2517FD (const uint8_t inCS, // CS input of MCP2517FD
                      SPIClass & inSPI, // Hardware SPI object
                      const uint8_t inINT) ; // INT output of MCP2517FD

//······················································································································
//   begin method (returns 0 if no error)
//······················································································································

  public: uint32_t begin (const ACAN2517FDSettings & inSettings,
                          void (* inInterruptServiceRoutine) (void)) ;

  public: uint32_t begin (const ACAN2517FDSettings & inSettings,
                          void (* inInterruptServiceRoutine) (void),
                          const ACAN2517FDFilters & inFilters) ;

//--- Error code returned by begin
  public: static const uint32_t kRequestedConfigurationModeTimeOut  = 1 <<  0 ;
  public: static const uint32_t kReadBackErrorWith1MHzSPIClock      = 1 <<  1 ;
  public: static const uint32_t kTooFarFromDesiredBitRate           = 1 <<  2 ;
  public: static const uint32_t kInconsistentBitRateSettings        = 1 <<  3 ;
  public: static const uint32_t kINTPinIsNotAnInterrupt             = 1 <<  4 ;
  public: static const uint32_t kISRIsNull                          = 1 <<  5 ;
  public: static const uint32_t kFilterDefinitionError              = 1 <<  6 ;
  public: static const uint32_t kMoreThan32Filters                  = 1 <<  7 ;
  public: static const uint32_t kControllerReceiveFIFOSizeIsZero    = 1 <<  8 ;
  public: static const uint32_t kControllerReceiveFIFOSizeGreaterThan32 = 1 << 9 ;
  public: static const uint32_t kControllerTransmitFIFOSizeIsZero    = 1 << 10 ;
  public: static const uint32_t kControllerTransmitFIFOSizeGreaterThan32 = 1 << 11 ;
  public: static const uint32_t kControllerRamUsageGreaterThan2048   = 1 << 12 ;
  public: static const uint32_t kControllerTXQPriorityGreaterThan31  = 1 << 13 ;
  public: static const uint32_t kControllerTransmitFIFOPriorityGreaterThan31 = 1 << 14 ;
  public: static const uint32_t kControllerTXQSizeGreaterThan32     = 1 << 15 ;
  public: static const uint32_t kRequestedModeTimeOut               = 1 << 16 ;
  public: static const uint32_t kX10PLLNotReadyWithin1MS            = 1 << 17 ;
  public: static const uint32_t kReadBackErrorWithFullSpeedSPIClock = 1 << 18 ;
  public: static const uint32_t kISRNotNullAndNoIntPin              = 1 << 19 ;

//······················································································································
//   Send a message
//······················································································································

  public: bool tryToSend (const CANMessage & inMessage) ;

  public: bool tryToSend (const CANFDMessage & inMessage) ;

//······················································································································
//    Receive a message
//······················································································································

  public: bool receive (CANFDMessage & outMessage) ;
  public: bool available (void) ;
  public: typedef void (*tFilterMatchCallBack) (const uint32_t inFilterIndex) ;
  public: bool dispatchReceivedMessage (const tFilterMatchCallBack inFilterMatchCallBack = NULL) ;

//--- Call back function array
  private: ACANFDCallBackRoutine * mCallBackFunctionArray = NULL ;

//······················································································································
//    Get error counters
//······················································································································

  public: uint32_t readErrorCounters (void) ;

//······················································································································
//    Private properties
//······················································································································

  private: SPISettings mSPISettings ;
  private: SPIClass & mSPI ;
  private: uint8_t mCS ;
  private: uint8_t mINT ;
  private: bool mUsesTXQ ;
  private: bool mControllerTxFIFOFull ;
  private: bool mHasDataBitRate ;
  private: uint8_t mTransmitFIFOPayload ; // in byte count
  private: uint8_t mTXQBufferPayload ; // in byte count
  private: uint8_t mReceiveFIFOPayload ; // in byte count

//······················································································································
//    Receive buffer
//······················································································································

  private: ACANBuffer mDriverReceiveBuffer ;

//······················································································································
//    Transmit buffer
//······················································································································

  private: ACANBuffer mDriverTransmitBuffer ;

  public: uint32_t driverTransmitBufferSize (void) const { return mDriverTransmitBuffer.size () ; }

  public: uint32_t driverTransmitBufferCount (void) const { return mDriverTransmitBuffer.count () ; }

  public: uint32_t driverTransmitBufferPeakCount (void) const { return mDriverTransmitBuffer.peakCount () ; }

//······················································································································
//    Private methods
//······················································································································

  private: void readCommandSPI (const uint16_t inRegisterAddress) ;
  private: void writeCommandSPI (const uint16_t inRegisterAddress) ;
  private: uint32_t readWordSPI (void) ;
  private: void writeWordSPI (const uint32_t inValue) ;

  private: void writeRegisterSPI (const uint16_t inRegisterAddress, const uint32_t inValue) ;
  private: uint32_t readRegisterSPI (const uint16_t inRegisterAddress) ;
  private: void writeByteRegisterSPI (const uint16_t inRegisterAddress, const uint8_t inValue) ;
  private: uint8_t readByteRegisterSPI (const uint16_t inRegisterAddress) ;
  private: void assertCS (void) ;
  private: void deassertCS (void) ;

  private: void reset2517FD (void) ;
  private: void writeRegister (const uint16_t inAddress, const uint32_t inValue) ;
  private: uint32_t readRegister (const uint16_t inAddress) ;
  private: void writeByteRegister (const uint16_t inRegisterAddress, const uint8_t inValue) ;
  private: uint8_t readByteRegister (const uint16_t inAddress) ;

  private: bool sendViaTXQ (const CANFDMessage & inMessage) ;
  private: bool enterInTransmitBuffer (const CANFDMessage & inMessage) ;
  private: void appendInControllerTxFIFO (const CANFDMessage & inMessage) ;

//······················································································································
//    Polling
//······················································································································

  public: void poll (void) ;

//······················································································································
//    Interrupt service routine
//······················································································································

  public: void isr (void) ;
  public: bool isr_core (void) ;
  private: void receiveInterrupt (void) ;
  private: void transmitInterrupt (void) ;
  #ifdef ARDUINO_ARCH_ESP32
    public: SemaphoreHandle_t mISRSemaphore ;
  #endif

//······················································································································
//    No copy
//······················································································································

  private: ACAN2517FD (const ACAN2517FD &) ;
  private: ACAN2517FD & operator = (const ACAN2517FD &) ;

//······················································································································

} ;

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

