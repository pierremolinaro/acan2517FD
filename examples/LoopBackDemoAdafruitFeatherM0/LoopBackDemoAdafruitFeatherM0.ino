//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517FD Demo in loopback mode, for Adafruit M0
//——————————————————————————————————————————————————————————————————————————————

#ifndef ARDUINO_SAMD_ZERO
  #error "Select 'Adafruit Feather M0' board"
#endif

//——————————————————————————————————————————————————————————————————————————————

#include <ACAN2517FD.h>
#include <SPI.h>
#include <wiring_private.h>

//——————————————————————————————————————————————————————————————————————————————
// Very very important: put a 10kΩ resistor between CS and VDD of MCP2517FD

static const byte MCP2517_CS  =  6 ; // CS input of MCP2517
static const byte MCP2517_INT =  5 ; // INT output of MCP2517
static const byte MCP2517_SO  = 10 ; // SO output of MCP2517
static const byte MCP2517_SI  = 11 ; // SI input of MCP2517
static const byte MCP2517_CLK = 12 ; // CLK input of MCP2517

//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517FD Driver object
//——————————————————————————————————————————————————————————————————————————————

SPIClass mySPI (&sercom1, MCP2517_SO, MCP2517_CLK, MCP2517_SI, SPI_PAD_0_SCK_3, SERCOM_RX_PAD_2);

ACAN2517FD can (MCP2517_CS, mySPI, MCP2517_INT) ;

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————

void setup () {
//--- Start serial
  Serial.begin (115200) ;
//--- Wait for serial (blink led at 10 Hz during waiting)
  while (!Serial) {
    delay (50) ;
  }
//----------------------------------- Begin SPI
  mySPI.begin () ;
  pinPeripheral (MCP2517_SI, PIO_SERCOM);
  pinPeripheral (MCP2517_CLK, PIO_SERCOM);
  pinPeripheral (MCP2517_SO, PIO_SERCOM);
//--- Configure ACAN2517FD
  Serial.print ("sizeof (ACAN2517FDSettings): ") ;
  Serial.print (sizeof (ACAN2517FDSettings)) ;
  Serial.println (" bytes") ;
  Serial.println ("Configure ACAN2517FD") ;
//--- For version >= 2.1.0
  ACAN2517FDSettings settings (ACAN2517FDSettings::OSC_20MHz, 500UL * 1000UL, DataBitRateFactor::x1) ;
  settings.mRequestedMode = ACAN2517FDSettings::InternalLoopBack ; // Select loopback mode
//--- Default values are too high for an Arduino Uno that contains 2048 bytes of RAM: reduce them
  settings.mDriverTransmitFIFOSize = 1 ;
  settings.mDriverReceiveFIFOSize = 1 ;
//--- RAM Usage
  Serial.print ("MCP2517FD RAM Usage: ") ;
  Serial.print (settings.ramUsage ()) ;
  Serial.println (" bytes") ;
//--- Begin
  const uint32_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
  if (errorCode == 0) {
    Serial.print ("Bit Rate prescaler: ") ;
    Serial.println (settings.mBitRatePrescaler) ;
    Serial.print ("Arbitration Phase segment 1: ") ;
    Serial.println (settings.mArbitrationPhaseSegment1) ;
    Serial.print ("Arbitration Phase segment 2: ") ;
    Serial.println (settings.mArbitrationPhaseSegment2) ;
    Serial.print ("Arbitration SJW:") ;
    Serial.println (settings.mArbitrationSJW) ;
    Serial.print ("Actual Arbitration Bit Rate: ") ;
    Serial.print (settings.actualArbitrationBitRate ()) ;
    Serial.println (" bit/s") ;
    Serial.print ("Exact Arbitration Bit Rate ? ") ;
    Serial.println (settings.exactArbitrationBitRate () ? "yes" : "no") ;
    Serial.print ("Arbitration Sample point: ") ;
    Serial.print (settings.arbitrationSamplePointFromBitStart ()) ;
    Serial.println ("%") ;
  }else{
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//——————————————————————————————————————————————————————————————————————————————
//   LOOP
//——————————————————————————————————————————————————————————————————————————————

static uint32_t gSendDate = 0 ;
static uint32_t gReceiveDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gSentFrameCount = 0 ;

//——————————————————————————————————————————————————————————————————————————————

void loop () {
  CANFDMessage frame ;
  if (gSendDate < millis ()) {
    frame.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH ;
    gSendDate += 1000 ;
    const bool ok = can.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
      Serial.print ("Sent: ") ;
      Serial.print (gSentFrameCount) ;
    }else{
      Serial.print ("Send failure") ;
    }
    Serial.print (", receive overflows: ") ;
    Serial.println (can.hardwareReceiveBufferOverflowCount ()) ;
  }
  if (gReceiveDate < millis ()) {
    gReceiveDate += 4567 ;
    while (can.available ()) {
      can.receive (frame) ;
      gReceivedFrameCount +=1 ;
      Serial.print ("Received: ") ;
      Serial.println (gReceivedFrameCount) ;
    }
  }
}

//——————————————————————————————————————————————————————————————————————————————
