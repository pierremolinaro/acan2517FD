#ifdef ARDUINO

#ifndef _ARDUINO_SPI_H_
#define _ARDUINO_SPI_H_

#include <Arduino.h>
#include <SPI.h>

#include "SPIHardwareInterface.h"

class ArduinoHardwareSPI : public SPIHardwareInterface
{
public:
  ArduinoHardwareSPI(SPIClass & dev, const uint8_t cs, const uint32_t spiClock);

  virtual void beginTransaction(bool configuration_mode = false) = 0;

  virtual void endTransaction();

  virtual int transfer(const uint8_t *buffer, int length);

  virtual int transfer16(const uint16_t data);

  virtual void initCS();

  virtual inline void assertCS();

  virtual inline void deassertCS();

private:
  //  SPI
  SPIClass & _dev;
  //  CS
  const uint8_t _cs;
  //  SPI settings
  SPISettings _normalSPISettings;
  SPISettings _configurationSPISettings;
  //  SPI max clock
  const uint32_t _spiClock;

};

#endif

#endif