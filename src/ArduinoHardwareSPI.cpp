#ifdef ARDUINO

#include "ArduinoHardwareSPI.h"

ArduinoHardwareSPI::ArduinoHardwareSPI(SPIClass & dev, const uint8_t cs, const uint32_t spiClock)
  : _dev(dev), _cs(cs), _spiClock(spiClock)
{
  //  init spi settings
  //  1Mhz
  _configurationSPISettings = SPISettings (1000UL * 1000, MSBFIRST, SPI_MODE0);
  //  full speed
  _normalSPISettings = SPISettings(_spiClock, MSBFIRST, SPI_MODE0);
}

void ArduinoHardwareSPI::beginTransaction(bool configuration_mode)
{
  if (configuration_mode)
  {
    _dev.beginTransaction(_configurationSPISettings);
  }
  else
  {
    _dev.beginTransaction(_normalSPISettings);
  }
}

void ArduinoHardwareSPI::endTransaction()
{
  _dev.endTransaction();
}

int ArduinoHardwareSPI::transfer(const uint8_t *buffer, int length)
{
  _dev.transfer(buffer, length);

  return 0;
}

int ArduinoHardwareSPI::transfer16(const uint16_t data)
{
  _dev.transfer16(data);

  return 0;
}

void ArduinoHardwareSPI::initCS() {
  //  init CS pin
  pinMode(_cs, OUTPUT);
}

inline void ArduinoHardwareSPI::assertCS() {
  digitalWrite(_cs, LOW);
}

inline void ArduinoHardwareSPI::deassertCS() {
  digitalWrite(_cs, HIGH);
}

#endif