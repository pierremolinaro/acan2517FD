#if defined(__arm__) && defined(MBED_SB)

#include "MbedHardwareSPI.h"

MbedHardwareSPI::MbedHardwareSPI(SPI & dev, const PinName csPin)
  : _dev(dev), _cs(csPin)
{
    //  init spi settings
    //  configuration mode
    _configuration_mode = true;
    //  frequency
    _dev.frequency(1000UL * 1000);
    //  8bits mode_0
    _dev.format(8, 0);
}

void MbedHardwareSPI::beginTransaction(bool configuration_mode)
{
    if(_configuration_mode != configuration_mode) {
        if(configuration_mode) {
            //  frequency
            _dev.frequency(1000UL * 1000);
        }
        else {
            //  frequency
            _dev.frequency(_spiClock);
        }

        _configuration_mode = configuration_mode;
    }

    //  TODO: lock?
}

void MbedHardwareSPI::endTransaction()
{
    //  nope
    //  TODO: unlock?
}

int MbedHardwareSPI::transfer(const uint8_t *buffer, int length)
{
    _dev.write((char*) buffer, length, (char*) buffer, length);

    return 0;
}

int MbedHardwareSPI::transfer16(const uint16_t data)
{
    _dev.write(data);
    return 0;
}

void MbedHardwareSPI::initCS()
{
    //  nop
}

inline void MbedHardwareSPI::assertCS()
{
    _cs.write(0);
}

inline void MbedHardwareSPI::deassertCS()
{
    _cs.write(1);
}

void MbedHardwareSPI::setSPIClock(const uint32_t spiClock)
{
    //  super
    SPIHardwareInterface::setSPIClock(spiClock);
}

#endif