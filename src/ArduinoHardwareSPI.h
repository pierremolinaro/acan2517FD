#ifdef ARDUINO

#ifndef _ARDUINO_SPI_H_
#define _ARDUINO_SPI_H_

#include <Arduino.h>
#include <SPI.h>

#include "SPIHardwareInterface.h"

namespace acan2517fd {


class ArduinoHardwareSPI : public SPIHardwareInterface
{
public:
    ArduinoHardwareSPI(SPIClass & dev, const uint8_t cs);

    void beginTransaction(bool configuration_mode = false) override;

    void endTransaction() override;

    int transfer(const uint8_t *buffer, int length) override;

    int transfer16(const uint16_t data) override;

    void initCS() override;

    inline void assertCS() override;

    inline void deassertCS() override;

    void setSPIClock(const uint32_t spiClock) override;

private:
    //  SPI
    SPIClass & _dev;
    //  CS
    const uint8_t _cs;
    //  SPI settings
    SPISettings _normalSPISettings;
    SPISettings _configurationSPISettings;
};

}

#endif

#endif