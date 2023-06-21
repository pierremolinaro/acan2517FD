#if defined(__arm__) && defined(MBED_SB)

#ifndef _MBED_HARDWARE_SPI_H_
#define _MBED_HARDWARE_SPI_H_

#include "mbed.h"

#include "SPIHardwareInterface.h"

namespace acan2517fd {


class MbedHardwareSPI : public SPIHardwareInterface
{
public:
    MbedHardwareSPI(SPI & dev, const PinName csPin);

    virtual void beginTransaction(bool configuration_mode = false);

    virtual void endTransaction();

    virtual int transfer(const uint8_t *buffer, int length);

    virtual int transfer16(const uint16_t data);

    virtual void initCS();

    virtual inline void assertCS();

    virtual inline void deassertCS();

    void setSPIClock(const uint32_t spiClock) override;

private:
    //  SPI
    SPI & _dev;
    //  CS
    DigitalOut _cs;

    //  mode
    bool _configuration_mode;
};

}

#endif

#endif
