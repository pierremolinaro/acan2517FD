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

    virtual void beginTransaction(bool configuration_mode = false) override;

    virtual void endTransaction() override;

    virtual int transfer(const uint8_t *buffer, int length) override;

    virtual int transfer16(const uint16_t data) override;

    virtual void initCS() override;

    virtual inline void assertCS() override;

    virtual inline void deassertCS() override;

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
