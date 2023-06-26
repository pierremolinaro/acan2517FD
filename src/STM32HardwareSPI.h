#if defined(__arm__) && defined(STM32)

#ifndef _STM32_HARDWARE_SPI_H_
#define _STM32_HARDWARE_SPI_H_

#include "SPIHardwareInterface.h"
#include "stdint.h"
#include "main.h"

namespace acan2517fd {

class STM32HardwareSPI : public SPIHardwareInterface
{
public:
    STM32HardwareSPI(SPI_HandleTypeDef *dev, GPIO_TypeDef *cs_port, uint16_t cs_pin);

    virtual void beginTransaction(bool configuration_mode = false) override;

    virtual void endTransaction() override;

    virtual int transfer(const uint8_t *buffer, int length) override;

    virtual int transfer16(const uint16_t data) override;

    virtual void initCS() override;

    virtual inline void assertCS() override;

    virtual inline void deassertCS() override;

private:
    //  SPI
    SPI_HandleTypeDef *_dev;

    //  GPIO Port
    GPIO_TypeDef *_cs_port;
    uint16_t _cs_pin;
};

}

#endif

#endif