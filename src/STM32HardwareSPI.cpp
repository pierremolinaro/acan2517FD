#if defined(__arm__) && defined(STM32)

#include "STM32HardwareSPI.h"

namespace acan2517fd {

STM32HardwareSPI::STM32HardwareSPI(SPI_HandleTypeDef & dev, GPIO_TypeDef & cs_port, uint16_t cs_pin)
    : _dev(dev), _cs_port(cs_port), _cs_pin(cs_pin)
{
    __HAL_SPI_ENABLE(_dev);
}

void STM32HardwareInterface::beginTransaction(bool configuration_mode)
{
    //  TODO: lock?
}

void STM32HardwareInterface::endTransaction()
{
    //  TODO: unlock?
}

int STM32HardwareInterface::transfer(const uint8_t *buffer, int length)
{
    HAL_SPI_TransmitReceive(_dev, buffer, length, buffer, length);

    return 0;
}

int STM32HardwareInterface::transfer16(const uint16_t data)
{
    HAL_SPI_TransmitReceive(_dev, (uint8_t*) data, 2, (uint8_t*) data, 2);

    return 0;
}

void STM32HardwareInterface::initCS()
{
    //  nop
}

inline void STM32HardwareInterface::assertCS()
{
    HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
}

inline void STM32HardwareInterface::deassertCS()
{
    HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
}

}
#endif