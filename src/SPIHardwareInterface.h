#ifndef _SPI_HARDWARE_INTERFACE_H_
#define _SPI_HARDWARE_INTERFACE_H_

#include "stdint.h"

namespace acan2517fd {


    class SPIHardwareInterface {
    public:

        /////////////////////////////////
        //  Chip select
        /////////////////////////////////

        /**
         * @brief Configure spi settings and begin transaction
         *
         * @param configuration_mode If this is true, configure spi as configuration mode
         */
        virtual void beginTransaction(bool configuration_mode = false) = 0;

        /**
         * @brief Finish transaction
         *
         */
        virtual void endTransaction() = 0;

        /**
         * @brief Transfer data specified length
         *
         * @param buffer The data buffer which will transfer
         * @param length The data length
         * @return int The number of bytes written and read from the device
         */
        virtual int transfer(const uint8_t *buffer, int length) = 0;

        /**
         * @brief Transfer 16-bit data
         *
         * @param data The data to transfer
         * @return int The number of bytes written and read from the device
         */
        virtual int transfer16(const uint16_t data) = 0;

        /////////////////////////////////
        //  Chip select
        /////////////////////////////////

        /**
         * @brief initialize CS pin
         *
         */
        virtual void initCS() = 0;

        /**
         * @brief assert cs
         *
         */
        virtual inline void assertCS() = 0;

        /**
         * @brief assert cs
         *
         */
        virtual inline void deassertCS() = 0;

        /////////////////////////////////
        //  settings
        /////////////////////////////////

        /**
         * @brief set system clock
         *
         */
        virtual void setSPIClock(const uint32_t spiClock) {
            _spiClock = spiClock;
        }

    protected:
        uint32_t _spiClock;
    };

}

#endif
