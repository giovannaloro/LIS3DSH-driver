#include <iostream>
#include <miosix.h>
#include "LIS3DSH.h"
#include <utils/Debug.h>


using namespace miosix;
using namespace Boardcore;
/*
SPIBus bus(SPI1);
GpioPin cs(GPIOA_BASE, 4);
GpioPin sck(GPIOA_BASE, 5);
GpioPin miso(GPIOA_BASE, 6);
GpioPin mosi(GPIOA_BASE, 7);
*/

int main() {
    /*
    sck.mode(Mode::ALTERNATE);
    sck.alternateFunction(5);
    miso.mode(Mode::ALTERNATE);
    miso.alternateFunction(5);
    mosi.mode(Mode::ALTERNATE);
    mosi.alternateFunction(5);
    cs.mode(Mode::OUTPUT);
    cs.high();
    LIS3DSH sensor(bus, cs);
    */
    //sensor.init();
    while (true) {
        //AccelerometerData data = sensor.sampleImpl();
        //TRACE("baboab");
        std::cout << "Z" << std::endl;//data.accelerationZ << "Y" << data.accelerationY << "X" << data.accelerationX << "endl"; 
        sleep(200);
    }
    return 0;
}
