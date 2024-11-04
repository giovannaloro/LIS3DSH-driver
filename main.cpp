#include <iostream>
#include <miosix.h>

int main() {
    /* Outline of what this entrypoint should do:
        - Initialize the SPI1 bus and the pins for SCK, MISO, MOSI and CS
        - Create an instance of the LIS3DSH sensor
        - Initialize the sensor
        - Loop forever:
            - Sample the sensor
            - Get the last sample
            - Print the sample data
            - Sleep for 200ms
    */

    while (true) {
        std::cout << "Hello there, skywarder!" << std::endl;
        miosix::Thread::sleep(1000);
    }

    return 0;
}
