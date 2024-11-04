#pragma once

#include<miosix.h>
#include<sensors/Sensor.h>
#include<drivers/spi/SPIDriver.h>
#include<sensors/SensorData.h> 
#include<drivers/spi/SPIBusInterface.h>
#include <diagnostic/PrintLogger.h>


namespace Boardcore
{
/**
 * @brief Driver for LIS3DSH STM32F407VG embedded accelerometer.
 */

class LIS3DSH : public Sensor<AccelerometerData>{
public:
    
    //custom bus config constructor
    LIS3DSH(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig);

    //bus config constructor
    LIS3DSH(SPIBusInterface& bus, miosix::GpioPin cs );


    //distructor
    ~LIS3DSH();

    //initialization method 
    bool init() override;

    //correct functioning control method 
    bool selfTest() override;

    //sample method 
    AccelerometerData sampleImpl();

private:

    //The addresses of the sensor's register used by the driver 
    enum Register : uint8_t
    {
        WHO_AM_I_ADD = 0x0F,
        OUT_X_L_ADD = 0x28,
        OUT_X_H_ADD = 0x29,
        OUT_Y_L_ADD = 0x2A,
        OUT_Y_H_ADD = 0x2B,
        OUT_Z_L_ADD = 0x2C,
        OUT_Z_H_ADD = 0x2D,
        CTRL5_ADD = 0x34,
    };

    //The selectable range of outputs values
    enum Range : uint8_t
    {
        FS_2_G = 0x01,  
        FS_4_G = 0x2,  
        FS_6_G = 0x03,  
        FS_8_G = 0x04,  
        FS_16_G = 0x5, 
    }; 

    //The auto test maximum delta for x and y registers
    static constexpr uint16_t MAX_DELTA_X_Y = 140;

    //The auto test maximum delta for z register
    static constexpr uint16_t MAX_DELTA_Z = 590;

    //A fixed value stored in a specific sensor register, it's used to check proper working
    static constexpr uint8_t WHO_AM_I_KEY = 0x3F;

    //The FS_2_G conversion factor in mg/digit
    static constexpr float FS_2_G_CONV_FACTOR = 0.06f;

    //The FS_4_G conversion factor in mg/digit
    static constexpr float FS_4_G_CONV_FACTOR = 0.12f;

    //The FS_6_G conversion factor in mg/digit
    static constexpr float FS_6_G_CONV_FACTOR = 0.18f;

    //The FS_8_G conversion factor in mg/digit
    static constexpr float FS_8_G_CONV_FACTOR = 0.24f;

    //The FS_16_G conversion factor in mg/digit
    static constexpr float FS_16_G_CONV_FACTOR = 0.73f;

    static constexpr uint8_t SET_2_G_SCALE = (uint8_t) 0 << 3;

    static constexpr uint8_t SET_4_G_SCALE = (uint8_t) 0 << 3;

    static constexpr uint8_t SET_6_G_SCALE = (uint8_t) 1 << 3;

    static constexpr uint8_t SET_8_G_SCALE = (uint8_t) 2 << 3;

    static constexpr uint8_t SET_16_G_SCALE = (uint8_t) 3 << 3;

    

    //The enable test const
    static constexpr uint8_t ENABLE_TEST_MODE = (uint8_t) 1;

    //The disable test const
    static constexpr uint8_t DISABLE_TEST_MODE = (uint8_t) 0;

    //This struct represents the controlled device, hence the sensor
    SPISlave spiSlave;

    //A value that represents the initialization of the sensor 
    bool isInit = false; 

    //The error logger 
    PrintLogger logger = Logging::getLogger("LIS3DSH");

    //The current output range scale
    Range currentScale = FS_2_G; 

    //The current conversion factor
    float currentConversionFactor = FS_2_G_CONV_FACTOR;

    //A private method to check correct board sensor communication
    bool checkWhoAmI();

    //A private method used to modify the output range
    bool setRange(Range selectedRange);

    float readSingleAxis(Register msBitsAdd, Register lsBitsAdd);

    //A private method to read the acceleration (mg) value of the x register
    float readX();

    //A private method to read the acceleration (mg) value of the y register
    float readY();

    //A private method to read the acceleration (mg) value of the z register
    float readZ();

    //A private method to check correct values reading as suggested in the datasheet 
    bool checkReadValue();

}; 
}

/*
TO DO
Make error management better 
Refactor comments
Test and Debug
*/
