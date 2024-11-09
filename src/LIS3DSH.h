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

    //Selectable ranges of outputs values
    enum Range : uint8_t
    {
        FS_2_G = 0x01,  
        FS_4_G = 0x2,  
        FS_6_G = 0x03,  
        FS_8_G = 0x04,  
        FS_16_G = 0x5, 
    }; 

    //Custom bus config constructor
    LIS3DSH(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig);

    //Bus config constructor
    LIS3DSH(SPIBusInterface& bus, miosix::GpioPin cs );

    //Initialization 
    bool init() override;

    //Internal sensor self test, check both proper communication and correctness of read values 
    bool selfTest() override;

    //Sample method on all axis
    AccelerometerData sampleImpl();

    //Output range selection  
    bool selectRange(Range selectedRange);

private:

    //Addresses of the sensor's registers used by the driver 
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

    //Bit masks used in setting sensor's registers
    enum Mask : uint8_t
    {
        SET_2_G_SCALE = (uint8_t) 0 << 3,
        SET_4_G_SCALE = (uint8_t) 1 << 3,
        SET_6_G_SCALE = (uint8_t) 2 << 3,
        SET_8_G_SCALE = (uint8_t) 3 << 3,
        SET_16_G_SCALE = (uint8_t) 4 << 3,
        CLEAN_FS = ((uint8_t) (7 << 3)),
        ENABLE_TEST_MODE = (uint8_t) 1 << 1,
        DISABLE_TEST_MODE = ((uint8_t) (0 << 1)),
    };

    //Auto test maximum delta for x and y registers in mg/digit
    static constexpr uint16_t MAX_DELTA_X_Y = 140;

    //Auto test maximum delta for z register in mg/digit 
    static constexpr uint16_t MAX_DELTA_Z = 590;

    //Fixed value stored in whoami register, it's used to check proper sensor board communication
    static constexpr uint8_t WHO_AM_I_KEY = 0x3F;

    //FS_2_G conversion factor in mg/digit
    static constexpr float FS_2_G_CONV_FACTOR = 0.06f;

    //FS_4_G conversion factor in mg/digit
    static constexpr float FS_4_G_CONV_FACTOR = 0.12f;

    //FS_6_G conversion factor in mg/digit
    static constexpr float FS_6_G_CONV_FACTOR = 0.18f;

    //FS_8_G conversion factor in mg/digit
    static constexpr float FS_8_G_CONV_FACTOR = 0.24f;

    //FS_16_G conversion factor in mg/digit
    static constexpr float FS_16_G_CONV_FACTOR = 0.73f;

    //Controlled device, aka the sensor
    SPISlave spiSlave;

    //Initialization state of the sensor 
    bool isInit = false; 

    //Error logger 
    PrintLogger logger = Logging::getLogger("LIS3DSH");

    //Current output range 
    Range currentRange = FS_2_G; 

    //Current conversion factor
    float currentConversionFactor = FS_2_G_CONV_FACTOR;

    //Check correct board sensor communication
    bool checkWhoAmI();

    //Read value from a single axis in mg 
    float readSingleAxis(Register msBitsAdd, Register lsBitsAdd);

    //Check correct values reading as suggested in the datasheet 
    bool checkReadValue();

    //Private support method for range selection selectRange
    void setRange(Range scale, Mask setMask, float conversionFactor);



}; 
}//namespace Boardcore

/*
TO DO
Test and Debug
*/
