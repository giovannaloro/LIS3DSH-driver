#include"LIS3DSH.h"
#include<miosix.h>

namespace Boardcore{

LIS3DSH::LIS3DSH(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig spiConfig) : spiSlave(bus, cs, spiConfig) {} 

LIS3DSH::LIS3DSH(SPIBusInterface& bus, miosix::GpioPin cs ) : LIS3DSH(bus, cs, SPIBusConfig{}) {
    spiSlave.config.clockDivider  = SPI::ClockDivider::DIV_16;
}

bool LIS3DSH::init(){
    if(isInit){
        LOG_ERR(logger, "Sensor already init");
        lastError = SensorErrors::ALREADY_INIT;
        return true;
    }

    if(!checkWhoAmI()){
        LOG_ERR(logger, "Read wrong whoAmI");
        lastError = SensorErrors::INVALID_WHOAMI;
        return false;
    }

    if(checkReadValue()){
        LOG_ERR(logger, "Self test failed");
        lastError = SensorErrors::SELF_TEST_FAIL;
        return false;
    }

    return isInit = true;
}

bool LIS3DSH::selfTest(){
    isInit = false;
    if(!checkReadValue()){
        LOG_ERR(logger, "Self test failed");
        lastError = SensorErrors::SELF_TEST_FAIL;
        return false;
    }
    return isInit = true;
}

AccelerometerData LIS3DSH::sampleImpl(){
    if(!isInit){
        LOG_ERR(logger, "Self test failed");
        lastError = SensorErrors::NOT_INIT;
    }

    AccelerometerData sampledData;
    sampledData.accelerationX = readX();
    sampledData.accelerationY = readY();
    sampledData.accelerationZ = readZ();
    return sampledData; 
}

bool LIS3DSH::checkWhoAmI(){
    SPITransaction spi(spiSlave);
    if(spi.readRegister(WHO_AM_I_ADD) == WHO_AM_I_KEY){
        return true;
    }
    return false;
}

bool LIS3DSH::setRange(Range selectedRange){
    if(!isInit){
        LOG_ERR(logger, "Sensor not initialized");
        lastError = SensorErrors::NOT_INIT;
        return false;
    }
    SPITransaction spi(spiSlave);
    switch(selectedRange)
    {
        case FS_2_G:
            currentScale = FS_2_G;
            currentConversionFactor = FS_2_G_CONV_FACTOR;
            spi.writeRegister(CTRL5_ADD, spi.readRegister(CTRL5_ADD) ^ SET_2_G_SCALE);
            break;

        case FS_4_G:
            currentScale = FS_4_G;
            currentConversionFactor = FS_4_G_CONV_FACTOR;
            spi.writeRegister(CTRL5_ADD, spi.readRegister(CTRL5_ADD) ^ SET_4_G_SCALE);
            break;

        case FS_6_G:
            currentScale = FS_6_G;
            currentConversionFactor = FS_6_G_CONV_FACTOR;
            spi.writeRegister(CTRL5_ADD, spi.readRegister(CTRL5_ADD) ^ SET_6_G_SCALE);
            break;

        case FS_8_G:
            currentScale = FS_8_G;
            currentConversionFactor = FS_8_G_CONV_FACTOR;
            spi.writeRegister(CTRL5_ADD, spi.readRegister(CTRL5_ADD) ^ SET_8_G_SCALE);
            break;

        case FS_16_G:
            currentScale = FS_16_G;
            currentConversionFactor = FS_16_G_CONV_FACTOR;
            spi.writeRegister(CTRL5_ADD, spi.readRegister(CTRL5_ADD) ^ SET_16_G_SCALE);
            break;
    }
    return true;
}


float LIS3DSH::readSingleAxis(Register msBitsAdd, Register lsBitsAdd){
    SPITransaction spi(spiSlave);
    uint16_t bitContainer = 0;
    bitContainer = (uint16_t) spi.readRegister(msBitsAdd)<<8 | spi.readRegister(lsBitsAdd);
    int16_t xOutput = static_cast<int16_t>(bitContainer);
    return (float) xOutput * currentConversionFactor;
    }

float LIS3DSH::readX(){
    return readSingleAxis(OUT_X_H_ADD, OUT_X_L_ADD);
}

float LIS3DSH::readY(){
    return readSingleAxis(OUT_Y_H_ADD, OUT_Y_L_ADD);
}

float LIS3DSH::readZ(){
    return readSingleAxis(OUT_Z_H_ADD, OUT_Z_L_ADD);
}

bool LIS3DSH::checkReadValue(){
    SPITransaction spi(spiSlave);
    float xReadFirst;
    float yReadFirst;
    float zReadFirst;
    float xReadSecond;
    float yReadSecond;
    float zReadSecond;
    //read values before setting the sensor in test mode
    xReadFirst = readX();
    yReadFirst = readY();
    zReadFirst = readZ();
    //set the sensor in test mode and the scale to the proper one 
    spi.writeRegister(CTRL5_ADD, spi.readRegister(CTRL5_ADD) ^ ENABLE_TEST_MODE); 
    spi.writeRegister(CTRL5_ADD, spi.readRegister(CTRL5_ADD) ^ SET_2_G_SCALE);
    //read values after sensor mode activation
    xReadSecond = readX();
    yReadSecond = readY();
    zReadSecond = readZ();
    //reset sensor in normal mode and reselect the current scale
    spi.writeRegister(CTRL5_ADD, spi.readRegister(CTRL5_ADD) ^ DISABLE_TEST_MODE); 
    spi.writeRegister(CTRL5_ADD, spi.readRegister(CTRL5_ADD) ^ currentScale);
    //compute deltas and check correct functioning 
    return (xReadFirst - xReadSecond < MAX_DELTA_X_Y) && (yReadFirst - yReadSecond < MAX_DELTA_X_Y) && (zReadFirst - zReadSecond < MAX_DELTA_Z);
}
}

