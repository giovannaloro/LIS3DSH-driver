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
    sampledData.accelerationX = readSingleAxis(OUT_X_H_ADD, OUT_X_L_ADD);
    sampledData.accelerationY = readSingleAxis(OUT_Y_H_ADD, OUT_Y_L_ADD);
    sampledData.accelerationZ = readSingleAxis(OUT_Z_H_ADD, OUT_Z_L_ADD);
    return sampledData; 
}

bool LIS3DSH::selectRange(Range selectedRange){
    if(!isInit){
        LOG_ERR(logger, "Sensor not initialized");
        lastError = SensorErrors::NOT_INIT;
        return false;
    }

    SPITransaction spi(spiSlave);

    switch(selectedRange)
    {
        case FS_2_G:
            setRange(FS_2_G, SET_2_G_SCALE, FS_2_G_CONV_FACTOR);
            break;

        case FS_4_G:
            setRange(FS_4_G, SET_4_G_SCALE, FS_4_G_CONV_FACTOR);
            break;

        case FS_6_G:
            setRange(FS_6_G, SET_6_G_SCALE, FS_6_G_CONV_FACTOR);
            break;

        case FS_8_G:
            setRange(FS_8_G, SET_8_G_SCALE, FS_8_G_CONV_FACTOR);
            break;

        case FS_16_G:
            setRange(FS_16_G, SET_16_G_SCALE, FS_16_G_CONV_FACTOR);
            break;
    }
    return true;
}

bool LIS3DSH::checkWhoAmI(){
    SPITransaction spi(spiSlave);
    if(spi.readRegister(WHO_AM_I_ADD) == WHO_AM_I_KEY){
        return true;
    }
    return false;
}

float LIS3DSH::readSingleAxis(Register msBitsAdd, Register lsBitsAdd){
    SPITransaction spi(spiSlave);
    uint16_t bitContainer = 0;
    bitContainer = (uint16_t) spi.readRegister(msBitsAdd)<<8 | spi.readRegister(lsBitsAdd);
    int16_t xOutput = static_cast<int16_t>(bitContainer);
    return (float) xOutput * currentConversionFactor;
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
    xReadFirst = readSingleAxis(OUT_X_H_ADD, OUT_X_L_ADD);
    yReadFirst = readSingleAxis(OUT_Y_H_ADD, OUT_Y_L_ADD);
    zReadFirst = readSingleAxis(OUT_Z_H_ADD, OUT_Z_L_ADD);
    //set the sensor in test mode and the scale to the proper one 
    Range previousRange = currentRange;
    spi.writeRegister(CTRL5_ADD, spi.readRegister(CTRL5_ADD) | ENABLE_TEST_MODE); 
    selectRange(FS_2_G);
    //read values after sensor test  mode activation
    xReadSecond = readSingleAxis(OUT_X_H_ADD, OUT_X_L_ADD);
    yReadSecond = readSingleAxis(OUT_Y_H_ADD, OUT_Y_L_ADD);
    zReadSecond = readSingleAxis(OUT_Z_H_ADD, OUT_Z_L_ADD);
    //reset sensor in normal mode and reselect the current scale
    spi.writeRegister(CTRL5_ADD, spi.readRegister(CTRL5_ADD) & ~DISABLE_TEST_MODE); 
    selectRange(previousRange);
    //compute deltas and check correct functioning 
    return (xReadFirst - xReadSecond < MAX_DELTA_X_Y) && (yReadFirst - yReadSecond < MAX_DELTA_X_Y) && (zReadFirst - zReadSecond < MAX_DELTA_Z);
}

void LIS3DSH::setRange(Range range, Mask rangeMask, float conversionFactor){
    SPITransaction spi(spiSlave);
    currentRange = range;
    currentConversionFactor = conversionFactor;
    spi.writeRegister(CTRL5_ADD, spi.readRegister(CTRL5_ADD) & ~CLEAN_FS);
    spi.writeRegister(CTRL5_ADD, spi.readRegister(CTRL5_ADD) | rangeMask);
}

}//namespace Boardcore

