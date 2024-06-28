#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <witmotion_imu/wt901_register.h>
#include <array>
#include <string>

namespace witmotion_imu
{
class I2cInterface
{
public:
    I2cInterface(TwoWire* wire, uint8_t address);
    ~I2cInterface();
    
    // read
    uint16_t getTime(const std::string& str);
    double getAccX();
    double getAccY();
    double getAccZ();
    std::array<double, 3> getAcc();

    double getGyroX();
    double getGyroY();
    double getGyroZ();
    std::array<double, 3> getGyro();

    double getMagX();
    double getMagY();
    double getMagZ();
    std::array<double, 3> getMag();

    double getRoll();
    double getPitch();
    double getYaw();
    std::array<double, 3> getAngle();

    double getTemperature();
    int32_t getPressure();
    int32_t getAltitude();
    int16_t getD0Status();
    int16_t getD1Status();
    int16_t getD2Status();
    int16_t getD3Status();
    int32_t getLongitude();
    int32_t getLatitude();
    double getGPSH();
    double getGPSY();
    double getGPSV();

    // w, x, y, z
    std::array<double, 4> getQuaternion();

    // write
    void saveConfig();
    void quitCalibration();
    void calibrateImu();
    void calibrateMag();

private:
    bool readRegisters(uint8_t start_register, 
                    uint8_t read_size, 
                    uint8_t* res);
    void writeRegisters(uint8_t start_register, 
                        uint8_t write_size, 
                        uint8_t* data);
    int32_t reverse(int32_t var)
    {
        return ((var&0xFFFF) << 16) | (var >> 16);
    }

    TwoWire* wire_;
    const uint8_t dev_address_;

    // buffers
    struct 
    {
        uint8_t year_;
        uint8_t month_;
        uint8_t day_;
        uint8_t hour_;
        uint8_t minute_;
        uint8_t second_;
        uint16_t milisecond_;
    } time_;

    struct 
    {
        int16_t x_;
        int16_t y_;
        int16_t z_;
    } acc_;

    struct 
    {
        int16_t x_;
        int16_t y_;
        int16_t z_;
    } gyro_;

    struct 
    {
        int16_t x_;
        int16_t y_;
        int16_t z_;
    } mag_;

    struct 
    {
        int16_t roll_;
        int16_t pitch_;
        int16_t yaw_;
    } angle_;
    
    int16_t temperature_;

    struct
    {
        int16_t d0_;
        int16_t d1_;
        int16_t d2_;
        int16_t d3_;
    } d_status_;

    int32_t pressure_;
    int32_t altitude_;
    int32_t longitude_;
    int32_t latitude_;

    struct 
    {
        int16_t height_;
        int16_t yaw_;
        int32_t vel_;
    } gps_;

    struct 
    {   
        int16_t w_;
        int16_t x_;
        int16_t y_;
        int16_t z_;
    } quat_;


    static constexpr double kAccConversion_ = 1.0/(32768.0/16.0);
    static constexpr double kGyroConversion_ = 1.0/(32768.0/2000.0)*(M_PI/180.0);
    static constexpr double kMagConversion_ = 1.0/(32768.0/180.0);
    static constexpr double kAngleConversion_ = 1.0/(32768.0/180.0)*(M_PI/180.0);
    static constexpr double kQuatConversion_ = 1.0/(32768.0);
};

}