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
    bool getTime(const std::string& str, uint16_t& time);
    bool getAccX(float& value);
    bool getAccY(float& value);
    bool getAccZ(float& value);
    bool getAcc(std::array<float, 3>& values);

    bool getGyroX(float& value);
    bool getGyroY(float& value);
    bool getGyroZ(float& value);
    bool getGyro(std::array<float, 3>& values);

    bool getMagX(float& value);
    bool getMagY(float& value);
    bool getMagZ(float& value);
    bool getMag(std::array<float, 3>& values);

    bool getRoll(float& value);
    bool getPitch(float& value);
    bool getYaw(float& value);
    bool getAngle(std::array<float, 3>& values);

    bool getTemperature(float& value);
    bool getPressure(int32_t& value);
    bool getAltitude(int32_t& value);
    bool getD0Status(int16_t& value);
    bool getD1Status(int16_t& value);
    bool getD2Status(int16_t& value);
    bool getD3Status(int16_t& value);
    bool getLongitude(int32_t& value);
    bool getLatitude(int32_t& value);
    bool getGPSH(float& value);
    bool getGPSY(float& value);
    bool getGPSV(float& value);

    // w, x, y, z
    bool getQuaternion(std::array<float, 4>& value);

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


    static constexpr float kAccConversion_ = 1.0/(32768.0/16.0);
    static constexpr float kGyroConversion_ = 1.0/(32768.0/2000.0)*(M_PI/180.0);
    static constexpr float kMagConversion_ = 1.0/(32768.0/180.0);
    static constexpr float kAngleConversion_ = 1.0/(32768.0/180.0)*(M_PI/180.0);
    static constexpr float kQuatConversion_ = 1.0/(32768.0);
};

}