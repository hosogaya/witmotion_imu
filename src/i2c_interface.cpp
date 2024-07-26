#include <witmotion_imu/i2c_interface.h>

namespace witmotion_imu
{
I2cInterface::I2cInterface(TwoWire* wire, uint8_t address)
: wire_(wire), dev_address_(address)
{}

I2cInterface::~I2cInterface() {}


bool I2cInterface::getTime(const std::string& str, uint16_t& time)
{
    if (!readRegisters(REG_YYMM, 8, (uint8_t*)(&time_))) return false;

    if (str == "year") time = time_.year_;
    if (str == "month") time = time_.month_;
    if (str == "day") time = time_.day_;
    if (str == "hour") time = time_.hour_;
    if (str == "minute") time = time_.minute_;
    if (str == "second") time = time_.second_;
    if (str == "milisecond") time = time_.milisecond_;

    return true;
}

/*************************
 *  Acceleration [g]
 **************************/

bool I2cInterface::getAccX(float& value)
{
    if (!readRegisters(REG_AX, 2, (uint8_t*)(&acc_.x_))) return false;
    value = acc_.x_*kAccConversion_;

    return true;
}

bool I2cInterface::getAccY(float& value)
{
    if (!readRegisters(REG_AY, 2, (uint8_t*)(&acc_.y_))) return false;
    value = acc_.y_*kAccConversion_;
    return true;
}

bool I2cInterface::getAccZ(float& value)
{
    if (!readRegisters(REG_AZ, 2, (uint8_t*)(&acc_.z_))) return false;
    value = acc_.z_*kAccConversion_;
    
    return true;
}

bool I2cInterface::getAcc(std::array<float, 3>& values)
{
    if (!readRegisters(REG_AX, 6, (uint8_t*)(&acc_))) return false;    
    
    values[0] = acc_.x_*kAccConversion_; 
    values[1] = acc_.y_*kAccConversion_; 
    values[2] = acc_.z_*kAccConversion_;

    return true;
}


/*************************
 *  Gyro [rad/s]
 **************************/
bool I2cInterface::getGyroX(float& value)
{
    if (!readRegisters(REG_GX, 2, (uint8_t*)(&gyro_.x_))) return false;
    value = gyro_.x_*kGyroConversion_;

    return true;
}

bool I2cInterface::getGyroY(float& value)
{
    if (!readRegisters(REG_GY, 2, (uint8_t*)(&gyro_.y_))) return false;
    value = gyro_.y_*kGyroConversion_;

    return true;
}

bool I2cInterface::getGyroZ(float& value)
{
    if (!readRegisters(REG_GZ, 2, (uint8_t*)(&gyro_.z_))) return false;
    value = gyro_.z_*kGyroConversion_;

    return true;
}

bool I2cInterface::getGyro(std::array<float, 3>& values)
{
    if (!readRegisters(REG_GX, 6, (uint8_t*)(&gyro_))) return false;
    
    values[0] = gyro_.x_*kGyroConversion_; 
    values[1] = gyro_.y_*kGyroConversion_; 
    values[2] = gyro_.z_*kGyroConversion_;

    return true;
}

/*************************
 *  magnitude
 **************************/
bool I2cInterface::getMagX(float& value)
{
    if (!readRegisters(REG_HX, 2, (uint8_t*)(&mag_.x_))) return false;
    value = mag_.x_*kMagConversion_;

    return true;
}

bool I2cInterface::getMagY(float& value)
{
    if (!readRegisters(REG_HY, 2, (uint8_t*)(&mag_.y_))) return false;
    value = mag_.y_*kMagConversion_;

    return true;
}

bool I2cInterface::getMagZ(float& value)
{
    if (!readRegisters(REG_HZ, 2, (uint8_t*)(&mag_.z_))) return false;
    value = mag_.z_*kMagConversion_;

    return true;
}

bool I2cInterface::getMag(std::array<float, 3>& values)
{
    if (!readRegisters(REG_HX, 6, (uint8_t*)(&mag_))) return false;
    
    values[0] = mag_.x_*kMagConversion_; 
    values[1] = mag_.y_*kMagConversion_; 
    values[2] = mag_.z_*kMagConversion_;

    return true;
}

/*************************
 *  Euler [rad]
 **************************/
bool I2cInterface::getRoll(float& value)
{
    if (!readRegisters(REG_Roll, 2, (uint8_t*)(&angle_.roll_))) return false;
    value = angle_.roll_*kAngleConversion_;;
}

bool I2cInterface::getPitch(float& value)
{
    if (!readRegisters(REG_Pitch, 2, (uint8_t*)(&angle_.pitch_))) return false;
    value = angle_.pitch_*kAngleConversion_;

    return true;
}

bool I2cInterface::getYaw(float& value)
{
    if (!readRegisters(REG_Yaw, 2, (uint8_t*)(&angle_.yaw_))) return false;
    value = angle_.yaw_*kAngleConversion_;

    return true;
}

bool I2cInterface::getAngle(std::array<float, 3>& values)
{
    if (!readRegisters(REG_Roll, 6, (uint8_t*)(&angle_))) return false;
    
    values[0] = angle_.roll_*kAngleConversion_;
    values[1] = angle_.pitch_*kAngleConversion_; 
    values[2] = angle_.yaw_*kAngleConversion_;

    return true;
}

bool I2cInterface::getTemperature(float& value)
{
    if (!readRegisters(REG_TEMP, 2, (uint8_t*)(&temperature_))) return false;
    value = temperature_*0.01; // C^o

    return true;
}

bool I2cInterface::getPressure(int32_t& value)
{
    if (!readRegisters(REG_PressureL, 4, (uint8_t*)(&pressure_))) return false;
    value = reverse(pressure_); // Pa
    
    return true;
}

bool I2cInterface::getAltitude(int32_t& value)
{
    if (!readRegisters(REG_HeightL, 4, (uint8_t*)(&altitude_))) return false;
    value = reverse(altitude_); // cm

    return true;
}

/************************************
 *  D status
 ************************************/

bool I2cInterface::getD0Status(int16_t& value)
{
    if (!readRegisters(REG_D0Status, 2, (uint8_t*)(&d_status_.d0_))) return false;
    value = d_status_.d0_;

    return true;
}

bool I2cInterface::getD1Status(int16_t& value)
{
    if (!readRegisters(REG_D1Status, 2, (uint8_t*)(&d_status_.d1_))) return false;
    value = d_status_.d1_;

    return true;
}

bool I2cInterface::getD2Status(int16_t& value)
{
    if (!readRegisters(REG_D2Status, 2, (uint8_t*)(&d_status_.d2_))) return false;
    value = d_status_.d2_;
    
    return true;
}

bool I2cInterface::getD3Status(int16_t& value)
{
    if (!readRegisters(REG_D3Status, 2, (uint8_t*)(&d_status_.d3_))) return false;
    value = d_status_.d3_;
    
    return true;
}

bool I2cInterface::getLongitude(int32_t& value)
{
    if (!readRegisters(REG_LonL, 4, (uint8_t*)(&longitude_))) return false;
    value = reverse(longitude_);

    return true; 
}

bool I2cInterface::getLatitude(int32_t& value)
{
    if (!readRegisters(REG_LatL, 4, (uint8_t*)(&latitude_))) return false;
    value = reverse(latitude_);

    return true; 
}

bool I2cInterface::getGPSH(float& value)
{
    if (!readRegisters(REG_GPSHeight, 2, (uint8_t*)(&gps_.height_))) return false;
    value = gps_.height_*0.1; // m

    return true;
}

bool I2cInterface::getGPSY(float& value)
{
    if (!readRegisters(REG_GPSYAW, 2, (uint8_t*)(&gps_.yaw_))) return false;
    value = gps_.yaw_*0.1; // deg

    return true;
}

bool I2cInterface::getGPSV(float& value)
{
    if (!readRegisters(REG_GPSVL, 4, (uint8_t*)(&gps_.vel_))) return false;
    gps_.vel_ = reverse(gps_.vel_);
    value = gps_.vel_*0.001; // km/h

    return true;
}

bool I2cInterface::getQuaternion(std::array<float, 4>& values)
{
    if (!readRegisters(REG_Q0, 8, (uint8_t*)(&quat_))) return false;
    values[0] = quat_.w_*kQuatConversion_;
    values[1] = quat_.x_*kQuatConversion_;
    values[2] = quat_.y_*kQuatConversion_;
    values[3] = quat_.z_*kQuatConversion_;
    return true;
}


bool I2cInterface::readRegisters(
    uint8_t start_register, 
    uint8_t read_size, 
    uint8_t* res
)
{
    wire_->beginTransmission(dev_address_);
    wire_->write(start_register);
    wire_->endTransmission(false);

    wire_->requestFrom(dev_address_, read_size);

    if (wire_->available() < read_size) return false;

    wire_->readBytes(res, read_size);
    return true;
}

void I2cInterface::saveConfig()
{
    uint8_t cmd[2] = {0x00, 0x00};
    writeRegisters(REG_SAVE, 2, cmd);
}

void I2cInterface::quitCalibration()
{
    uint8_t cmd[2] = {0x00, 0x00};
    writeRegisters(REG_CALSW, 2, cmd);
}

void I2cInterface::calibrateImu()
{
    uint8_t cmd[2] = {0x01, 0x00};
    writeRegisters(REG_CALSW, 2, cmd);
}

void I2cInterface::calibrateMag()
{
    uint8_t cmd[2] = {0x02, 0x00};
    writeRegisters(REG_CALSW, 2, cmd);
}

void I2cInterface::writeRegisters(
    uint8_t start_register, 
    uint8_t write_size, 
    uint8_t* data
)
{
    wire_->beginTransmission(dev_address_);
    wire_->write(start_register);
    wire_->write(data, write_size);
    wire_->endTransmission();
}

}