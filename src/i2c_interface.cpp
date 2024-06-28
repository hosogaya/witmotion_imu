#include <witmotion_imu/i2c_interface.h>

namespace witmotion_imu
{
I2cInterface::I2cInterface(TwoWire* wire, uint8_t address)
: wire_(wire), dev_address_(address)
{}

I2cInterface::~I2cInterface() {}


uint16_t I2cInterface::getTime(const std::string& str)
{
    readRegisters(REG_YYMM, 8, (uint8_t*)(&time_));

    if (str == "year") return time_.year_;
    if (str == "month") return time_.month_;
    if (str == "day") return time_.day_;
    if (str == "hour") return time_.hour_;
    if (str == "minute") return time_.minute_;
    if (str == "second") return time_.second_;
    if (str == "milisecond") return time_.milisecond_;

    return 0;
}

/*************************
 *  Acceleration [g]
 **************************/

double I2cInterface::getAccX()
{
    readRegisters(REG_AX, 2, (uint8_t*)(&acc_.x_));

    return acc_.x_*kAccConversion_;
}

double I2cInterface::getAccY()
{
    readRegisters(REG_AY, 2, (uint8_t*)(&acc_.y_));

    return acc_.y_*kAccConversion_;
}

double I2cInterface::getAccZ()
{
    readRegisters(REG_AZ, 2, (uint8_t*)(&acc_.z_));

    return acc_.z_*kAccConversion_;
}

std::array<double, 3> I2cInterface::getAcc()
{
    readRegisters(REG_AX, 6, (uint8_t*)(&acc_));
    
    std::array<double, 3> acc{
        acc_.x_*kAccConversion_, 
        acc_.y_*kAccConversion_, 
        acc_.z_*kAccConversion_ 
    };

    return acc;
}


/*************************
 *  Gyro [rad/s]
 **************************/
double I2cInterface::getGyroX()
{
    readRegisters(REG_GX, 2, (uint8_t*)(&gyro_.x_));

    return gyro_.x_*kGyroConversion_;
}

double I2cInterface::getGyroY()
{
    readRegisters(REG_GY, 2, (uint8_t*)(&gyro_.y_));

    return gyro_.y_*kGyroConversion_;
}

double I2cInterface::getGyroZ()
{
    readRegisters(REG_GZ, 2, (uint8_t*)(&gyro_.z_));

    return gyro_.z_*kGyroConversion_;
}

std::array<double, 3> I2cInterface::getGyro()
{
    readRegisters(REG_GX, 6, (uint8_t*)(&gyro_));
    
    std::array<double, 3> gyro{
        gyro_.x_*kGyroConversion_, 
        gyro_.y_*kGyroConversion_, 
        gyro_.z_*kGyroConversion_ 
    };

    return gyro;
}

/*************************
 *  magnitude
 **************************/
double I2cInterface::getMagX()
{
    readRegisters(REG_HX, 2, (uint8_t*)(&mag_.x_));

    return mag_.x_*kMagConversion_;
}

double I2cInterface::getMagY()
{
    readRegisters(REG_HY, 2, (uint8_t*)(&mag_.y_));

    return mag_.y_*kMagConversion_;
}

double I2cInterface::getMagZ()
{
    readRegisters(REG_HZ, 2, (uint8_t*)(&mag_.z_));

    return mag_.z_*kMagConversion_;
}

std::array<double, 3> I2cInterface::getMag()
{
    readRegisters(REG_HX, 6, (uint8_t*)(&mag_));
    
    std::array<double, 3> mag{
        mag_.x_*kMagConversion_, 
        mag_.y_*kMagConversion_, 
        mag_.z_*kMagConversion_ 
    };

    return mag;
}

/*************************
 *  Euler [rad]
 **************************/
double I2cInterface::getRoll()
{
    readRegisters(REG_Roll, 2, (uint8_t*)(&angle_.roll_));

    return angle_.roll_*kAngleConversion_;;
}

double I2cInterface::getPitch()
{
    readRegisters(REG_Pitch, 2, (uint8_t*)(&angle_.pitch_));

    return angle_.pitch_*kAngleConversion_;
}

double I2cInterface::getYaw()
{
    readRegisters(REG_Yaw, 2, (uint8_t*)(&angle_.yaw_));

    return angle_.yaw_*kAngleConversion_;
}

std::array<double, 3> I2cInterface::getAngle()
{
    readRegisters(REG_Roll, 6, (uint8_t*)(&angle_));
    
    std::array<double, 3> angle{
        angle_.roll_*kAngleConversion_, 
        angle_.pitch_*kAngleConversion_, 
        angle_.yaw_*kAngleConversion_ 
    };

    return angle;
}

double I2cInterface::getTemperature()
{
    readRegisters(REG_TEMP, 2, (uint8_t*)(&temperature_));

    return temperature_*0.01; // C^o
}

int32_t I2cInterface::getPressure()
{
    readRegisters(REG_PressureL, 4, (uint8_t*)(&pressure_));
    pressure_ = reverse(pressure_); // reverse
    return pressure_; // Pa
}

int32_t I2cInterface::getAltitude()
{
    readRegisters(REG_HeightL, 4, (uint8_t*)(&altitude_));
    altitude_ = reverse(altitude_);

    return altitude_; // cm
}

/************************************
 *  D status
 ************************************/

int16_t I2cInterface::getD0Status()
{
    readRegisters(REG_D0Status, 2, (uint8_t*)(&d_status_.d0_));

    return d_status_.d0_;
}

int16_t I2cInterface::getD1Status()
{
    readRegisters(REG_D1Status, 2, (uint8_t*)(&d_status_.d1_));

    return d_status_.d1_;
}

int16_t I2cInterface::getD2Status()
{
    readRegisters(REG_D2Status, 2, (uint8_t*)(&d_status_.d2_));

    return d_status_.d2_;
}

int16_t I2cInterface::getD3Status()
{
    readRegisters(REG_D3Status, 2, (uint8_t*)(&d_status_.d3_));

    return d_status_.d3_;
}

int32_t I2cInterface::getLongitude()
{
    readRegisters(REG_LonL, 4, (uint8_t*)(&longitude_));
    longitude_ = reverse(longitude_);

    return longitude_; 
}

int32_t I2cInterface::getLatitude()
{
    readRegisters(REG_LatL, 4, (uint8_t*)(&latitude_));
    latitude_ = reverse(latitude_);

    return latitude_; 
}

double I2cInterface::getGPSH()
{
    readRegisters(REG_GPSHeight, 2, (uint8_t*)(&gps_.height_));
    return gps_.height_*0.1; // m
}

double I2cInterface::getGPSY()
{
    readRegisters(REG_GPSYAW, 2, (uint8_t*)(&gps_.yaw_));

    return gps_.yaw_*0.1; // deg
}

double I2cInterface::getGPSV()
{
    readRegisters(REG_GPSVL, 4, (uint8_t*)(&gps_.vel_));
    gps_.vel_ = reverse(gps_.vel_);
    return gps_.vel_*0.001; // km/h
}

std::array<double, 4> I2cInterface::getQuaternion()
{
    readRegisters(REG_Q0, 8, (uint8_t*)(&quat_));
    std::array<double, 4> quat{
        quat_.w_*kQuatConversion_,
        quat_.x_*kQuatConversion_,
        quat_.y_*kQuatConversion_,
        quat_.z_*kQuatConversion_
    };

    return quat;
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