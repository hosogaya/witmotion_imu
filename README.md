# witmotion imu
Imu driver for IIC communication with jy901 and wt901 provided by witmotion. 

# Example Code
```cpp
#include <Arduino.h>
#include <witmotion_imu/i2c_interface.h>

#define WITMOTION_WIRE Wire2

witmotion_imu::I2cInterface imu_interface(&WITMOTION_WIRE, 0x50);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Please begin and set clock hz of the wire object in setup(). 
  WITMOTION_WIRE.begin();
  WITMOTION_WIRE.setClock(400e3);
  WITMOTION_WIRE.endTransmission();
}

std::array<float, 3> angle;
void loop() {
  // roll, pitch, yaw angles
  if (!imu_interface.getAngle(angle))
  {
    Serial.println("faild to read angle");
  }
  else 
  {
    for (const auto& a : angle)
    {
      Serial.print(a); Serial.print(", ");
    }
    Serial.println();
  }
  delay(100);
}
```

# Note
Please begin and set clock hz of the wire object in setup(). 

```cpp
void setup()
{
    Wire.begin();
    Wire.setClock(400e3);
}
```

# Reference 
* [wt901 datasheet](https://images-na.ssl-images-amazon.com/images/I/B11fVGszLsS.pdf)