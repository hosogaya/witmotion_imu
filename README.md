# witmotion imu
Imu driver for IIC communication with wt901 provided by witmotion. 

# Note
Please begin and set clock hz of the wire object in setup(). 

```cpp
void setup()
{
    Wire.begin();
    Wire.setClock(400e3);
}
```

# Refrence 
* [wt901 datasheet](https://images-na.ssl-images-amazon.com/images/I/B11fVGszLsS.pdf)