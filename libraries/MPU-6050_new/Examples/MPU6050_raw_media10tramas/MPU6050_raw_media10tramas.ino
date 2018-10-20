#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

int16_t axm,aym,azm,gxm,gym,gzm; /*______*/

#define OUTPUT_READABLE_ACCELGYRO
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void loop() {

    axm=0;aym=0;azm=0;gxm=0;gym=0;gzm=0;
    
    for(int i=0; i<200;i++)
    {
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      axm+=ax;aym+=ay;azm+=az;gxm+=gx;gym+=gy;gzm+=gz;
    }
 

        Serial.print("a/g:\t");
        Serial.print(axm); Serial.print("\t");
        Serial.print(aym); Serial.print("\t");
        Serial.print(azm); Serial.print("\t");
        Serial.print(gxm); Serial.print("\t");
        Serial.print(gym); Serial.print("\t");
        Serial.println(gzm);

}
