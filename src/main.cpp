#include <Arduino.h>
#include <Wire.h>

#define ACCEL_XOUTH 0x3B
#define ACCEL_XOUTL 0x3C
#define ACCEL_YOUTH 0x3D
#define ACCEL_YOUTL 0x3E
#define ACCEL_ZOUTH 0x3F
#define ACCEL_ZOUTL 0x40

#define GYRO_XOUTH 0x43
#define GYRO_XOUTL 0x44
#define GYRO_YOUTH 0x45
#define GYRO_YOUTL 0x46
#define GYRO_ZOUTH 0x47
#define GYRO_ZOUTL 0x48

const int MPU = 0x68; // MPU-6050 I2C address
float Accel_X, Accel_Y, Accel_Z;
float Gyro_X, Gyro_Y, Gyro_Z;
float acc_X, acc_Y, acc_Z;
float roll, pitch, yaw;
float offsetAX, offsetAY, offsetAZ;
float offsetGX, offsetGY, offsetGZ;

float elapsedTime, currentTime = 0, previousTime;
float dt = 0.015; // 15 ms
bool filter = false;

void mpuCalibration();

void setup()
{
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);
    Serial.begin(115200);
}

void loop()
{
    if (filter == false)
    {
        mpuCalibration();
        filter = true;
    }

    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = currentTime - previousTime;
    dt = elapsedTime / 1000; // Convert milliseconds to seconds

    Wire.beginTransmission(MPU);
    Wire.write(ACCEL_XOUTH);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 14, true);

    Accel_X = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;   // Full-scale Range: 2g || LSB Sensity: Accel/16384 = g
    Accel_Y = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
    Accel_Z = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;

    Wire.read(); // Skip temperature data
    Wire.read();

    Gyro_X = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;      // Full-scale Range: 250 deg/s || LSB Sensity: Gyro/131 = deg/s
    Gyro_Y = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
    Gyro_Z = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;

    acc_X = (Accel_X - offsetAX) * 9.81; // Convert to m/s^2
    acc_Y = (Accel_Y - offsetAY) * 9.81;
    acc_Z = (Accel_Z - offsetAZ) * 9.81;

    roll += (Gyro_X - offsetGX) * dt;                          // Calculating the angle
    pitch += (Gyro_Y - offsetGY) * dt;
    yaw += (Gyro_Z - offsetGZ) * dt;

    Serial.print(">Accel_X: "); Serial.println(acc_X);
    Serial.print(">Accel_Y: "); Serial.println(acc_Y);
    Serial.print(">Accel_Z: "); Serial.println(acc_Z);
    
    Serial.print("Roll: "); Serial.println(roll);
    Serial.print("Pitch: "); Serial.println(pitch);
    Serial.print("Yaw: "); Serial.println(yaw);             
}

void mpuCalibration() {
    float sumAX = 0, sumAY = 0, sumAZ = 0;
    float sumGX = 0, sumGY = 0, sumGZ = 0;
    int samples = 1000;

    Serial.println("Calibrating...Please do not move the device!");

    for (int i = 0; i < samples; i++)
    {
        Wire.beginTransmission(MPU);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 14, true);

        sumAX += (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
        sumAY += (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
        sumAZ += (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
        
        Wire.read(); Wire.read();

        sumGX += (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
        sumGY += (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
        sumGZ += (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
        
        if (i % 100 == 0) Serial.print("."); 
    }

    offsetAX = sumAX / samples;
    offsetAY = sumAY / samples;
    offsetAZ = (sumAZ / samples) - 1.0;

    offsetGX = sumGX / samples;
    offsetGY = sumGY / samples;
    offsetGZ = sumGZ / samples;

    Serial.println("\nCalibration completed!");
}