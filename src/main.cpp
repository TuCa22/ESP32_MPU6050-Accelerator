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
// float Accel_X, Accel_Y, Accel_Z;
float Gyro_X, Gyro_Y, Gyro_Z, Accel_X, Accel_Y, Accel_Z;
float acc_X, acc_Y, acc_Z;
float roll, pitch, yaw, froll, fpitch, fyaw;

float elapsedTime, currentTime = 0, previousTime;
float dt = 0.015; // 15 ms
bool filter = false;

void gyroFilter();

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
        gyroFilter();
        filter = true;
    }

    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = currentTime - previousTime;
    dt = elapsedTime / 1000; // Convert milliseconds to seconds

    Wire.beginTransmission(MPU);
    Wire.write(GYRO_XOUTH);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    Accel_X = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;   // Full-scale Range: 2g || LSB Sensity: Accel/16384 = g
    Accel_Y = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
    Accel_Z = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;

    Gyro_X = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;      // Full-scale Range: 250 deg/s || LSB Sensity: Gyro/131 = deg/s
    Gyro_Y = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
    Gyro_Z = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;

    acc_X = Accel_X * 9.81; // Convert to m/s^2
    acc_Y = Accel_Y * 9.81;
    acc_Z = Accel_Z * 9.81;

    roll += (Gyro_X - froll) * dt;                          // Calculating the angle
    pitch += (Gyro_Y - fpitch) * dt;
    yaw += (Gyro_Z - fyaw) * dt;
    Serial.print(">Roll: "); Serial.println(roll);
    Serial.print(">Pitch: "); Serial.println(pitch);
    Serial.print(">Yaw: "); Serial.println(yaw);                
}

void gyroFilter()
{
    Serial.println("Starting Gyro Filter...");
    delay(500);
    for (int i = 0; i < 10; i++)
    {
        Serial.print("*");
        delay(100);
    }
    Serial.println("*");

    for (int x = 0; x < 1000; x++)
    {
        Wire.beginTransmission(MPU);
        Wire.write(GYRO_XOUTH);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true);
        Gyro_X = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
        Gyro_Y = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
        Gyro_Z = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;

        froll += Gyro_X;                          // Simple low-pass filter
        fpitch += Gyro_Y;
        fyaw += Gyro_Z;
    }

    froll /= 1000;
    fpitch /= 1000;
    fyaw /= 1000;
    Serial.println("Gyro Filter Completed!");
    Serial.print("froll: "); Serial.print(froll);
    Serial.print(" | fpitch: "); Serial.print(fpitch);
    Serial.print(" | fyaw: "); Serial.println(fyaw);
}