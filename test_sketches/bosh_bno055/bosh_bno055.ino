/*
 * ESP32 Test Sketch for Bosch BNO055 IMU Module
 * 
 * This sketch initializes the BNO055 IMU sensor and reads
 * acceleration, gyroscope, magnetometer, and orientation data.
 * 
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// BNO055 I2C address
#define BNO055_ADDRESS 0x28  // Default I2C address (can be 0x29 if ADR pin is HIGH)

// Create the BNO055 sensor object
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS);

void setup() {
  // Initialize serial communication at 115200 bps
  Serial.begin(500000);
  while (!Serial) delay(10);  // Wait for serial port to open

  Serial.println("ESP32 BNO055 IMU Test");
  Serial.println("---------------------");
  
  // Initialize I2C communication
  Wire.begin(18,19, 400000);

  // Initialize the BNO055 sensor
  if (!bno.begin()) {
    Serial.println("Failed to initialize BNO055! Check your connections.");
    while (1);
  }
  
  delay(1000);
  
  // Display sensor information
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       "); Serial.println(sensor.name);
  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");

  // Perform system calibration
  Serial.println("Please calibrate the sensor by moving it in a figure-8 pattern");
  Serial.println("Calibration status: 0=uncalibrated, 3=fully calibrated");
  
  // Wait a second for sensor to stabilize
  delay(1000);
}

unsigned int t = 0;
void loop() {

  Serial.print(millis()-t);
  t = millis();
  // Get calibration status (0-3 for each system)
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("  Calib  Sys=");
  Serial.print(system);
  // Serial.print(" Gyro=");
  // Serial.print(gyro);
  // Serial.print(" Accel=");
  // Serial.print(accel);
  // Serial.print(" Mag=");
  // Serial.print(mag);
  // Serial.print("  ");

  // Get and display the acceleration data
  // imu::Vector<3> accelData = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // Serial.print("Accel: X=");
  // Serial.print(accelData.x());
  // Serial.print(" Y=");
  // Serial.print(accelData.y());
  // Serial.print(" Z=");
  // Serial.print(accelData.z());
  // Serial.print(" m/s²  ");

  // Get and display the gyroscope data
  // imu::Vector<3> gyroData = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  // Serial.print("Gyro: X=");
  // Serial.print(gyroData.x());
  // Serial.print(" Y=");
  // Serial.print(gyroData.y());
  // Serial.print(" Z=");
  // Serial.print(gyroData.z());
  // Serial.print(" rad/s  ");

  // Get and display the magnetometer data
  // imu::Vector<3> magData = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  // Serial.print("Mag: X=");
  // Serial.print(magData.x());
  // Serial.print(" Y=");
  // Serial.print(magData.y());
  // Serial.print(" Z=");
  // Serial.print(magData.z());
  // Serial.print(" μT  ");

  // Get and display euler orientation data
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print("  X=");
  Serial.print(euler.x());
  Serial.print(" Y=");
  Serial.print(euler.y());
  Serial.print(" Z=");
  Serial.print(euler.z());
  Serial.println(" deg  ");
  // Get temperature
  // int8_t temp = bno.getTemp();
  // Serial.print(" Temp=");
  // Serial.print(temp);
  // Serial.println("°C");

}