#include <Servo.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include "MPU6050_res_define.h"
#include <math.h>

// Define the SPI object
SPIClass SPI(0);

// Define the I2C object
TwoWire Wire(0);
const int MPU_addr = 0x68;    // Default address for MPU-6050

// Address of the transmitter and receiver
const uint64_t pipeOut = 101001;

// receiver object and the CE and CSN pins
RF24 receiver(9, 10);

// Servo objects for four motors
Servo motorFr; // Front Right Motor
Servo motorFl; // Front Left Motor
Servo motorBr; // Back Right Motor
Servo motorBl; // Back Left Motor

// Pins for the motors (ESCs)
#define MOTOR_FR 0
#define MOTOR_FL 1
#define MOTOR_BR 2
#define MOTOR_BL 3

// PID constants
#define KP 1
#define KI 0
#define KD 0

// Struct to carry the data
struct Signal {
  byte height;
  byte frwrvr;
  byte rgtlft;
  byte yaw;
};

// Struct to carry the raw Accelerometer, Temperature and Gyroscope data
struct Gyro {
  float ax;
  float ay;
  float az;
  float temp;
  float wx;
  float wy;
  float wz;
};

// Struct to carry processed data for the PID
struct procGyro{
  int16_t Ax;
  int16_t Ay;
  int16_t Az;
  int16_t Temp;
  int16_t Wx;
  int16_t Wy;
  int16_t Wz;
}

// Struct to carry the Roll, Pitch and Yaw data
struct RPY {
  float roll;
  float pitch;
  float yaw;
};

// Signal object to carry the data
Signal data;

// Gyro object to carry the raw Gyroscope data
Gyro gyro;

// RPY object to carry the Roll, Pitch and Yaw data
RPY rpy;

// procGyro object to carry the processed Gyroscope data
procGyro pGyro;

// Variables to keep track of timestamps
unsigned long currentTime = 0;
unsigned long previousTime = 0;

// Function to get the Gyroscope data
Gyro gyroData(Gyro &gyro) {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  gyro.ax = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  gyro.ay = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  gyro.az = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  gyro.temp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyro.wx = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyro.wy = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyro.wz = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Wire.endTransmission(true);
}

// Function to reset the data
void resetData(Signal &signal) {
  signal.height = 0;
  signal.frwrvr = 0;
  signal.rgtlft = 0;
  signal.yaw = 0;
}

// Function to receive the data
void receiveData() {
  if (receiver.available()) {
    receiver.read(&data, sizeof(data));
    previousTime = millis();
    Serial.print("height: ");
    Serial.print(data.height);
    Serial.print(" frwrvr: ");
    Serial.print(data.frwrvr);
    Serial.print(" rgtlft: ");
    Serial.print(data.rgtlft);
    Serial.print(" Yaw: ");
    Serial.print(data.yaw);
  }
}

// Function to control the motors
void motorControl(Signal &signal) {
  
  int a=1000 + signal.height + signal.frwrvr + signal.rgtlft + signal.yaw;
  map(a, 1000, 2000, 0, 180);
  motorFr.write(a);
  motorFl.write(a);
  motorBr.write(a);
  motorBl.write(a);
}

// Function to implement PID and calculate the input
void calcInput(Signal &signal, Gyro &gyro) {
  // Process the Accelerometer, Temperatue and Gyroscope data
  pGyro.Ax = gyro.ax/16384.0; // Acceleration in X-axis in m/s^2, 0.04 is the offset
  pGyro.Ay = gyro.ay/16384.0; // Acceleration in Y-axis in m/s^2, 0.04 is the offset
  pGyro.Az = gyro.az/16384.0; // Acceleration in Z-axis in m/s^2, 0.04 is the offset
  pGyro.Temp = gyro.temp/340.0 + 36.53; // Temperature in degree Celsius
  pGyro.Wx = gyro.wx/131.0; // Angular velocity in X-axis in degrees per second
  pGyro.Wy = gyro.wy/131.0; // Angular velocity in Y-axis in degrees per second
  pGyro.Wz = gyro.wz/131.0; // Angular velocity in Z-axis in degrees per second

  // Calculate the Roll, Pitch and Yaw
  rpy.roll = atan2(pGyro.Ay, pGyro.Az) * 180 / M_PI;
  rpy.pitch = atan2(-pGyro.Ax, sqrt(pGyro.Ay * pGyro.Ay + pGyro.Az * pGyro.Az)) * 180 / M_PI;
  rpy.yaw = atan2(pGyro.Wz, sqrt(pGyro.Wx * pGyro.Wx + pGyro.Wy * pGyro.Wy)) * 180 / M_PI;
 
  // Implement the PID here
  

  motorControl(signal);
  
}

void setup() {
  // put your setup code here, to run once:
  // Start the serial communication
  Serial.begin(115200);
  // Start the I2C communication
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(50);
  Wire.beginTransmission(MPU_addr);
  // Attach the motors to the pins
  motorFr.attach(MOTOR_FR);
  motorFl.attach(MOTOR_FL);
  motorBr.attach(MOTOR_BR);
  motorBl.attach(MOTOR_BL);
  // Reset the data
  resetData(data);
  // Configure the receiver
  receiver.begin();
  // Hold in infinite loop to wait for the nRF24L01 module to be connected
  while (!receiver.begin()) {
    Serial.println("Receiver not started");
    delay(1000);
  }
  receiver.setChannel(100); // Set the channel
  receiver.setAutoAck(false); // Disable auto acknowledgment
  receiver.setDataRate(RF24_250KBPS); // Set the data rate
  receiver.setPALevel(RF24_PA_MAX); // Set the power level
  receiver.startListening(); // Start listening
}

void loop() {
  // put your main code here, to run repeatedly:
  receiveData();
  currentTime = millis();
  if (currentTime - previousTime > 1000) {
    resetData(data); // Reset the data if no signal is received for 1 second
  }
  gyroData(gyro);
  calcInput(data, gyro);
}
