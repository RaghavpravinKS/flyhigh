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

// Variables to keep track of the PID errors
float integral_roll = 0.0;
float integral_pitch = 0.0;
float integral_yaw = 0.0;
float derivative_roll = 0.0;
float derivative_pitch = 0.0;
float derivative_yaw = 0.0;

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
struct PID {
  float Kp;
  float Ki;
  float Kd;
};

// Struct to carry the data
struct Signal {
  byte height;
  byte frwrvr;
  byte rgtlft;
  byte yaw;
};

// Struct to carry the raw Accelerometer, Temperature and Gyroscope data
struct Gyro {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t temp;
  int16_t wx;
  int16_t wy;
  int16_t wz;
};

// Struct to carry processed data for the PID
struct procGyro{
  float Ax;
  float Ay;
  float Az;
  float Temp;
  float Wx;
  float Wy;
  float Wz;
};

// Struct to carry the Roll, Pitch and Yaw data
struct RPY {
  float roll;
  float pitch;
  float yaw;
};

// Struct to carry the reference Throttle, Roll, Pitch and Yaw data
struct REF_RPY {
  float throttle;
  float roll;
  float pitch;
  float yaw;
};

// Struct to carry the PID output
struct PIDoutput {
  float roll;
  float pitch;
  float yaw;
};

// Variabkes to keep track of the PID errors
float prev_error_roll = 0.0;
float prev_error_pitch = 0.0;
float prev_error_yaw = 0.0;

// Signal object to carry the data
Signal data;

// Gyro object to carry the raw Gyroscope data
Gyro gyro;

// RPY object to carry the Roll, Pitch and Yaw data
RPY rpy;

// PRY object to carry the reference Roll, Pitch and Yaw data
REF_RPY ref_rpy;

// procGyro object to carry the processed Gyroscope data
procGyro pGyro;

// PID object to carry the PID constants
PID pid_roll;
PID pid_pitch;
PID pid_yaw;

// PIDoutput object to carry the PID output
PIDoutput pidOutput;

// Variables to keep track of timestamps for RF communication
unsigned long currentRFTime = 0;
unsigned long previousRFTime = 0;

// Variables to keep track of timestamps for PID calculation
unsigned long currentPIDTime = 0;
unsigned long previousPIDTime = 0;

// Function to set initial values for the PID
void resetPID(PID &pid) {
  pid.Kp = 0.0;
  pid.Ki = 0.0;
  pid.Kd = 0.0;
}

// Function to get the Gyroscope data
void gyroData(Gyro &gyro) {
  Wire.beginTransmission(MPU_addr); // Start the transmission
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false); // End the transmission
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  gyro.ax = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  gyro.ay = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  gyro.az = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  gyro.temp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyro.wx = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyro.wy = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyro.wz = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Wire.endTransmission(true); // End the transmission
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
    previousRFTime = millis();
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
void motorControl(REF_RPY &ref_rpy, PIDoutput &pidOutput) {
  // Calculate the motor speeds
  int motorFrSpeed = ref_rpy.throttle + pidOutput.roll - pidOutput.pitch + pidOutput.yaw;
  int motorFlSpeed = ref_rpy.throttle - pidOutput.roll - pidOutput.pitch - pidOutput.yaw;
  int motorBrSpeed = ref_rpy.throttle - pidOutput.roll + pidOutput.pitch + pidOutput.yaw;
  int motorBlSpeed = ref_rpy.throttle + pidOutput.roll + pidOutput.pitch - pidOutput.yaw;

  // Map the motor speeds to 0-180
  motorFrSpeed = map(motorFrSpeed, 0, 22200, 0, 180);
  motorFlSpeed = map(motorFlSpeed, 0, 22200, 0, 180);
  motorBrSpeed = map(motorBrSpeed, 0, 22200, 0, 180);
  motorBlSpeed = map(motorBlSpeed, 0, 22200, 0, 180);

  // Write the motor speeds
  motorFr.write(motorFrSpeed);
  motorFl.write(motorFlSpeed);
  motorBr.write(motorBrSpeed);
  motorBl.write(motorBlSpeed);
}

// Function to implement the PID
float calcPID(float ref, PID &pid, float actual, float &prev_error, float &integral, float &derivative, float &dt) {

  // Calculate the error
  float error = ref - actual;

  // Calculate the integral
  integral = integral + error*dt;

  // Calculate the derivative
  derivative = (error - prev_error)/dt;

  // Calculate the PID output
  float output = pid.Kp * error + pid.Ki * integral + pid.Kd * derivative;

  // Update the previous error
  prev_error = error;

  return output;

}

// Function to calculate the reference input
void calcReferenceInput(Signal &signal, REF_RPY &rpy) {
  rpy.throttle = signal.height;
  rpy.roll = signal.frwrvr;
  rpy.pitch = signal.rgtlft ;
  rpy.yaw = signal.yaw;
}

// Function to calculate the input
void calcInput(Signal &signal, Gyro &gyro, PIDoutput &pidOutput) {

  // Calculate the time difference
  currentPIDTime = micros();
  float dt = (currentPIDTime - previousPIDTime)/1000000.0;
  previousPIDTime = currentPIDTime;

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
  //  Map rpy.roll from -90- 90 to 0-11100
  rpy.roll = map(rpy.roll, -90, 90, 0, 11100);
  //  Map rpy.pitch from -90- 90 to 0-11100
  rpy.pitch = map(rpy.pitch, -90, 90, 0, 11100);
  //  Map rpy.yaw from -90- 90 to 0-11100
  rpy.yaw = map(rpy.yaw, -90, 90, 0, 11100);


  // Calculate the reference input
  calcReferenceInput(signal, ref_rpy);
  
  // PID calculation
  pidOutput.roll = calcPID(ref_rpy.roll, pid_roll, rpy.roll, prev_error_roll, integral_roll, derivative_roll, dt);
  pidOutput.pitch = calcPID(ref_rpy.pitch, pid_pitch, rpy.pitch, prev_error_pitch, integral_pitch, derivative_pitch, dt);
  pidOutput.yaw = calcPID(ref_rpy.yaw, pid_yaw, rpy.yaw, prev_error_yaw, integral_yaw, derivative_yaw, dt);

  motorControl(ref_rpy, pidOutput);
  
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

  // Reset the PID
  resetPID(pid_roll);
  resetPID(pid_pitch);
  resetPID(pid_yaw);
  
  // Reset the PID errors
  // resetPIDError(pidError);
}

void loop() {
  // put your main code here, to run repeatedly:
  receiveData();
  currentRFTime = millis();
  if (currentRFTime - previousRFTime > 1000) {
    resetData(data); // Reset the data if no signal is received for 1 second
  }
  gyroData(gyro);
  calcInput(data, gyro, pidOutput);
}
