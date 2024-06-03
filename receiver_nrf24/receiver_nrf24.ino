/*
  @file receiver_nrf24.ino
  @brief Interfacing NRF24L01 with ARIES Board using SPI.
  @detail The demo to receive the data from transmitter controller using NRF24L01

   Aries V2 pinout: https://vegaprocessors.in/blog/interfacing-8x8-led-dot-matrix-to-aries-v2-board/
   NRF24L01 pi88nout: https://howtomechatronics.com/wp-content/uploads/2017/02/NRF24L01-Pinout-NRF24L01-PA-LNA-.png

   * Library Name : RF24 (by TMRH20, Avamander)

   *** A Radio Transceiver Module (NRF24L01+PA/LNA) ***

   Connections:
   NRF24L01      Aries Board
   VCC          -   3.3V
   GND          -   GND
   CE           -   GPIO-9
   CNS          -   GPIO-10
   SCK          -   SCLK0
   MISO         -   MISO0
   MOSI         -   MOSI0
*/

#include <Servo.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include "MPU6050_res_define.h"
#include <math.h>

SPIClass SPI(0);

// Define the I2C object
TwoWire Wire(1);
const int mpu6050 = 0x68;    // Default address for MPU-6050

#define TRUE 1
#define FALSE 0

RF24 receiver(9, 10); // CE, CSN
const byte address[6] = "101001";
// boolean button_state = 0;
// int led_pin = 23;

Servo bldc;

struct Gyro {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t temp;
  int16_t wx;
  int16_t wy;
  int16_t wz;
};

struct procGyro{
  float Ax;
  float Ay;
  float Az;
  float Temp;
  float Wx;
  float Wy;
  float Wz;
};


struct PID {
  float Kp;
  float Ki;
  float Kd;
};

struct Signal {
  byte height;
  byte frwrvr;
  byte rgtlft;
  byte yaw;
};

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


float integral_roll = 0.0;
float integral_pitch = 0.0;
float integral_yaw = 0.0;
float derivative_roll = 0.0;
float derivative_pitch = 0.0;
float derivative_yaw = 0.0;


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

struct rate{
  float roll;
  float pitch;
  float yaw;
};

struct caliberation{
  float roll;
  float pitch;
  float yaw;

};


// void update_rates(rate &rate, caliberation &caliberation){
//       rate.roll-=caliberation.roll;
//       rate.pitch-=caliberation.pitch;
//       rate.yaw-=caliberation.yaw;
// }




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

void setconst(){
pid_roll.Kp=1;
pid_roll.Ki=1;
pid_roll.Kd=1;

pid_pitch.Kp=1;
pid_pitch.Ki=1;
pid_pitch.Kd=1;

pid_yaw.Kp=1;
pid_yaw.Ki=1;
pid_yaw.Kd=1;
}
// Carrying caliberation values

caliberation CALIBER;

// Carrying rates
rate Rate;


// PIDoutput object to carry the PID output
PIDoutput pidOutput;

unsigned long currentRFTime = 0;
unsigned long previousRFTime = 0;

// Variables to keep track of timestamps for PID calculation
unsigned long currentPIDTime = 0;
unsigned long previousPIDTime = 0;

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

void gyroData(Gyro &gyro) {
  Wire.beginTransmission(mpu6050); // Start the transmission
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false); // End the transmission
  Wire.requestFrom(mpu6050, 14, true); // request a total of 14 registers
  gyro.ax = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  gyro.ay = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  gyro.az = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  gyro.temp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyro.wx = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyro.wy = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyro.wz = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Wire.endTransmission(true); // End the transmission
}


void calib (rate &rate,caliberation &caliberation)
  {

    for ( int iterations=0;iterations<3000;iterations++){
      gyroData(gyro);

  rate.roll = gyro.wx/131; // Angular velocity in X-axis in degrees per second
  rate.pitch = gyro.wy/131; // Angular velocity in Y-axis in degrees per second
  rate.yaw = gyro.wz/131; // Angular velocity in Z-axis in degrees per second

  
      caliberation.roll+=rate.roll;
      caliberation.pitch+=rate.pitch;
      caliberation.yaw+=rate.yaw;
      delay(1);

    }

      caliberation.roll/=3000;
      caliberation.pitch/=3000;
      caliberation.yaw/=3000;

  }


void calcReferenceInput(Signal &signal, REF_RPY &rpy) {
  rpy.throttle = signal.height;
  rpy.roll = signal.frwrvr;
  rpy.pitch = signal.rgtlft ;
  rpy.yaw = signal.yaw;

  rpy.roll  =map(rpy.roll,0,3.3,-15,15);
  rpy.pitch =map(rpy.roll,0,3.3,-15,15);
  rpy.yaw   =map(rpy.roll,0,3.3,-15,15);

}




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
  Rate.roll = gyro.wx/131.0-CALIBER.roll; // Angular velocity in X-axis in degrees per second
  Rate.pitch = gyro.wy/131.0-CALIBER.pitch; // Angular velocity in Y-axis in degrees per second
  Rate.yaw = gyro.wz/131.0-CALIBER.yaw; // Angular velocity in Z-axis in degrees per second
  
  Serial.print(" Rate roll: ");
  Serial.print(Rate.roll);
  Serial.print(" Rate pitch: ");
  Serial.print(Rate.pitch);
  Serial.print(" Rate yaw: ");
  Serial.println(Rate.yaw);


  // Calculate the reference input
  calcReferenceInput(signal, ref_rpy);
  
  // PID calculation
  
  pidOutput.roll = calcPID(ref_rpy.roll, pid_roll, Rate.roll, prev_error_roll, integral_roll, derivative_roll, dt);
  pidOutput.pitch = calcPID(ref_rpy.pitch, pid_pitch, Rate.pitch, prev_error_pitch, integral_pitch, derivative_pitch, dt);
  pidOutput.yaw = calcPID(ref_rpy.yaw, pid_yaw, Rate.yaw, prev_error_yaw, integral_yaw, derivative_yaw, dt);

  motorControl(ref_rpy, pidOutput);
  
}




void receiveData() {
  if (receiver.available()) {
    Serial.println("DATA AVAILABLE");
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





void setup() {

 // set the serial monitor baud rate

 Serial.begin(115200);

  //Start the I2C communication
  // Wire.begin();
  // Wire.beginTransmission(MPU_addr);
  // Wire.write(0x6B);  // PWR_MGMT_1 register
  // Wire.write(1);     // set to zero (wakes up the MPU-6050)
  // Wire.endTransmission(true);
  // delay(50);
  // Wire.beginTransmission(MPU_addr);

 Wire.beginTransmission(mpu6050); // Start with device write address 
  Wire.write(SMPLRT_DIV);   //Write to sample rate register 0x19
  Wire.write(0x07);     //1KHz sample rate
  delayMicroseconds(100);
  Wire.endTransmission(true);
  delayMicroseconds(100); 

  Wire.beginTransmission(mpu6050);
  Wire.write(PWR_MGMT_1);    //Write to power management register 0x6B
  Wire.write(0x01);    //X axis gyroscope reference frequency
  Wire.endTransmission(true);
  delayMicroseconds(100);
  
  
  Wire.beginTransmission(mpu6050);
  Wire.write(CONFIG);      // Write to Configuration register 0x1A
  Wire.write(0x00);    //Fs = 8KHz
  Wire.endTransmission(true);
  delayMicroseconds(100);
  
  
  Wire.beginTransmission(mpu6050);
  Wire.write(GYRO_CONFIG);   //Write to Gyro configuration register 0x1B
  Wire.write(0x18);    //Full scale range +/- 2000 degree/C
  Wire.endTransmission(true);
  delayMicroseconds(100);

  calib(Rate, CALIBER);

motorFr.attach(MOTOR_FR);
  motorFl.attach(MOTOR_FL);
  motorBr.attach(MOTOR_BR);
  motorBl.attach(MOTOR_BL);



receiver.begin();
if (!receiver.begin()) { Serial.println(F("radio hardware not responding!"));}
receiver.openReadingPipe(0, address);   //Setting the address at which we will receive the data
receiver.setPALevel(RF24_PA_MAX);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
receiver.startListening();       //This sets the module as receiver
Serial.println("WORKING PROPERLY");

setconst();

}



int a=0;

void loop(){

receiveData();


gyroData(gyro);
// update_rates();
calcInput(data, gyro, pidOutput);

// Serial.print("   Pitch=");
// Serial.print(pidOutput.pitch);
// Serial.print("   Roll=");
// Serial.print(pidOutput.roll);
// Serial.print("   Yaw=");
// Serial.print(pidOutput.yaw);

// Serial.print("   Ax=");
// Serial.print(gyro.ax);
// Serial.print("   Ay=");
// Serial.print(gyro.ay);
// Serial.print("   Az=");
// Serial.print(gyro.az);

// Serial.print("   Wx=");
// Serial.print(gyro.wx);
// Serial.print("   Wy=");
// Serial.print(gyro.wy);
// Serial.print("   Wz=");
// Serial.println(gyro.wz);

delay(200);

}



