#include <Servo.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include "MPU6050_res_define.h"
#include <pwm.h>
#include <math.h>

signed short Ax, Ay, Az, Tmp, Gx, Gy, Gz; // Accelerometer and gyroscope readings
// float Ax_filter, Ay_filter, Az_filter, Gx_filter, Gy_filter, Gz_filter; // Filtered accelerometer and gyroscope readings
// float Ax_sum, Ay_sum, Az_sum, Gx_sum, Gy_sum, Gz_sum; // Sum of accelerometer and gyroscope readings
float Ax_conv, Ay_conv, Az_conv, Gx_conv, Gy_conv, Gz_conv; // Converted accelerometer and gyroscope readings
const int MPU6050 = 0x68;
SPIClass SPI(0);
TwoWire Wire(0);

//define leds
int redLed = 24;
int greenLed = 22;
int blueLed = 23;

RF24 receiver(9, 10); // Initialize the radio
const byte address[6] = "100001"; // Address for the radio

// Define PID gains for roll
float Kp_roll = 1.7;
float Ki_roll = 0.8;
float Kd_roll = 0.9;
float pid_max_roll = 400.0;

// Define PID gains for pitch
float Kp_pitch = 1.3;
float Ki_pitch = 0.8;
float Kd_pitch = 0.9;
float pid_max_pitch = 400.0;

// Define PID gains for yaw
float Kp_yaw = 4.0;
float Ki_yaw = 0.5;
float Kd_yaw = 0.0;
float pid_max_yaw = 400.0;

//  Taking 2 variables to note time and set the time for the loop as 4 ms
unsigned long time, time1, time2 = 0;

float Ax_prev = 0.0;
float Ay_prev = 0.0;
float Az_prev = 0.0;
float Gx_prev = 0.0;
float Gy_prev = 0.0;
float Gz_prev = 0.0;

// Caliberated values for Ax, Ay, Az, Gx, Gy, Gz
float Ax_cal = 0.0;
float Ay_cal = 0.0;
float Az_cal = 0.0;
float Gx_cal = 0.0;
float Gy_cal = 0.0;
float Gz_cal = 0.0;

// Integral term for PID
float integral_roll = 0.0;
float integral_pitch = 0.0;
float integral_yaw = 0.0;

// Error terms for PID
float error_roll = 0.0;
float error_pitch = 0.0;
float error_yaw = 0.0;

// Previous errors for derivative term
float error_roll_prev = 0.0;
float error_pitch_prev = 0.0;
float error_yaw_prev = 0.0;

// PID terms
float angle_roll = 0.0;
float angle_pitch = 0.0;
float angle_yaw = 0.0;

float pid_roll = 0.0;
float pid_pitch = 0.0;
float pid_yaw = 0.0;

// Define motor outputs
int motor_1 = 0;
int motor_2 = 0;
int motor_3 = 0;
int motor_4 = 0;

// Define arrays for moving average filter on accelerometer readings
float Ax_filter[20];
float Ay_filter[20];
float Az_filter[20];
float Ax_filtered, Ay_filtered, Az_filtered = 0.0;

void setup() {

    // Set up the NRF24L01 radio
    receiver.begin();
    if (!receiver.begin()){
        Serial.println("Receiver failed to start");
        digitalWrite(redLed, LOW);
    }
    receiver.openReadingPipe(0, address); // Open the radio pipe
    receiver.setPALevel(RF24_PA_MAX); // Set the radio power level
    receiver.startListening(); // Start listening for incoming messages
    
    
    
    digitalWrite(blueLed, LOW);
    Serial.begin(115200); // Start the serial communication
    Serial.println("Starting setup");
    PWM.PWMC_Set_Period(1, 2000000);
    PWM.PWMC_Set_OnOffTime(1, 87800);
    PWM.PWMC_init(1);
    PWM.PWMC_Enable();
    delay(100);

    PWM.PWMC_Set_Period(2, 2000000);
    PWM.PWMC_Set_OnOffTime(2, 87800);
    PWM.PWMC_init(2);
    PWM.PWMC_Enable();
    delay(100);

    PWM.PWMC_Set_Period(3, 2000000);
    PWM.PWMC_Set_OnOffTime(3, 87800);
    PWM.PWMC_init(3);
    PWM.PWMC_Enable();
    delay(100);

    PWM.PWMC_Set_Period(4, 2000000);
    PWM.PWMC_Set_OnOffTime(4, 87800);
    PWM.PWMC_init(4);
    PWM.PWMC_Enable();
    delay(100);

    Wire.beginTransmission(MPU6050);
    Wire.write(SMPLRT_DIV);
    Wire.write(0x07);
    delayMicroseconds(100);
    Wire.endTransmission(true);
    delayMicroseconds(100);
    Wire.beginTransmission(MPU6050);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission(true);
    delayMicroseconds(100);
    Wire.beginTransmission(MPU6050);
    Wire.write(CONFIG);
    Wire.write(0x02);
    Wire.endTransmission(true);
    delayMicroseconds(100);
    Wire.beginTransmission(MPU6050);
    Wire.write(GYRO_CONFIG);
    Wire.write(0x08); // 0x00 for 250, 0x08 for 500, 0x10 for 1000, 0x18 for 2000
    Wire.endTransmission(true);
    delayMicroseconds(100);
    Wire.beginTransmission(MPU6050);
    Wire.write(ACCEL_CONFIG);
    Wire.write(0x00); // 0x00 for 2g, 0x08 for 4g, 0x10 for 8g, 0x18 for 16g
    Wire.endTransmission(true);
    delayMicroseconds(100);
    
    // Caliberate Ax, Ay, Az, Gx, Gy, Gz
    for (int i = 0; i < 1000; i++){
        Wire.beginTransmission(MPU6050);
        Wire.write(ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU6050, 14, true);
        Ax = ((Wire.read() << 8) | Wire.read());
        Ay = ((Wire.read() << 8) | Wire.read());
        Az = ((Wire.read() << 8) | Wire.read());
        Tmp = ((Wire.read() << 8) | Wire.read());
        Gx = ((Wire.read() << 8) | Wire.read());
        Gy = ((Wire.read() << 8) | Wire.read());
        Gz = ((Wire.read() << 8) | Wire.read());
        Wire.endTransmission(true);
        Ax_cal += (float)Ax/16384.0;
        Ay_cal += (float)Ay/16384.0;
        Az_cal += (float)Az/16384.0;
        Gx_cal += (float)Gx/65.5;
        Gy_cal += (float)Gy/65.5;
        Gz_cal += (float)Gz/65.5;
        delay(10);
    }

    // Turn off blue led
    digitalWrite(blueLed, HIGH);
    
    Ax_cal /= 1000;
    Ay_cal /= 1000;
    Az_cal /= 1000;
    Gx_cal /= 1000;
    Gy_cal /= 1000;
    Gz_cal /= 1000;
    Serial.println("Caliberation done");
  }

int val;
int pwmval = 87800;
int throttle = 0;

void loop() {
    if (receiver.available()) {
        digitalWrite(redLed,HIGH);
        digitalWrite(greenLed,LOW);
        // digitalWrite(blueLed,HIGH);
        receiver.read(&pwmval, sizeof(pwmval));
        // Serial.println(pwmval);
    } else {
        digitalWrite(redLed,LOW);
        digitalWrite(greenLed,HIGH);
        // digitalWrite(blueLed,HIGH);
        // rx_data.height = rx_data.height;
        // rx_data.frwrvr = rx_data.frwrvr;
        // rx_data.rgtlft = rx_data.rgtlft;
        // rx_data.yaw = rx_data.yaw;
    }

    time = micros();
    // val = analogRead(A3);
    // if (val > 1200){
    //     pwmval += 50;
    // } else if (val < 100){
    //     pwmval = 87000;
    // }
    // Trun on Green Led
    // digitalWrite(22, LOW);

    Wire.beginTransmission(MPU6050);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, 14, true);
    Ax = ((Wire.read() << 8) | Wire.read());
    Ay = ((Wire.read() << 8) | Wire.read());
    Az = ((Wire.read() << 8) | Wire.read());
    Tmp = ((Wire.read() << 8) | Wire.read());
    Gx = ((Wire.read() << 8) | Wire.read());
    Gy = ((Wire.read() << 8) | Wire.read());
    Gz = ((Wire.read() << 8) | Wire.read());
    Wire.endTransmission(true);

    Ax_conv = (float)Ax/16384.0 - Ax_cal;
    Ay_conv = (float)Ay/16384.0 - Ay_cal;
    Az_conv = (float)Az/16384.0 - (Az_cal-1);
    Gx_conv = (float)Gx/65.5 - Gx_cal;
    Gy_conv = (float)Gy/65.5 - Gy_cal;
    Gz_conv = (float)Gz/65.5 - Gz_cal;
    Gx_prev = 0.7* Gx_prev + 0.3* Gx_conv;
    Gy_prev = 0.7* Gy_prev + 0.3* Gy_conv;
    Gz_prev = 0.7* Gz_prev + 0.3* Gz_conv;

    // Calculate moving average filter for accelerometer readings
    // for (int i = 0; i < 19; i++){
    //     Ax_filter[i] = Ax_filter[i+1];
    //     Ay_filter[i] = Ay_filter[i+1];
    //     Az_filter[i] = Az_filter[i+1];
    // }
    // Ax_filter[19] = Ax_conv;
    // Ay_filter[19] = Ay_conv;
    // Az_filter[19] = Az_conv;
    // for (int i = 0; i < 20; i++){
    //     Ax_filtered += Ax_filter[i];
    //     Ay_filtered += Ay_filter[i];
    //     Az_filtered += Az_filter[i];
    // }
    // Ax_filtered /= 20.0;
    // Ay_filtered /= 20.0;
    // Az_filtered /= 20.0;

    // angle_roll = 0.7 *(angle_roll + Gx_conv * 0.004) + 0.3 * (atan2(Ay_conv, sqrt(Ax_conv**2 + Az_conv**2) ) * 57.324);
    // angle_pitch = 0.7 *(angle_pitch + Gy_conv * 0.004) + 0.3 * (atan2(Ax_conv, sqrt(Ay_conv**2 + Az_conv**2) ) * 57.324);

    angle_roll = 0.9996 * (angle_roll + Gx_prev * 0.004) + 0.0004 * (atan2(Ay_conv, sqrt(Ax_conv*Ax_conv + Az_conv*Az_conv)) * 57.324);

    angle_pitch = 0.9996 * (angle_pitch + Gy_prev * 0.004) + 0.0004 * (atan2(Ax_conv, sqrt(Ay_conv*Ay_conv + Az_conv*Az_conv)) * 57.324);

    error_roll = angle_roll - 0.0;
    error_pitch = angle_pitch - 0.0;
    error_yaw = Gz_prev - 0.0;

    integral_roll += error_roll * 0.004;
    if (integral_roll > pid_max_roll){
        integral_roll = pid_max_roll;
    } else if (integral_roll < -pid_max_roll){
        integral_roll = -pid_max_roll;
    }

    integral_pitch += error_pitch * 0.004;
    if (integral_pitch > pid_max_pitch){
        integral_pitch = pid_max_pitch;
    } else if (integral_pitch < -pid_max_pitch){
        integral_pitch = -pid_max_pitch;
    }

    integral_yaw += error_yaw * 0.004;
    if (integral_yaw > pid_max_yaw){
        integral_yaw = pid_max_yaw;
    } else if (integral_yaw < -pid_max_yaw){
        integral_yaw = -pid_max_yaw;
    }

    pid_roll = Kp_roll * error_roll + Ki_roll * integral_roll + Kd_roll * (error_roll - error_roll_prev) / 0.004;
    if (pid_roll > pid_max_roll){
        pid_roll = pid_max_roll;
    } else if (pid_roll < -pid_max_roll){
        pid_roll = -pid_max_roll;
    }
    error_roll_prev = error_roll;

    pid_pitch = Kp_pitch * error_pitch + Ki_pitch * integral_pitch + Kd_pitch * (error_pitch - error_pitch_prev) / 0.004;
    if (pid_pitch > pid_max_pitch){
        pid_pitch = pid_max_pitch;
    } else if (pid_pitch < -pid_max_pitch){
        pid_pitch = -pid_max_pitch;
    }
    error_pitch_prev = error_pitch;


    pid_yaw = Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * (error_yaw - error_yaw_prev) / 0.004;
    if (pid_yaw > pid_max_yaw){
        pid_yaw = pid_max_yaw;
    } else if (pid_yaw < -pid_max_yaw){
        pid_yaw = -pid_max_yaw;
    }
    error_yaw_prev = error_yaw;

    // Map pwmval from 87800 to 220100 to 1000 to 2000
    throttle = map(pwmval, 87800, 220100, 1000, 2000);

    motor_1 = throttle - pid_roll - pid_pitch - pid_yaw; // Front right motor
    motor_2 = throttle + pid_roll - pid_pitch + pid_yaw; // Front left motor
    motor_3 = throttle - pid_roll + pid_pitch - pid_yaw; // Rear right motor
    motor_4 = throttle + pid_roll + pid_pitch + pid_yaw; // Rear left motor

    // Map motor outputs from 1000 to 2000 to 87800 to 220100
    motor_1 = map(motor_1, 1000, 2000, 87800, 220100);
    motor_2 = map(motor_2, 1000, 2000, 87800, 220100);
    motor_3 = map(motor_3, 1000, 2000, 87800, 220100);
    motor_4 = map(motor_4, 1000, 2000, 87800, 220100);

    // Error correction for ESC offset
    motor_2 += 5000;
     
    if (motor_1 < 87800){
        motor_1 = 87800;
    } else if (motor_1 > 220100){
        motor_1 = 220100;
    }
    PWM.PWMC_Set_OnOffTime(1, motor_1);
    PWM.PWMC_init(1);
    delayMicroseconds(20);

    if (motor_2 < 87800){
        motor_2 = 87800;
    } else if (motor_2 > 220100){
        motor_2 = 220100;
    }
    PWM.PWMC_Set_OnOffTime(2, motor_2);
    PWM.PWMC_init(2);
    delayMicroseconds(20);

    if (motor_3 < 87800){
        motor_3 = 87800;
    } else if (motor_3 > 220100){
        motor_3 = 220100;
    }
    PWM.PWMC_Set_OnOffTime(3, motor_3);
    PWM.PWMC_init(3);
    delayMicroseconds(20);
    
    if (motor_4 < 87800){
        motor_4 = 87800;
    } else if (motor_4 > 220100){
        motor_4 = 220100;
    }
    PWM.PWMC_Set_OnOffTime(4, motor_4);
    PWM.PWMC_init(4);
    delayMicroseconds(20);


    if (abs(Gz_prev)<0.03) Gz_prev = 0.0;

    // Serial.println(Gz_prev*100);

    // Print the accelerometer and gyroscope readings
    Serial.print(Gx_prev);
    Serial.print(" ");
    Serial.print(Gy_prev);
    Serial.print(" ");
    Serial.print(Gz_prev);
    Serial.print(" ");
    Serial.print(angle_roll);
    Serial.print(" ");
    Serial.println(angle_pitch);

    time1 = micros();
    if (time1 - time < 4000){
        delayMicroseconds(4000 - (time1 - time));
    }
    
    // time2 = micros();
    // Serial.println(time2 - time);
}
