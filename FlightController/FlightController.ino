#include <Servo.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>

SPIClass SPI(0);

// Address of the transmitter and receiver
const uint64_t pipeOut = 101001;

// receiver object and the CE and CSN pins
RF24 receiver(9, 10);

// Struct to carry the data
struct Signal {
  byte height;
  byte frwrvr;
  byte rgtlft;
  byte yaw;
};

// Signal object to carry the data
Signal data;

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

// Function to reset the data
void resetData(Signal &signal) {
  signal.height = 0;
  signal.frwrvr = 0;
  signal.rgtlft = 0;
  signal.yaw = 0;
}

// Variables to keep track of timestamps
unsigned long currentTime = 0;
unsigned long previousTime = 0;

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

void setup() {
  // put your setup code here, to run once:
  // Start the serial communication
  Serial.begin(115200);
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
  
}
