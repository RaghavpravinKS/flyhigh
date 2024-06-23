/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

// #include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

struct Signal {
  int height;
  int frwrvr;
  int rgtlft;
  int yaw;
};

int blueLed = 23;
int redLed = 22;
int greenLed = 21;

Signal data;

SPIClass SPI(0);

RF24 radio(9, 10); // CE, CSN         
const byte address[6] = "101000";     //Byte of array representing the address. This is the address where we will send the data. This should be same on the receiving side.

// Servo myservo;  // create servo object to control a servo

int potpin = A1;  // analog pin used to connect the potentiometer
int val = 0;    // variable to read the value from the analog pin
byte escval = 0;

// int xPin = A0;
int yPin = A1;
int xPin = A2;
float stack=0;

void setup() {
  digitalWrite(greenLed, LOW);
  // myservo.attach(7);  // attaches the servo on pin 9 to the servo object
  Serial.begin(115200);
  radio.begin();                  //Starting the Wireless communication
  radio.openWritingPipe(address); //Setting the address where we will send the data
  radio.setPALevel(RF24_PA_MAX);  //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.stopListening();          //This sets the module as transmitter
  Serial.println("Starting");
  
}

void setData(int escval, int rgtlft){
  data.rgtlft = 0;
  data.frwrvr = 0;
  data.height = escval;
  data.yaw = 0;
}

int index = 0;

void loop() {
//   if (index == 0) {
//     int xVal= analogRead(xPin);
//     for (int i = 0; i < 70; i++) {
//       if (stack >= 13000) {
//         stack = 13000;
    
//     } else if (val >= 10000) {
//         stack += 50;
//         // delay(400);
//     }
//      else {
    
//         stack += 1000;
    
// //   delay(400);                           // waits for the servo to get there
//     }                            // scale it to use it with the servo (value between 0 and 180)
// delay(100);
  
// //   ESC.writeMicroseconds(0);           // sets the servo position according to the scaled value
//   Serial.print("Servo position: ");
//   Serial.println(stack);

//       setData(stack, xVal);
//       radio.write(&data, sizeof(data)); //Sending the data
//       // delay(100);
//     }
//     index++;
//   } else {
    
  int yVal = analogRead(yPin);
  int xVal= analogRead(xPin);
    if (yVal>1100){
      if (stack>170000){
          stack=stack;
        }
        else {
        stack+=100;}
    }

    else {

      if (yVal<500 ){
        if (stack<0){
          stack=stack;
        }
        else {
        stack-=200;}

      }
    }

    val=stack;


  // xVal=map(xVal,0,1600,-20,20);


  // Serial.print("    Y = ");
  // Serial.print(yVal);
  // Serial.print("    raw_roll= ");
  // Serial.print(xVal);
  if (xVal < 30){
    val=0;
    stack=0;
  }
  Serial.print(" Stack=  ");
  
  Serial.print(stack);
  Serial.print(" Button state: ");
  Serial.println(xVal);

  
  // Serial.print("  val=  ");
  
  // Serial.println(val);
  setData(val, xVal);
  radio.write(&data, sizeof(data)); //Sending the data
  digitalWrite(greenLed, HIGH);
  digitalWrite(blueLed, LOW);

  // delay(10);
    }

// }

