#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

SPIClass SPI(1);

RF24 radio(20, 21); // CE, CSN         
const byte address[6] = "100001"; // Address for communication

struct DataPacket {
    int throttle;
    int kp;
    int kd;
    int ki;
    int pitch;
};

DataPacket data;
unsigned long time1,time2;

void setup() {
    Serial.begin(115200); // Start the serial communication
    Serial.println("Starting setup");
    pinMode(13, INPUT_PULLUP);  // Configure GPIO 13 as input with pull-up resistor
    
    radio.begin();                  
    radio.openWritingPipe(address); 
    radio.setPALevel(RF24_PA_MAX);  
    radio.stopListening();   
    data.ki=35;  
    data.kp=200;
    data.kd=158; 
    data.pitch=0;

}

void loop() {
    
    int x = analogRead(A1);
    float y = analogRead(A2);
    float z = analogRead(A3);
    float w = analogRead(A0);


    if (x > 1500 && data.throttle<=120000) {
      data.throttle += 250;
    }
    if (x < 200 && data.throttle<=120000) {
      data.throttle -= 250;
    }

    if (x > 1500 && data.throttle>=120000) {
      data.throttle += 100;
    }
    if (x < 200 && data.throttle>=120000) {
      data.throttle -= 100;
    }

    if (data.throttle <= 83800) {
        data.throttle = 83800;
    } 
    if (data.throttle >= 145000) {
        data.throttle = 145000;
    } 
    
    data.ki = (y / 1649) * 400; // Scale y to 0-400 range
    
    if (digitalRead(16) == HIGH) { 
        data.kd = (z / 1649) * 400;  // Change Kd when GPIO 13 is HIGH
    } else {
        data.ki = (z / 1649) * 300;  // Change Ki when GPIO 13 is LOW
    }

  
    while (!radio.write(&data, sizeof(DataPacket))) {
        delayMicroseconds(100);
    }
    
    Serial.print("Throttle: ");
    Serial.println(data.throttle);

     Serial.print("Kp: ");
    Serial.print(data.kp);
    Serial.print(" ");
    Serial.print("Kd: ");

    Serial.print(data.kd);

    Serial.print(" "); 
    Serial.print("Ki: ");
    Serial.println(data.ki);

    delayMicroseconds(100);
}

