/*
  @file transmitter_nrf24.ino
  @brief Interfacing NRF24L01 with ARIES Board using SPI.
  @detail The demo to transmit the data to receiver controller using NRF24L01

   Aries V2 pinout: https://vegaprocessors.in/blog/interfacing-8x8-led-dot-matrix-to-aries-v2-board/
   NRF24L01 pinout: https://howtomechatronics.com/wp-content/uploads/2017/02/NRF24L01-Pinout-NRF24L01-PA-LNA-.png

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

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

SPIClass SPI(1);

RF24 radio(14, 15); // CE, CSN         
const byte address[6] = "00010";     //Byte of array representing the address. This is the address where we will send the data. This should be same on the receiving side.
int throttle = 87800;

void setup() {
  Serial.begin(115200);
  radio.begin();                  //Starting the Wireless communication
  radio.openWritingPipe(address); //Setting the address where we will send the data
  radio.setPALevel(RF24_PA_MAX);  //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.stopListening();           //This sets the module as transmitter
}

unsigned long a1, a2;
int x = 0;

void loop()
{
  a1 = millis();
  x = analogRead(A1);

  if (x>1200){
    throttle+=50;
  } else if (x<100){
    throttle-=1000;
  }

  if (throttle<=87800){
    throttle=87800;
  }

  radio.write(&throttle, sizeof(throttle));
  Serial.print("Time taken: ");
  // Serial.println(throttle);
  a2 = millis();
  
  Serial.println(a2-a1);
}