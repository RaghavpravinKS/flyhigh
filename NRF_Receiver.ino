/*
  @file receiver_nrf24.ino
  @brief Interfacing NRF24L01 with ARIES Board using SPI.
  @detail The demo to receive the data from transmitter controller using NRF24L01

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

SPIClass SPI(0);

RF24 radio(9, 10); // CE, CSN
const byte address[6] = "10101";
boolean button_state = 0;
int led_pin = 23;

void setup() {
Serial.begin(250000);                // set the serial monitor baud rate to 250000
radio.begin();
radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
radio.setPALevel(RF24_PA_MAX);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
radio.startListening();       //This sets the module as receiver
Serial.println("WORKING PROPERLY");

}



void loop()
{
if (radio.available())   {           //Looking for the data.

char text[32] = "";                 //Saving the incoming data
int a=0;

radio.read(&a, sizeof(a));    //Reading the data
// radio.read(&button_state, sizeof(button_state));    //Reading the data

Serial.println("DATA AVAILABLE");
Serial.println(a);

 }


else {
  Serial.println("DATA NOT AVAILABLE");
}
delay(700);

}
