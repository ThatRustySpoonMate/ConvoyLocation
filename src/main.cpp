#include <Arduino.h> // For ....
#include "SPI.h" // Dependency for Display
#include <TinyGPS++.h> // For GPS 
#include <LoRa.h> // For Lora
#include "SSD1306Wire.h"  // For display

// Definitions
#define SS      18     // GPIO18 -- SX1278's CS
#define RST     14     // GPIO14 -- SX1278's RESET
#define DI0     26     // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define EXT_LED 0      // GPIO0 -- External LED (Drive High to turn on)
#define BROADCAST 0xff // Broadcast network address
#define MAX_RETRIES 10 // Number of times to rety sending the reply after reRecvRetransmit is reached
#define BUTTON_PIN 38  // Pin between PWR and RST 
#define MASTER         // Uncomment for slave mode
// Variable Declarations
TinyGPSPlus gps;

uint8_t increment_low = 0; // Low order byte of increment
uint8_t increment_high = 0; // high order byte of increment
uint32_t interval = 2000; // Time that LED stays on after receiving a message
uint32_t lastRecvTime = 0; // Tracks time of last message
uint32_t noRecvRetransmit = 5000; // If we havent received a reply in this time, send again (ms)
uint8_t retryCount = MAX_RETRIES; 

#ifdef MASTER
  uint8_t networkAddress = 0x01; // Master sending address

#else 
  uint8_t networkAddress = 0x02; // Slave sending address
#endif

// Function Declarations
void SendIncrement();
void OnReceive(int packetSize);
void clearFIFO();

void clearFIFO() {
  while(LoRa.available()) {
    LoRa.read();
  }
}

// Function Definitions
void SendIncrement() {
  Serial.println("Sending packet...");
  LoRa.beginPacket();
  LoRa.print(networkAddress); // Preface with network address
  LoRa.print(increment_low);
  LoRa.print(increment_high);
  LoRa.endPacket();
}

void onReceive(int packetSize) {
  // Polling mode
  if (packetSize == 0) { // if there's no packet, return
    return;
  }  
  Serial.println("RECEIVED!");

  // Blocking mode        
  //while(packetSize == 0) {
  //  packetSize = LoRa.parsePacket();
  //}

  digitalWrite(EXT_LED, HIGH);  // turn on external led
  lastRecvTime = millis();      // timestamp the message

  // read packet bytes:
  uint8_t senderAddress = LoRa.read();
  increment_low = LoRa.read();   // Low order byte of increment
  increment_high = LoRa.read(); // high order byte of increment
  // Maybe read here until LoRa !available? -- To clear FIFO buffer

  if(senderAddress != networkAddress) {
    retryCount = MAX_RETRIES;
    uint16_t increment = ((uint16_t) increment_high << 8) | increment_low; // put the two bytes together into a 16 bit value
    // Print details:
    Serial.println("Sender: " + String(senderAddress));
    Serial.print("Received: ");
    Serial.print(increment_low);
    Serial.print("  ");
    Serial.println(increment_high);
    Serial.println("RSSI: " + String(LoRa.packetRssi()));
    Serial.println("Snr: " + String(LoRa.packetSnr()));
    Serial.println();

    // Increment increment variable
    increment_low = increment_low + 1;
    // Handle overflow
    if(increment_low == 0xff){ 
      increment_low = 0x00;
      increment_high = increment_high + 1;
    }

    clearFIFO();
    delay(100);
    SendIncrement();
    

  } else {
    Serial.println("Picked up own transmission.");
    // Received our own message,  return
    return;
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  LoRa.setPins(SS, RST, DI0);

  pinMode(EXT_LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  


  LoRa.setSpreadingFactor(12); // Maximum spreading factor -- More time on air, longer range, less bw
  LoRa.setSignalBandwidth(62.5E3); // Low bandwidth -- Longer range 
  LoRa.setTxPower(20); // Maximum TX power -- Longer range


  delay(1000);

  #ifdef MASTER
    Serial.println("LoRa started in master mode.");
    SendIncrement();
  #else
    Serial.println("LoRa started in slave mode.");
  #endif


}

void loop() {
  // put your main code here, to run repeatedly:

  if ( (millis() - lastRecvTime > noRecvRetransmit ) && retryCount > 0) {
    SendIncrement();
    lastRecvTime = millis();
    //delay(2000);// Retry every 2s until we get a reply
    //retryCount--;
    //Serial.println("Retries remaining: " + String(retryCount));
  } //else {
    //if(retryCount == 0) {
    //    if(digitalRead(BUTTON_PIN) == 1){ // Reset max retries if button pressed
    //      retryCount = MAX_RETRIES
    //    }
    //  digitalWrite(EXT_LED, HIGH); // Turn on external LED
    //  delay(50);
    //  digitalWrite(EXT_LED, LOW); // Turn off external LED
    //  delay(50);
    //
    //}
  //}

  onReceive(LoRa.parsePacket());  // Check for a packet, and call onReceive with the result:
  //delay(interval);
  digitalWrite(EXT_LED, LOW); // Turn off external LED
  
}


