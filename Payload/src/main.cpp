#include "main.h"
#include <RH_RF69.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_sensor.h>
#include <Adafruit_BME680.h>
#include <string.h>

#define RF69_FREQ        915.0
#define RFM69_RST        A3
#define RFM69_CS         A5
#define RFM69_INT        1
#define LED              8


RH_RF69 rf69(RFM69_CS, RFM69_INT);

Adafruit_BME680 bme;



void convertRounding(uint32_t input, uint8_t pressure[4]) {
    // convert the uint32_t to a float to enable proper rounding
    float temp = static_cast<float>(input);

    // round the float to the nearest integer
    uint32_t rounded = static_cast<uint32_t>(temp + 0.5f);

    // use memcpy to copy the rounded uint32_t into the output buffer
    memcpy(pressure, &rounded, sizeof(uint32_t));
}







void setup() {
  Serial.begin(9600);

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

 // while(!Serial);

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);


  Serial.println("start");
  if(!bme.begin()) {
    Serial.println("BME failed to init");
    while(1);
  }
  Serial.println("start1");

  if(!rf69.init()){
    Serial.println("fail");
    Serial.println("RFM69 failed to init");
    while(1);
  }
  Serial.println("start2");
  if(!rf69.setFrequency(RF69_FREQ)){
    Serial.println("RFM69 failed to set frequency");
  while(1);
  }
Serial.println("mid");
  rf69.setTxPower(15, true);

// Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  Serial.println("finish");
}


void loop() {
  
  //updates bme.data variables
  bme.performReading();

  Serial.println("Sending to rf69_server");
  // Send a message to rf69_server
  
  
  
// send temp
  float f = bme.temperature;
  uint8_t data[sizeof(f)];
  memcpy(data, &f, sizeof(float));

  Serial.println(f);

 // rf69.send(data, sizeof(data));
  //rf69.waitPacketSent();
  
//end temp



// start pressure

uint32_t input = bme.pressure;
uint8_t pressure[sizeof(float)];
convertRounding(input, pressure);
Serial.println(bme.pressure);
rf69.send(pressure, sizeof(pressure));
rf69.waitPacketSent();

// end pressure


  Serial.print("rssi: "); Serial.println(rf69.lastRssi());




  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf69.waitAvailableTimeout(500))
  { 
    // Should be a reply message for us now   
    if (rf69.recv(buf, &len))
    {
      Serial.print("got reply: ");
      Serial.println((char*)buf);
    }
    else
    {
      Serial.println("recv failed");
    }
  }
  else
  {
    Serial.println("No reply, is rf69_server running?");
  }


  delay(1000);
}