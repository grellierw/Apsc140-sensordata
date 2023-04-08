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
#define VBATPIN          A9

RH_RF69 rf69(RFM69_CS, RFM69_INT);

//#define BME_CS           A4

Adafruit_BME680 bme;


//define headers for each measurement
#define TEMP_HEADER      0x01
#define PRESSURE_HEADER  0x02
#define HUMIDITY_HEADER  0x03
#define GAS_HEADER       0x04
#define VOLTAGE_HEADER   0x05


/******************FUNCTIONS********************/
void convertRounding(uint32_t input, uint8_t array[4]) {
    // convert the uint32_t to a float to enable proper rounding
    float temp = static_cast<float>(input);

    // round the float to the nearest integer
    uint32_t rounded = static_cast<uint32_t>(temp + 0.5f);

    // use memcpy to copy the rounded uint32_t into the output buffer
    memcpy(&array[1], &rounded, sizeof(uint32_t));
}

void sendData(const uint8_t* data, size_t len) {
   // digitalWrite(RFM69_CS, HIGH);
    delay(500);
    rf69.send(data, len);
    rf69.waitPacketSent();
    Serial.println("Sent");
    
   // digitalWrite(RFM69_CS, LOW);
}

void waitForRX(){

  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  int time = millis();
  if (rf69.waitAvailableTimeout(500))
  { 
    if (rf69.recv(buf, &len))
    {
      Serial.print("got reply: ");
      Serial.println((char*)buf);
    }
    else
    {
      delay(1000);
    }
  }
  else
      delay(1000);

}


/***************SETUP*****************/
void setup() {
  Serial.begin(9600);

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  pinMode(RFM69_CS, OUTPUT);
  //pinMode(BME_CS, OUTPUT);
  
  //while(!Serial);

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);



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
  Serial.println("start");
  delay(1000);

  //digitalWrite(RFM69_CS, LOW);

  if(!bme.begin()) {
    Serial.println("BME failed to init");
    while(1);
  }
  Serial.println("start");


// Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  Serial.println("finish"); 
  
 // digitalWrite(RFM69_CS, LOW);
}


/******************LOOP*******************/

void loop() {
  

  //updates bme.data variables
  //digitalWrite(BME_CS, HIGH);
  bme.performReading();
  //digitalWrite(BME_CS, LOW);


  Serial.println("Sending to rf69_server"); // Send a message to rf69_server
  
  
  // Send temperature OLD 
  /* float tempInput = bme.temperature;
  uint8_t dataTemp[sizeof(tempInput)];
  memcpy(dataTemp, &tempInput, sizeof(float));
  sendData(dataTemp, sizeof(dataTemp)); 
  */
 
  // Send temperature
  float tempInput = bme.temperature;
  uint8_t dataTemp[sizeof(tempInput) + 1];          // Add an extra byte for the header
  dataTemp[0] = TEMP_HEADER;                        // Add the header
  memcpy(&dataTemp[1], &tempInput, sizeof(float)); 
  Serial.print("Temperature: ");
  Serial.println(tempInput); // Copy the temperature data
  sendData(dataTemp, sizeof(dataTemp));
  waitForRX();            
  //digitalWrite(RFM69_CS, HIGH);
       // rf69.send(dataTemp, sizeof(dataTemp));
      //  rf69.waitPacketSent();
  //digitalWrite(RFM69_CS, LOW);
  
  // Send pressure  
  uint32_t pressurebme = bme.pressure;
  
  float pressureInput = pressurebme;
  pressureInput = pressureInput / 10000;

  uint8_t pressure[sizeof(pressureInput)];
  pressure[0] = PRESSURE_HEADER;
  //convertRounding(pressureInput, pressure);
 memcpy(&pressure[1], &pressureInput, sizeof(float));

  Serial.print("Pressure: ");
  Serial.println(pressureInput);
  sendData(pressure, sizeof(pressure));   
  waitForRX();  

  //Serial.println(pressure[1]);

  float receivedTemp;
  memcpy(&receivedTemp, &pressure[1], sizeof(float));
 // Serial.println(receivedTemp);


  // Send Humidity
  float humidityInput = bme.humidity;
  uint8_t dataHumidity[sizeof(humidityInput) + 1];
  dataHumidity[0] = HUMIDITY_HEADER;
  memcpy(&dataHumidity[1], &humidityInput, sizeof(float));
    Serial.print("Humidity: ");
  Serial.println(humidityInput);
  sendData(dataHumidity, sizeof(dataHumidity));
  waitForRX();  









  // Send Gas Data
  uint32_t gasbme = bme.gas_resistance;
  float gasInput = (float)gasbme;
  gasInput = gasInput / 1000;

  uint8_t gas[sizeof(gasInput)];
  gas[0] = GAS_HEADER;
  //convertRounding(gasInput, gas);
  memcpy(&gas[1], &gasInput, sizeof(float));

  Serial.print("Gas Resistance: ");
  Serial.println(gasInput);
  Serial.println(gasbme);
  sendData(gas, sizeof(gas));
  waitForRX();  


  // Send Voltage Data 

  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2; // we divided by 2, so multiply back
  measuredvbat *= 3.3; // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  uint8_t dataVoltage[sizeof(measuredvbat) + 1];
  dataVoltage[0] = VOLTAGE_HEADER;
  memcpy(&dataVoltage[1], &measuredvbat, sizeof(float));
  sendData(dataVoltage, sizeof(dataVoltage));
  waitForRX();  
  Serial.print("VBat: " ); 
  Serial.println(measuredvbat);

  






// OLD send temp 
/*
float temp = bme.temperature;
uint8_t data[sizeof(temp)];
memcpy(data, &temp, sizeof(float));
rf69.send(data, sizeof(data));
rf69.waitPacketSent();

Serial.print("Temperature: ");
Serial.println(temp);
*/
//end temp

// OLD start pressure
/*
uint32_t input = bme.pressure;
uint8_t pressure[sizeof(float)];
convertRounding(input, pressure);
Serial.println(bme.pressure);
rf69.send(pressure, sizeof(pressure));
rf69.waitPacketSent();
*/
// end pressure


  Serial.print("rssi: "); Serial.println(rf69.lastRssi());


}