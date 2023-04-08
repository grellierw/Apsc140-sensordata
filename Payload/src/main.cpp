// Include libraries
#include <RH_RF69.h>
#include <Adafruit_BME680.h>
#include <SD.h>
#include <Adafruit_sensor.h>

// Pin definitions
#define RFM69_RST  A3
#define RFM69_CS   A5
#define RFM69_INT  1
#define LED        8
#define VBATPIN    A9

// RFM69 settings
#define RF69_FREQ 915.0
#define RF69_TX_POWER 15

// Measurement headers
#define TEMP_HEADER      0x01
#define PRESSURE_HEADER  0x02
#define HUMIDITY_HEADER  0x03
#define GAS_HEADER       0x04
#define VOLTAGE_HEADER   0x05

// Initialize RFM69 module
RH_RF69 rf69(RFM69_CS, RFM69_INT);
void initRFM69() {
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  if (!rf69.init()) {
    Serial.println("RFM69 failed to initialize");
    while(1);
  }
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("RFM69 failed to set frequency");
    while(1);
  }
  rf69.setTxPower(RF69_TX_POWER, true);
}

// Initialize BME680 sensor
Adafruit_BME680 bme;
void initBME680() {
  if (!bme.begin()) {
    Serial.println("BME680 failed to initialize");
    while(1);
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);
}

// Helper function to round and convert uint32_t to uint8_t array
void convertRounding(uint32_t input, uint8_t array[4]) {
  float temp = static_cast<float>(input);
  uint32_t rounded = static_cast<uint32_t>(temp + 0.5f);
  memcpy(&array[1], &rounded, sizeof(uint32_t));
}

// Function to send data via RFM69
void sendData(const uint8_t* data, size_t len) {
  rf69.send(data, len);
  rf69.waitPacketSent();
}

// Function to wait for RFM69 response
void waitForRX() {
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (rf69.waitAvailableTimeout(500)) {
    if (rf69.recv(buf, &len)) {
      Serial.print("Received reply: ");
      Serial.println((char*)buf);
    }
    else {
      delay(1000);
    }
  }
  else {
    delay(1000);
  }
}

// Main setup function
void setup() {
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  initRFM69();
  initBME680();
}

/******************LOOP*******************/

void loop() {

  // Update BME data variables
  bme.performReading();

  // Send temperature data
  float tempInput = bme.temperature;
  uint8_t dataTemp[sizeof(tempInput) + 1];
  dataTemp[0] = TEMP_HEADER;
  memcpy(&dataTemp[1], &tempInput, sizeof(float));
  Serial.print("Temperature: ");
  Serial.println(tempInput);
  sendData(dataTemp, sizeof(dataTemp));
  waitForRX();

  // Send pressure data
  uint32_t pressureBME = bme.pressure;
  float pressureInput = pressureBME / 10000.0;
  uint8_t pressure[sizeof(float) + 1];
  pressure[0] = PRESSURE_HEADER;
  memcpy(&pressure[1], &pressureInput, sizeof(float));
  Serial.print("Pressure: ");
  Serial.println(pressureInput);
  sendData(pressure, sizeof(pressure));
  waitForRX();

  // Send humidity data
  float humidityInput = bme.humidity;
  uint8_t dataHumidity[sizeof(humidityInput) + 1];
  dataHumidity[0] = HUMIDITY_HEADER;
  memcpy(&dataHumidity[1], &humidityInput, sizeof(float));
  Serial.print("Humidity: ");
  Serial.println(humidityInput);
  sendData(dataHumidity, sizeof(dataHumidity));
  waitForRX();

  // Send gas resistance data
  uint32_t gasBME = bme.gas_resistance;
  float gasInput = gasBME / 1000.0;
  uint8_t gas[sizeof(float) + 1];
  gas[0] = GAS_HEADER;
  memcpy(&gas[1], &gasInput, sizeof(float));
  Serial.print("Gas Resistance: ");
  Serial.println(gasInput);
  sendData(gas, sizeof(gas));
  waitForRX();

  // Send voltage data
  float measuredVBat = analogRead(VBATPIN) * 2 * 3.3 / 1024.0;
  uint8_t dataVoltage[sizeof(measuredVBat) + 1];
  dataVoltage[0] = VOLTAGE_HEADER;
  memcpy(&dataVoltage[1], &measuredVBat, sizeof(float));
  Serial.print("VBat: ");
  Serial.println(measuredVBat);
  sendData(dataVoltage, sizeof(dataVoltage));
  waitForRX();

  Serial.print("RSSI: ");
  Serial.println(rf69.lastRssi());
}