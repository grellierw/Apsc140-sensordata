#include <SPI.h>
#include <RH_RF69.h>

#define FREQUENCY       915.0
#define CS_PIN          A5
#define INT_PIN         1
#define RST_PIN         A3
#define LED_PIN         8

// Header definitions for each measurement
#define TEMP_HEADER      0x01
#define PRESSURE_HEADER  0x02
#define HUMIDITY_HEADER  0x03
#define GAS_HEADER       0x04
#define VOLTAGE_HEADER   0x05

// Radio driver instance
RH_RF69 rf69(CS_PIN, INT_PIN);

int16_t packetNum = 0;

void Blink(byte pin, byte delayMs, byte loops);

void setup()
{
  Serial.begin(9600);

  // Wait until serial console is open
  while (!Serial);

  pinMode(LED_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW);

  Serial.println("Feather RFM69 RX Test!");

  // Manual reset
  digitalWrite(RST_PIN, HIGH);
  delay(10);
  digitalWrite(RST_PIN, LOW);
  delay(10);

  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  
  Serial.println("RFM69 radio init OK!");

  if (!rf69.setFrequency(FREQUENCY)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);

  Serial.print("RFM69 radio @ ");  
  Serial.print((int)FREQUENCY);  
  Serial.println(" MHz");
}


// Loop through and receive data
void loop() {
  int count = 0;
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  // Wait for a message with a timeout of 500 milliseconds
  if (rf69.waitAvailableTimeout(500)) {

    // Receive the message
    if (rf69.recv(buf, &len)) {
      uint8_t header = buf[0];

      // Check the header to determine what type of data was received
      switch (header) {
        case TEMP_HEADER:
          // Extract temperature data from the message
          float tempData;
          memcpy(&tempData, buf + 1, sizeof(float));
          Serial.print("temperature:");
          Serial.print(tempData);
          Serial.print(",");
          break;

        case PRESSURE_HEADER:
          // Extract pressure data from the message
          float pressureData;
          memcpy(&pressureData, buf + 1, sizeof(uint32_t));
          Serial.print("pressure:");
          Serial.print(pressureData);
          Serial.print(",");
          break;

        case HUMIDITY_HEADER:
          // Extract humidity data from the message
          float humidityData;
          memcpy(&humidityData, buf + 1, sizeof(float));
          Serial.print("humidity:");
          Serial.print(humidityData);
          Serial.print(",");
          break;

        case GAS_HEADER:
          // Extract gas data from the message
          float gasData;
          memcpy(&gasData, buf + 1, sizeof(uint32_t));
          Serial.print("Gas:");
          Serial.print(gasData);
          Serial.print(",");
          break;

        case VOLTAGE_HEADER:
          // Extract voltage data from the message
          float voltageData;
          memcpy(&voltageData, buf + 1, sizeof(uint32_t));
          Serial.print("Battery:");
          Serial.print(voltageData);
          Serial.print(",");
          count = 1;
          break;

        default:
          Serial.println("No preamble");
          break;
      }

      // Print the radio signal strength
      if (count == 1) {
        Serial.println("");
        Serial.print("RSSI:");
        Serial.print(abs(rf69.lastRssi()), DEC);
        Serial.print(",");
        uint8_t data[] = "RX";
        rf69.send(data, sizeof(data));
        rf69.waitPacketSent();
      }
    }
  }

  // Blink an LED
  if (count == 1) {
    Blink(LED_PIN, 40, 3); //blink LED 3 times, 40ms between blinks
  }
}

// Function to blink an LED
void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}