// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem 
// configuration

#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

// Feather 32u4 w/Radio
  #define RFM69_CS      A5
  #define RFM69_INT     1
  #define RFM69_RST     A3
  #define LED           8

//define headers for each measurement
#define TEMP_HEADER      0x01
#define PRESSURE_HEADER  0x02
#define HUMIDITY_HEADER  0x03
#define GAS_HEADER       0x04
#define VOLTAGE_HEADER   0x05

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

void Blink(byte PIN, byte DELAY_MS, byte loops);

void setup() 
{
  Serial.begin(9600);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 RX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}


void loop() {
 if (rf69.available()) {

     /*
     uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];        //creates variable to store incoming data 
     uint8_t len = sizeof(buf);			              //creates variable to store length of data
   
      if (rf69.recv(buf, &len)) { 		           	//receives information
      if (!len) return;
      buf[len] = 0;

      // Receiving Pressure
      uint32_t pressure;			                    // Variable for pressure
      memcpy(&pressure, buf, sizeof(uint32_t));   // Converts from uint8_t to a uint32_t
      Serial.print("Received pressure:");
      Serial.println(pressure);

      // Receiving Temperature
      float temp;					                  // Variable for temperature
      memcpy(&temp, buf, sizeof(float));		// Converts from uint8_t to a float 
      Serial.print("Received temperature:");
      Serial.println(temp);
      						
      Serial.print("RSSI: ");				        // Prints the Radio Signal Strength
      Serial.println(rf69.lastRssi(), DEC);

      }
      else {
      Serial.println("Receive failed");
      } 
      */
  
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);

if (rf69.waitAvailableTimeout(500))
{ 
  // Should be a reply message for us now   
  if (rf69.recv(buf, &len))
  {
    uint8_t header = buf[0];
    switch (header) {
      case TEMP_HEADER:
        // Extract temperature data from the rest of the byte array
        float tempData;
        memcpy(&tempData, buf + 1, sizeof(float));
        // Do something with the temperature data
        break;

      case PRESSURE_HEADER:
        // Extract pressure data from the rest of the byte array
        uint32_t pressureData;
        memcpy(&pressureData, buf + 1, sizeof(uint32_t));
        // Do something with the pressure data
        break;

      case HUMIDITY_HEADER:
        // Extract humidity data from the rest of the byte array
        float humidityData;
        memcpy(&humidityData, buf + 1, sizeof(float));
        // Do something with the humidity data
        break;

      case GAS_HEADER:
        // Extract gas data from the rest of the byte array
        uint32_t gasData;
        memcpy(&gasData, buf + 1, sizeof(uint32_t));
        // Do something with the gas data
        break;

      case VOLTAGE_HEADER:

        uint32_t voltageData;
        memcpy(&voltageData, buf + 1, sizeof(uint32_t));

        break;

      default:
        // Unknown header, do nothing
        break;
    }
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









      if (strstr((char *)buf, "Hello World")) {
        // Send a reply!
        uint8_t data[] = "And hello back to you";
        rf69.send(data, sizeof(data));
        rf69.waitPacketSent();
        Serial.println("Sent a reply");
        Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks
      }
    } else {
      Serial.println("Receive failed");
    }
  }



void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}