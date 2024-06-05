#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ScioSense_ENS16x.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define HELTEC_POWER_BUTTON   // must be before "#include <heltec_unofficial.h>"
#include <heltec_unofficial.h>

#include "ens16x_i2c_interface.h"

using namespace ScioSense;

#define SEALEVELPRESSURE_HPA (1013.25)

#define I2C_ADDRESS 0x52
I2cInterface i2c;

// I2C PINOUT
#define SDA_PIN 33
#define SCL_PIN 34

#define USE_INTERRUPT
#define INTN 2

#define BUZZER_PIN 26

// Pause between transmited packets in seconds.
// Set to zero to only transmit a packet when pressing the user button
// Will not exceed 1% duty cycle, even if you set a lower value.
#define PAUSE               300

// Frequency in MHz. Keep the decimal point to designate float.
// Check your own rules and regulations to see what is legal where you are.
// #define FREQUENCY           866.3       // for Europe
#define FREQUENCY           905.2       // for US

// LoRa bandwidth. Keep the decimal point to designate float.
// Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.
#define BANDWIDTH           250.0

// Number from 5 to 12. Higher means slower but higher "processor gain",
// meaning (in nutshell) longer range and more robust against interference. 
#define SPREADING_FACTOR    9

// Transmit power in dBm. 0 dBm = 1 mW, enough for tabletop-testing. This value can be
// set anywhere between -9 dBm (0.125 mW) to 22 dBm (158 mW). Note that the maximum ERP
// (which is what your antenna maximally radiates) on the EU ISM band is 25 mW, and that
// transmissting without an antenna can damage your hardware.
#define TRANSMIT_POWER      0

String rxdata;
volatile bool rxFlag = false;
long counter = 0;
uint64_t last_tx = 0;
uint64_t tx_time;
uint64_t minimum_pause;


// object ens160
ENS160 ens160; 

//object bme280
Adafruit_BME280 bme; // I2C

// Set the LCD address to 0x27 or 0x3F for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);  // Adjust the address (0x27) if necessary

unsigned long delayTime;


void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  // begin serial
    //Serial.begin(9600);
  heltec_setup(); 
  both.println("Radio init");
  RADIOLIB_OR_HALT(radio.begin());
  // Set the callback function for received packets
  radio.setDio1Action(rx);
  // Set radio parameters
  both.printf("Frequency: %.2f MHz\n", FREQUENCY);
  RADIOLIB_OR_HALT(radio.setFrequency(FREQUENCY));
  both.printf("Bandwidth: %.1f kHz\n", BANDWIDTH);
  RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH));
  both.printf("Spreading Factor: %i\n", SPREADING_FACTOR);
  RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));
  both.printf("TX power: %i dBm\n", TRANSMIT_POWER);
  RADIOLIB_OR_HALT(radio.setOutputPower(TRANSMIT_POWER));
  // Start receiving
  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));

  // Initialize I2C with the chosen pins
  Wire1.begin(SDA_PIN, SCL_PIN);
  i2c.begin(Wire1, I2C_ADDRESS);

  //////////////////////////////////////////////////////////////// ENS 160 Setup 
  ens160.enableDebugging(Serial);

  Serial.println("begin..");
  while (ens160.begin(&i2c) != true)
  {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("success");
  #ifdef USE_INTERRUPT
    ens160.setInterruptPin(INTN);
    ens160.writeConfiguration
    (
          ENS160::Configuration::InterruptEnable
        | ENS160::Configuration::NewGeneralPurposeData
        | ENS160::Configuration::NewData
    );
  #endif
    ens160.startStandardMeasure();


  //////////////////////////////////////////////////////////////// BME 280 Setup
  // while(!Serial);    // time to get serial running
  // Serial.println(F("BME280 test"));

  // unsigned status;
    
  // // default settings
  // status = bme.begin(0x76); 

  // if (!status) {
  //   Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
  //   Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
  //   Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
  //   Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
  //   Serial.print("        ID of 0x60 represents a BME 280.\n");
  //   Serial.print("        ID of 0x61 represents a BME 680.\n");
  //   while (1) delay(10);
  // }
    
  // Serial.println("-- Default Test --");
  // delayTime = 1000;

  // Serial.println();

  //////////////////////////////////////////////////////////////// Initialize the LCD
  lcd.init();
  lcd.backlight();  // Turn on the backlight
  
}
long test;

void loop() {
  ens160.wait();

  // Enable Tools->Serial Plotter to see the sensor output as a graph

  if (ens160.update() == ENS16x::Result::Ok)
  {
    if (hasFlag(ens160.getDeviceStatus(), ENS16x::DeviceStatus::NewData)){
      Serial.print("AQI UBA:"); Serial.print((uint8_t)ens160.getAirQualityIndex_UBA());
      Serial.print("\tTVOC:"); Serial.print(ens160.getTvoc());
      Serial.print("\tECO2:"); Serial.println(ens160.getEco2());
    }

    if (hasFlag(ens160.getDeviceStatus(), ENS16x::DeviceStatus::NewGeneralPurposeData)){
      Serial.print("RS0:"); Serial.print(ens160.getRs0());
      Serial.print("\tRS1:"); Serial.print(ens160.getRs1());
      Serial.print("\tRS2:"); Serial.print(ens160.getRs2());
      Serial.print("\tRS3:"); Serial.println(ens160.getRs3());
    }  

  }
  test = (uint8_t)ens160.getAirQualityIndex_UBA();
  Serial.print(test);
  // if (bme.readHumidity() > 50.0) {
  //   digitalWrite(BUZZER_PIN, HIGH);
  // }
  // else {
  //   digitalWrite(BUZZER_PIN, LOW);
  // }

  //display information on LCD
  lcd.setCursor(0, 0); // Set the cursor to column 0, line 1
  lcd.print("AQI: " + String((uint8_t)ens160.getAirQualityIndex_UBA()));

  // lcd.setCursor(18, 0);
  // lcd.print((uint8_t)ens160.getAirQualityIndex_UBA());

  // lcd.setCursor(0, 1); // Set the cursor to column 0, line 1
  // lcd.print("Temp.: " + String(bme.readTemperature()) + (char)223 + "C");
  // // lcd.setCursor(13, 1);
  // // lcd.print(bme.readTemperature());
  // // lcd.setCursor(18, 1);
  // // lcd.print("°C");

  // lcd.setCursor(0, 2); // Set the cursor to column 0, line 1
  // lcd.print("Air Pres.: " + String(bme.readPressure()/ 100.0F) + "hPa");
  // // lcd.setCursor(13, 2);
  // // lcd.print(bme.readPressure()/ 100.0F);

  // lcd.setCursor(0, 3); // Set the cursor to column 0, line 1
  // lcd.print("Humidity: "+String(bme.readHumidity())+"%");

  heltec_loop();
  
  bool tx_legal = millis() > last_tx + minimum_pause;
  // Transmit a packet every PAUSE seconds or when the button is pressed
  if ((PAUSE && tx_legal && millis() - last_tx > (PAUSE * 1000)) || button.isSingleClick()) {
    // In case of button click, tell user to wait
    if (!tx_legal) {
      both.printf("Legal limit, wait %i sec.\n", (int)((minimum_pause - (millis() - last_tx)) / 1000) + 1);
      return;
    }
    both.printf("TX [%s] ", String(counter).c_str());
    radio.clearDio1Action();
    heltec_led(50); // 50% brightness is plenty for this LED
    tx_time = millis();
    RADIOLIB(radio.transmit(String(test).c_str()));
    tx_time = millis() - tx_time;
    heltec_led(0);
    if (_radiolib_status == RADIOLIB_ERR_NONE) {
      both.printf("OK (%i ms)\n", (int)tx_time);
    } else {
      both.printf("fail (%i)\n", _radiolib_status);
    }
    // Maximum 1% duty cycle
    minimum_pause = tx_time * 100;
    last_tx = millis();
    radio.setDio1Action(rx);
    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
  }

  // If a packet was received, display it and the RSSI and SNR
  if (rxFlag) {
    rxFlag = false;
    radio.readData(rxdata);
    if (_radiolib_status == RADIOLIB_ERR_NONE) {
      both.printf("RX [%s]\n", rxdata.c_str());
      both.printf("  RSSI: %.2f dBm\n", radio.getRSSI());
      both.printf("  SNR: %.2f dB\n", radio.getSNR());
    }
    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
  }

  // printValues();


  delay(delayTime);

}

// void printValues() {
//     Serial.print("Temperature = ");
//     Serial.print(bme.readTemperature());
//     Serial.println(" °C");

//     Serial.print("Pressure = ");
//     Serial.print(bme.readPressure() / 100.0F);
//     Serial.println(" hPa");

//     Serial.print("Approx. Altitude = ");
//     Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
//     Serial.println(" m");

//     Serial.print("Humidity = ");
//     Serial.print(bme.readHumidity());
//     Serial.println(" %");

//     Serial.println();
// }

void rx() {
  rxFlag = true;
}