#include "SSLClient.h"
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_PN532.h>
#include <AESLib.h>
#include <SPI.h>

// Define the SPI pins for PN532
#define PN532_SCK 18
#define PN532_MISO 19
#define PN532_MOSI 23
#define PN532_SS 15

#define SEALEVELPRESSURE_HPA (1013.25)  // Define sea level pressure

Adafruit_BME280 bme;  // Create BME280 object

//Please enter your certificate
#include "Secretss.h"

// ESP32 LilyGO-T-SIM7000G pins definition
#define MODEM_UART_BAUD 115200
#define MODEM_DTR 25
#define MODEM_TX 17
#define MODEM_RX 16
#define MODEM_PWRKEY 4
#define LED_PIN 32

#define SDA_PIN 21
#define SCL_PIN 22

// Create TwoWire instance for the I2C bus
TwoWire I2C1 = TwoWire(0);  // First I2C bus

// Set serial for debug console (to the Serial Monitor)
#define SerialMon Serial
// Set serial for AT commands (to the SIM7000 module)
#define SerialAT Serial2

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM7600   // Modem is SIM7000
#define TINY_GSM_RX_BUFFER 1024  // Set RX buffer to 1Kb

#define AWS_IOT_PUBLISH_TOPIC "subham"
#define AWS_IOT_SUBSCRIBE_TOPIC 

// Include after TinyGSM definitions
#include <TinyGsmClient.h>

// Your GPRS credentials (leave empty, if missing)
const char apn[] = "airtelgprs.com";  // Your APN
const char gprs_user[] = "";      // User
const char gprs_pass[] = "";      // Password
const char simPIN[] = "";         // SIM card PIN code, if any

// MQTT Config
const char client_name[] = "MyEsp32";
const char mqtt_broker[] = "a2u9gyfv7s162-ats.iot.ap-south-1.amazonaws.com";
int secure_port = 8883;  // TCP-TLS Port

// Layers stack
TinyGsm sim_modem(SerialAT);
TinyGsmClient gsm_transpor_layer(sim_modem);
SSLClient secure_presentation_layer(&gsm_transpor_layer);
PubSubClient client(secure_presentation_layer);

float latitude, longitude;

Adafruit_PN532 nfc(PN532_SS);
byte aesKey[] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0xF8, 0x09, 0xCF, 0x4F, 0x3C, 0x76 };  // 128-bit AES key
byte aesIV[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };   // 128-bit IV
AESLib aesLib;
uint8_t success;
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
uint8_t uidLength;
bool check = false;

String pilotID;
String droneID = "UA005M4S1EX";

// For read the MQTT events
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// To connect to the broker
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(client_name)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(AWS_IOT_PUBLISH_TOPIC, "hello world");
      // ... and resubscribe
      client.subscribe(AWS_IOT_PUBLISH_TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("...try again in 5 seconds");
      delay(5000);
    }
  }
}

void setupModem() {
  // To skip it, call init() instead of restart()
  SerialMon.print("Initializing modem...");
  sim_modem.restart();
  String modemInfo = sim_modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);
  setupModem();
}

void getData() {
  // Read altitude based on sea level pressure
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  SerialAT.println("AT+CGPSINFO");
  delay(1000);

  //Serial.println(SerialAT.available());
  if (SerialAT.available()) {

    // Read incoming GPS data
    String gpsData = SerialAT.readString();

    // Check if the received data is a valid NMEA sentence
    if (gpsData.indexOf("+CGPSINFO:") != -1) {
      parseGPSData(gpsData);
      Serial.print("Latitude: ");
      Serial.println(latitude, 8);
      Serial.print("Longitude: ");
      Serial.println(longitude, 8);
      Serial.print("Altitude: ");
      Serial.println(altitude);
      Serial.print("Pilot ID: ");
      Serial.println(pilotID);
    }
  }
  StaticJsonDocument<200> doc;
  doc["lat"] = latitude;
  doc["lng"] = longitude;
  doc["alt"] = altitude;
  doc["uin"] = droneID;
  doc["pno"] = pilotID;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);  // print to client

  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}

void parseGPSData(const String& gpsData) {
  //+CGPSINFO: 2017.557807,N,08544.589645,E,020724,063351.0,101.1,0.0,
  // Find the position of the first colon and comma
  int colonIndex = gpsData.indexOf(':');
  int firstCommaIndex = gpsData.indexOf(',');

  // Extract the latitude and longitude substrings
  String latitudeStr = gpsData.substring(colonIndex + 2, firstCommaIndex);
  char latitudeDir = gpsData.charAt(firstCommaIndex + 1);
  int secondCommaIndex = gpsData.indexOf(',', firstCommaIndex + 3);
  String longitudeStr = gpsData.substring(firstCommaIndex + 3, secondCommaIndex);
  char longitudeDir = gpsData.charAt(secondCommaIndex + 1);

  // Print the extracted latitude and longitude
  Serial.print("Latitude: ");
  Serial.print(latitudeStr);
  Serial.print(" ");
  Serial.println(latitudeDir);

  Serial.print("Longitude: ");
  Serial.print(longitudeStr);
  Serial.print(" ");
  Serial.println(longitudeDir);

  float latitudeValue = latitudeStr.substring(0, 2).toFloat() + latitudeStr.substring(2).toFloat() / 60.0;
  float longitudeValue = longitudeStr.substring(0, 3).toFloat() + longitudeStr.substring(3).toFloat() / 60.0;

  if (latitudeDir == 'S') {
    latitudeValue = -latitudeValue;
  }
  if (longitudeDir == 'W') {
    longitudeValue = -longitudeValue;
  }

  latitude = latitudeValue;
  longitude = longitudeValue;
}

void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("incoming: ");
  Serial.println(topic);

  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  Serial.println(message);
}

void verify() {
  Serial.println("Found an NFC card!");
  Serial.print("UID Length: ");
  Serial.print(uidLength, DEC);
  Serial.println(" bytes");
  Serial.print("UID Value: ");
  for (uint8_t i = 0; i < uidLength; i++) {
    Serial.print(" 0x");
    Serial.print(uid[i], HEX);
  }
  Serial.println("");

  if (uidLength == 4) {
    uint8_t data[16];
    // Authenticate using the default key (0xFF 0xFF 0xFF 0xFF 0xFF 0xFF)
    success = nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 1, 0, (uint8_t*)"\xFF\xFF\xFF\xFF\xFF\xFF");
    if (success) {
      // Read 16 bytes from block 1
      success = nfc.mifareclassic_ReadDataBlock(1, data);
      if (success) {
        Serial.println("Encrypted data read from card successfully.");
        Serial.print("Encrypted data: ");
        for (uint8_t i = 0; i < 16; i++) {
          Serial.print(data[i], HEX);
          Serial.print(" ");
        }
        Serial.println();

        byte decryptedData[16];
        aesLib.decrypt(data, 16, decryptedData, aesKey, 128, aesIV);

        String decryptedString;
        for (uint8_t i = 0; i < 16; i++) {
          decryptedString += (char)decryptedData[i];
        }
        Serial.println();

        // Additional condition: first byte is a vowel, third byte is a consonant, and thirteenth byte is '2'
        if (isVowel(decryptedData[0]) && isConsonant(decryptedData[9]) && isVowel(decryptedData[15])) {
          String part1 = decryptedString.substring(1,9);
          String part2 = decryptedString.substring(10,15);
          pilotID = part1 + part2;
          Serial.println("Card verified");
          check = true;
          delay(1000);
        } else {
          Serial.println("Card verification failed.");
        }
      } else {
        Serial.println("Failed to read data from block 1.");
      }
    } else {
      Serial.println("Authentication failed.");
    }
  }
}

bool isVowel(char c) {
  c = tolower(c);
  return (c == 'a' || c == 'e' || c == 'i' || c == 'o' || c == 'u');
}

bool isConsonant(char c) {
  c = tolower(c);
  return (c >= 'a' && c <= 'z' && !isVowel(c));
  delay(1000);
}

void setup() {
  SerialMon.begin(9600);
  delay(100);

  // Initialize the first I2C bus
  I2C1.begin(SDA, SCL, 100000);
  Serial.println("I2C initialized successfully");

  if (!bme.begin(0x76, &I2C1)) {  // Specify custom I2C address
    Serial.println("Could not find BME280 sensor, check wiring!");
    while (1);
  }

  Serial.println("BME280 initialized successfully");

  // Initialize the LED pin as an output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize the PN532 module
  nfc.begin();

  // Check if the PN532 is connected properly by reading its firmware version
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("Didn't find PN53x board");
    while (1);
  }

  // Configure the PN532 to read NFC cards
  nfc.SAMConfig();
  Serial.println("Waiting for an NFC card ...");
  delay(1000);

  // Set SIM module baud rate and UART pins
  SerialAT.begin(MODEM_UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);

  //Add CA Certificate
  secure_presentation_layer.setCACert(AWS_CERT_CA);
  secure_presentation_layer.setCertificate(AWS_CERT_CRT);
  secure_presentation_layer.setPrivateKey(AWS_CERT_PRIVATE);

  // SIM modem initial setup
  sim_modem.restart();

  // MQTT init
  client.setServer(mqtt_broker, secure_port);
  client.setCallback(callback);

  Serial.println("GPS initialized");
  SerialAT.println("AT+CGPS=1");
}

void loop() {
  SerialMon.print("Initializing modem...");
  if (!sim_modem.init()) {
    SerialMon.print(" fail... restarting modem...");
    // Restart takes quite some time
    // Use modem.init() if you don't need the complete restart
    if (!sim_modem.restart()) {
      SerialMon.println(" fail... even after restart");
      return;
    }
  }
  SerialMon.println(" OK");

  // General information
  String name = sim_modem.getModemName();
  Serial.println("Modem Name: " + name);
  String modem_info = sim_modem.getModemInfo();
  Serial.println("Modem Info: " + modem_info);

  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && sim_modem.getSimStatus() != 3) {
    sim_modem.simUnlock(simPIN);
  }

  // Set modes
  /*
    2 Automatic
    13 GSM only
    38 LTE only
    51 GSM and LTE only
  * * * */
  sim_modem.setNetworkMode(2);
  delay(3000);
  /*
    1 CAT-M
    2 NB-Iot
    3 CAT-M and NB-IoT
  * * */
  //sim_modem.setPreferredMode(3);
  //delay(3000);

  // Wait for network availability
  SerialMon.print("Waiting for network...");
  if (!sim_modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" OK");

  // Connect to the GPRS network
  SerialMon.print("Connecting to network...");
  if (!sim_modem.isNetworkConnected()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" OK");

  // Connect to APN
  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!sim_modem.gprsConnect(apn)) {
    SerialMon.println(" fail");
    return;
  }
  digitalWrite(LED_PIN, HIGH);
  SerialMon.println(" OK");

  // More info..
  Serial.println("");
  String ccid = sim_modem.getSimCCID();
  Serial.println("CCID: " + ccid);
  String imei = sim_modem.getIMEI();
  Serial.println("IMEI: " + imei);
  String cop = sim_modem.getOperator();
  Serial.println("Operator: " + cop);
  IPAddress local = sim_modem.localIP();
  Serial.println("Local IP: " + String(local));
  int csq = sim_modem.getSignalQuality();
  Serial.println("Signal quality: " + String(csq));

  // Check if an NFC card is present
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  if (success) {
    verify();
  a
    }
  }
}