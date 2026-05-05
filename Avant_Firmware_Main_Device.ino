#include <ArduinoBLE.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

// --- HARDWARE SETTINGS ---
Adafruit_MPU6050 mpu1; // Gyro 1
Adafruit_MPU6050 mpu2; // Gyro 2

const int batteryPin = A0;
const int servoPin = D9; 

// --- NATIVE PWM ---
const int servoChannel = 4; 
const int freq = 50;        
const int resolution = 12;  

// --- BATTERY CONSTANTS ---
const float dividerRatio = 3.1276;
const float minVoltage = 6.0; 
const float maxVoltage = 8.4;
float calibrationFactor = 1.1;
float batteryVoltage = 7.4; 

// --- BLE SETTINGS ---
BLEService gyroService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEStringCharacteristic dataChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLENotify | BLEWrite, 32);
unsigned long lastBatteryCheck = 0;

// --- ESP-NOW SETTINGS ---
typedef struct struct_message {
  float x;
  float y;
  float z;
} struct_message;

struct_message incomingData;
String g3Data = "0.00,0.00,0.00"; 

// Callback: This runs automatically in the background whenever G3 data hits the antenna
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingDataPtr, int len) {
  memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
  // Format it instantly for the iPhone
  g3Data = String(incomingData.x, 2) + "," + String(incomingData.y, 2) + "," + String(incomingData.z, 2);
}

void moveServo(int angle) {
  int duty = map(angle, 0, 270, 102, 512);
  ledcWrite(servoChannel, duty);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);
  Serial.println("--- Avant Actuation: Multi-Gyro Hub ---");

  // 1. INITIALIZE WI-FI & ESP-NOW (Do this first for antenna priority)
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
  } else {
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("ESP-NOW Receiver Active.");
  }

  2. INITIALIZE GYROS.                                     
  if (!mpu1.begin(0x68)) {
    Serial.println("G1 (0x68) FAILED! Check wiring.");
  } else { Serial.println("G1 Ready."); }
  
  if (!mpu2.begin(0x69)) {
    Serial.println("G2 (0x69) FAILED! Ensure AD0 is wired to 3.3V.");
  } else { Serial.println("G2 Ready."); }

  // 3. INITIALIZE BLE
  if (!BLE.begin()) {
    Serial.println("BLE FAILED!");
    while (1);
  }
  BLE.setLocalName("AvantProsthetic");
  BLE.setAdvertisedService(gyroService);
  gyroService.addCharacteristic(dataChar);
  BLE.addService(gyroService);
  BLE.advertise();
  Serial.println("Bluetooth OK.");

  // 4. PWM SETUP
  ledcSetup(servoChannel, freq, resolution);
  ledcAttachPin(servoPin, servoChannel);
  
  // Hardware Verification
  moveServo(0); delay(400);
  moveServo(90); delay(400);
  moveServo(0);

  Serial.println("--- SYSTEM READY ---");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    while (central.connected()) {
      // 1. RECEIVE COMMANDS
      if (dataChar.written()) {
        String command = dataChar.value();
        if (command.startsWith("D:")) {
          int angle = command.substring(2).toInt();
          if (batteryVoltage >= minVoltage) {
            angle = constrain(angle, 0, 270);
            moveServo(angle);
            Serial.print("BLE Move: ");
            Serial.println(angle);
          } else {
            Serial.println("Blocked: Low Battery");
          }
        }
      }

      // 2. STREAM G1
      sensors_event_t a1, g1, temp1;
      if (mpu1.getEvent(&a1, &g1, &temp1)) {
        dataChar.writeValue("G1:" + String(g1.gyro.x, 2) + "," + String(g1.gyro.y, 2) + "," + String(g1.gyro.z, 2));
        delay(15); 
      }

      // 3. STREAM G2
      sensors_event_t a2, g2, temp2;
      if (mpu2.getEvent(&a2, &g2, &temp2)) {
        dataChar.writeValue("G2:" + String(g2.gyro.x, 2) + "," + String(g2.gyro.y, 2) + "," + String(g2.gyro.z, 2));
        delay(15); 
      }

      // 4. STREAM G3 (Data is updated continuously in the background by ESP-NOW)
      dataChar.writeValue("G3:" + g3Data);
      delay(15);

      // 5. BATTERY CHECK
      if (millis() - lastBatteryCheck > 5000) {
        float rawADC = analogRead(batteryPin);
        batteryVoltage = (rawADC / 4095.0) * 3.3 * dividerRatio * calibrationFactor;
        float percentage = constrain(((batteryVoltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0, 0.0, 100.0);
        dataChar.writeValue("B:" + String(percentage, 1));
        lastBatteryCheck = millis();
      }
    }
    Serial.println("Disconnected.");
  }
}