#include <MichaelTLB-project-1_inferencing.h>
#include <ArduinoBLE.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <HardwareSerial.h>

// --- FIRMWARE VERSION ---
const String FW_VERSION = "2.0.0";

// --- HARDWARE SETTINGS ---
Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;

const int batteryPin = A0;
const int servoPin = 9;

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
  // NOTE: Consider adding gx, gy, gz here so G3 has full 6-axis data!
} struct_message;

struct_message incomingData;
String g3Data = "0.00,0.00,0.00";

// --- OTA SETTINGS ---
WebServer server(80);
bool isOTAMode = false;
bool otaRequested = false;

// --- MULTICORE & AI SETTINGS ---
TaskHandle_t TaskHardware;
TaskHandle_t TaskAI;
SemaphoreHandle_t sensorMutex; // Protects the AI buffer from being read while written

// The rolling buffer to feed Edge Impulse
float features_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
volatile int feature_index = 0;
volatile bool buffer_ready = false;

// Shared States
volatile int current_ai_state = 0; // The class ID predicted by AI
volatile float target_servo_angle = 90.0;
volatile float current_servo_angle = 90.0;

// --- GAIT TRAJECTORIES (Example) ---
// Define smooth arrays of angles for different states
const int TRAJ_LENGTH = 10;
int walk_traj[TRAJ_LENGTH] = {90, 85, 70, 60, 50, 60, 75, 85, 90, 90};
int stairs_traj[TRAJ_LENGTH] = {90, 80, 50, 30, 40, 60, 80, 90, 90, 90};
int traj_index = 0;

// Callback: runs when G3 data is received over ESP-NOW
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingDataPtr, int len) {
  memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
  g3Data = String(incomingData.x, 2) + "," + String(incomingData.y, 2) + "," + String(incomingData.z, 2);
}

void moveServo(int angle) {
  int duty = map(angle, 0, 270, 102, 512);
  ledcWrite(servoChannel, duty);
}

void startOTAMode() {
  // ... [Keep your exact original OTA code here] ...
  Serial.println("--- Entering OTA Mode ---");
  isOTAMode = false;
  BLE.stopAdvertise();
  delay(200);
  BLE.end();
  delay(500);
  esp_now_deinit();
  delay(200);
  WiFi.disconnect(true, true);
  delay(200);
  WiFi.mode(WIFI_OFF);
  delay(300);
  WiFi.mode(WIFI_AP);
  delay(300);

  bool apStarted = WiFi.softAP("Avant-Update");
  if (!apStarted) {
    Serial.println("Failed to start OTA AP");
    return;
  }
  Serial.print("OTA AP IP Address: ");
  Serial.println(WiFi.softAPIP());
  
  server.on("/update", HTTP_POST, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", Update.hasError() ? "FAIL" : "OK");
      delay(1000);
      ESP.restart();
    }, []() {
      HTTPUpload &upload = server.upload();
      if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("Receiving Firmware: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) Serial.printf("Success! Size: %u Bytes\n", upload.totalSize);
        else Update.printError(Serial);
      } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        Serial.println("OTA upload aborted");
      }
    });

  server.begin();
  isOTAMode = true;
  Serial.println("HTTP OTA Server Ready.");
}


// -----------------------------------------
// CORE 0: HARDWARE LOOP (Sensors & Servo)
// -----------------------------------------
void HardwareTask(void * parameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(50); // Exact 20Hz timing for AI (50ms interval)
  xLastWakeTime = xTaskGetTickCount();

  for(;;) {
    if (!isOTAMode) {
      sensors_event_t a1, g1, temp1;
      sensors_event_t a2, g2, temp2;
      
      mpu1.getEvent(&a1, &g1, &temp1);
      mpu2.getEvent(&a2, &g2, &temp2);

      // Mutex lock to update the AI buffer safely
      if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Shift buffer down to make room for new 18 axes
        for (int i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 18; i++) {
            features_buffer[i] = features_buffer[i + 18];
        }
        
        // Append new data at the end (Must match 'fusion_string' order!)
        int end = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 18;
        features_buffer[end + 0] = a1.acceleration.x;
        features_buffer[end + 1] = a1.acceleration.y;
        features_buffer[end + 2] = a1.acceleration.z;
        features_buffer[end + 3] = g1.gyro.x;
        features_buffer[end + 4] = g1.gyro.y;
        features_buffer[end + 5] = g1.gyro.z;
        
        features_buffer[end + 6] = a2.acceleration.x;
        features_buffer[end + 7] = a2.acceleration.y;
        features_buffer[end + 8] = a2.acceleration.z;
        features_buffer[end + 9] = g2.gyro.x;
        features_buffer[end + 10] = g2.gyro.y;
        features_buffer[end + 11] = g2.gyro.z;
        
        // ESP-NOW G3 Data
        features_buffer[end + 12] = incomingData.x;
        features_buffer[end + 13] = incomingData.y;
        features_buffer[end + 14] = incomingData.z;
        features_buffer[end + 15] = 0.0; // PADDING (MISSING G3 GYRO)
        features_buffer[end + 16] = 0.0; // PADDING (MISSING G3 GYRO)
        features_buffer[end + 17] = 0.0; // PADDING (MISSING G3 GYRO)

        if (feature_index < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            feature_index += 18;
        } else {
            buffer_ready = true; // Buffer is completely full and ready for AI
        }
        xSemaphoreGive(sensorMutex);
      }

      // -- SERVO TRAJECTORY LOGIC --
      // Advance trajectory every 50ms based on AI state
      if (current_ai_state == 1) {       // Assume 1 is Walking
          target_servo_angle = walk_traj[traj_index];
      } else if (current_ai_state == 3) { // Assume 3 is Stairs
          target_servo_angle = stairs_traj[traj_index];
      } else {                           // Idle / Default
          target_servo_angle = 90;
      }
      
      traj_index++;
      if(traj_index >= TRAJ_LENGTH) traj_index = 0;

      // Smoothly interpolate servo current angle toward target angle
      current_servo_angle += (target_servo_angle - current_servo_angle) * 0.2; 
      moveServo((int)current_servo_angle);
    }

    // FreeRTOS strict delay to maintain exactly 20Hz sensor sampling
    vTaskDelayUntil(&xLastWakeTime, xFrequency); 
  }
}

// -----------------------------------------
// CORE 1: AI INFERENCE LOOP
// -----------------------------------------
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        memcpy(out_ptr, features_buffer + offset, length * sizeof(float));
        xSemaphoreGive(sensorMutex);
        return 0;
    }
    return -1; // Failed to get mutex
}

void AITask(void * parameter) {
  for(;;) {
    if (buffer_ready && !isOTAMode) {
        ei_impulse_result_t result = { 0 };
        signal_t features_signal;
        features_signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
        features_signal.get_data = &raw_feature_get_data;

        EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);
        
        if (res == EI_IMPULSE_OK) {
            // Find the class with highest probability
            int best_class = 0;
            float best_score = 0.0;
            
            for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                if (result.classification[i].value > best_score) {
                    best_score = result.classification[i].value;
                    best_class = i;
                }
            }
            
            // Only update state if confident
            if (best_score > 0.70) {
                current_ai_state = best_class;
                Serial.printf("AI State Update: %s (%.2f)\n", result.classification[best_class].label, best_score);
            }
        }
    }
    // Yield to allow background WiFi/BLE tasks to process
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

// -----------------------------------------
// STANDARD ARDUINO SETUP
// -----------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- Avant Actuation: Dual-Core AI Hub ---");

  Wire.begin();
  
  // Create Mutex for shared variables
  sensorMutex = xSemaphoreCreateMutex();

  // Network and MPU setup...
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) Serial.println("ESP-NOW Init Failed");
  else { esp_now_register_recv_cb(OnDataRecv); Serial.println("ESP-NOW Active."); }

  if (!mpu1.begin(0x68)) Serial.println("G1 (0x68) FAILED!");
  else Serial.println("G1 Ready.");
  if (!mpu2.begin(0x69)) Serial.println("G2 (0x69) FAILED!");
  else Serial.println("G2 Ready.");

  if (!BLE.begin()) { Serial.println("BLE FAILED!"); while (1) {} }
  BLE.setLocalName("AvantProsthetic");
  BLE.setAdvertisedService(gyroService);
  gyroService.addCharacteristic(dataChar);
  BLE.addService(gyroService);
  BLE.advertise();
  Serial.println("Bluetooth OK.");

  ledcSetup(servoChannel, freq, resolution);
  ledcAttachPin(servoPin, servoChannel);
  moveServo(90);

  // --- PIN TASKS TO CORES ---
  xTaskCreatePinnedToCore(HardwareTask, "HardwareTask", 10000, NULL, 2, &TaskHardware, 0); // Core 0 (Priority 2)
  xTaskCreatePinnedToCore(AITask, "AITask", 20000, NULL, 1, &TaskAI, 1);                   // Core 1 (Priority 1)

  Serial.println("--- SYSTEM READY ---");
}

// -----------------------------------------
// MAIN LOOP (OTA & BLE Management)
// -----------------------------------------
void loop() {
  if (isOTAMode) {
    server.handleClient();
    delay(2);
    return;
  }
  
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());
    bool fwSent = false;

    while (central.connected()) {
      if (otaRequested) break;

      if (!fwSent && dataChar.subscribed()) {
        delay(100);
        dataChar.writeValue("FW:" + FW_VERSION);
        fwSent = true;
      }

      if (dataChar.written()) {
        String command = dataChar.value();
        command.trim();
        if (command == "D:OTA") otaRequested = true;
        // Manual servo overrides disabled to let AI control, but can be added back here
      }

      // Battery check
      if (millis() - lastBatteryCheck > 5000) {
        float rawADC = analogRead(batteryPin);
        batteryVoltage = (rawADC / 4095.0) * 3.3 * dividerRatio * calibrationFactor;
        float percentage = constrain(((batteryVoltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0, 0.0, 100.0);
        dataChar.writeValue("B:" + String(percentage, 1));
        lastBatteryCheck = millis();
      }
      
      delay(20); // Small loop delay for BLE
    }

    if (otaRequested) {
      otaRequested = false;
      startOTAMode();
    }
    
    if (!isOTAMode) Serial.println("Disconnected.");
  }
  delay(10);
}