/*
 * ESP32-S3 Dron PID Tuning System
 * Brushless motor 4x A2212/13T control
 * MPU6050 stabilization
 * WiFi Web Joystick Interface
 * 
 * Motor Pins: 4,5,6,7 (PWM 1000-2000µs)
 * MPU6050: SDA=1, SCL=2, Address=0x69
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps612.h>
#include <math.h>

// ============================================
// CONFIGURATION
// ============================================

// WiFi
const char* ssid = "ESP32_DRONE";
const char* password = "drone12345";
const int wifiPort = 80;

// Motor Pins (PWM)
#define MOTOR_FRONT_RIGHT 4
#define MOTOR_FRONT_LEFT  5
#define MOTOR_BACK_LEFT   6
#define MOTOR_BACK_RIGHT  7

#define PWM_FREQ 50        // 50Hz ESC frequency
#define PWM_BITS 16        // 16-bit resolution
#define PWM_MIN 1000       // 1000µs
#define PWM_MAX 2000       // 2000µs
#define PWM_IDLE 1000      // ESC idle

// MPU6050
#define MPU_ADDR 0x69
#define SDA_PIN 1
#define SCL_PIN 2

// PID Parameters (bularni tuning qilasiz)
float kp_roll = 1.5;
float ki_roll = 0.05;
float kd_roll = 0.8;

float kp_pitch = 1.5;
float ki_pitch = 0.05;
float kd_pitch = 0.8;

float kp_yaw = 2.0;
float ki_yaw = 0.1;
float kd_yaw = 0.4;

// ============================================
// GLOBAL VARIABLES
// ============================================

WebServer server(wifiPort);
MPU6050 mpu;

// Motor throttle values (0-255, converted to PWM)
uint16_t motor[4] = {1000, 1000, 1000, 1000};

// IMU Data
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float roll, pitch, yaw;
float rollRate, pitchRate, yawRate;

// Control inputs from joystick
float controlRoll = 0;    // -100 to 100
float controlPitch = 0;   // -100 to 100
float controlYaw = 0;     // -100 to 100
float controlThrottle = 0; // 0 to 100

// PID state
float rollError_prev = 0, rollIntegral = 0;
float pitchError_prev = 0, pitchIntegral = 0;
float yawError_prev = 0, yawIntegral = 0;

// Status
bool droneArmed = false;
unsigned long lastTime = 0;
float dt = 0.01; // 10ms loop

// MPU6050 DMP setup
uint8_t mpuIntStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3] = {0, 0, 0};

// ============================================
// SETUP
// ============================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\nESP32-S3 Dron PID Test Tizimi");
  Serial.println("================================");
  
  // Initialize I2C for MPU6050
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  delay(100);
  
  // Initialize MPU6050
  initMPU6050();
  
  // Initialize PWM channels for motors
  initMotorPWM();
  
  // Setup WiFi
  setupWiFi();
  
  // Setup web server routes
  setupWebServer();
  
  Serial.println("Tizim tayyor! WiFi: ESP32_DRONE (password: drone12345)");
  Serial.println("Web server: http://192.168.4.1");
  
  lastTime = millis();
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {
  // Handle web requests
  server.handleClient();
  
  // Update IMU data
  readMPU6050();
  
  // Calculate time delta
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  if (dt > 0.1) dt = 0.01; // Safety check
  
  // Run PID controller
  if (droneArmed) {
    calculatePID();
    updateMotors();
  } else {
    // All motors idle
    motor[0] = motor[1] = motor[2] = motor[3] = 1000;
    updateMotors();
  }
  
  // Send telemetry every 100ms
  static unsigned long lastTelem = 0;
  if (millis() - lastTelem > 100) {
    sendTelemetry();
    lastTelem = millis();
  }
  
  delay(10); // 100Hz loop
}

// ============================================
// MPU6050 INITIALIZATION
// ============================================

void initMPU6050() {
  Serial.println("MPU6050 ishga tushirilmoqda...");
  
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("ERROR: MPU6050 topilmadi!");
    while(1);
  }
  
  Serial.println("MPU6050 ulanildi!");
  
  // Initialize DMP
  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus != 0) {
    Serial.printf("DMP xatosi: %d\n", devStatus);
    while(1);
  }
  
  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();
  
  // Calibration (tuning uchun)
  mpu.setXGyroOffset(50);
  mpu.setYGyroOffset(20);
  mpu.setZGyroOffset(-10);
  
  Serial.println("MPU6050 tayyor!");
}

// ============================================
// MPU6050 DATA READ
// ============================================

void readMPU6050() {
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;
  
  // Get quaternion values
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  
  // Get gravity
  mpu.dmpGetGravity(&gravity, &q);
  
  // Get Euler angles (roll, pitch, yaw)
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
  // Convert to degrees
  roll = ypr[2] * 180 / M_PI;
  pitch = ypr[1] * 180 / M_PI;
  yaw = ypr[0] * 180 / M_PI;
  
  // MPU 45 degree rotation correction
  // Dron X-shape (+shape emas), shuning uchun:
  float temp_roll = roll;
  roll = (roll * cos(45 * M_PI / 180) - pitch * sin(45 * M_PI / 180));
  pitch = (temp_roll * sin(45 * M_PI / 180) + pitch * cos(45 * M_PI / 180));
  
  // Get gyro rates
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  
  rollRate = gx / 131.0;  // LSB sensitivity 131 LSB/(°/s)
  pitchRate = gy / 131.0;
  yawRate = gz / 131.0;
}

// ============================================
// PID CONTROLLER
// ============================================

void calculatePID() {
  // Roll PID
  float rollError = controlRoll - roll;
  rollIntegral += rollError * dt;
  rollIntegral = constrain(rollIntegral, -100, 100);
  float rollDerivative = (rollError - rollError_prev) / dt;
  float rollOutput = kp_roll * rollError + ki_roll * rollIntegral + kd_roll * rollDerivative;
  rollError_prev = rollError;
  rollOutput = constrain(rollOutput, -100, 100);
  
  // Pitch PID
  float pitchError = controlPitch - pitch;
  pitchIntegral += pitchError * dt;
  pitchIntegral = constrain(pitchIntegral, -100, 100);
  float pitchDerivative = (pitchError - pitchError_prev) / dt;
  float pitchOutput = kp_pitch * pitchError + ki_pitch * pitchIntegral + kd_pitch * pitchDerivative;
  pitchError_prev = pitchError;
  pitchOutput = constrain(pitchOutput, -100, 100);
  
  // Yaw PID
  float yawError = controlYaw - yawRate;
  yawIntegral += yawError * dt;
  yawIntegral = constrain(yawIntegral, -100, 100);
  float yawDerivative = (yawError - yawError_prev) / dt;
  float yawOutput = kp_yaw * yawError + ki_yaw * yawIntegral + kd_yaw * yawDerivative;
  yawError_prev = yawError;
  yawOutput = constrain(yawOutput, -100, 100);
  
  // Motor mixing (+shape configuration)
  // Front-Right (4)
  motor[0] = controlThrottle + rollOutput - pitchOutput + yawOutput;
  // Front-Left (5)
  motor[1] = controlThrottle - rollOutput - pitchOutput - yawOutput;
  // Back-Left (6)
  motor[2] = controlThrottle - rollOutput + pitchOutput + yawOutput;
  // Back-Right (7)
  motor[3] = controlThrottle + rollOutput + pitchOutput - yawOutput;
  
  // Constrain to throttle range
  for (int i = 0; i < 4; i++) {
    motor[i] = constrain(motor[i], 0, 100);
  }
}

// ============================================
// MOTOR CONTROL
// ============================================

void initMotorPWM() {
  Serial.println("Motor PWM-ni sozlash...");
  
  ledcSetup(0, PWM_FREQ, PWM_BITS);
  ledcSetup(1, PWM_FREQ, PWM_BITS);
  ledcSetup(2, PWM_FREQ, PWM_BITS);
  ledcSetup(3, PWM_FREQ, PWM_BITS);
  
  ledcAttachPin(MOTOR_FRONT_RIGHT, 0);
  ledcAttachPin(MOTOR_FRONT_LEFT, 1);
  ledcAttachPin(MOTOR_BACK_LEFT, 2);
  ledcAttachPin(MOTOR_BACK_RIGHT, 3);
  
  // Set all motors to idle
  updateMotors();
  
  Serial.println("Motor PWM tayyor!");
}

void updateMotors() {
  // Convert 0-100 to 1000-2000µs
  uint16_t pwm0 = PWM_MIN + (motor[0] * (PWM_MAX - PWM_MIN)) / 100;
  uint16_t pwm1 = PWM_MIN + (motor[1] * (PWM_MAX - PWM_MIN)) / 100;
  uint16_t pwm2 = PWM_MIN + (motor[2] * (PWM_MAX - PWM_MIN)) / 100;
  uint16_t pwm3 = PWM_MIN + (motor[3] * (PWM_MAX - PWM_MIN)) / 100;
  
  // 16-bit resolution = 65536 ticks per 20ms
  // 1µs = 65536/20000 ≈ 3.27 ticks
  ledcWrite(0, pwm0 * 3.27);
  ledcWrite(1, pwm1 * 3.27);
  ledcWrite(2, pwm2 * 3.27);
  ledcWrite(3, pwm3 * 3.27);
}

// ============================================
// WIFI & WEB SERVER
// ============================================

void setupWiFi() {
  Serial.println("\nWiFi hotspot ishga tushirilmoqda...");
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.printf("AP IP address: %s\n", IP.toString().c_str());
}

void setupWebServer() {
  // Web interface
  server.on("/", HTTP_GET, handleRoot);
  
  // Joystick control
  server.on("/control", HTTP_POST, handleControl);
  
  // Arm/Disarm
  server.on("/arm", HTTP_POST, handleArm);
  
  // Telemetry
  server.on("/telemetry", HTTP_GET, handleTelemetry);
  
  // PID settings
  server.on("/pid", HTTP_POST, handlePIDUpdate);
  
  server.begin();
  Serial.println("Web server boshlanmoqda...");
}

void handleRoot() {
  String html = getWebInterface();
  server.send(200, "text/html", html);
}

void handleControl() {
  if (!server.hasArg("roll") || !server.hasArg("pitch") || 
      !server.hasArg("yaw") || !server.hasArg("throttle")) {
    server.send(400, "application/json", "{\"error\":\"Noto'g'ri parametrlar\"}");
    return;
  }
  
  controlRoll = server.arg("roll").toFloat();
  controlPitch = server.arg("pitch").toFloat();
  controlYaw = server.arg("yaw").toFloat();
  controlThrottle = server.arg("throttle").toFloat();
  
  // Constrain values
  controlRoll = constrain(controlRoll, -30, 30);
  controlPitch = constrain(controlPitch, -30, 30);
  controlYaw = constrain(controlYaw, -100, 100);
  controlThrottle = constrain(controlThrottle, 0, 100);
  
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleArm() {
  if (!server.hasArg("armed")) {
    server.send(400, "application/json", "{\"error\":\"Parametr kerak\"}");
    return;
  }
  
  String armedStr = server.arg("armed");
  droneArmed = (armedStr == "true");
  
  if (droneArmed) {
    Serial.println("🟢 DRON ISHGA TUSHDI!");
    controlRoll = controlPitch = controlYaw = 0;
    controlThrottle = 20; // Minimal throttle
  } else {
    Serial.println("🔴 DRON TO'XTADI!");
    controlRoll = controlPitch = controlYaw = controlThrottle = 0;
  }
  
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleTelemetry() {
  String json = "{";
  json += "\"roll\":" + String(roll, 2) + ",";
  json += "\"pitch\":" + String(pitch, 2) + ",";
  json += "\"yaw\":" + String(yaw, 2) + ",";
  json += "\"armed\":" + String(droneArmed ? "true" : "false") + ",";
  json += "\"motors\":[" + String(motor[0]) + "," + String(motor[1]) + "," 
          + String(motor[2]) + "," + String(motor[3]) + "]";
  json += "}";
  
  server.send(200, "application/json", json);
}

void handlePIDUpdate() {
  if (server.hasArg("kp_roll")) kp_roll = server.arg("kp_roll").toFloat();
  if (server.hasArg("ki_roll")) ki_roll = server.arg("ki_roll").toFloat();
  if (server.hasArg("kd_roll")) kd_roll = server.arg("kd_roll").toFloat();
  
  if (server.hasArg("kp_pitch")) kp_pitch = server.arg("kp_pitch").toFloat();
  if (server.hasArg("ki_pitch")) ki_pitch = server.arg("ki_pitch").toFloat();
  if (server.hasArg("kd_pitch")) kd_pitch = server.arg("kd_pitch").toFloat();
  
  if (server.hasArg("kp_yaw")) kp_yaw = server.arg("kp_yaw").toFloat();
  if (server.hasArg("ki_yaw")) ki_yaw = server.arg("ki_yaw").toFloat();
  if (server.hasArg("kd_yaw")) kd_yaw = server.arg("kd_yaw").toFloat();
  
  Serial.println("PID yangilandi!");
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void sendTelemetry() {
  Serial.printf("Roll: %.1f° | Pitch: %.1f° | Yaw: %.1f° | Armed: %s\n",
                roll, pitch, yaw, droneArmed ? "YES" : "NO");
  Serial.printf("Motors: [%d, %d, %d, %d]\n", motor[0], motor[1], motor[2], motor[3]);
}

// ============================================
// WEB INTERFACE (HTML+JS)
// ============================================

String getWebInterface() {
  return R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>🚁 Dron PID Test</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      min-height: 100vh;
      padding: 20px;
      color: #333;
    }
    .container {
      max-width: 1200px;
      margin: 0 auto;
      background: white;
      border-radius: 15px;
      box-shadow: 0 20px 60px rgba(0,0,0,0.3);
      overflow: hidden;
    }
    header {
      background: #2c3e50;
      color: white;
      padding: 20px;
      text-align: center;
    }
    .content {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 20px;
      padding: 30px;
    }
    @media (max-width: 768px) {
      .content { grid-template-columns: 1fr; }
    }
    .section {
      background: #f8f9fa;
      border: 2px solid #dee2e6;
      border-radius: 10px;
      padding: 20px;
    }
    .section h2 {
      color: #667eea;
      margin-bottom: 15px;
      font-size: 18px;
    }
    .joystick-container {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 15px;
      margin-bottom: 20px;
    }
    .joystick {
      background: #fff;
      border: 3px solid #667eea;
      border-radius: 10px;
      width: 100%;
      aspect-ratio: 1;
      position: relative;
      touch-action: none;
      cursor: grab;
    }
    .joystick:active { cursor: grabbing; }
    .joystick canvas {
      width: 100%;
      height: 100%;
      display: block;
    }
    .throttle-container {
      margin-bottom: 20px;
    }
    .throttle-slider {
      width: 100%;
      height: 30px;
      margin: 10px 0;
    }
    .status {
      padding: 15px;
      border-radius: 8px;
      margin-bottom: 15px;
      font-weight: bold;
    }
    .status.armed {
      background: #d4edda;
      color: #155724;
      border: 2px solid #28a745;
    }
    .status.disarmed {
      background: #f8d7da;
      color: #721c24;
      border: 2px solid #dc3545;
    }
    .button-group {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin-bottom: 20px;
    }
    button {
      padding: 12px 20px;
      border: none;
      border-radius: 8px;
      font-size: 16px;
      font-weight: bold;
      cursor: pointer;
      transition: all 0.3s;
    }
    .btn-arm {
      background: #28a745;
      color: white;
    }
    .btn-arm:hover {
      background: #218838;
      transform: scale(1.05);
    }
    .btn-disarm {
      background: #dc3545;
      color: white;
    }
    .btn-disarm:hover {
      background: #c82333;
      transform: scale(1.05);
    }
    .btn-reset {
      background: #ffc107;
      color: #333;
    }
    .btn-reset:hover {
      background: #e0a800;
    }
    .telemetry-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin-bottom: 20px;
    }
    .telemetry-item {
      background: white;
      padding: 15px;
      border-radius: 8px;
      border-left: 4px solid #667eea;
    }
    .telemetry-label {
      font-size: 12px;
      color: #666;
      text-transform: uppercase;
      margin-bottom: 5px;
    }
    .telemetry-value {
      font-size: 24px;
      font-weight: bold;
      color: #667eea;
    }
    .motor-indicators {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 10px;
      margin-bottom: 20px;
    }
    .motor {
      background: white;
      padding: 15px;
      border-radius: 8px;
      text-align: center;
      border: 2px solid #dee2e6;
    }
    .motor-label {
      font-size: 12px;
      color: #666;
      margin-bottom: 5px;
    }
    .motor-bar {
      height: 150px;
      background: #e9ecef;
      border-radius: 5px;
      overflow: hidden;
      margin-bottom: 10px;
    }
    .motor-fill {
      height: 100%;
      background: linear-gradient(to top, #667eea, #764ba2);
      width: 100%;
      transition: all 0.05s;
    }
    .motor-value {
      font-size: 16px;
      font-weight: bold;
      color: #333;
    }
    .pid-control {
      display: grid;
      grid-template-columns: 1fr;
      gap: 15px;
    }
    .pid-row {
      background: white;
      padding: 12px;
      border-radius: 8px;
      border: 1px solid #dee2e6;
    }
    .pid-label {
      font-size: 12px;
      font-weight: bold;
      color: #667eea;
      margin-bottom: 5px;
    }
    .pid-inputs {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 8px;
    }
    .pid-input-group {
      display: flex;
      flex-direction: column;
    }
    .pid-input-group label {
      font-size: 10px;
      color: #666;
      margin-bottom: 2px;
    }
    .pid-input-group input {
      padding: 5px;
      border: 1px solid #ddd;
      border-radius: 4px;
      font-size: 12px;
    }
  </style>
</head>
<body>
  <div class="container">
    <header>
      <h1>🚁 ESP32-S3 Dron PID Test Tizimi</h1>
      <p>Joystik bilan boshqaring va PID parametrlarini sozlang</p>
    </header>
    
    <div class="content">
      <!-- LEFT SIDE: CONTROLS -->
      <div>
        <!-- Status -->
        <div id="status" class="status disarmed">⚫ ISHGA TUSHMAGAN</div>
        
        <!-- Arm/Disarm Buttons -->
        <div class="button-group">
          <button class="btn-arm" onclick="armDrone()">🟢 ISHGA TUSH</button>
          <button class="btn-disarm" onclick="disarmDrone()">🔴 TO'XTISH</button>
        </div>
        
        <!-- Joysticks -->
        <div class="section">
          <h2>Joystik Boshqaruvi</h2>
          <div class="joystick-container">
            <div>
              <div class="joystick" id="leftJoystick"></div>
              <p style="text-align: center; margin-top: 10px; font-size: 12px; color: #666;">
                ↑↓ Throttle + Yaw
              </p>
            </div>
            <div>
              <div class="joystick" id="rightJoystick"></div>
              <p style="text-align: center; margin-top: 10px; font-size: 12px; color: #666;">
                Roll + Pitch
              </p>
            </div>
          </div>
        </div>
        
        <!-- Throttle Slider -->
        <div class="section">
          <h2>Qayta Tekislash</h2>
          <label>Throttle: <span id="throttleValue">0</span>%</label>
          <input type="range" min="0" max="100" value="0" class="throttle-slider" 
                 id="throttleSlider" oninput="updateThrottle(this.value)">
        </div>
      </div>
      
      <!-- RIGHT SIDE: TELEMETRY & PID -->
      <div>
        <!-- Telemetry -->
        <div class="section">
          <h2>📊 Sensor Ma'lumotlari</h2>
          <div class="telemetry-grid">
            <div class="telemetry-item">
              <div class="telemetry-label">Roll</div>
              <div class="telemetry-value" id="telemetryRoll">0.0°</div>
            </div>
            <div class="telemetry-item">
              <div class="telemetry-label">Pitch</div>
              <div class="telemetry-value" id="telemetryPitch">0.0°</div>
            </div>
            <div class="telemetry-item">
              <div class="telemetry-label">Yaw</div>
              <div class="telemetry-value" id="telemetryYaw">0.0°</div>
            </div>
            <div class="telemetry-item">
              <div class="telemetry-label">Status</div>
              <div class="telemetry-value" id="telemetryStatus" style="font-size: 18px;">🔴</div>
            </div>
          </div>
        </div>
        
        <!-- Motor Indicators -->
        <div class="section">
          <h2>⚙️ Motorlar</h2>
          <div class="motor-indicators">
            <div class="motor">
              <div class="motor-label">Motor 1</div>
              <div class="motor-bar"><div class="motor-fill" id="motor0"></div></div>
              <div class="motor-value" id="motorValue0">0</div>
            </div>
            <div class="motor">
              <div class="motor-label">Motor 2</div>
              <div class="motor-bar"><div class="motor-fill" id="motor1"></div></div>
              <div class="motor-value" id="motorValue1">0</div>
            </div>
            <div class="motor">
              <div class="motor-label">Motor 3</div>
              <div class="motor-bar"><div class="motor-fill" id="motor2"></div></div>
              <div class="motor-value" id="motorValue2">0</div>
            </div>
            <div class="motor">
              <div class="motor-label">Motor 4</div>
              <div class="motor-bar"><div class="motor-fill" id="motor3"></div></div>
              <div class="motor-value" id="motorValue3">0</div>
            </div>
          </div>
        </div>
        
        <!-- PID Tuning -->
        <div class="section">
          <h2>🎛️ PID Sozlash</h2>
          <div class="pid-control">
            <div>
              <div class="pid-label">Roll PID</div>
              <div class="pid-inputs">
                <div class="pid-input-group">
                  <label>KP</label>
                  <input type="number" step="0.1" value="1.5" id="kp_roll" onchange="updatePID()">
                </div>
                <div class="pid-input-group">
                  <label>KI</label>
                  <input type="number" step="0.01" value="0.05" id="ki_roll" onchange="updatePID()">
                </div>
                <div class="pid-input-group">
                  <label>KD</label>
                  <input type="number" step="0.1" value="0.8" id="kd_roll" onchange="updatePID()">
                </div>
              </div>
            </div>
            <div>
              <div class="pid-label">Pitch PID</div>
              <div class="pid-inputs">
                <div class="pid-input-group">
                  <label>KP</label>
                  <input type="number" step="0.1" value="1.5" id="kp_pitch" onchange="updatePID()">
                </div>
                <div class="pid-input-group">
                  <label>KI</label>
                  <input type="number" step="0.01" value="0.05" id="ki_pitch" onchange="updatePID()">
                </div>
                <div class="pid-input-group">
                  <label>KD</label>
                  <input type="number" step="0.1" value="0.8" id="kd_pitch" onchange="updatePID()">
                </div>
              </div>
            </div>
            <div>
              <div class="pid-label">Yaw PID</div>
              <div class="pid-inputs">
                <div class="pid-input-group">
                  <label>KP</label>
                  <input type="number" step="0.1" value="2.0" id="kp_yaw" onchange="updatePID()">
                </div>
                <div class="pid-input-group">
                  <label>KI</label>
                  <input type="number" step="0.01" value="0.1" id="ki_yaw" onchange="updatePID()">
                </div>
                <div class="pid-input-group">
                  <label>KD</label>
                  <input type="number" step="0.1" value="0.4" id="kd_yaw" onchange="updatePID()">
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>

  <script>
    let armed = false;
    let leftJoyX = 0, leftJoyY = 0;
    let rightJoyX = 0, rightJoyY = 0;
    
    // Joystick handler
    class Joystick {
      constructor(elementId, onMove) {
        this.element = document.getElementById(elementId);
        this.onMove = onMove;
        this.touch = false;
        
        this.canvas = document.createElement('canvas');
        this.canvas.width = this.canvas.height = 200;
        this.element.appendChild(this.canvas);
        this.ctx = this.canvas.getContext('2d');
        
        this.centerX = this.canvas.width / 2;
        this.centerY = this.canvas.height / 2;
        this.radius = 80;
        this.handleRadius = 25;
        this.handleX = this.centerX;
        this.handleY = this.centerY;
        
        this.element.addEventListener('mousedown', (e) => this.handleStart(e));
        this.element.addEventListener('mousemove', (e) => this.handleMove(e));
        this.element.addEventListener('mouseup', () => this.handleEnd());
        
        this.element.addEventListener('touchstart', (e) => this.handleStart(e.touches[0]));
        this.element.addEventListener('touchmove', (e) => this.handleMove(e.touches[0]));
        this.element.addEventListener('touchend', () => this.handleEnd());
        
        this.draw();
      }
      
      handleStart(e) {
        this.touch = true;
        this.handleMove(e);
      }
      
      handleMove(e) {
        if (!this.touch) return;
        
        const rect = this.canvas.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        
        const dx = x - this.centerX;
        const dy = y - this.centerY;
        const dist = Math.sqrt(dx*dx + dy*dy);
        
        if (dist > this.radius) {
          this.handleX = this.centerX + (dx / dist) * this.radius;
          this.handleY = this.centerY + (dy / dist) * this.radius;
        } else {
          this.handleX = x;
          this.handleY = y;
        }
        
        const x_norm = (this.handleX - this.centerX) / this.radius * 100;
        const y_norm = -(this.handleY - this.centerY) / this.radius * 100;
        
        this.onMove(x_norm, y_norm);
        this.draw();
      }
      
      handleEnd() {
        this.touch = false;
        this.handleX = this.centerX;
        this.handleY = this.centerY;
        this.onMove(0, 0);
        this.draw();
      }
      
      draw() {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        
        // Background circle
        this.ctx.fillStyle = '#f0f0f0';
        this.ctx.beginPath();
        this.ctx.arc(this.centerX, this.centerY, this.radius, 0, Math.PI*2);
        this.ctx.fill();
        
        // Center dot
        this.ctx.fillStyle = '#ddd';
        this.ctx.beginPath();
        this.ctx.arc(this.centerX, this.centerY, 5, 0, Math.PI*2);
        this.ctx.fill();
        
        // Handle
        this.ctx.fillStyle = '#667eea';
        this.ctx.beginPath();
        this.ctx.arc(this.handleX, this.handleY, this.handleRadius, 0, Math.PI*2);
        this.ctx.fill();
        
        // Border
        this.ctx.strokeStyle = '#667eea';
        this.ctx.lineWidth = 3;
        this.ctx.beginPath();
        this.ctx.arc(this.centerX, this.centerY, this.radius, 0, Math.PI*2);
        this.ctx.stroke();
      }
    }
    
    // Initialize joysticks
    new Joystick('leftJoystick', (x, y) => {
      leftJoyX = x;
      leftJoyY = y;
      sendControl();
    });
    
    new Joystick('rightJoystick', (x, y) => {
      rightJoyX = x;
      rightJoyY = y;
      sendControl();
    });
    
    function updateThrottle(val) {
      document.getElementById('throttleValue').innerText = val;
      sendControl();
    }
    
    function sendControl() {
      const throttle = parseFloat(document.getElementById('throttleSlider').value);
      const pitch = leftJoyY;
      const yaw = leftJoyX;
      const roll = rightJoyX;
      
      fetch('/control', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: `roll=${roll}&pitch=${pitch}&yaw=${yaw}&throttle=${throttle}`
      });
    }
    
    function armDrone() {
      armed = true;
      fetch('/arm?armed=true', { method: 'POST' });
      updateStatus();
    }
    
    function disarmDrone() {
      armed = false;
      document.getElementById('throttleSlider').value = 0;
      document.getElementById('throttleValue').innerText = '0';
      fetch('/arm?armed=false', { method: 'POST' });
      updateStatus();
    }
    
    function updateStatus() {
      const status = document.getElementById('status');
      if (armed) {
        status.className = 'status armed';
        status.innerText = '🟢 ISHGA TUSHMASHDI';
      } else {
        status.className = 'status disarmed';
        status.innerText = '⚫ ISHGA TUSHMAGAN';
      }
    }
    
    function updatePID() {
      const data = {
        kp_roll: document.getElementById('kp_roll').value,
        ki_roll: document.getElementById('ki_roll').value,
        kd_roll: document.getElementById('kd_roll').value,
        kp_pitch: document.getElementById('kp_pitch').value,
        ki_pitch: document.getElementById('ki_pitch').value,
        kd_pitch: document.getElementById('kd_pitch').value,
        kp_yaw: document.getElementById('kp_yaw').value,
        ki_yaw: document.getElementById('ki_yaw').value,
        kd_yaw: document.getElementById('kd_yaw').value
      };
      
      const params = new URLSearchParams(data);
      fetch('/pid', {
        method: 'POST',
        body: params
      });
    }
    
    function updateTelemetry() {
      fetch('/telemetry')
        .then(r => r.json())
        .then(data => {
          document.getElementById('telemetryRoll').innerText = data.roll.toFixed(1) + '°';
          document.getElementById('telemetryPitch').innerText = data.pitch.toFixed(1) + '°';
          document.getElementById('telemetryYaw').innerText = data.yaw.toFixed(1) + '°';
          document.getElementById('telemetryStatus').innerText = data.armed ? '🟢' : '⚫';
          
          for (let i = 0; i < 4; i++) {
            const pct = (data.motors[i] / 100) * 100;
            document.getElementById('motor' + i).style.height = pct + '%';
            document.getElementById('motorValue' + i).innerText = data.motors[i];
          }
        });
    }
    
    setInterval(updateTelemetry, 100);
  </script>
</body>
</html>
)";
}
