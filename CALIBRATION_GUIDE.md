# MPU6050 Kalibratsiya Va Sosttuvchi Qo'llanma

## 1. Gyroscope (Gyro) Kalibratsiyasi

MPU6050 qo'lda gyroscope zero offset'ni sozlash kerak bo'ladi:

### Kod:
```cpp
// Sketch'ning setup()iga qo'shing:
mpu.setXGyroOffset(50);      // Qo'sh-solib aniqlang (X o'qi)
mpu.setYGyroOffset(20);      // Y o'qi
mpu.setZGyroOffset(-10);     // Z o'qi

// Accelerometer offsets
mpu.setXAccelOffset(0);
mpu.setYAccelOffset(0);
mpu.setZAccelOffset(0);
```

### Kalibratsiya Jarayoni:
1. ESP32 ni flat, level surface'da qo'ying
2. Hiç harakat qilmang 10 sekund
3. Serial Monitor'dan offset qiymatlarini o'qing
4. Qo'sh-solib `setXGyroOffset()` va boshqalarga qo'shing

## 2. Accelerometer Kalibratsiyasi

### Kod:
```cpp
void calibrateAccel() {
  Serial.println("Accelerometer kalibratsiyasi...");
  int16_t ax, ay, az;
  
  float sumX = 0, sumY = 0, sumZ = 0;
  
  for (int i = 0; i < 100; i++) {
    mpu.getAcceleration(&ax, &ay, &az);
    sumX += ax;
    sumY += ay;
    sumZ += az;
    delay(10);
  }
  
  float offsetX = sumX / 100.0;
  float offsetY = sumY / 100.0;
  float offsetZ = (sumZ / 100.0) - 16384; // 16384 = 1g for ±2g range
  
  Serial.printf("Offsets: X=%.0f Y=%.0f Z=%.0f\n", offsetX, offsetY, offsetZ);
  
  mpu.setXAccelOffset((int16_t)-offsetX);
  mpu.setYAccelOffset((int16_t)-offsetY);
  mpu.setZAccelOffset((int16_t)-offsetZ);
}
```

## 3. 45° Rotation Correctionsi

Dron X-shape bo'lib, MPU **45°** qilib o'rnatilgan:

```
Haqiqiy Dron:        MPU Orientation:
      +                    /
     /|\                  /|
    / | \                / |
   M1 | M2              M1 | M2 (45° rotated)
   |  |  |              |  |  |
---+--+--+---       ---+--+--+---
   |  |  |              |  |  |
   M4 | M3              M4 | M3
    \ | /                \ |
     \|/                  \|
      -                    \
```

### Rotation Matritsa:
```cpp
// setup() da qo'shing:
float roll_raw, pitch_raw, yaw_raw;

// loop() da IMU o'qishdan keyin:
float angle_45 = 45.0 * M_PI / 180.0; // 45 degrees to radians

float roll_corrected = roll_raw * cos(angle_45) - pitch_raw * sin(angle_45);
float pitch_corrected = roll_raw * sin(angle_45) + pitch_raw * cos(angle_45);

roll = roll_corrected;
pitch = pitch_corrected;
```

## 4. MPU6050 Boshqa Kalibratsiya Usuli (Advance)

### Firmware Calibration:
```cpp
void completeMPUCalibration() {
  Serial.println("To'liq kalibratsiya boshlanmoqda...");
  Serial.println("MPU ni 6 ta turli position'da 2 soniya turib qoying:");
  
  int16_t offsets[6] = {0, 0, 0, 0, 0, 0};
  String positions[] = {"X+", "X-", "Y+", "Y-", "Z+", "Z-"};
  
  for (int p = 0; p < 6; p++) {
    Serial.printf("Position %d/%d: %s\n", p+1, 6, positions[p].c_str());
    delay(2000);
    
    int16_t ax, ay, az;
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    
    for (int i = 0; i < 50; i++) {
      mpu.getAcceleration(&ax, &ay, &az);
      sumX += ax;
      sumY += ay;
      sumZ += az;
      delay(10);
    }
    
    Serial.printf("Average: X=%d Y=%d Z=%d\n", sumX/50, sumY/50, sumZ/50);
  }
  
  Serial.println("Kalibratsiya tugadi!");
}
```

## 5. Temperature Compensation

MPU6050 harorat o'zgarganda drift qiladi:

```cpp
// Harorat o'qish:
int16_t temp;
mpu.getTemperature(&temp);
float temperature = temp / 340.0 + 36.53; // Celsius

// Agar 5°C dan ko'p o'zgarsa:
if (abs(temperature - lastTemperature) > 5.0) {
  Serial.println("Harorat o'zgargan, kalibratsiya qayta tekshirilsin!");
  lastTemperature = temperature;
}
```

## 6. Senorni Tekshirish (Debug)

```cpp
void testMPU6050() {
  int16_t ax, ay, az, gx, gy, gz;
  int16_t temp;
  
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  mpu.getTemperature(&temp);
  
  Serial.printf("Accel: X=%d Y=%d Z=%d\n", ax, ay, az);
  Serial.printf("Gyro: X=%d Y=%d Z=%d\n", gx, gy, gz);
  Serial.printf("Temp: %.2f°C\n", temp / 340.0 + 36.53);
}
```

## 7. I2C Debug

Agar MPU6050 aloqa qilmasa:

```cpp
void scanI2C() {
  Serial.println("I2C scanning...");
  
  for (int addr = 0; addr < 128; addr++) {
    Wire.beginTransmission(addr);
    int error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.printf("Device found at address: 0x%02X\n", addr);
    }
  }
}
```

## 8. Real-time Monitoring

Serial Plotter'da ko'rish:

```cpp
void printTelemetry() {
  // Serial Plotter formats
  Serial.printf("Roll:%.2f,Pitch:%.2f,Yaw:%.2f\n", roll, pitch, yaw);
  
  // Yoki JSON format (Python script bilan)
  Serial.printf("{\"r\":%.2f,\"p\":%.2f,\"y\":%.2f}\n", roll, pitch, yaw);
}
```

## PID Test Scenarios

### Test 1: Roll Stabilization
- Dronni sag tomonga 45° eğ (tilt)
- Qo'l bilan qo'yib turdim
- Roll value qaytmalimi?
- Agar oscillation bo'lsa: KP kamaytiring
- Agar sekin bo'lsa: KP orttirig, KD qo'shing

### Test 2: Static Hold
- Throttle 30%
- Joystik centered (0,0,0,0)
- Dron o'z level'ini saqlamalimi?
- Agar drift bo'lsa: KI orttirig

### Test 3: Response Speed
- Joystik 1 degree inject qiling
- Dron 100-200ms da qaytmalimi?
- Agar sekin: KP orttirig
- Agar juda tez/oscillate: KD orttirig

## Final Checklist

- [ ] I2C ulanish tekshirildi (0x69 topildi)
- [ ] Gyroscope kalibratsiyasi
- [ ] Accelerometer zero offset noto'g'ri emas
- [ ] 45° rotation qayta ko'ntrolni o'tdi
- [ ] Serial output smooth va stable
- [ ] PID tahmini boshlang: KP=1.0, KI=0, KD=0.5
- [ ] Dron statik position saqlamasligi tekshirildi

---

**Eslatma:** Har bir test'dan oldin kalibratsiyani qayta o'tkazing, agar harorat yoki joyni o'zgartirsangiz!
