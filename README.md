# ESP32-S3 Dron PID Testing System 🚁

Brushless motorli (A2212/13T) dron PID tuning uchun to'liq sistem - WiFi joystik boshqaruvi bilan.

## Hardware Setup

### Motor Pin Configuration
```
ESP32-S3 Pin  |  Motor
   4          |  Front-Right (Motor 1)
   5          |  Front-Left  (Motor 2)
   6          |  Back-Left   (Motor 3)
   7          |  Back-Right  (Motor 4)
```

### MPU6050 Pin Configuration
```
ESP32-S3 Pin  |  MPU6050
   1 (SDA)    |  SDA
   2 (SCL)    |  SCL
   GND        |  GND
   3.3V       |  VCC
```
- I2C Address: **0x69**
- MPU6050 **45° qilib o'rnatilgan** (X-shape dron uchun)

### ESC Configuration
```
40A ESC Settings:
- Frequency: 50Hz
- Pulse: 1000-2000 µs (1ms = arm, 1.5ms = 50%, 2ms = 100%)
- Min Throttle: 1000 µs
- Max Throttle: 2000 µs
```

## Installation

### 1. Arduino IDE Setup
1. **Board qo'shing:**
   - File → Preferences
   - Additional Boards Manager URLs:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
   - Tools → Board Manager → "ESP32" qidiring va o'rnatang

2. **Board Tanlang:**
   - Tools → Board → ESP32 → ESP32-S3 Dev Module

3. **Port Tanlang:**
   - Tools → Port → /dev/ttyUSB0 (yoki COM3, COM4...)

4. **Upload Speed:**
   - Tools → Upload Speed → 921600

### 2. Library O'rnatish
Sketch'da quyidagi library'larni o'rnatish kerak:
- **MPU6050** (I2CdevLib versions)
- WiFi (built-in)
- WebServer (built-in)

**Qo'lda o'rnatish:**
```
Sketch → Include Library → Add .ZIP Library
```

## Joystik Boshqaruvi

### WiFi Ulanish
1. Telefon/Kompyuterda WiFi qidiring
2. SSID: `ESP32_DRONE`
3. Password: `drone12345`
4. Browser'da kiriting: `192.168.4.1`

### Joystik Kontrolleri
```
LEFT JOYSTICK:
- Vertical (Up/Down)  → Throttle & Pitch
- Horizontal (L/R)    → Yaw

RIGHT JOYSTICK:
- Vertical (Up/Down)  → Pitch
- Horizontal (L/R)    → Roll
```

## Motor Mixing (+Shape)

```
        Front
    M1  (4)  M2
     \ (5) /
      \ | /
   M4--[+]--M2
      / | \
     / (6) \
    M4  (7)  M3
       Back
```

**PID Output → Motor Mixing:**
```cpp
M1 = Throttle + Roll - Pitch + Yaw
M2 = Throttle - Roll - Pitch - Yaw
M3 = Throttle - Roll + Pitch + Yaw
M4 = Throttle + Roll + Pitch - Yaw
```

## PID Tuning Guide

### Dron to'qish usuli:
1. Dronni **X-frame** qamchi yoki mahkam qo'l bilan ushlab turish
2. Throttle minimal (20-30%)
3. Qo'l bilan dronni bir tarafga nayish berish

### PID Parameters

#### Roll & Pitch (Stabilizatsiya)
- **KP** (Proportional): 1.0 - 2.5
  - Kam → sekin reaktsiya
  - Ko'p → oscillation (tebranish)
- **KI** (Integral): 0.01 - 0.1
  - Xatoning to'plami
- **KD** (Derivative): 0.5 - 1.5
  - Damping (salbash)

#### Yaw (Aylanish)
- **KP**: 1.5 - 3.0
- **KI**: 0.05 - 0.2
- **KD**: 0.2 - 0.6

### Tuning Ketma-ketligi
1. **KP bilan boshlang** (KI=0, KD=0)
   - Dronni eğ (tilt) berish → tez qaytishi kerak
   - Oscillation bo'lsa → KP ni kamaytiring

2. **KD qo'shing** (Damping)
   - Oscillation'ni kamaytiradiy
   - Juda ko'p → sekin va "mushy" feel

3. **KI qo'shing** (Steady-state error)
   - Dronni tilting qilib turdim → qaytmasligi uchun
   - Kam qiymatlarga boshlang (0.01-0.05)

## Test Jarayoni

### 1. Roll Tuning (Yon egilish)
```
- "ARM" tugmasini bosing
- Throttle 20-30%
- Right joystick: X axis → Roll qiymatiga karing
- Dron qanday reaktsiya bersa, KP/KD o'zgartiring
```

### 2. Pitch Tuning (Oldinga-orqaga egilish)
```
- Throttle 20-30%
- Right joystick: Y axis → Pitch qiymatiga karing
- Roll kabi sozlang
```

### 3. Yaw Tuning (Aylanish)
```
- Throttle 20-30%
- Left joystick: X axis → Yaw boshqarish
- Yaw rate'ini monitor qiling
```

## Troubleshooting

### Dron bitta tomonga qaraydi
- Gyroscope calibration xatasi
- Dronni flat surface'da 10 soniya turib qoying

### Motor irregular
- ESC calibration qilish kerak
- Motor pins tekshiring

### WiFi ulanmaydi
- ESP32 ni reset qiling (EN tugmasini bosing)
- Password noto'g'ri emas yoki SSID o'zgargan

### Sensor ma'lumoti xira
- I2C wiring tekshiring
- Pull-up resistor qo'shing (4.7kΩ SDA va SCL'ga)

## File Tuzilishi

```
esp32-drone-pid/
├── esp32-drone-pid.ino       # Main sketch
├── README.md                  # Bu fayl
└── calibration_guide.md      # Kalibratsiya qo'llanmasi
```

## Kerakli Kutubxonalar

```
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps612.h>
```

## License

MIT

## Author

Dron PID Test System v1.0

---

**Xavfsizlik Eslatmasi:** 
- Dronni har doim mahkam qo'l bilan ushlab turib test qiling
- Motor propeller'larini olib tashlang testdan oldin
- Haqiqiy uchurishdan avval dastlab stend'da test qiling
