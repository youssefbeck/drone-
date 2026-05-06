# 🚁 Quick Start Guide (Tezkor Boshlanish)

## 1-DAQIQA SETUP

### Step 1: Library'larni o'rnatish (Arduino IDE'da)
```
Sketch → Include Library → Manage Libraries

Qidiring va o'rnatang:
1. "MPU6050" → Installed by Jeff Rowberg
2. ESP32 board qo'shing (Boards Manager'dan)
```

### Step 2: Board va Port
```
Tools → Board → ESP32 → ESP32-S3 Dev Module
Tools → Port → /dev/ttyUSB0 (yoki COM3, COM4...)
Tools → Upload Speed → 921600
```

### Step 3: Kod Upload
```
Sketch → Upload
(yoki Ctrl+U)
```

### Step 4: WiFi Ulanish
```
Telefon WiFi'sini aching:
- SSID: ESP32_DRONE
- Password: drone12345
- Browser: 192.168.4.1
```

## Birinchi Test

### Dronni Sozlash
1. Dronni **xavfsiz qamchi** bilan mahkam qo'ling
2. **Propeller'larni olib tashlang** (XAVFSIZLIK!)
3. Throttle slider 0'da turib qoyin

### Web Interfeysi
```
1. "🟢 ISHGA TUSH" tugmasini bosing
2. Throttle slider'ni 20-30% ga o'tkazing
3. Right joystick'ni chala chapdega eğ
4. Dron shakllashmalimi? ✓

Bu test uchun shuncha!
```

## Sensor Tekshirish

### Serial Monitor'da Ma'lumotlarni Ko'ring
```
Arduino IDE: Tools → Serial Monitor
Baud: 115200

Chiqish:
Roll: 0.5° | Pitch: -1.2° | Yaw: 0.0° | Armed: NO
Motors: [1000, 1000, 1000, 1000]
```

### Agar MPU6050 topilmasa ❌
```
ERROR: MPU6050 topilmadi!

Tekshirish:
1. I2C wiring (pins 1, 2)
2. Power va GND
3. Pull-up resistors (4.7kΩ)
4. I2C address: 0x69 (AD0 pin HIGH)
```

## PID Tuning Jarayoni

### Boshlang'ich Qiymatlar
```
Roll: KP=1.0, KI=0.0, KD=0.5
Pitch: KP=1.0, KI=0.0, KD=0.5
Yaw: KP=2.0, KI=0.0, KD=0.3
```

### Test 1: Roll Tuning
```
1. ISHGA TUSH
2. Throttle 25%
3. Right Joystick: FULL LEFT (Roll = -30°)
4. Ko'ring: Dron qaytmalimi?

Agar sekin → KP +0.5
Agar oscillate → KP -0.3, KD +0.2
```

### Test 2: PID Optimization
```
Web interfeys'da:
- KP/KI/KD input'larni o'zgartirez
- 2 soniya kutib qoling
- Effektni yo'qlaing
- Talab bo'yicha o'zgartiring

Qayta-qayta, dronni "hoza" qilguncha!
```

## Motor Tekshirish

### Agar Motorlar Rotatsiya Qilmasa
```
1. ESC'ni arm qilish (1000µs signal kerak)
2. PWM wiring'ni tekshiring (pins 4,5,6,7)
3. ESC'ni reset qiling (ON/OFF)
4. Throttle 0 → 100 → 0
```

### Motor Ism'aritmasi
```
Web dashboard'da Motor bars:
[Motor 1] [Motor 2] [Motor 3] [Motor 4]
  |         |         |         |
 Pin 4     Pin 5     Pin 6     Pin 7

Barcha 0'da bo'lmaliki ishga tushganda
Throttle > 20% → motors ishtiya boshlaydi
```

## Xavfsizlik Cheklisti ✓

Haqiqiy uchen avval:
- [ ] Dronni qamchi bilan tusdim
- [ ] Propeller'lar olib ketilgan
- [ ] WiFi ulanishi tekshirilgan
- [ ] Sensor ma'lumoti stable
- [ ] PID'lar optimal
- [ ] Motor aralashmasida xato yo'q
- [ ] Batareyaning voltaji tekshirilgan

## Tez Muammo hal Qilish

| Muammo | Sababi | Hal |
|--------|--------|-----|
| WiFi topilmaydi | Reset kerak | EN tugmasini bosing |
| Sensor error | I2C xatosi | Wiring tekshiring |
| Motor don'tishi | ESC arm xatosi | Throttle 0→100 test |
| Oscillation | KP juda yuqori | KP ni 20% kamaytiring |
| Sekin reaktsiya | KP juda past | KP ni 20% orttirig |

## Dokumentatsiyalar

Ushbu papka'da:
- `README.md` - Umumiy qo'llanma
- `WIRING_GUIDE.md` - Pinning diagramma
- `CALIBRATION_GUIDE.md` - Sensor kalibratsiyasi
- `PID_TUNING_GUIDE.md` - PID reference

## Arduino IDE vs Web Interface

### Arduino IDE (Upload/Debug)
```
- Kod o'zgartirishlar
- Library management
- Serial debugging
- Compilation
```

### Web Interface (192.168.4.1)
```
- Real-time joystick control
- Live telemetry
- PID tuning
- Motor visualization
- ARM/DISARM
```

## Kerakli Commands

### Arduino IDE Terminal'da (Linux)
```bash
# ESP32 boards o'rnatish
# File → Preferences → 
# Additional Boards Manager URLs dan qo'shing

# Sekuncia:
# Tools → Board Manager → "esp32" → Install

# Library:
# Sketch → Include Library → Manage Libraries
# "MPU6050" qidiring → Installed by Jeff Rowberg → Install
```

## Tips & Tricks

### 💡 WiFi Password o'zgartirib qo'yish
```cpp
// esp32-drone-pid.ino line ~25:
const char* password = "drone12345";
// O'zgartirib qo'ying keraksa
```

### 💡 Motor pins o'zgartirib qo'yish
```cpp
// esp32-drone-pid.ino line ~22-25:
#define MOTOR_FRONT_RIGHT 4
#define MOTOR_FRONT_LEFT  5
#define MOTOR_BACK_LEFT   6
#define MOTOR_BACK_RIGHT  7
```

### 💡 Serial debugging
```cpp
// Loop'da qo'shing:
Serial.println("Debug message");

// Ko'rish uchun: Tools → Serial Monitor
```

## Next Steps

1. ✓ Setup va upload
2. ✓ WiFi connection
3. ✓ Sensor tekshirish
4. ✓ Motor test
5. → **PID tuning** (3-5 soat)
6. → Flight code'ga transfer
7. → First flight!

---

**Murakkab muammolar uchun:**
- CALIBRATION_GUIDE.md → Sensor tekshirish
- PID_TUNING_GUIDE.md → PID reference
- WIRING_GUIDE.md → Hardware debugging

**Omad!** 🚀

Savollaringiz bo'lsa, README.md dagi kontaktga yozing.
