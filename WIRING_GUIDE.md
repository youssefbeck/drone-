# ESP32-S3 Drone Wiring Guide

## Wiring Diagram (Text Format)

```
┌─────────────────────────────────────────────────────────┐
│                      ESP32-S3                           │
│                                                         │
│    VCC   GND   SDA(1)  SCL(2)  GPIO4  GPIO5  GPIO6  GPIO7│
│     │     │      │       │       │      │      │      │   │
│     │     │      │       │       │      │      │      │   │
│     ├─────┼──────┼───────┼───────┼──────┼──────┼──────┤   │
│     │     │      │       │       │      │      │      │   │
└─────┼─────┼──────┼───────┼───────┼──────┼──────┼──────┘   │
      │     │      │       │       │      │      │      │
      │     │      │       │       │      │      │      │
      │   [GND]    │       │       │      │      │      │
      │     │      │       │       │      │      │      │
      │     │      │       │       │      │      │      │
    [3.3V] │      │       │       │      │      │      │
      │     │      │       │       │      │      │      │
      │    GND    SDA     SCL     │      │      │      │
      │     │      │       │       │      │      │      │
      │     │    ┌─┴──────┬┘       │      │      │      │
      │     │    │        │       │      │      │      │
      │     │   [MPU6050]  │       │      │      │      │
      │     │    │        │       │      │      │      │
      └─────┴────┼────────┘       │      │      │      │
              (ad)                │      │      │      │
                          
         Pin 4              Pin 5              Pin 6              Pin 7
         │                  │                  │                  │
      [PWM]              [PWM]              [PWM]              [PWM]
         │                  │                  │                  │
    ┌────┴───┐         ┌────┴───┐        ┌────┴───┐         ┌────┴───┐
    │  ESC   │         │  ESC   │        │  ESC   │         │  ESC   │
    │   40A  │         │   40A  │        │   40A  │         │   40A  │
    └────┬───┘         └────┬───┘        └────┬───┘         └────┬───┘
         │                  │                  │                  │
    ┌────┴──────┐      ┌────┴──────┐     ┌────┴──────┐      ┌────┴──────┐
    │  Brushless │      │ Brushless  │    │ Brushless  │     │ Brushless  │
    │  Motor 1   │      │  Motor 2   │    │  Motor 3   │     │  Motor 4   │
    │ (Pin4)     │      │ (Pin5)     │    │ (Pin6)     │     │ (Pin7)     │
    │Front-Right │      │ Front-Left │    │ Back-Left  │     │ Back-Right │
    └────────────┘      └────────────┘    └────────────┘     └────────────┘
```

## Pin Connection Table

| ESP32 Pin | Component | Function | Notes |
|-----------|-----------|----------|-------|
| 1 (SDA) | MPU6050 | I2C Data | 4.7kΩ pull-up to 3.3V |
| 2 (SCL) | MPU6050 | I2C Clock | 4.7kΩ pull-up to 3.3V |
| 3.3V | MPU6050 | Power | 3.3V |
| GND | MPU6050 | Ground | Common GND |
| 4 (GPIO4) | ESC 1 | PWM Signal | 1000-2000 µs |
| 5 (GPIO5) | ESC 2 | PWM Signal | 1000-2000 µs |
| 6 (GPIO6) | ESC 3 | PWM Signal | 1000-2000 µs |
| 7 (GPIO7) | ESC 4 | PWM Signal | 1000-2000 µs |
| GND | ESC 1-4 | Ground | Common GND |

## Motor Configuration

### Motor Positions (+ Shape)

```
        M1 (Front-Right)
         |
         |
M4 ------+------ M2
(Back-   |   (Front-
 Right)  |    Left)
         |
         |
        M3 (Back-Left)
```

### ESC Signal Interpretation
- **1000 µs** = Motor OFF (ESC arm state)
- **1500 µs** = 50% throttle
- **2000 µs** = 100% throttle

### PWM Frequency: 50Hz (20ms period)

```
Timeline (20ms):
┌────────────────────────────────────────┐
│ High ┐                                  │
│      │ Pulse Width 1-2ms                │
│ Low  └──────────────────────────────────┴─
└────────────────────────────────────────┘
  0ms          5-15ms                20ms
```

## I2C Pull-up Resistors

MPU6050 requires I2C pull-up resistors:

```
         3.3V
         ┌───┐
         │ R │ 4.7kΩ
         │   │
    ┌────┴───┴───┐
    │            │
   SDA          SCL
    │            │
   [MPU6050]    [MPU6050]
```

### Installation:
```
Option 1: ESP32 Pin → 4.7kΩ Resistor → 3.3V
Option 2: Motherboard'da built-in bo'lishi mumkin
Option 3: MPU6050 breakout board'da built-in bo'lishi mumkin
```

## Power Requirements

### ESP32-S3
- **Operating Voltage:** 3.0 - 3.6V (typically 3.3V)
- **Current Draw:** ~100-200 mA

### MPU6050
- **Operating Voltage:** 3.0 - 3.4V
- **Current Draw:** ~3-4 mA

### ESC 40A + Motors
- **Operating Voltage:** 7.4V - 14.8V (2-4S LiPo)
- **Current Draw:** Peak 40A per motor
- **Note:** ESP32 va MPU6050 boshqa power supply'dan quvvat olishi kerak!

### Total System Power Budget:
```
┌──────────────────────────────────┐
│   Main LiPo Battery (2-4S)       │
│   (7.4V - 14.8V, 40A-50A)        │
└──────┬───────────────────────────┘
       │
       ├─────→ ESC 1 (M1)
       ├─────→ ESC 2 (M2)
       ├─────→ ESC 3 (M3)
       ├─────→ ESC 4 (M4)
       │
       └─────→ BEC/UBEC → 5V
               │
               ├─→ ESP32 5V pin
               └─→ 3.3V regulator (or built-in)
                  │
                  ├─→ ESP32 3.3V pin
                  └─→ MPU6050 VCC
```

## Alternative Configurations

### Option 1: Integrated ESC
Agar 4x ESC o'rniga 1x integrated 4-in-1 ESC bo'lsa:
```
Pin 4,5,6,7 → 4-in-1 ESC PWM inputs (1-4)
              ↓
              Motors 1-4
```

### Option 2: Separate Power Modules
```
Main Batt → Power Distribution Board
           ├→ ESC 1,2,3,4
           └→ UBEC (5V output) → ESP32 5V pin
```

## Voltage Divider (Optional for Battery Monitoring)

Agar batareyani voltaj ko'rish kerak bo'lsa:

```
LiPo Battery (3S = 11.1V max)
     │
     ├─→ R1 (10kΩ)
     │
     ├─→ R2 (2.2kΩ) → ESP32 ADC Pin
     │
    GND
```

Voltage at ADC = Battery_Voltage × R2 / (R1 + R2)
= 11.1V × 2.2 / (10 + 2.2) = ~2.0V (safe for ESP32 ADC)

## Troubleshooting Connection Issues

### Motors not responding
1. Check ESC signal wiring (GPIO 4,5,6,7)
2. Verify ESC is armed (1000 µs signal sent)
3. Check power to ESC

### MPU6050 not detected
1. Check I2C wiring (pins 1, 2)
2. Verify address: 0x69 (AD0 pulled high)
3. Check 3.3V supply
4. Add pull-up resistors if not present

### Intermittent connection
1. Check wire quality
2. Add ferrite beads on I2C lines
3. Separate power and signal grounds

## Material List

```
Electronics:
- ESP32-S3 DevKit (1x)
- MPU6050 Breakout (1x)
- ESC 40A (4x)
- Brushless Motor A2212/13T (4x)
- Propellers (8x, 2x backup)

Wiring:
- 22 AWG Servo Wire (1m) - Motor control
- 18 AWG Power Wire (2m) - ESC power
- I2C Wire (0.5m) - Pull-ups available

Resistors:
- 4.7kΩ (2x) - I2C pull-ups
- 10kΩ (1x) - Battery divider (optional)
- 2.2kΩ (1x) - Battery divider (optional)

Connectors:
- XT60 (2x) - Battery connector
- JST-XH (8x) - Motor connectors
```

---

**Wiring Checklist:**
- [ ] ESP32 ← → MPU6050 I2C
- [ ] All motors connected to ESC 1-4
- [ ] ESC signal wires to GPIO 4,5,6,7
- [ ] Common GND between all components
- [ ] 3.3V power stable
- [ ] Pull-up resistors installed
- [ ] No loose connections
