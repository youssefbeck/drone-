# PID Tuning Quick Reference

## What's PID?

```
Error = Target - Current
        ↓
    ┌───┴────┬────────┬────────┐
    ↓        ↓        ↓        ↓
 Input → Proportional Integral Derivative → Output
          (P)        (I)        (D)
          ↓          ↓          ↓
        Kp*e      Ki*∫e      Kd*de/dt
            ↓      ↓      ↓
            └──────┴──────┘
               ↓
           Motor Output
```

### Components:
- **P (Proportional):** How fast to react
- **I (Integral):** How to eliminate steady error
- **D (Derivative):** How to reduce overshoot

## Starting Values

### For Roll & Pitch (Angle Stabilization)
```
First Try:
KP = 1.0
KI = 0.0
KD = 0.5
```

### For Yaw (Rate Control)
```
First Try:
KP = 2.0
KI = 0.0
KD = 0.3
```

## Tuning Flow Chart

```
START
  ↓
Set KP only (KI=0, KD=0)
  ↓
Tilt drone 45° and release
  ↓
┌─────────────────────────────────┐
│ Does it oscillate?              │
├──────────────────┬──────────────┤
│ YES              │ NO           │
├──────────────────┼──────────────┤
│ Decrease KP      │ Increase KP  │
│ Try -20%         │ Try +20%     │
└──────────────────┴──────────────┘
  ↓
Good response? → NO → loop up
  ↓ YES
Add KD (Damping)
  ↓
Set KD to smooth oscillation
  ↓
Add KI for steady-state
  ↓
Test static hold
  ↓
DONE!
```

## Tuning Parameters

### KP (Proportional Gain)

| Value | Effect | Problem |
|-------|--------|---------|
| Too Low | Slow response, drone lags | Won't correct tilt |
| Just Right | Quick & stable correction | None |
| Too High | Oscillates (wobbles) | Jitters, hard to control |

**How to find optimal KP:**
1. Set KI=0, KD=0
2. Tilt drone 45°
3. Watch how fast it returns
4. Should return in 0.5-1 second smoothly
5. If oscillates: decrease by 20%
6. If slow: increase by 20%

### KD (Derivative Gain)

| Value | Effect | Problem |
|-------|--------|---------|
| Too Low | Oscillates after settling | Bouncy feel |
| Just Right | Smooth damping | None |
| Too High | Slow, "mushy" response | Sluggish control |

**How to find optimal KD:**
1. After tuning KP
2. Set KD = KP * 0.3 to 0.8
3. Tilt drone and release
4. Should smoothly return without bounce
5. Increase KD if oscillating
6. Decrease if feeling sluggish

### KI (Integral Gain)

| Value | Effect | Problem |
|-------|--------|---------|
| Too Low | Steady error remains | Dron leans slightly |
| Just Right | Eliminates drift | None |
| Too High | Slow integration windup | Feels unresponsive |

**How to find optimal KI:**
1. With good KP and KD
2. Hold drone tilted 20°
3. Let go slowly
4. Should return to level automatically
5. Start with KI = KP * 0.05
6. Increase if drifts, decrease if slow

## Test Procedures

### Test 1: Step Response
```
Command: Tilt 45° manually
Expected: Return to level in 0.5-1s smoothly
Problem detection:
  - Oscillates → Decrease KD or KP
  - Too slow → Increase KP
  - Doesn't stabilize → Increase KI
```

### Test 2: Steady State Hold
```
Command: Hold at 20° for 5 seconds
Expected: Angle stays ~20°
Problem detection:
  - Creeps down → Increase KI
  - Bounces → Increase KD
  - Goes wild → Decrease KP
```

### Test 3: Ramp Input
```
Command: Slowly move joystick left (15°/sec)
Expected: Drone follows smoothly
Problem detection:
  - Lags → Increase KP
  - Overshoots → Increase KD
  - Hunting → Decrease KI
```

## Common Problems & Solutions

### Problem: Oscillation (Wobbles)
```
Symptom: Drone shakes side-to-side
Causes: KP too high, KD too low
Fix: 
  - Decrease KP by 10-20%
  - Increase KD by 20-30%
```

### Problem: Slow Response
```
Symptom: Drone takes 2+ seconds to stabilize
Causes: KP too low, KD too high
Fix:
  - Increase KP by 20-30%
  - Decrease KD by 10-20%
```

### Problem: Constant Drift
```
Symptom: Drone tilts slowly to one side
Causes: Sensor offset, KI too low
Fix:
  - Re-calibrate MPU6050
  - Increase KI by 20-50%
  - Check for uneven motor thrust
```

### Problem: Jittering
```
Symptom: Motor values jump rapidly
Causes: KP too high, sensor noise
Fix:
  - Decrease KP
  - Add sensor smoothing filter
  - Check I2C wiring
```

### Problem: Unresponsive
```
Symptom: Joystick input has no effect
Causes: Loop rate too low, gains wrong
Fix:
  - Check loop timing (should be ~100Hz)
  - Increase all gains slightly
  - Verify joystick signal is received
```

## PID Gain Scaling

### For Different Loop Frequencies:

```cpp
// If loop is 100 Hz (10ms): use base values
// If loop is 50 Hz (20ms): multiply gains by 0.5
// If loop is 200 Hz (5ms): multiply gains by 2.0

float loop_factor = 0.01 / dt;  // dt in seconds
kp_final = kp_base * loop_factor;
```

## Fine-Tuning Checklist

- [ ] Loop running at consistent 100Hz?
- [ ] All gains positive numbers?
- [ ] Sensor data smooth (not noisy)?
- [ ] Motor mixing correct?
- [ ] Throttle not saturating motors?
- [ ] Roll & Pitch roughly similar?
- [ ] Yaw separate tuning?
- [ ] No integral windup (limits set)?
- [ ] Joystick calibrated?

## Advanced Tuning

### Ziegler-Nichols Method (Professional)

1. Set KI=0, KD=0
2. Slowly increase KP until drone oscillates consistently
3. Note this **Ku** (critical gain) and **Tu** (period)
4. Calculate:
   - KP = 0.6 × Ku
   - KI = 1.2 × Ku / Tu
   - KD = 0.075 × Ku × Tu

### Frequency Response Method

Monitor oscillation frequency and adjust:
- **Too low freq (< 1 Hz):** Increase KP
- **Too high freq (> 5 Hz):** Increase KD or decrease KP

## Logging & Analysis

### Serial Output Format:
```cpp
Serial.printf("%.2f,%.2f,%.2f,%d,%d,%d,%d\n",
  roll, pitch, yaw,
  motor[0], motor[1], motor[2], motor[3]
);
```

### In Python:
```python
import matplotlib.pyplot as plt

# Parse CSV from serial
data = np.loadtxt('drone_log.csv', delimiter=',')

plt.plot(data[:, 0], label='Roll')
plt.plot(data[:, 1], label='Pitch')
plt.plot(data[:, 2], label='Yaw')
plt.legend()
plt.show()
```

## Default Tuning for Different Builds

### Lightweight Drone (< 500g)
```
KP_ROLL = 2.0    KP_PITCH = 2.0    KP_YAW = 2.5
KI_ROLL = 0.1    KI_PITCH = 0.1    KI_YAW = 0.15
KD_ROLL = 0.8    KD_PITCH = 0.8    KD_YAW = 0.4
```

### Medium Drone (500g-1kg)
```
KP_ROLL = 1.5    KP_PITCH = 1.5    KP_YAW = 2.0
KI_ROLL = 0.05   KI_PITCH = 0.05   KI_YAW = 0.1
KD_ROLL = 0.8    KD_PITCH = 0.8    KD_YAW = 0.4
```

### Heavy Drone (> 1kg)
```
KP_ROLL = 1.0    KP_PITCH = 1.0    KP_YAW = 1.5
KI_ROLL = 0.02   KI_PITCH = 0.02   KI_YAW = 0.05
KD_ROLL = 0.6    KD_PITCH = 0.6    KD_YAW = 0.3
```

---

**Remember:** Small changes (±10-20%) work better than big jumps!
Start conservative, tune gradually.
