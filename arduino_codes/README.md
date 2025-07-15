# Wheel Encoder RPM & Speed Measurement (Arduino)

To measures the RPM (Revolutions Per Minute) and linear speed (in meters per second) of a wheel using an encoder.

## Purpose

This code is useful for real-time velocity estimation in mobile robotics, RC cars, and other motion-controlled systems. It calculates:

- **Wheel RPM** using encoder pulses
- **Linear velocity (m/s)** using the estimated wheel diameter

## ⚙️ How It Works

- An **interrupt** is triggered on every encoder pulse (rising edge).
- The **time difference** between two consecutive pulses is measured.
- This time is used to compute:
  - Pulses per second
  - Revolutions per second (considering encoder resolution)
  - RPM
  - Linear speed: `v = ω * r = (rev/s) * 2πr`

## 🛠️ Configuration

Update these values in the code to match your hardware setup:

```cpp
const int pulsesPerRevolution = 3;           // Number of encoder pulses per full wheel revolution
const float wheelDiameterMeters = 0.105;     // Diameter of your wheel in meters
```
## Timing and Sampling
-The loop() runs at 10Hz (delay(100))
-Timeout of 1 second is used to determine when the wheel has stopped (can change this )

## Notes
- Make sure the encoder is connected to a hardware interrupt-capable pin (like pin 2 or 3 on most Arduino boards).
