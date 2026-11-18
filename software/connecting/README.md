# Setup and configuration of mobile apps

**⏱️ Time:** 10-20 minutes per app | **🎯 Difficulty:** Beginner to Intermediate (varies by platform)

This directory contains detailed setup instructions for connecting BonoGPS to various lap timer applications.

## Supported Apps

| App | iOS | Android | Recommended Connection | Features | Setup Guide |
| --- | --- | ------- | ---------------------- | -------- | ----------- |
| Harry's Lap Timer | ✓ | ✓ | BLE (iOS), BT-SPP (Android) | Satellite view, 20Hz support, battery status | [Guide](harrylaptimer) |
| TrackAddict | ✗ | ✓ | BT-SPP | Standard lap timing | [Guide](trackaddict) |
| RaceChrono | ✓ | ✓ | TCP-IP (iOS), BLE, BT-SPP or TCP-IP (Android); BLE untested on iOS | Satellite view (NMEA connections only), clean UX, 25Hz over BLE | [Guide](racechrono) |
| RaceTime | ✗ | ✓ | BT-SPP | Lightweight, simple setup | [Guide](racetime) |

**ESP32-S3 boards (e.g. LilyGO T-Energy-S3)** have no BT-SPP. Use BLE (Harry's Lap Timer on iOS, RaceChrono) or the WiFi (TCP-IP) presets instead.

## Quick Start

1. Choose your app from the table above
2. Follow the detailed setup guide for your platform
3. Load the appropriate preset from the BonoGPS web interface (*Device > Load Preset*)
4. Connect and start recording!
