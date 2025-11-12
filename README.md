# What is BonoGPS?

- [What is BonoGPS?](#what-is-bonogps)
  - [For the track day enthusiast](#for-the-track-day-enthusiast)
  - [For the SW Engineer / maker](#for-the-sw-engineer--maker)
    - [Diagram of project components](#diagram-of-project-components)
  - [Quick Start Guide](#quick-start-guide)
  - [Known Good Configurations](#known-good-configurations)
  - [Complete Bill of Materials (BOM)](#complete-bill-of-materials-bom)
  - [How to Verify Everything is Working](#how-to-verify-everything-is-working)
  - [Common Mistakes to Avoid](#common-mistakes-to-avoid)
  - [HowTo](#howto)
    - [Daily Usage](#daily-usage)
      - [Save a configuration](#save-a-configuration)
    - [Connecting to an app](#connecting-to-an-app)
  - [Hardware build instructions](#hardware-build-instructions)
    - [GPS Choice and setup preconfiguration](#gps-choice-and-setup-preconfiguration)
  - [Software build instructions](#software-build-instructions)
  - [Technical Specifications](#technical-specifications)
  - [Troubleshooting and FAQ](#troubleshooting-and-faq)
  - [Possible enhancements and ideas](#possible-enhancements-and-ideas)
  - [Glossary](#glossary)
  - [Credits and tools](#credits-and-tools)

## For the track day enthusiast

The main goal of this device is to **make GPS data (speed, location) available to mobile apps that can record data up to 25 Hz, for example for track riding**. While there are many good (and not even too expensive) solutions out there, building one for yourself is a great experience.

![Map of Buttonwillow lateral acceleration](software/using/buttonwillow_map.png)

Currently these apps are supported

1. [Harry's Lap Timer](https://www.gps-laptimer.de) >> details [here](software/connecting/harrylaptimer)
2. [TrackAddict](https://www.hptuners.com/product/trackaddict-app/) >> details [here](software/connecting/trackaddict)
3. [RaceChrono](https://racechrono.com/) >> details [here](software/connecting/racechrono)
4. [RaceTime](https://www.racetimeapp.com/en/) >> details [here](software/connecting/racetime)

If you are not into the maker thing or if you don't know anyone who might be, [look into RaceBox products like mini/miniS](https://www.racebox.pro/) and even [a 25Hz kit](https://www.racebox.pro/products/racebox-micro), or [a XGPS160](https://gps.dualav.com/explore-by-product/xgps160).

## For the SW Engineer / maker

This repo contains software, a list of hardware, and (simple) schematics to build an ESP32 device that reads NMEA sentences from a GPS receiver compatible with u-blox M10 and M8 series, for example:

| GPS Module | Chipset | Max Refresh | Antenna Type | Recommended | Link |
| ---------- | ------- | ----------- | ------------ | ----------- | ---- |
| **BK880** | M10 | **25 Hz** | Active | ✓ Best | [Store](https://store.beitian.com/products/beitian-compass-qmc5883l-amp2-6-pix4-pixhawk-gnss-gps-glonass-dual-flight-control-gps-module-bn-880q?variant=44977758011679) |
| **BK280** | M10 | **25 Hz** | Passive | ✓ Best | [Store](https://store.beitian.com/collections/gps-module/products/beitian-gps-module-with-antenna-ubx-m10050-gnss-chip-ultra-low-power-gnss-receiver-for-track-be-180?variant=44859232420127) |
| BN880 | M8 | 10 Hz | Active | Good | [Store](https://store.beitian.com/products/beitian-ubx-m8030-g-mouse-supports-gps-qzss-and-sbas-fixed-wing-traversing-aircraft-gps-module-antenna-bn-180-220-280-357-880-880q?variant=46725104730399&_pos=1&_sid=fe8b2c602&_ss=r) |
| BN220 | M8 | 10 Hz | Passive | Budget | [Store](https://store.beitian.com/products/beitian-ubx-m8030-g-mouse-supports-gps-qzss-and-sbas-fixed-wing-traversing-aircraft-gps-module-antenna-bn-180-220-280-357-880-880q?variant=46694929989919&_pos=1&_sid=fe8b2c602&_ss=r) |
| DIYmall NEO-M8N | M8N | 10 Hz | Active | Compatible | [Amazon](https://www.amazon.com/DIYmall-NEO-M8N-Module-HMC5983-Antenna/dp/B012RNLG0K) |

**Note:** Active antennas provide better signal quality. M10 modules (BK880/BK280) offer 25Hz refresh rate which is ideal for track use. See [GPS setup guide](hardware/GPS) for detailed configuration.

and repeats them back to a logger device, either

1. a Bluetooth Low Energy (BLE) service
2. a BT Classic (BT-SPP) stream
3. a TCP-IP socket

The logger device is likely going to be one of the *Track Lap time apps* listed above running on your phone.

Examples of actual devices are in [hardware/assembled](hardware/assembled), at a cost of 25$ and 40$ (GPS receivers with active antennas are a little bit more expensive but worth it, and a larger battery helps as well)

![Prototype picture](hardware/assembled/bonogps_bn220_side.jpg)

A web configuration (basic responsive HTML) panel allows changing configuration and selecting GPS parameters on the fly: access it via [http://bonogps.local](http://bonogps.local) (when in WiFi AP mode, this becomes [http://10.0.0.1](http://10.0.0.1) on Android without mDNS resolution).

The settings required by each app are available as presets that you can load for your device (iOS, Android). More information on what version, features, and connections of each app are in [software/connecting](software/connecting).

You can also interface your GPS to [uBlox u-center](https://www.u-blox.com/en/product/u-center) via TCP-IP. In this case, be aware that the UART speed is fixed.

### Diagram of project components

![Project diagram](software/bonogps_project_diagram.png)

## Quick Start Guide

**New to BonoGPS? Get started in 5 steps:**

### Step 1: Get the Hardware (Budget: $30-60)

**Minimum setup (~$30):**
- ESP32 DevKit board (~$8-12) - [Amazon](https://www.amazon.com/s?k=esp32+devkit)
- BN220 GPS module (~$15-20) - [Beitian Store](https://store.beitian.com/)
- USB cable for programming/power
- Basic jumper wires

**Recommended setup (~$45):**
- ESP32 DevKit or LOLIN D32 PRO (~$12-20)
- BN880 GPS with active antenna (~$25) - [Beitian Store](https://store.beitian.com/)
- Li-Ion battery (650-2000mAh, ~$5-10)
- Case or 3D printed enclosure

**Best performance (~$60):**
- LOLIN D32 PRO with built-in battery charger (~$15-20)
- BK880 or BK280 GPS (M10, 25Hz) (~$35-45) - [Beitian Store](https://store.beitian.com/)
- 2000mAh Li-Ion battery (~$8-12)
- Custom 3D printed mount

### Step 2: Wire It Up (15 minutes)

Connect GPS to ESP32 - only 4 wires needed:
- GPS **VCC** → ESP32 **3.3V** (power)
- GPS **GND** → ESP32 **GND** (ground)
- GPS **TX** → ESP32 **RX** (GPIO 16 on DevKit, GPIO 4 on LOLIN D32 PRO)
- GPS **RX** → ESP32 **TX** (GPIO 17 on DevKit, GPIO 2 on LOLIN D32 PRO)

See detailed wiring diagrams: [Generic ESP32](hardware/esp32) | [LOLIN D32 PRO](hardware/esp32/lolin_d32_pro.md)

### Step 3: Configure GPS Module (30 minutes)

**Critical:** GPS must be configured before first use.

1. Download [u-blox u-center](https://www.u-blox.com/en/product/u-center)
2. Connect GPS to computer via USB-to-serial adapter
3. Follow the [GPS configuration guide](hardware/GPS)
4. Set baudrate to **115200**
5. Enable required NMEA messages
6. Save configuration to GPS flash memory

### Step 4: Build and Upload Software (20-45 minutes)

**Option A: Arduino IDE (easier for beginners)**
1. Install [Arduino IDE 2.x](https://www.arduino.cc/en/software)
2. Add ESP32 board support
3. Install required libraries (see [library list](software/building#external-libraries))
4. Open `bonogps.ino`
5. Select "ESP32 Dev Module" or "LOLIN D32 PRO"
6. Select "Minimal SPIFFS (1.9MB)" partition scheme
7. Upload to ESP32

**Option B: PlatformIO (recommended for developers)**
1. Install [VS Code + PlatformIO](https://platformio.org/install/ide?install=vscode)
2. Clone this repository: `git clone https://github.com/renatobo/bonogps.git`
3. Open project folder in VS Code
4. Select build target for your board
5. Build and upload

See detailed instructions: [Software build guide](software/building)

### Step 5: Connect to Your App (10 minutes)

1. Power on BonoGPS - wait for GPS fix (LED blinks once/second)
2. Connect phone to **BonoGPS-XXXX** WiFi network
3. Open browser to [http://10.0.0.1](http://10.0.0.1)
4. Go to **Device > Load Preset** - select your app and platform
5. Enable appropriate connection (BLE/BT-SPP/TCP-IP)
6. Open your lap timer app and connect

**Supported apps:** [Harry's Lap Timer](software/connecting/harrylaptimer) | [TrackAddict](software/connecting/trackaddict) | [RaceChrono](software/connecting/racechrono) | [RaceTime](software/connecting/racetime)

**Need help?** See [Troubleshooting FAQ](#troubleshooting-and-faq) below.

---

## Known Good Configurations

These hardware and app combinations have been tested and confirmed working. Use these as reference if you want a guaranteed-to-work setup.

### Budget Build - iOS ($45)
**Hardware:**
- ESP32 DevKit (~$10)
- BN880 GPS with active antenna (~$25)
- 1000mAh Li-Ion battery (~$6)
- Micro USB cable for programming
- Basic transparent case

**Software & Connection:**
- Harry's Lap Timer (iOS)
- Connection: BLE
- Preset: "Harry's Lap Timer iOS BLE"
- Expected performance: 10Hz, good accuracy (2-3m)

**Why this works:** BN880 active antenna provides reliable signal, BLE is native to iOS, Harry's Lap Timer has best BLE support.

### Best Performance - iOS ($65)
**Hardware:**
- LOLIN D32 PRO (~$18)
- BK880 GPS M10 with active antenna (~$40)
- 2000mAh Li-Ion battery (~$10)
- Custom 3D printed mount

**Software & Connection:**
- Harry's Lap Timer (iOS)
- Connection: BLE
- Preset: "Harry's Lap Timer iOS BLE"
- Expected performance: 20Hz over BLE, excellent accuracy (<2m)

**Why this works:** M10 chipset supports 25Hz, LOLIN has battery management, BLE optimized for iOS. Best overall iOS solution.

### Budget Build - Android ($35)
**Hardware:**
- ESP32 DevKit (~$8)
- BN220 GPS with passive antenna (~$18)
- Powered via USB (no battery)
- Velcro mounting

**Software & Connection:**
- RaceChrono (Android)
- Connection: BT-SPP
- Preset: "RaceChrono Android BT-SPP"
- Expected performance: 10Hz, good accuracy in open areas

**Why this works:** BT-SPP is most reliable on Android, RaceChrono is free and widely used, no battery needed for testing.

### Recommended - Android ($50)
**Hardware:**
- ESP32 DevKit or LOLIN D32 PRO (~$12-18)
- BN880 GPS with active antenna (~$25)
- 1500mAh Li-Ion battery (~$8)
- Basic enclosure

**Software & Connection:**
- RaceChrono or TrackAddict (Android)
- Connection: BT-SPP
- Preset: App-specific BT-SPP preset
- Expected performance: 10Hz, excellent accuracy (2-3m)

**Why this works:** Active antenna ensures good signal, BT-SPP handles full NMEA messages reliably, proven battery life for track days.

### Advanced - Android with High Refresh ($65)
**Hardware:**
- LOLIN D32 PRO (~$18)
- BK880 M10 GPS with active antenna (~$40)
- 2000mAh Li-Ion battery (~$10)
- Professional mounting solution

**Software & Connection:**
- Harry's Lap Timer (Android)
- Connection: BT-SPP or TCP-IP
- Preset: "Harry's Lap Timer Android BT-SPP"
- Expected performance: 10Hz over BT-SPP (25Hz requires TCP-IP)

**Why this works:** M10 chipset provides maximum performance, larger battery for full track day, LOLIN handles power management. **Note:** 25Hz requires TCP-IP connection which has less range than Bluetooth.

### Universal WiFi Setup - iOS & Android ($45)
**Hardware:**
- ESP32 DevKit (~$10)
- BN880 GPS (~$25)
- 2000mAh Li-Ion battery (~$10) - important for WiFi power draw

**Software & Connection:**
- RaceChrono (iOS or Android)
- Connection: TCP-IP over WiFi
- IP: 10.0.0.1, Port: 1818
- Expected performance: 10Hz, works on any platform

**Why this works:** TCP-IP works universally, no Bluetooth pairing needed, RaceChrono supports custom TCP devices. Larger battery compensates for WiFi power consumption.

### What About TrackAddict?
**Hardware:** Any ESP32 + BN220/BN880
**Platform:** Android only (not available on iOS)
**Connection:** BT-SPP only
**Important:** Must use **GP Talker ID** (not GN). Load "TrackAddict Android BT-SPP" preset.

### Testing Configuration (Cheapest, $30)
For initial testing before investing in batteries/cases:
- ESP32 DevKit (~$8)
- BN220 GPS (~$18)
- USB power from laptop
- Any supported app
- Test app connection indoors via WiFi, then test GPS outdoors

---

## Complete Bill of Materials (BOM)

Use this comprehensive shopping list to gather all components for your BonoGPS build. Choose components based on your budget and performance requirements.

### Core Components (Required)

| Component | Options | Qty | Est. Cost | Purpose | Where to Buy | Notes |
|-----------|---------|-----|-----------|---------|--------------|-------|
| **ESP32 Board** | ESP32 DevKit (generic) | 1 | $8-12 | Main controller | [Amazon](https://www.amazon.com/s?k=esp32+devkit), AliExpress | Most common, cheapest |
| | LOLIN D32 PRO | 1 | $15-20 | Main controller + extras | [WEMOS/LOLIN Store](https://www.wemos.cc/) | Battery charger, PSRAM, TF card |
| **GPS Module** | BN220 (M8, passive) | 1 | $15-20 | Budget GPS | [Beitian Store](https://store.beitian.com/) | 10Hz max, passive antenna |
| | BN880 (M8, active) | 1 | $23-27 | Good GPS | [Beitian Store](https://store.beitian.com/) | 10Hz max, active antenna (recommended) |
| | BK280 (M10, passive) | 1 | $35-40 | Best GPS | [Beitian Store](https://store.beitian.com/) | 25Hz capable, passive antenna |
| | BK880 (M10, active) | 1 | $38-45 | Best GPS | [Beitian Store](https://store.beitian.com/) | 25Hz capable, active antenna (best performance) |
| | NEO-M8N | 1 | $25-30 | Compatible GPS | [Amazon](https://www.amazon.com/s?k=neo-m8n+gps) | Alternative M8 module |
| **USB Cable** | Micro USB cable | 1 | $3-5 | Programming & power | Amazon, any electronics store | For ESP32 DevKit |
| | USB-C cable | 1 | $5-8 | Programming & power | Amazon, any electronics store | For LOLIN D32 PRO |
| **Jumper Wires** | Male-to-female, 10cm | 4+ | $2-5 | GPS wiring | Amazon, AliExpress | Only needed if not soldering |

**Core components subtotal:** $30-60 depending on choices

### Power Components (Recommended for portable use)

| Component | Options | Qty | Est. Cost | Purpose | Where to Buy | Notes |
|-----------|---------|-----|-----------|---------|--------------|-------|
| **Battery** | 650mAh Li-Ion 3.7V | 1 | $5-7 | ~2-2.5 hrs runtime | Amazon, hobby stores | JST connector |
| | 1000mAh Li-Ion 3.7V | 1 | $6-8 | ~3-4 hrs runtime | Amazon, hobby stores | JST connector |
| | 1500mAh Li-Ion 3.7V | 1 | $8-10 | ~5-6 hrs runtime | Amazon, hobby stores | JST connector |
| | 2000mAh Li-Ion 3.7V | 1 | $8-12 | ~6-8 hrs runtime | Amazon, hobby stores | JST connector, best for track day |
| **Battery Charger** | USB Li-Ion charger | 1 | $3-6 | Battery charging | Amazon, AliExpress | Not needed for LOLIN D32 PRO (built-in) |
| | LiPo balance charger | 1 | $15-25 | Multi-battery charging | Amazon, hobby stores | Optional, for multiple batteries |

**Power components subtotal:** $5-40 depending on battery size and charger

### Assembly Components (Optional but recommended)

| Component | Options | Qty | Est. Cost | Purpose | Where to Buy | Notes |
|-----------|---------|-----|-----------|---------|--------------|-------|
| **Enclosure** | Transparent plastic case | 1 | $3-8 | Basic protection | Amazon, electronics stores | Various sizes available |
| | Custom 3D printed case | 1 | $5-15 | Perfect fit | Local makerspace, online 3D print services | STL files in [hardware/assembled](hardware/assembled) |
| | Weatherproof box | 1 | $10-20 | Professional protection | Amazon | For permanent motorcycle installations |
| **Mounting** | Velcro strips | 1 set | $3-5 | Removable mounting | Amazon, hardware stores | Easy attachment/removal |
| | Double-sided tape (3M VHB) | 1 roll | $5-10 | Semi-permanent | Amazon, hardware stores | Strong, vibration-resistant |
| | Zip ties | 10+ | $2-5 | Cable management | Amazon, hardware stores | Various sizes |
| **Wiring** | Heat shrink tubing | 1 set | $5-8 | Wire insulation | Amazon | Professional finish |
| | 22-24 AWG stranded wire | 1m | $3-5 | Custom wiring | Amazon, electronics stores | If making custom cables |
| | JST connectors | 5 sets | $5-10 | Battery connectors | Amazon, hobby stores | For custom battery connections |
| **Soldering** | Soldering iron kit | 1 | $15-40 | Permanent connections | Amazon | If you don't have one |
| | Solder (lead-free) | 1 roll | $8-15 | Soldering | Amazon, electronics stores | 60/40 or lead-free |
| | Flux pen | 1 | $5-8 | Better solder joints | Amazon | Makes soldering easier |

**Assembly components subtotal:** $0-80+ depending on tools and quality

### Configuration Tools (One-time purchase)

| Component | Options | Qty | Est. Cost | Purpose | Where to Buy | Notes |
|-----------|---------|-----|-----------|---------|--------------|-------|
| **USB-to-Serial Adapter** | CP2102 or FTDI adapter | 1 | $5-10 | GPS configuration | Amazon, AliExpress | Required for GPS setup with u-center |
| | USB-to-TTL with 3.3V/5V switch | 1 | $8-12 | GPS configuration (better) | Amazon | More versatile, safer |

**Configuration tools subtotal:** $5-12 (one-time, reusable)

### Software (Free)

| Software | Cost | Purpose | Where to Get |
|----------|------|---------|--------------|
| Arduino IDE 2.x | Free | Firmware compilation (beginner) | [arduino.cc](https://www.arduino.cc/en/software) |
| VS Code + PlatformIO | Free | Firmware compilation (advanced) | [platformio.org](https://platformio.org/) |
| u-blox u-center | Free | GPS configuration | [u-blox.com](https://www.u-blox.com/en/product/u-center) |
| Harry's Lap Timer | $0-100 | iOS/Android lap timing | App Store / Play Store |
| RaceChrono | Free-$20 | iOS/Android lap timing | App Store / Play Store |
| TrackAddict | Free-$25 | Android lap timing | Play Store |
| RaceTime | Free | Android lap timing | Play Store |

### Shopping List Templates

**Minimum viable build ($30-35):**
- [ ] ESP32 DevKit ($8-12)
- [ ] BN220 GPS ($15-20)
- [ ] Micro USB cable ($3-5)
- [ ] 4x jumper wires ($2-5) OR soldering supplies
- [ ] USB-to-serial adapter ($5-10) for GPS config
- **Total: ~$33-52**

**Recommended portable build ($50-65):**
- [ ] ESP32 DevKit or LOLIN D32 PRO ($10-18)
- [ ] BN880 GPS ($25)
- [ ] 1000-1500mAh battery ($6-10)
- [ ] USB cable ($3-8)
- [ ] Plastic case ($3-8)
- [ ] Velcro or double-sided tape ($3-5)
- [ ] USB-to-serial adapter ($5-10)
- **Total: ~$55-84**

**Best performance build ($70-90):**
- [ ] LOLIN D32 PRO ($15-20)
- [ ] BK880 M10 GPS ($38-45)
- [ ] 2000mAh battery ($8-12)
- [ ] USB-C cable ($5-8)
- [ ] 3D printed custom case ($5-15)
- [ ] Mounting solution ($3-10)
- [ ] USB-to-serial adapter ($5-10)
- [ ] Soldering supplies if needed ($15-40)
- **Total: ~$79-160**

### Notes on Purchasing

**GPS Modules:**
- Buy directly from Beitian Store for authentic modules
- Beware of clones on AliExpress/eBay (may have older firmware or fake chipsets)
- Active antennas significantly improve performance in difficult conditions

**Batteries:**
- Match voltage: 3.7V Li-Ion or LiPo only
- Check connector: JST 2-pin is most common
- Capacity = runtime: 650mAh → 2hrs, 2000mAh → 8hrs
- Never charge Li-Ion batteries below 0°C (32°F)

**ESP32 Boards:**
- DevKit: Universal compatibility, cheapest
- LOLIN D32 PRO: Built-in battery management worth the extra cost for portable use

**Where to save money:**
- Skip the case initially (test on desk)
- Use USB power instead of battery for testing
- Use jumper wires instead of soldering initially
- BN220 instead of BN880 (passive vs active antenna)

**Where NOT to save money:**
- GPS module quality (buy from reputable source)
- USB cable quality (cheap cables cause upload/power issues)
- Battery quality (cheap batteries fail quickly)

---

## How to Verify Everything is Working

After completing the Quick Start, use this checklist to confirm your BonoGPS is functioning correctly:

### Hardware & Boot Checks
- ✅ **ESP32 powers on:** Device responds when plugged in via USB or battery
- ✅ **Built-in LED indicates WiFi mode:**
  - Slow blink (500ms cycle) = Access Point mode active
  - Fast blink (250ms cycle) = Client mode active
  - Off = WiFi disabled
- ✅ **GPS module powers on:** GPS module LED is visible (if module has one)

### GPS Functionality
- ✅ **GPS acquires satellite fix:**
  - BN/BK modules: Red LED blinks once per second after fix
  - First time: Allow 5-15 minutes outdoors with clear sky view
  - Subsequent times: Should get fix in 1-5 seconds
- ✅ **Satellite count adequate:** At least 4 satellites visible for accurate positioning
  - Check in web interface under "Status" or "GPS Info"
  - 8+ satellites = excellent signal
  - 4-7 satellites = good signal
  - <4 satellites = insufficient for accurate position

### Connectivity & Web Interface
- ✅ **Can access web interface:**
  - Connect to BonoGPS-XXXX WiFi network
  - Navigate to http://10.0.0.1 (Android) or http://bonogps.local (iOS/desktop)
  - Configuration page loads successfully
- ✅ **Web interface shows correct device info:**
  - Device name displays (BonoGPS-XXXX)
  - Battery voltage shown (if using LOLIN D32 PRO with battery)
  - Firmware version visible

### App Connection
- ✅ **Mobile app discovers device:**
  - BLE: Device appears as "BonoGPS-XXXX" in Bluetooth settings
  - BT-SPP: Device pairs successfully via Bluetooth
  - TCP-IP: App connects to 10.0.0.1:1818
- ✅ **App receives GPS data:**
  - Speed readings update in real-time
  - Position shows on map (if app has map view)
  - Satellite count matches GPS module readings
- ✅ **Data update rate is correct:**
  - 10Hz modules: ~10 updates per second
  - 25Hz modules: ~25 updates per second (M10 chipset only)

### Track Day Ready
- ✅ **Battery runtime adequate:** Full charge provides expected runtime based on capacity
- ✅ **GPS maintains fix while moving:** No dropouts during test drive
- ✅ **Mounting secure:** Device stays in place under vibration
- ✅ **App records session successfully:** Can complete and save a test session

**If any check fails**, see the [Troubleshooting FAQ](#troubleshooting-and-faq) section below.

---

## Common Mistakes to Avoid

### Critical Mistakes (Will prevent device from working)

**❌ Forgetting to configure GPS before first use**
- **Why it matters:** GPS modules ship with default settings that may not work with BonoGPS or your app
- **Solution:** Follow the [GPS configuration guide](hardware/GPS) before connecting GPS to ESP32
- **Time cost:** 30-45 minutes to configure correctly

**❌ Wrong partition scheme during upload**
- **Why it matters:** Default partition is too small for BonoGPS firmware with Bluetooth
- **Solution:** Select "Minimal SPIFFS (1.9MB)" in Arduino IDE or PlatformIO
- **Error:** Build fails with "not enough space" or upload fails

**❌ Swapped TX/RX pins**
- **Why it matters:** UART communication requires TX→RX and RX→TX crossover
- **Solution:** Connect GPS TX to ESP32 RX, and GPS RX to ESP32 TX
- **Symptom:** No GPS data received, web interface shows "No GPS fix"

**❌ Testing GPS indoors**
- **Why it matters:** GPS requires direct line of sight to satellites
- **Solution:** Test outdoors with clear sky view, away from buildings and trees
- **First fix:** Allow 5-15 minutes for initial satellite almanac download

### App Compatibility Mistakes

**❌ Using GN Talker ID with RaceChrono or TrackAddict**
- **Why it matters:** These apps require GP Talker ID specifically
- **Solution:** Load the app-specific preset from web interface (sets Main Talker ID = GP)
- **Symptom:** App shows "no data" or "invalid GPS"

**❌ Wrong connection method for platform**
- **Why it matters:** iOS doesn't support BT-SPP, Android BLE has limitations
- **Solution:**
  - **iOS:** Use BLE (Harry's Lap Timer) or TCP-IP (RaceChrono)
  - **Android:** Use BT-SPP (most reliable) or TCP-IP as backup
- **Symptom:** Device not found or connection drops

**❌ Too high refresh rate with satellite messages enabled**
- **Why it matters:** Bluetooth bandwidth is limited
- **Solution:**
  - For BLE: Use 20Hz or lower, disable GSA/GSV or poll at 5-second intervals
  - For BT-SPP: 10Hz is safe with full messages
- **Symptom:** Connection drops, data corruption, app crashes

### Build and Upload Mistakes

**❌ Wrong NimBLE library version**
- **Why it matters:** Breaking changes between versions 1.x, 2.x, and 3.x
- **Solution:** Use NimBLE-Arduino version **2.x** specifically
- **Error:** Compilation fails with "class NimBLEDevice has no member..."

**❌ Not cloning from Git (downloading ZIP instead)**
- **Why it matters:** Build script uses Git to determine version info
- **Solution:** Use `git clone https://github.com/renatobo/bonogps.git`
- **Error:** "git_rev_macro.py" errors during PlatformIO build

**❌ Insufficient USB power**
- **Why it matters:** ESP32 + GPS + WiFi can draw up to 500mA
- **Solution:** Use quality USB cable and powered USB port (not USB hub)
- **Symptom:** Random resets, brownouts, WiFi disconnections

### Hardware Assembly Mistakes

**❌ Using 5V instead of 3.3V for GPS power**
- **Why it matters:** Most GPS modules are 3.3V only, 5V will damage them
- **Solution:** Connect GPS VCC to ESP32 **3.3V pin**, never 5V
- **Result:** Permanent GPS module damage

**❌ Weak solder connections for motorcycle use**
- **Why it matters:** Vibration will eventually break cold solder joints
- **Solution:** Use proper soldering technique or crimped connectors
- **Symptom:** Intermittent GPS connection, random disconnections

**❌ GPS antenna placement under metal or carbon fiber**
- **Why it matters:** Metal and carbon fiber block GPS signals completely
- **Solution:** Place antenna under plastic fairings/seat cowls only
- **Symptom:** Never gets satellite fix, satellite count always 0

### Configuration Mistakes

**❌ Not saving GPS configuration to flash**
- **Why it matters:** Configuration is lost when GPS powers off
- **Solution:** In u-center, send UBX-CFG-CFG command to save to flash
- **Symptom:** GPS works after configuration but reverts on next power-on

**❌ Mismatched baudrate between GPS and ESP32**
- **Why it matters:** Serial communication requires matching speeds
- **Solution:** Set GPS to 115200 baud, matches `GPS_STANDARD_BAUD_RATE` in code
- **Symptom:** Garbled data, no GPS messages received

**❌ Forgetting to enable required NMEA messages**
- **Why it matters:** Apps need specific messages (GGA, RMC minimum)
- **Solution:** Load app preset or manually enable messages in u-center
- **Symptom:** App connects but shows "no GPS data"

**Pro tip:** Follow the Quick Start Guide step-by-step and verify each stage before moving to the next. Most issues are caught early this way.

---

## HowTo

### Daily Usage

1. Turn on the ESP32 and wait a few minutes for the GPS to get a fix on position (on BN devices, the red led will blink every second)
2. Open your preferred mobile app and connect it to your BonoGPS-*ABCD* device (ABCD is a combination of 4 letters and numbers unique to your ESP32 device)
3. Enjoy your ride!

Most runtime configurations are managed via its web-based interface: you can use your phone or any device with a web browser which is able to connect to either the built-in Access Point the BonoGPS provides or (if you are home and you have configured the WiFi Client credentials) [http://bonogps.local](http://bonogps.local)

The local built-in access point will be named BonoGPS-*ABCD* device (ABCD is a combination of 4 letters and numbers unique to your ESP32 device) and the password to it is in the source code (you can change it to your discretion - it's simply meant to avoid anyone connecting by mistake)

![Main page of web configuration panel from mobile device](software/using/webinterface_root_mobile.png)

![Main page of web configuration panel from laptop](software/using/webinterface_root_laptop.png)

- **GPS runtime settings:** GPS messages, rate of updates, serial port speed
- **Connections:** List clients, enable BLE/BT-SPP connectivity, enable/disable the builtin TCP-IP messages repeater, switch between WiFi client to a well-known WiFi network or built-in Access Point
- **Device:** Put GPS in powersaving for some time, Disable WiFi, load preset configurations, save current configuration, collect information about current status of the device, save WiFi credentials for client mode restart the ESP32

The BOOT button (or another button of your choice) allows you to loop between WiFi modes on the fly:

- short press: toggle between WiFi Access Point (built in led, usually blue, slowly blinking 500 ms cycle) and no WiFi (built in led off)
- long press (at least 2 seconds): enable WiFi Client so that you can reach the unit from your preferred WiFi network (internal led/blue led light flashes rapidly with a 250 ms cycle)

#### Save a configuration

If you load a preset or if you change a runtime settings, you can preserve it across restart of the device.

Go to *Device > Save Config* and you'll find these options

![Save Config page](software/using/webinterface_saveconfig.png)

For convenience, the WiFi status (Start its own AP, or connect to a local WiFi) can be excluded or saved separately.

### Connecting to an app

There are many mobile apps to log lap times, few accept custom devices, in particular on iOS. The ones below are tested.

| App | Platform | Connection | Max Refresh | Tested Version | Setup Guide |
| --- | -------- | ---------- | ----------- | -------------- | ----------- |
| [Harry's Lap Timer](https://www.gps-laptimer.de) | iOS | **BLE** (recommended), TCP-IP | 20Hz (BLE) | v24.9.1 | [Guide](software/connecting/harrylaptimer) |
| [Harry's Lap Timer](https://www.gps-laptimer.de) | Android | **BT-SPP** (recommended), TCP-IP | 10Hz | v24.9.1 | [Guide](software/connecting/harrylaptimer) |
| [TrackAddict](https://www.hptuners.com/product/trackaddict-app/) | Android | BT-SPP | 10Hz | v4.6.0 | [Guide](software/connecting/trackaddict) |
| [RaceChrono](https://racechrono.com/) | iOS | TCP-IP | 10Hz | v7.0.10 | [Guide](software/connecting/racechrono) |
| [RaceChrono](https://racechrono.com/) | Android | BT-SPP | 10Hz | v7.0.10 | [Guide](software/connecting/racechrono) |
| [RaceTime](https://www.racetimeapp.com/en/) | Android | BT-SPP | 10Hz | v3.3.8 lite | [Guide](software/connecting/racetime) |

**Note:** Recommended connection methods are shown in **bold**. See detailed setup instructions in each guide.

You can load a preset configuration from the configuration page selecting *Device > Load Preset* and then choosing your mobile phone device and app combination (when there are alternatives, the recommended option is in bold)

![Loading presets](software/using/webinterface_loadpreset.png)

## Hardware build instructions

**⏱️ Time:** 30-60 minutes | **🎯 Difficulty:** Beginner (basic soldering skills helpful)

The minimum build is a ublox M8 series GPS receiver module connected to an ESP32:

- TX/RX from the GPS to a serial port (default in the code is UART2/Serial2) on ESP32. Remember: you need to connect RX on one device to TX on the other and viceversa
- the GPS power pins to ESP32: VCC to 3v3, GND to GND

BN220 comes with a 4 pin adapter cable for GND TX RX VCC, while BN880 includes 2 additionals pins for SDA and SCL of the IMU which are not used so you should leave them unconnected.

Schematics are relatively simple

- power the GPS module (3.3V on ESP32 to VCC on GPS receiver, GND on ESP32 to GND on GPS receiver)
- connect GPS RX to ESP32 UART2 TX and GPS TX to ESP32 UART2 RX (RX and TX are switched: GPS transmits, ESP32 receives)

![Schematics](hardware/esp32/esp32_to_gps_schem.png)

Examples are in [hardware/assembled](hardware/assembled) including an example with more options using an ESP32 device type Lolin D32 Pro which has an internal battery charger [hardware/esp32/lolin_d32_pro.md](hardware/esp32/lolin_d32_pro.md)

**See also:**
- [GPS Configuration Guide](hardware/GPS) - Configure GPS before use
- [Assembly Troubleshooting](hardware/assembled#hardware-assembly-troubleshooting) - Common wiring issues
- [LOLIN D32 PRO Specific Guide](hardware/esp32/lolin_d32_pro.md) - Battery charging setup

### GPS Choice and setup preconfiguration

**⏱️ Time:** 30-45 minutes | **🎯 Difficulty:** Intermediate | **Prerequisites:** USB-to-serial adapter, u-center software

Thanks to needs of a very active drone community, there are a lot of inexpensive GPS receivers: some considerations on performance/accuracy/cost are reported [in the hardware/GPS folder](hardware/GPS).

What you need to configure is documented [in the hardware/GPS folder](hardware/GPS): this is **important**, the performances of your GPS won't be optimal until you do, or the whole setup will not work at all.

**See also:**
- [GPS Module Comparison](hardware/GPS#performance-comparison) - Detailed specs for each module
- [GPS Troubleshooting](hardware/GPS#gps-hardware-troubleshooting) - Module-specific issues
- [u-blox u-center Guide](hardware/GPS) - Configuration software instructions

## Software build instructions

**⏱️ Time:** 20-45 minutes (Arduino IDE: 20-30 min, PlatformIO: 30-45 min first time) | **🎯 Difficulty:** Beginner to Intermediate | **Prerequisites:** Arduino IDE or VS Code with PlatformIO, USB cable

This code is developed specifically for ESP32, and tested with [PlatformIO](https://platformio.org/) (main development platform) and the [Arduino IDE version 2 (2.3.4)](https://www.arduino.cc/en/software). More information on what libraries are needed and software organization [in the software/building folder](software/building).

**See also:**
- [Build Troubleshooting](software/building#build-troubleshooting) - Library versions, compilation errors
- [Board Settings](include/README.md) - Pin definitions and configurations
- [Connecting to Apps](software/connecting) - After successful build

## Technical Specifications

### Performance Expectations

**GPS Accuracy:**
- **Horizontal position:** 2-3 meters (6-10 feet) typical with M8/M10 chipsets
- **Speed accuracy:** ±0.1 km/h at constant velocity
- **Update rates:** 10Hz (M8 modules), 25Hz (M10 modules)
- **Cold start time:** 5-15 minutes (first power-on)
- **Hot start time:** 1-5 seconds (after recent fix)
- **Signal quality:** C/N0 values typically 30-40 dBHz (higher is better)

**Power Consumption:**
- **ESP32:** 80-160mA (varies with WiFi/Bluetooth activity)
- **GPS module:** 30-80mA (depends on module and fix status)
- **LED indicators:** 10-40mA
- **Total typical:** 250-350mA average during operation
- **Peak consumption:** Up to 500mA when WiFi and GPS are both active

**Battery Life Examples:**
- 650mAh battery: ~2-2.5 hours runtime
- 1000mAh battery: ~3-4 hours runtime
- 2000mAh battery: ~6-8 hours runtime (full track day)

**Note:** Actual battery life depends on WiFi/Bluetooth usage, GPS update rate, and environmental conditions.

### Connectivity Specifications

**WiFi:**
- **Standard:** 802.11 b/g/n (2.4 GHz only)
- **Range:** ~30-50 meters open air (depends on environment)
- **Access Point mode:** Supports up to 4 simultaneous clients
- **TCP server port:** 1818 (configurable)
- **mDNS hostname:** bonogps.local

**Bluetooth Low Energy (BLE):**
- **Standard:** Bluetooth 4.2 / 5.0 (depends on ESP32 module)
- **Range:** ~10-30 meters typical
- **Service UUID:** 1819 (Location and Navigation Service)
- **Max throughput:** ~20Hz GPS data with minimal satellite info
- **iOS support:** Full (recommended connection method)

**Bluetooth Classic (BT-SPP):**
- **Standard:** Bluetooth 2.0 SPP (Serial Port Profile)
- **Range:** ~10-30 meters typical
- **Baudrate:** 115200 (matches GPS UART)
- **Max throughput:** ~10Hz GPS data with satellite info
- **Android support:** Full (recommended connection method)

### Build Size and Memory

**Flash Memory Usage:**
- **With BLE + BT-SPP:** ~1.5-1.7 MB (requires Minimal SPIFFS partition)
- **With BLE only:** ~1.3-1.4 MB
- **With BT-SPP only:** ~1.2-1.3 MB
- **Minimum partition:** 1.9 MB app space required

**RAM Usage:**
- **ESP32 SRAM:** ~120-180 KB used (out of 320 KB available)
- **PSRAM:** Optional, helps with stability if available (LOLIN D32 PRO has 4MB)

**SPIFFS Storage:**
- **Configuration files:** ~2-4 KB
- **Reserved space:** 190 KB (with Minimal SPIFFS partition)

### Supported NMEA Messages

**Standard output messages:**
- `GxGGA` - Global Positioning System Fix Data
- `GxRMC` - Recommended Minimum Specific GPS Data
- `GxGBS` - GNSS Satellite Fault Detection (accuracy estimates)
- `GxGSA` - GPS DOP and Active Satellites (optional, polled)
- `GxGSV` - GPS Satellites in View (optional, polled)
- `GxVTG` - Track Made Good and Ground Speed
- `GxZDA` - Time and Date

**Talker ID support:**
- `GN` - Multi-constellation GNSS (GPS+GLONASS+Galileo+BeiDou)
- `GP` - GPS only (required by some apps like RaceChrono)

**Message rates:** Configurable from 1Hz to 25Hz (hardware dependent)

### Environmental Specifications

**Operating conditions:**
- **Temperature:** -20°C to +70°C (-4°F to 158°F) typical
- **Note:** Li-Ion batteries should not be charged below 0°C (32°F)
- **Humidity:** Most ESP32 boards are not waterproof without enclosure
- **Vibration:** Solder connections recommended for motorcycle use
- **Mounting:** GPS antenna requires clear sky view (metal/carbon blocks signal)

**GPS antenna considerations:**
- **Active antennas:** Better signal quality, require 3.3V power, ~50mA draw
- **Passive antennas:** Lower signal quality, no extra power required
- **Placement:** Best under plastic fairings/seat cowls, avoid metal/carbon

### Limitations and Known Issues

**Connection limitations:**
- **BT-SPP reconnection:** Android may require device restart or re-pairing
- **BLE throughput:** Limited to ~20Hz with full NMEA messages
- **WiFi TCP:** Single client at a time for GPS data stream
- **Memory pressure:** Running BLE + BT-SPP + WiFi simultaneously may cause instability

**GPS limitations:**
- **Indoor use:** GPS will not work indoors or under metal roofs
- **Urban canyons:** Tall buildings can degrade accuracy
- **Tree cover:** Dense foliage can reduce satellite visibility
- **First fix:** Takes 5-15 minutes on first power-on to download almanac

**Software limitations:**
- **OTA updates:** Disabled by default to save flash space
- **Web interface:** Basic HTML/CSS, optimized for mobile browsers
- **Configuration backup:** Manual export/import via web interface

See [Troubleshooting FAQ](#troubleshooting-and-faq) for solutions to common problems.

## Troubleshooting and FAQ

### Common Issues and Solutions

#### GPS and Hardware Issues

**Q: My GPS is not getting a fix / takes very long to acquire satellites**

**A:** Several factors can affect GPS acquisition time:
- **First time setup:** Cold start can take 5-15 minutes to download almanac data
- **Antenna placement:** Ensure the GPS antenna has clear view of the sky. The BN-880 LED should blink once per second when it has a fix
- **Antenna orientation:** For BN-880/BK-880, the antenna should face upward (skyward)
- **Indoor use:** GPS will not work indoors or in areas with obstructed sky view
- **Configuration:** Verify your GPS is properly configured with the correct baudrate (115200) and messages enabled. See [GPS configuration guide](hardware/GPS)

**Q: No GPS data is reaching my mobile app**

**A:** Troubleshooting steps:
1. Check that the GPS has a fix (BN devices: red LED blinking every second)
2. Verify the baudrate matches: GPS port should be 115200 (check `GPS_STANDARD_BAUD_RATE` in code)
3. Confirm correct NMEA messages are enabled for your app (load the appropriate preset)
4. Check TX/RX wiring: GPS TX → ESP32 RX, GPS RX → ESP32 TX
5. Open an [issue](https://github.com/renatobo/bonogps/issues) with details if problem persists

**Q: Which GPS module should I buy?**

**A:** See the [GPS comparison table](#for-the-sw-engineer--maker) in this README. Quick recommendations:
- **Best performance:** BK880 or BK280 (M10 chipset, 25Hz, ~$35-45)
- **Good balance:** BN880 (M8 chipset with active antenna, 10Hz, ~$25)
- **Budget option:** BN220 (M8 chipset with passive antenna, 10Hz, ~$15-20)

Active antennas provide significantly better signal quality. Avoid NEO-M8N if you need multiple GNSS constellations.

#### Bluetooth and Connection Issues

**Q: ESP32 reboots when I change Bluetooth settings / No Bluetooth available**

**A:** This is typically caused by:
- **Insufficient partition space:** Ensure you're using the "Minimal SPIFFS" partition scheme (1.9MB app space)
- **Memory issues:** Both BLE and BT-SPP enabled simultaneously can cause memory pressure. Try disabling one if not needed
- **Library conflicts:** Ensure you're using NimBLE-Arduino version 2.x (not 1.x)
- Check the [closed issue #60](https://github.com/renatobo/bonogps/issues/60) for resolution details

**Q: Can't reconnect to RaceChrono via BT-SPP after first disconnect**

**A:** Known issue with Android BT-SPP. Workarounds:
- Restart the ESP32 device
- "Forget" the Bluetooth device on Android and re-pair
- Restart the RaceChrono app
- See [issue #30](https://github.com/renatobo/bonogps/issues/30) for ongoing discussion

**Q: BLE connection issues with Harry's Lap Timer on iOS**

**A:** Common solutions:
- Ensure BLE is enabled in the web configuration
- Verify the service UUID is **1819** and characteristic UUID is **2A67**
- Device name should be `BonoGPS-XXXX` (check web interface header)
- iOS may cache old Bluetooth data - try restarting your iPhone
- BLE works best at 20Hz or lower with GSA/GSV polling disabled or at low frequency (every 5 seconds)

#### WiFi and Web Interface Issues

**Q: Can't access the web interface at bonogps.local**

**A:** Troubleshooting:
- **Android users:** Use [http://10.0.0.1](http://10.0.0.1) instead (Android doesn't support mDNS by default)
- **Access Point mode:** Connect to the BonoGPS-XXXX WiFi network first (password is in the source code)
- **Client mode:** Ensure your device is on the same WiFi network as BonoGPS
- Check the built-in LED: Slow blink (500ms) = AP mode, Fast blink (250ms) = Client mode
- Use the BOOT button to cycle between WiFi modes (short press for AP/off, long press for Client)

**Q: How do I change WiFi modes?**

**A:** Use the BOOT button (or external button on LOLIN D32 PRO):
- **Short press:** Toggle between Access Point mode and WiFi off
- **Long press (2+ seconds):** Switch to Client mode (connects to your saved WiFi network)

The built-in blue LED indicates the mode:
- Slow blinking (500ms cycle): Access Point active
- Fast blinking (250ms cycle): Client mode active
- Off: WiFi disabled

#### Build and Compilation Issues

**Q: Arduino IDE compilation fails with NimBLE errors**

**A:** Solutions:
- Use NimBLE-Arduino version **2.x** (not 1.x or 3.x)
- Install the exact library versions from `platform.ini` `lib_deps` section
- Use EasyButton version **2.0.1** specifically (newer versions have breaking changes)
- Ensure you selected "Minimal SPIFFS (1.9MB)" partition scheme
- See [issue #50](https://github.com/renatobo/bonogps/issues/50) for NimBLE-specific solutions

**Q: Build fails with "not enough space" or partition errors**

**A:** The compiled binary is large due to Bluetooth stacks:
- Select `Tools > Partition Scheme > Minimal SPIFFS (1.9MB APP)`
- In PlatformIO: use `board_build.partitions = min_spiffs.csv`
- Disable unused features (BLE or BT-SPP) if you only need one connection type

**Q: Git revision macro errors during PlatformIO build**

**A:** The build uses `git_rev_macro.py` to get version info:
- Clone the repository with git (don't download as ZIP)
- If issues persist, comment out the script invocation in `platformio.ini`
- Manually define `GIT_REV` and `GIT_REPO` macros if needed

### App-Specific Issues

**Q: RaceChrono shows weird/incorrect data**

**A:** Check these settings:
- Main Talker ID must be **GP** (not GN) for RaceChrono
- Enable only `GPGGA`, `GPRMC`, optionally `GPGSA`/`GPGSV`
- Load the RaceChrono preset from *Device > Load Preset*
- See [RaceChrono setup guide](software/connecting/racechrono)

**Q: TrackAddict not receiving data**

**A:** TrackAddict requirements:
- Requires Main Talker ID = **GP**
- Must have `GPRMC`, `GPGGA`, and `GPGLL` enabled
- Only works via BT-SPP on Android
- Load the TrackAddict preset from the web interface

### Getting Help

If your issue isn't covered here:

1. Check the [GitHub Discussions](https://github.com/renatobo/bonogps/discussions) - many questions already answered
2. Review [closed issues](https://github.com/renatobo/bonogps/issues?q=is%3Aissue+is%3Aclosed) for similar problems
3. Check app-specific guides in [software/connecting](software/connecting)
4. Open a new [issue](https://github.com/renatobo/bonogps/issues) with:
   - Your hardware (ESP32 model, GPS model)
   - Software version and how you built it
   - What app you're connecting to
   - Detailed description of the problem
   - Any error messages or logs

## Possible enhancements and ideas

See [issues with label enhancement](https://github.com/renatobo/bonogps/issues?q=is%3Aissue+is%3Aopen+label%3Aenhancement) on the github project

## Glossary

**GPS & Navigation Terms:**

- **GPS Fix:** When the GPS receiver has locked onto enough satellites (minimum 4) to calculate position. Indicated by LED blinking once per second on BN/BK modules.
- **Cold Start:** First power-on when GPS has no recent satellite data. Takes 5-15 minutes to download almanac and ephemeris data.
- **Hot Start:** GPS restart when recent satellite data is still valid. Typically achieves fix in 1-5 seconds.
- **Warm Start:** GPS restart after some time, when almanac is still valid but ephemeris is outdated. Takes 30-60 seconds.
- **Satellite Almanac:** Coarse orbital data for all satellites, valid for months. Required for cold start.
- **Ephemeris:** Precise orbital data for specific satellites, valid for 2-4 hours. Required for accurate positioning.
- **GNSS (Global Navigation Satellite System):** Umbrella term for all satellite navigation systems (GPS, GLONASS, Galileo, BeiDou).
- **GPS (Global Positioning System):** US satellite navigation system, one type of GNSS.
- **M8 / M10:** u-blox GPS chipset generations. M8 supports 10Hz max, M10 supports 25Hz max.
- **C/N0 (Carrier-to-Noise Ratio):** Signal quality measurement in dBHz. Higher is better (30-40 dBHz typical, 45+ excellent).
- **HDOP (Horizontal Dilution of Precision):** Geometric accuracy indicator. Lower is better (<1.0 excellent, 1-2 good, >5 poor).
- **DOP (Dilution of Precision):** Measure of satellite geometry effect on position accuracy.

**GPS Antenna Terms:**

- **Active Antenna:** Has built-in amplifier (LNA), requires power, better signal quality. Typical for BN880/BK880.
- **Passive Antenna:** No amplifier, cheaper, lower signal quality. Typical for BN220/BK280.
- **LNA (Low Noise Amplifier):** Built-in amplifier in active antennas that boosts weak GPS signals.

**GPS Message Formats:**

- **NMEA (National Marine Electronics Association):** Standard text format for GPS data (e.g., $GPGGA,123456...).
- **UBX:** u-blox binary protocol, more compact than NMEA but requires parsing library.
- **Talker ID:** Two-letter prefix indicating satellite system source:
  - **GP:** GPS only (US satellites)
  - **GN:** Multi-constellation GNSS (GPS + GLONASS + Galileo + BeiDou)
- **NMEA Messages:**
  - **GGA:** Global Positioning System Fix Data (position, altitude, fix quality)
  - **RMC:** Recommended Minimum Specific GPS Data (position, speed, course, time)
  - **GSA:** GPS DOP and Active Satellites (satellite IDs, dilution of precision)
  - **GSV:** GPS Satellites in View (satellite count, signal strength per satellite)
  - **GBS:** GNSS Satellite Fault Detection (accuracy estimates)
  - **VTG:** Track Made Good and Ground Speed (course and speed)
  - **ZDA:** Time and Date (UTC time and date)

**Connectivity Terms:**

- **BLE (Bluetooth Low Energy):** Low-power Bluetooth standard (Bluetooth 4.0+). Recommended for iOS with Harry's Lap Timer. Max ~20Hz GPS data.
- **BT-SPP (Bluetooth Serial Port Profile):** Classic Bluetooth serial connection. Recommended for Android. Max ~10Hz with full messages.
- **TCP-IP:** Network connection over WiFi. Works on both iOS and Android. Single client at a time for GPS stream.
- **UART (Universal Asynchronous Receiver-Transmitter):** Serial communication protocol between GPS and ESP32.
- **Baudrate:** Speed of serial communication in bits per second. BonoGPS uses 115200 baud.
- **mDNS (Multicast DNS):** Network protocol that resolves bonogps.local to IP address. Doesn't work on Android by default.

**ESP32 Terms:**

- **ESP32:** Microcontroller with built-in WiFi and Bluetooth. Brain of BonoGPS device.
- **Flash Memory:** Non-volatile storage on ESP32 where firmware is stored (~4MB total).
- **RAM/SRAM:** Volatile memory for program execution (~320KB on standard ESP32).
- **PSRAM:** Optional extra RAM (4MB on LOLIN D32 PRO) for improved stability.
- **Partition Scheme:** How flash memory is divided. BonoGPS requires "Minimal SPIFFS (1.9MB)" for app space.
- **SPIFFS:** File system on ESP32 for storing configuration files (~190KB with minimal partition).
- **OTA (Over-The-Air):** Wireless firmware updates. Disabled by default in BonoGPS to save flash space.
- **GPIO (General Purpose Input/Output):** Programmable pins on ESP32 for connecting peripherals.
- **UART2/Serial2:** Second serial port on ESP32, used for GPS communication (TX=GPIO17, RX=GPIO16 on DevKit).

**Build & Development Terms:**

- **Arduino IDE:** Beginner-friendly development environment for programming ESP32.
- **PlatformIO:** Professional IDE extension for VS Code, preferred for BonoGPS development.
- **Library:** Pre-written code package that adds functionality (e.g., NimBLE-Arduino for Bluetooth).
- **NimBLE:** Lightweight Bluetooth Low Energy library for ESP32. BonoGPS requires version 2.x.
- **Preset:** Pre-configured settings for specific apps, loaded via web interface (Device > Load Preset).

**u-blox Configuration Terms:**

- **u-center:** Windows software from u-blox for configuring GPS modules.
- **UBX-CFG-xxx:** Configuration commands for u-blox GPS modules (e.g., UBX-CFG-PRT for port settings).
- **Save to Flash:** Persist GPS configuration so it survives power cycles. Critical step often forgotten!

**Hardware Terms:**

- **DevKit:** Generic ESP32 development board, most common and cheapest option (~$8-12).
- **LOLIN D32 PRO:** Premium ESP32 board with battery charger, PSRAM, and TF card slot (~$15-20).
- **Li-Ion / LiPo:** Rechargeable lithium battery types. 3.7V nominal, charge to 4.2V, discharge to 3.0V minimum.
- **JST Connector:** Small 2-pin connector commonly used for batteries.
- **VCC:** Power supply voltage pin (3.3V for GPS modules).
- **GND:** Ground / 0V reference voltage pin.
- **TX (Transmit):** Data output pin. GPS TX connects to ESP32 RX.
- **RX (Receive):** Data input pin. GPS RX connects to ESP32 TX.

**App-Specific Terms:**

- **Harry's Lap Timer (HLT):** Most feature-rich lap timer app, only iOS app supporting BLE GPS.
- **RaceChrono:** Lap timer with cleanest UX, requires GP Talker ID.
- **TrackAddict:** Android-only lap timer, requires GP Talker ID.
- **Hz (Hertz):** Updates per second. 10Hz = 10 position updates per second. 25Hz = 25 updates per second (M10 modules only).

## Credits and tools

- Very valuable information from the mobile apps developers: [Harry's Lap Timer forum](http://forum.gps-laptimer.de/viewforum.php?f=2), [HP Tuner Track Adict forum](https://forum.hptuners.com/forumdisplay.php?74-TrackAddict), [RaceChrono forum](https://racechrono.com/forum/categories/diy-builds)
- Email conversations with Harald Schlangmann (Harry's Lap Timer) and Roberto Morini (Racetime) who I thank for the time and effort in developing and supporting their apps
- There are several other similar projects on github, a few from which I learned a lot: [RaceChronoDYI-TBeam](https://github.com/0x8008135/RaceChronoDYI-TBeam) [RaceChrono BLE DIY device (GPS and CAN-Bus)](https://github.com/aollin/racechrono-ble-diy-device) [DAWA](https://github.com/quichedood/DAWA-6.x)
- Screenshot framing by [Mockuphone](https://mockuphone.com/) and [Android developers marketing tools
](https://developer.android.com/distribute/marketing-tools/device-art-generator)
- [The GPS Dictionary](https://www.u-blox.com/sites/default/files/the_gps_dictionary.pdf)
