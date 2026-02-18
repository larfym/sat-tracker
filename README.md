# 🛰️ Satellite Tracker

An ESP32-based automated dual-axis antenna tracking system for Low Earth
Orbit (LEO) satellites, featuring real-time orbital prediction,
closed-loop motor control, and a web-based telemetry interface.

------------------------------------------------------------------------

## 📡 Description

**Satellite Tracker** is an embedded platform designed to automatically
track Low Earth Orbit (LEO) satellites in real time using orbital
prediction algorithms.

The system is powered by an ESP32 microcontroller, which:

-   Computes satellite position using the SGP4 orbital propagation model
-   Synchronizes time and geographic location via GPS
-   Controls a motorized offset dish antenna in azimuth and elevation
-   Provides telemetry and configuration through an integrated web
    interface

The platform is modular, scalable, and suitable for experimentation,
amateur satellite tracking, and engineering research.

------------------------------------------------------------------------

## 🚀 Features

-   Real-time satellite tracking using SGP4 orbit prediction
-   Automatic next-pass prediction
-   GPS-based time and position synchronization
-   Integrated web interface for monitoring and configuration
-   JSON API for remote telemetry access
-   Configurable antenna mechanical offsets
-   Satellite visibility calculation (eclipse / daylight / visible)
-   Scalable and modular firmware architecture
-   Manual and automatic tracking modes
-   Dual-axis mount control (azimuth & elevation)
-   Position feedback using reed switches
-   Motor current monitoring for protection and diagnostics
-   PI-based closed-loop position control

------------------------------------------------------------------------

## 📁 Proyect Structure

    sat-tracker/Firmware
    │
    ├── src/                 # Source code
    ├── include/             # Headers y configurations
    ├── data/                # Web Page (SPIFFS / LittleFS)
    ├── lib/                 # Libs
    ├── platformio.ini       # PlatformIO config.

------------------------------------------------------------------------

## 🔧 Hardware Requirements

-   ESP32 DevKit V1
-   GPS Module
-   Custom Power Interface Board
-   Custom Main Control Board
-   Azimuth and Elevation actuators
-   36V / 5A power supply

------------------------------------------------------------------------

## 💻 Software Dependencies

-   SGP4 -- Satellite orbit prediction\
    https://github.com/Hopperpop/Sgp4-Library

-   ESPAsyncWebServer -- Asynchronous web server\
    https://github.com/me-no-dev/ESPAsyncWebServer

-   TinyGPS++ -- GPS NMEA parsing\
    https://github.com/mikalhart/TinyGPSPlus

-   ArduinoJson -- JSON serialization and parsing\
    https://arduinojson.org/

All dependencies are automatically managed via PlatformIO.

------------------------------------------------------------------------

## 🛠 Building and Installation

1.  Clone this repository:

git clone https://github.com/larfym/sat-tracker.git

2.  Install PlatformIO: https://platformio.org/

3.  Open the Firmware folder in VS Code with the PlatformIO extension.

4.  Build and upload:

platformio run --target uploadfs platformio run --target upload

------------------------------------------------------------------------

## 🌐 Web Interface

To access the web interface:

1.  Connect to the ESP32 WiFi Access Point (Default SSID: SatTracker)
2.  Open your browser and navigate to:

http://tracker.local

From the web interface you can:

-   Configure satellite TLE data
-   Adjust antenna offsets
-   Monitor real-time telemetry
-   Switch between manual and automatic tracking
-   View GPS and system status
-   Monitor next pass prediction

## 📜 License

This project is licensed under the GNU General Public License v3.0 - see [LICENSE](LICENSE) for details.

------------------------------------------------------------------------

🛰 Satellite Tracker -- Embedded autonomous satellite tracking platform.
