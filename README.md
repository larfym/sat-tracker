# Satellite Tracker

An ESP32-based automated dish antenna tracking system for satellites LEO with web interface control and telemetry.

## Features

- Real-time satellite tracking using SGP4 orbit prediction
- GPS time and position synchronization
- Web interface for configuration and monitoring
- Manual and automatic tracking modes 
- Dual-axis control (azimuth and elevation)
- Position feedback using reed switches
- Current monitoring for motor protection & control
- PID-based position control

## Hardware Requirements

- ESP32 development board
- GPS module (gy-neo6mv2)
- Custom Dual H-bridge motor driver with (L6205)
- ...
- ...

## Software Dependencies

- [SGP4](https://github.com/Hopperpop/Sgp4-Library) - Satellite orbit prediction
- [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer) - Async web server
- [TinyGPS++](https://github.com/mikalhart/TinyGPSPlus) - GPS NMEA parsing
- [ArduinoJson](https://arduinojson.org/) - JSON processing

## Building and Installation

1. Clone this repository
2. Install [PlatformIO](https://platformio.org/)
3. Open project in VS Code with PlatformIO extension
4. Build and upload:

```bash
platformio run --target uploadfs
platformio run -t upload
```

## Web Interface

Access the web interface by:

1. Connect to the ESP32 WiFi AP (default SSID: "SatTracker")
2. Navigate to http://tracker.local
3. Use the interface to:
   - Configure satellite TLE data.
   - Adjust antenna offsets.
   - View Telemetry data.
   - Manual Track.
   - Automatic Track.

## Project Structure

- `src/` - Source files
- `include/` - Header files (Hardware Components & Utilities)
- `data/` - Web interface files (Saved on SPIFFS)
- `lib/` - External libraries


## License

This project is licensed under the GNU General Public License v3.0 - see [LICENSE](LICENSE) for details.
