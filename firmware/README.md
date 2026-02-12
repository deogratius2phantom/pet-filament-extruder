# Firmware

This directory contains the PlatformIO-based firmware for the PET Filament Extrusion System controller.

## Requirements

- [PlatformIO Core](https://platformio.org/) or [PlatformIO IDE for VS Code](https://platformio.org/install/ide?install=vscode)
- USB cable for programming
- ESP32 development board (or configured target platform)

## Getting Started

### Installation

1. Install PlatformIO (if not already installed)
2. Clone this repository
3. Navigate to the `firmware/` directory

### Building

```bash
pio run
```

### Uploading

```bash
pio run --target upload
```

### Serial Monitor

```bash
pio device monitor
```

Or use the PlatformIO IDE interface in VS Code.

## Configuration

(Configuration options to be documented)

## Architecture

The firmware is organized into the following components:

- **Main Loop**: Core control logic
- **Motor Control**: Stepper/servo motor control for extrusion
- **Temperature Management**: Heater and sensor control
- **User Interface**: Input handling and display updates

(Detailed architecture documentation to be added)

## Development

### Project Structure

- `src/` - Source code files
- `include/` - Header files
- `lib/` - Custom libraries
- `test/` - Unit tests

### Adding Libraries

Edit `platformio.ini` and add libraries to the `lib_deps` section.
