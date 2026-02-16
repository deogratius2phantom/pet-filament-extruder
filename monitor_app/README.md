# PET Filament Extruder Monitor

Real-time monitoring and data logging application for the PET Filament Extruder Controller (ATmega2560).

## Features

- **Real-time Temperature Graphing**: Display temperature traces for all 5 heaters with color-coded lines
- **Motor Speed Monitoring**: Optional graph showing motor speeds (requires periodic polling)
- **CSV Data Logging**: Log all sensor data to CSV files with automatic rotation
- **Serial Console**: View raw serial output and send commands to the controller
- **Quick Commands**: One-click buttons for STATUS, SPEEDS, and HELP commands
- **Configurable History**: View 5, 10, 30, or 60 minutes of historical data

## Installation

### Prerequisites

- Python 3.7 or higher
- pip (Python package manager)

### Install Dependencies

```bash
cd monitor_app
pip install -r requirements.txt
```

The required packages are:
- `pyserial>=3.5` - Serial communication
- `PyQt5>=5.15.10` - GUI framework
- `pyqtgraph>=0.13.3` - Real-time graphing

## Usage

### Starting the Application

```bash
python main.py
```

Or make it executable:

```bash
chmod +x main.py
./main.py
```

### Connecting to the Extruder

1. **Select Port**: Choose your Arduino's serial port from the dropdown
   - macOS: `/dev/cu.usbmodem*` or `/dev/tty.usbmodem*`
   - Linux: `/dev/ttyACM*` or `/dev/ttyUSB*`
   - Windows: `COM*`

2. **Set Baud Rate**: Default is 115200 (matches firmware)

3. **Click Connect**: Wait for "Connected" message in console

### Using the Graphs

- **Temperature Graph**: Shows all 5 heater temperatures in real-time
  - Use checkboxes (H0-H4) to show/hide specific heaters
  - Colors: Red, Green, Blue, Yellow, Magenta

- **Motor Speed Graph**: Enable "Show Motor Speeds" checkbox
  - Automatically polls `SPEEDS` command every 5 seconds
  - Shows motor speeds for all 5 motors

- **History Duration**: Select 5, 10, 30, or 60 minutes from dropdown

### Data Logging

1. **Choose Directory** (optional): Click "Choose Directory" to select log location
   - Default: `./logs/` directory

2. **Start Logging**: Click "Start Logging" button
   - Creates CSV file: `extruder_log_YYYYMMDD_HHMMSS.csv`
   - File size shown in real-time

3. **Stop Logging**: Click "Stop Logging" button

4. **Automatic Rotation**: New file created when:
   - Current file reaches 100 MB
   - New day starts (midnight)

### CSV Format

Log files contain timestamped data with columns:

```
timestamp, temp_0, temp_1, ..., temp_4, 
stable_0, ..., stable_4, enabled_0, ..., enabled_4,
speed_0, ..., speed_4, running_0, ..., running_4
```

Example:
```csv
timestamp,temp_0,temp_1,temp_2,temp_3,temp_4,stable_0,...
2026-02-14 10:30:45.123,220.5,219.8,221.0,220.2,219.5,True,...
```

### Sending Commands

**Via Input Field:**
1. Type command in "Command:" field
2. Press Enter or click "Send"

**Quick Buttons:**
- **STATUS**: Display detailed system status
- **SPEEDS**: Show all motor speeds
- **HELP**: Display firmware command menu
- **Clear**: Clear console output

**Available Firmware Commands:**
- `TUNE <heater> <setpoint> <power>` - Auto-tune PID
- `PID <heater>` - Show PID values
- `SETPID <heater> <Kp> <Ki> <Kd>` - Set PID values
- `STATUS` - Detailed status of all heaters/motors
- `SPEED <motor> <steps/sec>` - Set motor speed
- `SPEEDS` - Show all motor speeds
- `MOTOR <motor>` - Select motor for encoder
- `RESET` - Reset all motor speeds to default

## Troubleshooting

### No Serial Ports Available

- **macOS/Linux**: Check permissions
  ```bash
  sudo usermod -a -G dialout $USER  # Linux
  # Log out and back in
  ```

- **All platforms**: Verify Arduino is connected via USB

### Connection Fails

- Check correct baud rate (115200)
- Close other serial monitors (Arduino IDE, PlatformIO)
- Try unplugging and reconnecting the Arduino
- Press "Refresh" to update port list

### Graph Not Updating

- Verify serial data is being received (check console)
- Ensure firmware is running and sending periodic status
- Check that status updates aren't paused (wait 15 seconds for auto-resume)

### High CPU Usage

- Reduce graph history duration (5 or 10 minutes)
- Disable motor speed polling if not needed

## Development

### Project Structure

```
monitor_app/
├── main.py              # Main application and UI
├── serial_handler.py    # Serial communication and parsing
├── graph_widget.py      # Real-time graphing widget
├── logger.py            # CSV data logging
├── requirements.txt     # Python dependencies
├── README.md           # This file
└── logs/               # Log files directory (created automatically)
```

### Extending the Application

**Adding New Data Types:**
1. Update regex patterns in `serial_handler.py`
2. Add data buffers and parsing logic
3. Create new graph traces in `graph_widget.py`
4. Add columns to CSV in `logger.py`

**Customizing Graphs:**
- Colors: Edit `HEATER_COLORS` and `MOTOR_COLORS` in `graph_widget.py`
- Update frequency: Modify timer interval in `GraphWidget.__init__()`
- Buffer size: Change `max_buffer_size` in `SerialReader.__init__()`

## License

This software is provided as-is for monitoring the PET Filament Extruder Controller.

## Support

For issues or questions related to:
- **Firmware**: Check `/firmware/` directory
- **Serial Protocol**: See firmware's `main.cpp` for output format
- **Python Application**: Check console output for error messages
