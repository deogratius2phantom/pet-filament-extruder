"""
Serial communication handler for PET Filament Extruder Monitor.
Handles serial port communication, data parsing, and buffering.
"""

import serial
import serial.tools.list_ports
import re
from datetime import datetime
from collections import deque
from PyQt5.QtCore import QThread, pyqtSignal
import time


class SerialReader(QThread):
    """Thread for reading serial data from the extruder controller."""
    
    # Signals for communicating with the main thread
    data_received = pyqtSignal(dict)  # Parsed heater/motor data
    raw_data_received = pyqtSignal(str)  # Raw serial line for console
    connection_error = pyqtSignal(str)  # Connection error messages
    connected = pyqtSignal()  # Connection established
    disconnected = pyqtSignal()  # Connection lost
    
    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        self.running = False
        
        # Data buffers - keep last 10 minutes of data (600 samples at 1Hz)
        self.max_buffer_size = 600
        self.heater_data = {
            'timestamps': deque(maxlen=self.max_buffer_size),
            'temperatures': [deque(maxlen=self.max_buffer_size) for _ in range(5)],
            'stable': [deque(maxlen=self.max_buffer_size) for _ in range(5)],
            'enabled': [deque(maxlen=self.max_buffer_size) for _ in range(5)],
        }
        
        self.motor_data = {
            'timestamps': deque(maxlen=self.max_buffer_size),
            'speeds': [deque(maxlen=self.max_buffer_size) for _ in range(5)],
            'running': [deque(maxlen=self.max_buffer_size) for _ in range(5)],
        }
        
        # Regex patterns for parsing
        self.heater_pattern = re.compile(
            r'Heater\s+(\d+):\s+([\d.]+)C\s+\|\s+Stable:\s+(\w+)\s+\|\s+Enabled:\s+(\w+)',
            re.IGNORECASE
        )
        self.motor_pattern = re.compile(
            r'Motor\s+(\d+):\s+(\d+)\s+steps/sec\s+\|\s+Running:\s+(\w+)',
            re.IGNORECASE
        )
        
        # Motor speed polling
        self.poll_motor_speeds = False
        self.last_motor_poll = 0
        self.motor_poll_interval = 5.0  # Poll every 5 seconds
    
    def run(self):
        """Main thread loop - reads and parses serial data."""
        self.running = True
        
        try:
            self.serial_port = serial.Serial(
                self.port,
                self.baudrate,
                timeout=1.0
            )
            time.sleep(2)  # Wait for Arduino reset
            self.connected.emit()
            
            while self.running:
                try:
                    if self.serial_port.in_waiting > 0:
                        line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                        
                        if line:
                            # Emit raw data for console display
                            self.raw_data_received.emit(line)
                            
                            # Parse the data
                            self.parse_line(line)
                    
                    # Poll motor speeds if enabled
                    if self.poll_motor_speeds and (time.time() - self.last_motor_poll) > self.motor_poll_interval:
                        self.send_command("SPEEDS")
                        self.last_motor_poll = time.time()
                    
                    time.sleep(0.01)  # Small delay to prevent CPU hogging
                    
                except serial.SerialException as e:
                    self.connection_error.emit(f"Serial error: {str(e)}")
                    break
                except Exception as e:
                    self.connection_error.emit(f"Error: {str(e)}")
                    
        except serial.SerialException as e:
            self.connection_error.emit(f"Failed to open port: {str(e)}")
        except Exception as e:
            self.connection_error.emit(f"Unexpected error: {str(e)}")
        finally:
            self.close()
            self.disconnected.emit()
    
    def parse_line(self, line):
        """Parse a line of serial data."""
        # Try to parse as heater data
        heater_match = self.heater_pattern.search(line)
        if heater_match:
            heater_idx = int(heater_match.group(1))
            temperature = float(heater_match.group(2))
            stable = heater_match.group(3).lower() == 'yes'
            enabled = heater_match.group(4).lower() == 'yes'
            
            if 0 <= heater_idx < 5:
                timestamp = datetime.now()
                
                # Update heater data buffers
                if not self.heater_data['timestamps'] or \
                   (timestamp - self.heater_data['timestamps'][-1]).total_seconds() > 0.5:
                    self.heater_data['timestamps'].append(timestamp)
                
                self.heater_data['temperatures'][heater_idx].append(temperature)
                self.heater_data['stable'][heater_idx].append(stable)
                self.heater_data['enabled'][heater_idx].append(enabled)
                
                # Emit parsed data
                self.data_received.emit({
                    'type': 'heater',
                    'index': heater_idx,
                    'temperature': temperature,
                    'stable': stable,
                    'enabled': enabled,
                    'timestamp': timestamp
                })
        
        # Try to parse as motor data (from SPEEDS or STATUS command)
        motor_match = self.motor_pattern.search(line)
        if motor_match:
            motor_idx = int(motor_match.group(1))
            speed = int(motor_match.group(2))
            running = motor_match.group(3).lower() == 'yes'
            
            if 0 <= motor_idx < 5:
                timestamp = datetime.now()
                
                # Update motor data buffers
                if not self.motor_data['timestamps'] or \
                   (timestamp - self.motor_data['timestamps'][-1]).total_seconds() > 0.5:
                    self.motor_data['timestamps'].append(timestamp)
                
                self.motor_data['speeds'][motor_idx].append(speed)
                self.motor_data['running'][motor_idx].append(running)
                
                # Emit parsed data
                self.data_received.emit({
                    'type': 'motor',
                    'index': motor_idx,
                    'speed': speed,
                    'running': running,
                    'timestamp': timestamp
                })
    
    def send_command(self, command):
        """Send a command to the serial port."""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(f"{command}\n".encode('utf-8'))
                self.serial_port.flush()
            except Exception as e:
                self.connection_error.emit(f"Failed to send command: {str(e)}")
    
    def set_motor_polling(self, enabled):
        """Enable or disable periodic motor speed polling."""
        self.poll_motor_speeds = enabled
        if enabled:
            self.last_motor_poll = 0  # Trigger immediate poll
    
    def set_buffer_size(self, minutes):
        """Set the buffer size in minutes (assumes 1 sample/second)."""
        self.max_buffer_size = minutes * 60
        # Recreate buffers with new size
        self.heater_data = {
            'timestamps': deque(list(self.heater_data['timestamps']), maxlen=self.max_buffer_size),
            'temperatures': [deque(list(d), maxlen=self.max_buffer_size) for d in self.heater_data['temperatures']],
            'stable': [deque(list(d), maxlen=self.max_buffer_size) for d in self.heater_data['stable']],
            'enabled': [deque(list(d), maxlen=self.max_buffer_size) for d in self.heater_data['enabled']],
        }
        self.motor_data = {
            'timestamps': deque(list(self.motor_data['timestamps']), maxlen=self.max_buffer_size),
            'speeds': [deque(list(d), maxlen=self.max_buffer_size) for d in self.motor_data['speeds']],
            'running': [deque(list(d), maxlen=self.max_buffer_size) for d in self.motor_data['running']],
        }
    
    def get_heater_data(self):
        """Get all buffered heater data."""
        return self.heater_data
    
    def get_motor_data(self):
        """Get all buffered motor data."""
        return self.motor_data
    
    def close(self):
        """Close the serial connection."""
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
    
    def stop(self):
        """Stop the thread."""
        self.running = False
        self.wait()


def list_serial_ports():
    """List all available serial ports."""
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]
