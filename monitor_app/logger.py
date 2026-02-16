"""
CSV data logger for PET Filament Extruder Monitor.
Logs temperature and motor data to CSV files with automatic rotation.
"""

import csv
import os
from datetime import datetime
from PyQt5.QtCore import QObject, pyqtSignal


class DataLogger(QObject):
    """CSV data logger with automatic file rotation."""
    
    # Signals
    logging_started = pyqtSignal(str)  # Emits log file path
    logging_stopped = pyqtSignal()
    file_rotated = pyqtSignal(str)  # Emits new file path
    error_occurred = pyqtSignal(str)
    
    def __init__(self, log_directory="logs"):
        super().__init__()
        self.log_directory = log_directory
        self.log_file = None
        self.csv_writer = None
        self.is_logging = False
        self.current_file_path = None
        self.rows_written = 0
        
        # Rotation settings
        self.max_file_size_mb = 100
        self.rotate_daily = True
        self.current_date = None
        
        # CSV columns
        self.columns = [
            'timestamp',
            'temp_0', 'temp_1', 'temp_2', 'temp_3', 'temp_4',
            'stable_0', 'stable_1', 'stable_2', 'stable_3', 'stable_4',
            'enabled_0', 'enabled_1', 'enabled_2', 'enabled_3', 'enabled_4',
            'speed_0', 'speed_1', 'speed_2', 'speed_3', 'speed_4',
            'running_0', 'running_1', 'running_2', 'running_3', 'running_4'
        ]
        
        # Data buffer
        self.current_row = {col: '' for col in self.columns}
    
    def start_logging(self, custom_filename=None):
        """Start logging data to a CSV file."""
        if self.is_logging:
            self.error_occurred.emit("Already logging")
            return False
        
        # Create log directory if it doesn't exist
        try:
            os.makedirs(self.log_directory, exist_ok=True)
        except Exception as e:
            self.error_occurred.emit(f"Failed to create log directory: {str(e)}")
            return False
        
        # Generate filename
        if custom_filename:
            filename = custom_filename
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"extruder_log_{timestamp}.csv"
        
        self.current_file_path = os.path.join(self.log_directory, filename)
        self.current_date = datetime.now().date()
        
        try:
            self.log_file = open(self.current_file_path, 'w', newline='')
            self.csv_writer = csv.DictWriter(self.log_file, fieldnames=self.columns)
            self.csv_writer.writeheader()
            self.log_file.flush()
            self.is_logging = True
            self.rows_written = 0
            self.logging_started.emit(self.current_file_path)
            return True
        except Exception as e:
            self.error_occurred.emit(f"Failed to create log file: {str(e)}")
            return False
    
    def stop_logging(self):
        """Stop logging data."""
        if not self.is_logging:
            return
        
        try:
            if self.log_file:
                self.log_file.close()
                self.log_file = None
                self.csv_writer = None
            self.is_logging = False
            self.logging_stopped.emit()
        except Exception as e:
            self.error_occurred.emit(f"Error closing log file: {str(e)}")
    
    def log_data(self, data_dict):
        """
        Log a data point. Expected data_dict format:
        {
            'type': 'heater' or 'motor',
            'index': 0-4,
            'timestamp': datetime object,
            ... type-specific fields
        }
        """
        if not self.is_logging:
            return
        
        # Check if we need to rotate the file
        if self._should_rotate():
            self._rotate_file()
        
        # Update current row with new data
        timestamp = data_dict.get('timestamp', datetime.now())
        self.current_row['timestamp'] = timestamp.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        
        data_type = data_dict.get('type')
        index = data_dict.get('index')
        
        if data_type == 'heater' and 0 <= index < 5:
            self.current_row[f'temp_{index}'] = f"{data_dict.get('temperature', ''):.2f}"
            self.current_row[f'stable_{index}'] = str(data_dict.get('stable', ''))
            self.current_row[f'enabled_{index}'] = str(data_dict.get('enabled', ''))
        
        elif data_type == 'motor' and 0 <= index < 5:
            self.current_row[f'speed_{index}'] = str(data_dict.get('speed', ''))
            self.current_row[f'running_{index}'] = str(data_dict.get('running', ''))
    
    def flush_row(self):
        """Write the current row to the CSV file."""
        if not self.is_logging or not self.csv_writer:
            return
        
        try:
            self.csv_writer.writerow(self.current_row)
            self.log_file.flush()
            self.rows_written += 1
            
            # Clear data that was written (but keep timestamp)
            for col in self.columns:
                if col != 'timestamp':
                    self.current_row[col] = ''
        except Exception as e:
            self.error_occurred.emit(f"Error writing to log file: {str(e)}")
    
    def _should_rotate(self):
        """Check if the log file should be rotated."""
        if not self.current_file_path or not os.path.exists(self.current_file_path):
            return False
        
        # Check file size
        file_size_mb = os.path.getsize(self.current_file_path) / (1024 * 1024)
        if file_size_mb >= self.max_file_size_mb:
            return True
        
        # Check date change
        if self.rotate_daily and self.current_date != datetime.now().date():
            return True
        
        return False
    
    def _rotate_file(self):
        """Rotate to a new log file."""
        if not self.is_logging:
            return
        
        # Close current file
        if self.log_file:
            self.log_file.close()
        
        # Create new file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"extruder_log_{timestamp}.csv"
        self.current_file_path = os.path.join(self.log_directory, filename)
        self.current_date = datetime.now().date()
        
        try:
            self.log_file = open(self.current_file_path, 'w', newline='')
            self.csv_writer = csv.DictWriter(self.log_file, fieldnames=self.columns)
            self.csv_writer.writeheader()
            self.log_file.flush()
            self.rows_written = 0
            self.file_rotated.emit(self.current_file_path)
        except Exception as e:
            self.error_occurred.emit(f"Failed to rotate log file: {str(e)}")
            self.is_logging = False
    
    def get_current_file_path(self):
        """Get the current log file path."""
        return self.current_file_path
    
    def get_current_file_size(self):
        """Get the current log file size in MB."""
        if self.current_file_path and os.path.exists(self.current_file_path):
            return os.path.getsize(self.current_file_path) / (1024 * 1024)
        return 0.0
    
    def get_rows_written(self):
        """Get the number of rows written to the current file."""
        return self.rows_written
    
    def set_rotation_settings(self, max_size_mb=None, rotate_daily=None):
        """Update rotation settings."""
        if max_size_mb is not None:
            self.max_file_size_mb = max_size_mb
        if rotate_daily is not None:
            self.rotate_daily = rotate_daily
