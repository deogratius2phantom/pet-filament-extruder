#!/usr/bin/env python3
"""
PET Filament Extruder Monitor - Main Application
Real-time monitoring and data logging for the extruder controller.
"""

import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QComboBox, QLabel, 
                             QTextEdit, QLineEdit, QGroupBox, QStatusBar,
                             QSplitter, QFileDialog)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

from serial_handler import SerialReader, list_serial_ports
from graph_widget import GraphWidget
from logger import DataLogger


class MainWindow(QMainWindow):
    """Main application window."""
    
    def __init__(self):
        super().__init__()
        self.serial_reader = None
        self.data_logger = DataLogger()
        
        self.setWindowTitle("PET Filament Extruder Monitor")
        self.setGeometry(100, 100, 1400, 900)
        
        self.init_ui()
        self.init_connections()
        
        # Update status bar timer
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(1000)
    
    def init_ui(self):
        """Initialize the user interface."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Connection control panel
        connection_group = QGroupBox("Connection")
        connection_layout = QHBoxLayout()
        
        connection_layout.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.refresh_ports()
        connection_layout.addWidget(self.port_combo)
        
        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.refresh_ports)
        connection_layout.addWidget(self.refresh_button)
        
        connection_layout.addWidget(QLabel("Baud:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("115200")
        connection_layout.addWidget(self.baud_combo)
        
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        connection_layout.addWidget(self.connect_button)
        
        connection_layout.addStretch()
        connection_group.setLayout(connection_layout)
        main_layout.addWidget(connection_group)
        
        # Create splitter for graph and console
        splitter = QSplitter(Qt.Vertical)
        
        # Graph widget
        self.graph_widget = GraphWidget()
        splitter.addWidget(self.graph_widget)
        
        # Console and controls container
        bottom_widget = QWidget()
        bottom_layout = QVBoxLayout(bottom_widget)
        
        # Logging controls
        logging_group = QGroupBox("Data Logging")
        logging_layout = QHBoxLayout()
        
        self.log_button = QPushButton("Start Logging")
        self.log_button.clicked.connect(self.toggle_logging)
        logging_layout.addWidget(self.log_button)
        
        self.log_path_label = QLabel("Not logging")
        logging_layout.addWidget(self.log_path_label, stretch=1)
        
        self.log_size_label = QLabel("0.0 MB")
        logging_layout.addWidget(self.log_size_label)
        
        self.choose_log_dir_button = QPushButton("Choose Directory")
        self.choose_log_dir_button.clicked.connect(self.choose_log_directory)
        logging_layout.addWidget(self.choose_log_dir_button)
        
        logging_group.setLayout(logging_layout)
        bottom_layout.addWidget(logging_group)
        
        # Console output
        console_group = QGroupBox("Console Output")
        console_layout = QVBoxLayout()
        
        self.console_text = QTextEdit()
        self.console_text.setReadOnly(True)
        self.console_text.setMaximumHeight(200)
        font = QFont("Courier")
        font.setPointSize(9)
        self.console_text.setFont(font)
        console_layout.addWidget(self.console_text)
        
        # Command input
        command_layout = QHBoxLayout()
        command_layout.addWidget(QLabel("Command:"))
        self.command_input = QLineEdit()
        self.command_input.returnPressed.connect(self.send_command)
        command_layout.addWidget(self.command_input)
        
        self.send_button = QPushButton("Send")
        self.send_button.clicked.connect(self.send_command)
        command_layout.addWidget(self.send_button)
        
        # Quick command buttons
        quick_commands = [
            ("STATUS", "STATUS"),
            ("SPEEDS", "SPEEDS"),
            ("HELP", "HELP"),
            ("Clear", None)
        ]
        
        for label, cmd in quick_commands:
            btn = QPushButton(label)
            if cmd:
                btn.clicked.connect(lambda checked, c=cmd: self.send_quick_command(c))
            else:
                btn.clicked.connect(self.console_text.clear)
            command_layout.addWidget(btn)
        
        console_layout.addLayout(command_layout)
        console_group.setLayout(console_layout)
        bottom_layout.addWidget(console_group)
        
        # Heater control panel
        heater_control_group = QGroupBox("Heater Control")
        heater_control_layout = QHBoxLayout()
        
        self.heater_buttons = []
        for i in range(5):
            heater_widget = QWidget()
            heater_layout = QVBoxLayout(heater_widget)
            heater_layout.setSpacing(5)
            heater_layout.setContentsMargins(5, 5, 5, 5)
            
            label = QLabel(f"Heater {i}")
            label.setAlignment(Qt.AlignCenter)
            heater_layout.addWidget(label)
            
            btn_on = QPushButton("ON")
            btn_on.setCheckable(False)
            btn_on.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    border: none;
                    padding: 8px;
                    font-weight: bold;
                    border-radius: 4px;
                }
                QPushButton:hover {
                    background-color: #45a049;
                }
                QPushButton:pressed {
                    background-color: #3d8b40;
                }
            """)
            btn_on.clicked.connect(lambda checked, idx=i: self.enable_heater(idx))
            heater_layout.addWidget(btn_on)
            
            btn_off = QPushButton("OFF")
            btn_off.setCheckable(False)
            btn_off.setStyleSheet("""
                QPushButton {
                    background-color: #f44336;
                    color: white;
                    border: none;
                    padding: 8px;
                    font-weight: bold;
                    border-radius: 4px;
                }
                QPushButton:hover {
                    background-color: #da190b;
                }
                QPushButton:pressed {
                    background-color: #c41606;
                }
            """)
            btn_off.clicked.connect(lambda checked, idx=i: self.disable_heater(idx))
            heater_layout.addWidget(btn_off)
            
            self.heater_buttons.append((btn_on, btn_off))
            heater_control_layout.addWidget(heater_widget)
        
        # All heaters control
        all_heaters_widget = QWidget()
        all_heaters_layout = QVBoxLayout(all_heaters_widget)
        all_heaters_layout.setSpacing(5)
        all_heaters_layout.setContentsMargins(5, 5, 5, 5)
        
        all_label = QLabel("All Heaters")
        all_label.setAlignment(Qt.AlignCenter)
        all_heaters_layout.addWidget(all_label)
        
        btn_all_on = QPushButton("ALL ON")
        btn_all_on.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                padding: 8px;
                font-weight: bold;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        btn_all_on.clicked.connect(self.enable_all_heaters)
        all_heaters_layout.addWidget(btn_all_on)
        
        btn_all_off = QPushButton("ALL OFF")
        btn_all_off.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                border: none;
                padding: 8px;
                font-weight: bold;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #da190b;
            }
        """)
        btn_all_off.clicked.connect(self.disable_all_heaters)
        all_heaters_layout.addWidget(btn_all_off)
        
        heater_control_layout.addWidget(all_heaters_widget)
        heater_control_layout.addStretch()
        
        heater_control_group.setLayout(heater_control_layout)
        bottom_layout.addWidget(heater_control_group)
        
        splitter.addWidget(bottom_widget)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        
        main_layout.addWidget(splitter)
        
        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_label = QLabel("Disconnected")
        self.status_bar.addWidget(self.status_label)
        self.data_rate_label = QLabel("0 samples/sec")
        self.status_bar.addPermanentWidget(self.data_rate_label)
    
    def init_connections(self):
        """Initialize signal connections for data logger."""
        self.data_logger.logging_started.connect(self.on_logging_started)
        self.data_logger.logging_stopped.connect(self.on_logging_stopped)
        self.data_logger.file_rotated.connect(self.on_file_rotated)
        self.data_logger.error_occurred.connect(self.on_logger_error)
    
    def refresh_ports(self):
        """Refresh the list of available serial ports."""
        current_port = self.port_combo.currentText()
        self.port_combo.clear()
        ports = list_serial_ports()
        self.port_combo.addItems(ports)
        
        # Restore selection if port still exists
        if current_port in ports:
            self.port_combo.setCurrentText(current_port)
    
    def toggle_connection(self):
        """Connect or disconnect from the serial port."""
        if self.serial_reader is None or not self.serial_reader.isRunning():
            self.connect_serial()
        else:
            self.disconnect_serial()
    
    def connect_serial(self):
        """Establish serial connection."""
        port = self.port_combo.currentText()
        if not port:
            self.console_text.append("<b>Error:</b> No port selected\n")
            return
        
        baud = int(self.baud_combo.currentText())
        
        self.serial_reader = SerialReader(port, baud)
        self.serial_reader.data_received.connect(self.on_data_received)
        self.serial_reader.raw_data_received.connect(self.on_raw_data)
        self.serial_reader.connection_error.connect(self.on_connection_error)
        self.serial_reader.connected.connect(self.on_connected)
        self.serial_reader.disconnected.connect(self.on_disconnected)
        
        # Connect graph widget to serial handler
        self.graph_widget.serial_handler = self.serial_reader
        
        self.serial_reader.start()
        self.connect_button.setText("Disconnect")
        self.connect_button.setEnabled(False)  # Disable until connected
    
    def disconnect_serial(self):
        """Disconnect from serial port."""
        if self.serial_reader:
            self.serial_reader.stop()
            self.serial_reader.wait()
            self.serial_reader = None
        
        self.graph_widget.serial_handler = None
        self.connect_button.setText("Connect")
        self.status_label.setText("Disconnected")
    
    def send_command(self):
        """Send command from input field."""
        if not self.serial_reader or not self.serial_reader.isRunning():
            self.console_text.append("<b>Error:</b> Not connected\n")
            return
        
        command = self.command_input.text().strip()
        if command:
            self.serial_reader.send_command(command)
            self.console_text.append(f"<b>&gt; {command}</b>\n")
            self.command_input.clear()
    
    def send_quick_command(self, command):
        """Send a quick command button."""
        if not self.serial_reader or not self.serial_reader.isRunning():
            self.console_text.append("<b>Error:</b> Not connected\n")
            return
        
        self.serial_reader.send_command(command)
        self.console_text.append(f"<b>&gt; {command}</b>\n")
    
    def enable_heater(self, index):
        """Enable a specific heater."""
        command = f"ENABLE {index}"
        self.send_quick_command(command)
    
    def disable_heater(self, index):
        """Disable a specific heater."""
        command = f"DISABLE {index}"
        self.send_quick_command(command)
    
    def enable_all_heaters(self):
        """Enable all heaters."""
        self.send_quick_command("ENABLE ALL")
    
    def disable_all_heaters(self):
        """Disable all heaters."""
        self.send_quick_command("DISABLE ALL")
    
    def toggle_logging(self):
        """Start or stop data logging."""
        if self.data_logger.is_logging:
            self.data_logger.stop_logging()
        else:
            self.data_logger.start_logging()
    
    def choose_log_directory(self):
        """Choose a directory for log files."""
        directory = QFileDialog.getExistingDirectory(
            self, "Choose Log Directory", self.data_logger.log_directory
        )
        if directory:
            self.data_logger.log_directory = directory
            self.console_text.append(f"<b>Log directory:</b> {directory}\n")
    
    def on_data_received(self, data):
        """Handle parsed data from serial reader."""
        # Log the data if logging is enabled
        if self.data_logger.is_logging:
            self.data_logger.log_data(data)
            
            # Flush row after receiving heater data (complete update cycle)
            if data.get('type') == 'heater' and data.get('index') == 4:
                self.data_logger.flush_row()
    
    def on_raw_data(self, line):
        """Handle raw serial data for console display."""
        self.console_text.append(line)
        
        # Auto-scroll to bottom
        scrollbar = self.console_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
        
        # Limit console buffer
        if self.console_text.document().lineCount() > 1000:
            cursor = self.console_text.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.movePosition(cursor.Down, cursor.KeepAnchor, 100)
            cursor.removeSelectedText()
    
    def on_connection_error(self, error):
        """Handle connection errors."""
        self.console_text.append(f"<b style='color:red'>Error:</b> {error}\n")
    
    def on_connected(self):
        """Handle successful connection."""
        self.console_text.append("<b style='color:green'>Connected</b>\n")
        self.status_label.setText(f"Connected to {self.port_combo.currentText()}")
        self.connect_button.setText("Disconnect")
        self.connect_button.setEnabled(True)
    
    def on_disconnected(self):
        """Handle disconnection."""
        self.console_text.append("<b style='color:orange'>Disconnected</b>\n")
        self.status_label.setText("Disconnected")
        self.connect_button.setText("Connect")
        self.connect_button.setEnabled(True)
    
    def on_logging_started(self, filepath):
        """Handle logging started."""
        self.console_text.append(f"<b style='color:green'>Logging started:</b> {filepath}\n")
        self.log_button.setText("Stop Logging")
        self.log_path_label.setText(filepath)
    
    def on_logging_stopped(self):
        """Handle logging stopped."""
        self.console_text.append("<b style='color:orange'>Logging stopped</b>\n")
        self.log_button.setText("Start Logging")
        self.log_path_label.setText("Not logging")
        self.log_size_label.setText("0.0 MB")
    
    def on_file_rotated(self, filepath):
        """Handle log file rotation."""
        self.console_text.append(f"<b>Log file rotated:</b> {filepath}\n")
        self.log_path_label.setText(filepath)
    
    def on_logger_error(self, error):
        """Handle logger errors."""
        self.console_text.append(f"<b style='color:red'>Logger Error:</b> {error}\n")
    
    def update_status(self):
        """Update status bar information."""
        # Update log file size
        if self.data_logger.is_logging:
            size_mb = self.data_logger.get_current_file_size()
            self.log_size_label.setText(f"{size_mb:.2f} MB")
        
        # Update data rate (approximate based on buffer growth)
        if self.serial_reader and self.serial_reader.isRunning():
            heater_data = self.serial_reader.get_heater_data()
            if heater_data['timestamps']:
                # Rough estimate based on buffer size
                self.data_rate_label.setText("~1 sample/sec")
    
    def closeEvent(self, event):
        """Handle application close."""
        if self.serial_reader:
            self.disconnect_serial()
        if self.data_logger.is_logging:
            self.data_logger.stop_logging()
        event.accept()


def main():
    """Main application entry point."""
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Modern look
    
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
