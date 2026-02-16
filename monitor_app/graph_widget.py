"""
Real-time graphing widget for PET Filament Extruder Monitor.
Displays temperature and motor speed data using pyqtgraph.
"""

import pyqtgraph as pg
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QCheckBox, QLabel, QComboBox
from PyQt5.QtCore import QTimer
from datetime import datetime, timedelta
import numpy as np


class GraphWidget(QWidget):
    """Widget for displaying real-time temperature and motor speed graphs."""
    
    # Color scheme for 5 heaters
    HEATER_COLORS = [
        (255, 0, 0),      # Red - Heater 0
        (0, 255, 0),      # Green - Heater 1
        (0, 0, 255),      # Blue - Heater 2
        (255, 255, 0),    # Yellow - Heater 3
        (255, 0, 255),    # Magenta - Heater 4
    ]
    
    MOTOR_COLORS = [
        (255, 100, 100),  # Light Red - Motor 0
        (100, 255, 100),  # Light Green - Motor 1
        (100, 100, 255),  # Light Blue - Motor 2
        (255, 255, 100),  # Light Yellow - Motor 3
        (255, 100, 255),  # Light Magenta - Motor 4
    ]
    
    def __init__(self, serial_handler=None):
        super().__init__()
        self.serial_handler = serial_handler
        
        # Graph settings
        self.history_minutes = 10
        self.show_motors = False
        
        # Initialize UI
        self.init_ui()
        
        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_graphs)
        self.update_timer.start(1000)  # Update every second
    
    def init_ui(self):
        """Initialize the user interface."""
        layout = QVBoxLayout()
        
        # Control panel
        control_layout = QHBoxLayout()
        
        # History duration selector
        control_layout.addWidget(QLabel("History:"))
        self.history_combo = QComboBox()
        self.history_combo.addItems(["5 min", "10 min", "30 min", "60 min"])
        self.history_combo.setCurrentText("10 min")
        self.history_combo.currentTextChanged.connect(self.on_history_changed)
        control_layout.addWidget(self.history_combo)
        
        # Motor graph toggle
        self.motor_checkbox = QCheckBox("Show Motor Speeds")
        self.motor_checkbox.stateChanged.connect(self.on_motor_toggle)
        control_layout.addWidget(self.motor_checkbox)
        
        control_layout.addStretch()
        
        # Legend checkboxes for heaters
        self.heater_checkboxes = []
        for i in range(5):
            cb = QCheckBox(f"H{i}")
            cb.setChecked(True)
            cb.setStyleSheet(f"color: rgb{self.HEATER_COLORS[i]};")
            cb.stateChanged.connect(lambda state, idx=i: self.toggle_heater(idx, state))
            control_layout.addWidget(cb)
            self.heater_checkboxes.append(cb)
        
        layout.addLayout(control_layout)
        
        # Temperature graph
        self.temp_plot = pg.PlotWidget(title="Heater Temperatures")
        self.temp_plot.setLabel('left', 'Temperature', units='°C')
        self.temp_plot.setLabel('bottom', 'Time', units='s')
        self.temp_plot.showGrid(x=True, y=True, alpha=0.3)
        self.temp_plot.addLegend()
        
        # Create temperature curves
        self.temp_curves = []
        for i in range(5):
            curve = self.temp_plot.plot(
                pen=pg.mkPen(color=self.HEATER_COLORS[i], width=2),
                name=f"Heater {i}"
            )
            self.temp_curves.append(curve)
        
        layout.addWidget(self.temp_plot, stretch=2)
        
        # Motor speed graph (initially hidden)
        self.motor_plot = pg.PlotWidget(title="Motor Speeds")
        self.motor_plot.setLabel('left', 'Speed', units='steps/sec')
        self.motor_plot.setLabel('bottom', 'Time', units='s')
        self.motor_plot.showGrid(x=True, y=True, alpha=0.3)
        self.motor_plot.addLegend()
        self.motor_plot.hide()
        
        # Create motor speed curves
        self.motor_curves = []
        for i in range(5):
            curve = self.motor_plot.plot(
                pen=pg.mkPen(color=self.MOTOR_COLORS[i], width=2),
                name=f"Motor {i}"
            )
            self.motor_curves.append(curve)
        
        layout.addWidget(self.motor_plot, stretch=1)
        
        self.setLayout(layout)
    
    def on_history_changed(self, text):
        """Handle history duration change."""
        minutes = int(text.split()[0])
        self.history_minutes = minutes
        if self.serial_handler:
            self.serial_handler.set_buffer_size(minutes)
    
    def on_motor_toggle(self, state):
        """Toggle motor speed graph visibility."""
        self.show_motors = (state == 2)  # Qt.Checked = 2
        if self.show_motors:
            self.motor_plot.show()
            if self.serial_handler:
                self.serial_handler.set_motor_polling(True)
        else:
            self.motor_plot.hide()
            if self.serial_handler:
                self.serial_handler.set_motor_polling(False)
    
    def toggle_heater(self, index, state):
        """Toggle visibility of a heater trace."""
        visible = (state == 2)
        if visible:
            self.temp_curves[index].show()
        else:
            self.temp_curves[index].hide()
    
    def update_graphs(self):
        """Update graph data from serial handler buffers."""
        if not self.serial_handler:
            return
        
        # Update temperature graph
        heater_data = self.serial_handler.get_heater_data()
        if heater_data['timestamps']:
            # Calculate time axis (seconds from start)
            timestamps = list(heater_data['timestamps'])
            if timestamps:
                start_time = timestamps[0]
                time_axis = [(t - start_time).total_seconds() for t in timestamps]
                
                # Update each heater curve
                for i in range(5):
                    temps = list(heater_data['temperatures'][i])
                    if temps and len(temps) == len(time_axis):
                        self.temp_curves[i].setData(time_axis, temps)
                
                # Auto-scale to show recent data
                if time_axis:
                    max_time = time_axis[-1]
                    min_time = max(0, max_time - (self.history_minutes * 60))
                    self.temp_plot.setXRange(min_time, max_time)
        
        # Update motor speed graph if visible
        if self.show_motors:
            motor_data = self.serial_handler.get_motor_data()
            if motor_data['timestamps']:
                timestamps = list(motor_data['timestamps'])
                if timestamps:
                    start_time = timestamps[0]
                    time_axis = [(t - start_time).total_seconds() for t in timestamps]
                    
                    # Update each motor curve
                    for i in range(5):
                        speeds = list(motor_data['speeds'][i])
                        if speeds and len(speeds) == len(time_axis):
                            self.motor_curves[i].setData(time_axis, speeds)
                    
                    # Auto-scale to show recent data
                    if time_axis:
                        max_time = time_axis[-1]
                        min_time = max(0, max_time - (self.history_minutes * 60))
                        self.motor_plot.setXRange(min_time, max_time)
    
    def clear_graphs(self):
        """Clear all graph data."""
        for curve in self.temp_curves:
            curve.setData([], [])
        for curve in self.motor_curves:
            curve.setData([], [])
