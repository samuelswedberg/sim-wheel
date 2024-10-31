import sys
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QLabel, QSlider, QVBoxLayout, QHBoxLayout, QGridLayout, 
    QWidget, QPushButton, QCheckBox, QComboBox, QFrame, QGroupBox
)
from PyQt6.QtCore import Qt


class DeviceSettingsWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Device Settings")
        self.setGeometry(100, 100, 900, 500)
        
        # Main layout
        main_layout = QVBoxLayout()
        main_widget = QWidget()
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        # Disconnected Label
        disconnected_label = QLabel("Disconnected")
        disconnected_label.setStyleSheet("color: red; font-weight: bold; font-size: 16px;")
        device_label = QLabel("Alpha Mini")
        device_label.setStyleSheet("font-weight: bold; font-size: 18px;")
        header_layout = QHBoxLayout()
        header_layout.addWidget(disconnected_label)
        header_layout.addWidget(device_label)
        header_layout.addStretch()

        # Steer Settings Section
        steer_group = QGroupBox("Steer Settings")
        steer_layout = QVBoxLayout()

        angle_label = QLabel("0°")
        angle_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        steer_layout.addWidget(angle_label)

        center_button = QPushButton("Center")
        steer_layout.addWidget(center_button)

        angle_slider = QSlider(Qt.Orientation.Horizontal)
        angle_slider.setRange(0, 1080)
        angle_slider.setValue(900)
        steer_layout.addWidget(angle_slider)
        steer_layout.addWidget(QLabel("Angle"))

        angle_preset_layout = QHBoxLayout()
        angle_preset_layout.addWidget(QLabel("Angle Preset"))
        manage_button = QPushButton("Manage")
        angle_preset_layout.addWidget(manage_button)
        steer_layout.addLayout(angle_preset_layout)

        hard_lock_slider = QSlider(Qt.Orientation.Horizontal)
        hard_lock_slider.setRange(0, 1080)
        hard_lock_slider.setValue(900)
        steer_layout.addWidget(hard_lock_slider)
        steer_layout.addWidget(QLabel("Hard Lock Angle"))

        limit_strength_layout = QHBoxLayout()
        limit_strength_layout.addWidget(QLabel("Limit Strength"))
        limit_strength_layout.addStretch()
        steer_layout.addLayout(limit_strength_layout)

        steer_group.setLayout(steer_layout)

        # Basic Settings Section
        basic_settings_group = QGroupBox("Basic Settings")
        basic_layout = QVBoxLayout()

        ffb_label = QHBoxLayout()
        ffb_label.addWidget(QLabel("Force Feedback"))
        reverse_ffb_checkbox = QCheckBox("Reverse FFB")
        ffb_label.addWidget(reverse_ffb_checkbox)
        ffb_value = QLabel("100%")
        ffb_label.addWidget(ffb_value)
        basic_layout.addLayout(ffb_label)

        ffb_slider = QSlider(Qt.Orientation.Horizontal)
        ffb_slider.setRange(0, 100)
        ffb_slider.setValue(100)
        basic_layout.addWidget(ffb_slider)

        smoothness_label = QLabel("Smoothness")
        basic_layout.addWidget(smoothness_label)

        smoothness_slider = QSlider(Qt.Orientation.Horizontal)
        smoothness_slider.setRange(0, 10)
        smoothness_slider.setValue(3)
        basic_layout.addWidget(smoothness_slider)

        basic_settings_group.setLayout(basic_layout)

        # Vehicle Section
        vehicle_group = QGroupBox("Vehicle")
        vehicle_layout = QVBoxLayout()

        wheel_rotation_label = QLabel("Wheel Rotation Speed")
        vehicle_layout.addWidget(wheel_rotation_label)
        
        wheel_rotation_slider = QSlider(Qt.Orientation.Horizontal)
        wheel_rotation_slider.setRange(0, 100)
        wheel_rotation_slider.setValue(25)
        vehicle_layout.addWidget(wheel_rotation_slider)

        feedback_detail_label = QLabel("Feedback Detail")
        vehicle_layout.addWidget(feedback_detail_label)

        feedback_detail_slider = QSlider(Qt.Orientation.Horizontal)
        feedback_detail_slider.setRange(0, 10)
        feedback_detail_slider.setValue(9)
        vehicle_layout.addWidget(feedback_detail_slider)

        vehicle_group.setLayout(vehicle_layout)

        # Mechanical Section
        mechanical_group = QGroupBox("Mechanical")
        mechanical_layout = QVBoxLayout()

        max_torque_layout = QHBoxLayout()
        max_torque_layout.addWidget(QLabel("Max Torque"))
        max_torque_value = QLabel("10.0Nm")
        max_torque_layout.addWidget(max_torque_value)
        mechanical_layout.addLayout(max_torque_layout)

        max_torque_slider = QSlider(Qt.Orientation.Horizontal)
        max_torque_slider.setRange(0, 100)
        max_torque_slider.setValue(100)
        mechanical_layout.addWidget(max_torque_slider)

        mechanical_layout.addWidget(QLabel("Mechanical Damper"))
        damper_slider = QSlider(Qt.Orientation.Horizontal)
        damper_slider.setRange(0, 100)
        damper_slider.setValue(30)
        mechanical_layout.addWidget(damper_slider)

        mechanical_layout.addWidget(QLabel("Mechanical Friction"))
        friction_slider = QSlider(Qt.Orientation.Horizontal)
        friction_slider.setRange(0, 100)
        friction_slider.setValue(40)
        mechanical_layout.addWidget(friction_slider)

        mechanical_layout.addWidget(QLabel("Mechanical Inertia"))
        inertia_slider = QSlider(Qt.Orientation.Horizontal)
        inertia_slider.setRange(0, 100)
        inertia_slider.setValue(0)
        mechanical_layout.addWidget(inertia_slider)

        mechanical_layout.addWidget(QLabel("Feedback Frequency"))
        frequency_slider = QSlider(Qt.Orientation.Horizontal)
        frequency_slider.setRange(1, 10)
        frequency_slider.setValue(1)
        mechanical_layout.addWidget(frequency_slider)

        mechanical_group.setLayout(mechanical_layout)

        # Preset Section
        preset_layout = QHBoxLayout()
        preset_layout.addWidget(QLabel("Preset"))
        preset_combo = QComboBox()
        preset_combo.addItems(["Game Filter", "Tag Filter", "Assetto Corsa"])
        preset_layout.addWidget(preset_combo)

        save_button = QPushButton("Save")
        save_as_button = QPushButton("Save As")
        import_button = QPushButton("Import")
        share_button = QPushButton("Share")

        preset_layout.addWidget(save_button)
        preset_layout.addWidget(save_as_button)
        preset_layout.addWidget(import_button)
        preset_layout.addWidget(share_button)

        # Add sections to main layout
        grid_layout = QGridLayout()
        grid_layout.addWidget(steer_group, 0, 0)
        grid_layout.addWidget(basic_settings_group, 0, 1)
        grid_layout.addWidget(vehicle_group, 1, 0)
        grid_layout.addWidget(mechanical_group, 1, 1)

        main_layout.addLayout(header_layout)
        main_layout.addLayout(grid_layout)
        main_layout.addLayout(preset_layout)


app = QApplication(sys.argv)
window = DeviceSettingsWindow()
window.show()
sys.exit(app.exec())
