import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QSlider
from PyQt6.uic import loadUi

class SliderApp(QMainWindow):
    def __init__(self):
        super().__init__()

        # Load the UI file
        loadUi("wheelbase.ui", self)

        # Access the QSlider and QLabel from the UI
        self.slider = self.findChild(QSlider, "ffbStrengthSlider")  # Replace with your slider's object name
        self.label = self.findChild(QLabel, "ffbPercent")  # Replace with your label's object name

        # Connect slider's signal to a method
        self.slider.valueChanged.connect(self.update_label)

    def update_label(self):
        # Get the value of the slider
        value = self.slider.value()
        # Update the QLabel with the slider's value
        self.label.setText(f"{value}%")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SliderApp()
    window.show()
    sys.exit(app.exec())
