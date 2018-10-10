#!/usr/bin/env python

from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QHBoxLayout, QVBoxLayout, QSlider, QLCDNumber, QMainWindow, QInputDialog
from PyQt5.QtCore import Qt

class MuscleBar:
    def __init__(self, name, window, MIN_PRESSURE=3500, MAX_PRESSURE=10000):
        self.name = name
        self.min_pressure = MIN_PRESSURE
        self.max_pressure = MAX_PRESSURE
        self.pressure = self.min_pressure
        self.pressure_mpa = self.convert_to_mpa(self.pressure)
        self.makebar(window)

    def makebar(self,  window):
        self.window = window
        self.lcd = QLCDNumber(window)
        self.lcd.display(self.pressure)
        self.lcd.setSegmentStyle(QLCDNumber.Flat)
        self.sld = QSlider(Qt.Horizontal, window)
        self.sld.setFocusPolicy(Qt.NoFocus)
        self.sld.setRange(self.min_pressure, self.max_pressure)
        self.sld.setValue(self.pressure)
        self.sld.setTickPosition(QSlider.TicksBelow)
        self.sld.setTickInterval(500)
        self.lbl = QLabel('Muscle_' + self.name + ':  {:.4f} [MPa]'.format(self.pressure_mpa), window)
        self.button_edit = QPushButton("Edit", window)

        self.sld.valueChanged[int].connect(self.sldvaluechanged)
        self.button_edit.clicked.connect(self.editclicked)

        self.box = QHBoxLayout()
        vboxl = QVBoxLayout()
        vboxl.addWidget(self.lcd)
        vboxl.addWidget(self.button_edit)
        vbox = QVBoxLayout()
        vbox.addWidget(self.lbl)
        vbox.addWidget(self.sld)
        self.box.addLayout(vboxl)
        self.box.addLayout(vbox)

    def sldvaluechanged(self, value):
        self.pressure = value
        self.pressure_mpa = self.convert_to_mpa(self.pressure)
        self.lcd.display(self.pressure)
        self.lbl.setText('Muscle_' + self.name + ':  {:.4f} [MPa]'.format(self.pressure_mpa))

    def editclicked(self):
        value, ok_pressed = QInputDialog.getInt(self.window,
                                                "Edit Pressure",
                                                "Muscle_" + self.name + ":",
                                                self.pressure,
                                                self.min_pressure,
                                                self.max_pressure,
                                                500)
        if ok_pressed:
            self.pressure = value
            self.pressure_mpa = self.convert_to_mpa(self.pressure)
            self.lcd.display(self.pressure)
            self.sld.setValue(self.pressure)
            self.lbl.setText('Muscle_' + self.name + ':  {:.4f} [MPa]'.format(self.pressure_mpa))
            
    def convert_to_mpa(self, value):
        if value < 3500 or value > 10000:
            raise ValueError(value)
        mpa = ((value * 10.0 / 32768.0) - 1.0) / 4.0
        return mpa
