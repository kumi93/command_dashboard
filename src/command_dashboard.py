#!/usr/bin/env python
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QHBoxLayout, QVBoxLayout, QSlider, QLCDNumber, QMainWindow
from PyQt5.QtCore import Qt
import command_widget as cw

class UlrGui(QMainWindow):
    def __init__(self, parent=None):
        super(UlrGui,self).__init__(parent)

        self.alive = True
        self.title = "Command Dashboard"
        self.left = 30
        self.top = 30
        self.width = 1200
        self.height = 600

        self._initUI()

    def closeevent(self, event):
        self.alive = False

    def _initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        self.setCentralWidget(cw.CommandWidget(self))

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ulrgui = UlrGui()
    ulrgui.show()
    sys.exit(app.exec_())
