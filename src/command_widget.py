#!/usr/bin/env python
import sys
# sys.path.append('../../arl_commons/src/arl_commons')
from arl_commons.pyarlarm import ArlArm, ArlMusculature, ArlMuscleState, ArlMuscleCommand, ArlMuscleControlMode, ArlArmException
import os
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import PyQt5.QtCore
import musclebar as mb
import csv



class CommandWidget(QWidget):
    COMMAND_TYPE_PRESSURE_ID = 0
    COMMAND_TYPE_ACTIVATION_ID = 1
    MIN_PRESSURE = 3500
    MAX_PRESSURE = 10000

    def __init__(self, parent):
        super(CommandWidget, self).__init__(parent)

        self.parent = parent
        
        try:
            self.ros_master_uri = os.environ['ROS_MASTER_URI']
            if self.ros_master_uri == 'http://192.168.102.200:11311' or self.ros_master_uri == 'http://ulr-robot:11311':
                self.robot_name = 'ulr-robot'
                self.num_sensors = 32
                self.num_columns = 4
            elif self.ros_master_uri == 'http://192.168.102.220:11311' or self.ros_master_uri == 'http://robot-controller:11311':
                self.robot_name = 'robot-controller'
                self.num_sensors = 6
                self.num_columns = 1
            else:
                self.robot_name = 'unknown'
                self.num_sensors = 32
                self.num_columns = 4
                raise ValueError
        except ValueError:
            print '[WARNING] ROS_MASTER_URI is unknown. Please confirm $ROS_MASTER_URI'
        except KeyError:
            print '[ERROR] ROS_MASTER_URI not found. Please confirm $ROS_MASTER_URI'

        self.arm = ArlArm()
        self.commands = ArlMusculature()
        self.init_widget_ui()
        
    def init_widget_ui(self):
        self.mbars = []
        self.create_command_widget()
        
    def create_command_widget(self):
        # create muscle bars
        for i in range(1, self.num_sensors+1):
            self.mbars.append(mb.MuscleBar(str(i), self, MIN_PRESSURE=self.MIN_PRESSURE, MAX_PRESSURE=self.MAX_PRESSURE))
            
        # buttons and shortcuts
        button_pub = QPushButton('Publish', self)
        shortcut_pub = QShortcut(QKeySequence("Ctrl+P"), self)
        button_pub.clicked.connect(self.pubclicked)
        shortcut_pub.activated.connect(self.pubclicked)
        button_pub.setToolTip("[Ctrl+P]")

        shortcut_emerstop = QShortcut(QKeySequence("Ctrl+C"), self)
        shortcut_emerstop.activated.connect(self.emerstoppressed)
        
        button_resetpos = QPushButton('Reset', self)
        button_resetpos.setToolTip('Reset robot position to initial state. All bar value are also reset. [Ctrl+R]')
        shortcut_reset = QShortcut(QKeySequence("Ctrl+R"), self)
        button_resetpos.clicked.connect(self.resetposclicked)
        shortcut_reset.activated.connect(self.resetposclicked)

        button_save = QPushButton("Save", self)
        button_save.clicked.connect(self.saveclicked)
        button_save.setToolTip("Save current pressures as .csv file. [Ctrl+S]")
        shortcut_save = QShortcut(QKeySequence("Ctrl+S"), self)
        shortcut_save.activated.connect(self.saveclicked)

        button_load = QPushButton("Load", self)
        button_load.clicked.connect(self.loadclicked)
        button_load.setToolTip("Load pressures from .csv file. [Ctrl+O]")
        shortcut_load = QShortcut(QKeySequence("Ctrl+O"), self)
        shortcut_load.activated.connect(self.loadclicked)
        
        button_help = QPushButton('Help', self)
        button_help.clicked.connect(self.helpclicked)
        button_help.setToolTip('[Ctrl+H]')
        shortcut_help = QShortcut(QKeySequence("Ctrl+H"), self)
        shortcut_help.activated.connect(self.helpclicked)

        # labels
        self.lbl_robot_name = QLabel('<font size="6" color="blue">' + self.robot_name + "</font><br>"+self.ros_master_uri, self)
        self.lbl_robot_name.setFrameStyle(QFrame.WinPanel | QFrame.Sunken)
        self.lbl_status = QLabel('<font size="5">Stand by...</font>', self)
        self.lbl_status.setFrameStyle(QFrame.WinPanel | QFrame.Sunken)

        # shortcuts for edit
        shortcuts_edit = []
        if self.robot_name == 'robot-controller':
            for i in range(len(self.mbars)):
                shortcut = QShortcut(QKeySequence("F{}".format(i+1)), self)
                shortcut.activated.connect(self.mbars[i].editclicked)
                shortcuts_edit.append(shortcut)
        else:
            for i in range(1, 11):
                shortcut = QShortcut(QKeySequence("F{}".format(i)), self)
                shortcut.activated.connect(self.mbars[i-1].editclicked)
                shortcuts_edit.append(shortcut)
            for i in range(11, 21):
                shortcut = QShortcut(QKeySequence("Ctrl+F{}".format(i-10)), self)
                shortcut.activated.connect(self.mbars[i-1].editclicked)
                shortcuts_edit.append(shortcut)
            for i in range(21, 31):
                shortcut = QShortcut(QKeySequence("Shift+F{}".format(i-20)), self)
                shortcut.activated.connect(self.mbars[i-1].editclicked)
                shortcuts_edit.append(shortcut)
            for i in range(31, len(self.mbars)+1):
                shortcut = QShortcut(QKeySequence("Ctrl+Shift+F{}".format(i-30)), self)
                shortcut.activated.connect(self.mbars[i-1].editclicked)
                shortcuts_edit.append(shortcut)

        # muscle bar layout
        biggerhbox = QHBoxLayout()
        
        if self.robot_name == 'robot-controller':
            bigvboxl = QVBoxLayout()
            for i in range(self.num_sensors):
                bigvboxl.addLayout(self.mbars[i].box)

            biggerhbox.addLayout(bigvboxl)
        else:
            bigvboxl = QVBoxLayout()
            for i in range(8):
                bigvboxl.addLayout(self.mbars[i].box)
    
            bigvboxc = QVBoxLayout()
            for i in range(8, 16):
                bigvboxc.addLayout(self.mbars[i].box)
    
            bigvboxr = QVBoxLayout()
            for i in range(16, 24):
                bigvboxr.addLayout(self.mbars[i].box)
    
            bigvboxr2 = QVBoxLayout()
            for i in range(24, len(self.mbars)):
                bigvboxr2.addLayout(self.mbars[i].box)
    
            biggerhbox.addLayout(bigvboxl)
            biggerhbox.addLayout(bigvboxc)
            biggerhbox.addLayout(bigvboxr)
            biggerhbox.addLayout(bigvboxr2)
            
        # button layout
        buttonbox = QVBoxLayout()
        buttonbox.addWidget(self.lbl_robot_name, stretch=2)
        buttonbox.addWidget(self.lbl_status, stretch=6)
        buttonbox.addWidget(button_pub, stretch=1)
        buttonbox.addWidget(button_resetpos, stretch=1)
        buttonbox.addWidget(button_save, stretch=1)
        buttonbox.addWidget(button_load, stretch=1)
        buttonbox.addWidget(button_help, stretch=1)
        biggerhbox.addLayout(buttonbox)

        biggerhbox.setSpacing(10)
        self.setLayout(biggerhbox)
        
    def pubclicked(self):
        self.pubonce()
        self.lbl_status.setText('<font size="5" color="red">Published</font>')

    # publish current pressures
    @PyQt5.QtCore.pyqtSlot()
    def pubonce(self):
        musculature = {}

        for muscle_idx in range(len(self.mbars)):
            muscle_comm = ArlMuscleCommand()
            muscle_comm.name = 'muscle_' + self.mbars[muscle_idx].name
            muscle_comm.pressure = self.convert_to_mpa(self.mbars[muscle_idx].pressure)
            muscle_comm.control_mode = ArlMuscleControlMode.BY_PRESSURE
            musculature[muscle_comm.name] = muscle_comm

        self.commands.muscle_commands = musculature
        self.arm.sendMusculatureCommand(self.commands)
    
    # publish 0 pressures
    @PyQt5.QtCore.pyqtSlot()
    def pubzero(self):
        musculature = {}
    
        for muscle_idx in range(len(self.mbars)):
            muscle_comm = ArlMuscleCommand()
            muscle_comm.name = 'muscle_' + self.mbars[muscle_idx].name
            muscle_comm.pressure = 0.0
            muscle_comm.control_mode = ArlMuscleControlMode.BY_PRESSURE
            musculature[muscle_comm.name] = muscle_comm
    
        self.commands.muscle_commands = musculature
        self.arm.sendMusculatureCommand(self.commands)

    def resetposclicked(self):
        for mbar in self.mbars:
            mbar.pressure = self.MIN_PRESSURE
            mbar.lcd.display(mbar.pressure)
            mbar.sld.setValue(mbar.pressure)
        self.lbl_status.setText('<font size="5">Stand by...</font>')
        self.pubzero()

    def emerstoppressed(self):
        pressures_temp = []
        for mbar in self.mbars:
            pressures_temp.append(mbar.pressure)
            mbar.pressure = self.MIN_PRESSURE
        self.lbl_status.setText("<h1>Stop</h1>")
        self.pubzero()
        for i,mbar in enumerate(self.mbars):
            mbar.pressure = pressures_temp[i]
        self.close()
        self.parent.close()

    def saveclicked(self):
        send_pressures = []
        for mbar in self.mbars:
            send_pressures.append(mbar.pressure)

        savefilename = QFileDialog.getSaveFileName(self, "Save as csv file", "", "csv Files (*.csv)")[0]
        if savefilename:
            if not QFileInfo(savefilename).suffix():
                savefilename += ".csv"
            with open(savefilename, 'w') as fop:
                writer = csv.writer(fop)
                writer.writerow(send_pressures)

    def loadclicked(self):
        loadfilename = QFileDialog.getOpenFileName(self, "Load csv file", "", "csv Files (*.csv)")[0]
        if loadfilename:
            with open(loadfilename, 'r') as fld:
                reader = csv.reader(fld)
                load_pressures_str = next(reader)

            load_pressures = [int(load_pressure) for load_pressure in load_pressures_str]

            for i, mbar in enumerate(self.mbars):
                mbar.pressure = load_pressures[i]
                mbar.lcd.display(mbar.pressure)
                mbar.sld.setValue(mbar.pressure)
                
                
    def helpclicked(self):
        help_window = HelpWindow()
        help_window.show()

    def convert_to_mpa(self, value):
        if value < 3500 or value > 10000:
            raise ValueError(value)
        mpa = ((value * 10.0 / 32768.0) - 1.0) / 4.0
        return mpa
        

class HelpWindow(QWidget):
    def __init__ (self, parent=None):
        self.w = QDialog(parent)
        self.w.setWindowTitle('Help')

        help_content = '''
                        <h1>Keyboard Shortcuts</h1>
                        <p><font size="5"><b>Commands</b></font><br>
                        <font size="4">
                        [Ctrl+P]: Publish<br>
                        [Crtl+R]: Reset<br>
                        [Ctrl+S]: Save<br>
                        [Ctrl+O]: Load<br>
                        [Ctrl+H]: Help<br><br></font>
                        <font size="5"><b>Edit muscles</b></font><br>
                        <font size="4">
                        [F1~10]: Muscle_1~10<br>
                        [Ctrl+F1~10]: Muscle_11~20<br>
                        [Shift+F1~10]: Muscle_21~30<br>
                        [Ctrl+Shift+F1~2]: Muscle_31~32<br>
                        </font></p>
                        '''
        lbl_help = QLabel(help_content)
        layout = QHBoxLayout()
        layout.addWidget(lbl_help)
        self.w.setLayout(layout)
        
        
    def show(self):
        self.w.exec_()
