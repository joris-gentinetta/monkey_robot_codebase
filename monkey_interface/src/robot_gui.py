#!/usr/bin/env python3
'''
Author: Joris Gentinetta
E-Mail: jorisg@ethz.ch
Last changed: 27.2.24
'''

import sys
import threading

from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout,
                             QWidget, QTextEdit, QLineEdit, QLabel, QComboBox, QHBoxLayout)

from monkey_interface import MoveGroupInterface, Utils


# Create self.helper class
class RobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.helper = Utils(self)

        self.add_waypoint = False
        self.save_waypoints = False
        self.cont_waypoints = False

        self.initUI()

    def initUI(self):
        self.setWindowTitle('Robot Control GUI')
        self.setGeometry(100, 100, 600, 400)
        self.layout = QVBoxLayout()

        self.optionLabel = QLabel('Select an action:', self)
        self.optionCombo = QComboBox(self)
        self.optionCombo.addItem('Collect waypoints')
        self.optionCombo.addItem('Load and edit waypoints')
        self.optionCombo.addItem('Plan/Execute')
        self.optionCombo.currentIndexChanged.connect(self.updateUIBasedOnSelection)

        self.fileNameLabel = QLabel('File name:', self)
        self.fileNameEdit = QLineEdit(self)

        self.fileWidget = QWidget()
        self.fileNameLayout = QHBoxLayout(self.fileWidget)
        self.fileNameLayout.addWidget(self.fileNameLabel)
        self.fileNameLayout.addWidget(self.fileNameEdit)
        self.fileWidget.setVisible(False)

        self.confirmBtn = QPushButton('Confirm', self)
        self.confirmBtn.clicked.connect(self.confirmAction)

        self.outputText = QTextEdit(self)
        self.outputText.setReadOnly(True)

        self.layout.addWidget(self.optionLabel)
        self.layout.addWidget(self.optionCombo)
        self.layout.addWidget(self.fileWidget)
        self.layout.addWidget(self.confirmBtn)
        self.layout.addWidget(self.outputText)

        self.mainWidget = QWidget()
        self.mainWidget.setLayout(self.layout)
        self.setCentralWidget(self.mainWidget)

    def updateUIBasedOnSelection(self):
        if self.optionCombo.currentText() == 'Load and edit waypoints':
            self.fileWidget.setVisible(True)
        else:
            self.fileWidget.hide()

    def clear_layout(self):
        while self.layout.count():
            item = self.layout.takeAt(0)
            if item.widget():
                widget = item.widget()
                self.layout.removeWidget(widget)
                widget.setParent(None)
    def setupCollectWaypointsUI(self):
        self.clear_layout()
        self.fileWidget.setVisible(True)

        self.infoLabel = QLabel('Move the robot to desired positions and press "Add Waypoint"', self)
        self.addWaypointBtn = QPushButton('Add Waypoint', self)
        self.addWaypointBtn.setEnabled(False)  # Disable the button initially
        self.addWaypointBtn.clicked.connect(self.addWaypoint)

        self.saveBtn = QPushButton('Save Waypoints', self)
        self.saveBtn.setEnabled(False)  # Disable the button initially
        self.contBtn = QPushButton('Continue Collecting Waypoints', self)
        self.contBtn.setEnabled(False)  # Disable the button initially

        self.saveBtn.clicked.connect(self.saveWaypoints)
        self.contBtn.clicked.connect(self.contWaypoints)

        self.layout.addWidget(self.infoLabel)
        self.layout.addWidget(self.addWaypointBtn)
        self.layout.addWidget(self.contBtn)
        self.layout.addWidget(self.fileWidget)
        self.layout.addWidget(self.saveBtn)
        self.layout.addWidget(self.outputText)

    def setupPlanningUI(self):

        self.clear_layout()
        self.fileWidget.setVisible(True)

        self.infoLabel = QLabel('Planning UI', self)

        self.loadWPBtn = QPushButton('Load Waypoints', self)
        self.loadPlanBtn = QPushButton('Load Plan', self)

        self.planBtn = QPushButton('Plan Carthesian Path', self)
        self.planBtn.setEnabled(False)

        self.saveBtn = QPushButton('Save Plan', self)
        self.saveBtn.setEnabled(False)  # Disable the button initially

        self.interpLabel = QLabel('Interpolation Steps: ', self)
        self.interpEdit = QLineEdit(self)
        self.interpEdit.setText('10')
        self.interpWidget = QWidget()
        self.interpLayout = QHBoxLayout(self.interpWidget)
        self.interpLayout.addWidget(self.interpLabel)
        self.interpLayout.addWidget(self.interpEdit)
        # self.interpWidget.setVisible(False)
        self.executeBtn = QPushButton('Execute Plan', self)
        self.executeBtn.setEnabled(False)  # Disable the button initially

        self.startBtn = QPushButton('Start', self)
        self.startBtn.setEnabled(False)  # Disable the button initially
        self.start = False

        self.loadWPBtn.clicked.connect(self.loadWP)
        self.loadPlanBtn.clicked.connect(self.loadPlan)
        self.planBtn.clicked.connect(self.plan)
        self.saveBtn.clicked.connect(self.savePlan)
        self.executeBtn.clicked.connect(self.executePlan)
        self.startBtn.clicked.connect(self.startMovement)

        self.layout.addWidget(self.fileWidget)
        self.layout.addWidget(self.loadWPBtn)
        self.layout.addWidget(self.loadPlanBtn)
        self.layout.addWidget(self.planBtn)
        self.layout.addWidget(self.saveBtn)
        self.layout.addWidget(self.interpWidget)
        self.layout.addWidget(self.executeBtn)
        self.layout.addWidget(self.startBtn)
        self.layout.addWidget(self.outputText)

    def restoreMainUI(self):
        self.clear_layout()
        self.fileWidget.setVisible(False)
        # self.layout.addWidget(self.infoLabel)
        self.layout.addWidget(self.optionLabel)
        self.layout.addWidget(self.optionCombo)
        self.layout.addWidget(self.fileWidget)
        self.layout.addWidget(self.confirmBtn)
        self.layout.addWidget(self.outputText)

    def addWaypoint(self):
        self.add_waypoint = True

    def saveWaypoints(self):
        if self.fileNameEdit.text() == '':
            self.outputText.append('Please enter a file name')
        else:
            self.save_waypoints = True
            self.restoreMainUI()

    def contWaypoints(self):
        self.cont_waypoints = True

    def loadWP(self):
        self.planBtn.setEnabled(False)
        self.executeBtn.setEnabled(False)
        self.saveBtn.setEnabled(False)
        self.startBtn.setEnabled(False)
        fileName = self.fileNameEdit.text()
        if self.helper.loadWaypointsFromJSON(fileName):
            self.planBtn.setEnabled(True)
            self.helper.initial_wp_count = len(self.helper.ifaceLA.loaded_json_wpoints.poses) + 1

    def loadPlan(self):
        fileName = self.fileNameEdit.text()
        self.planBtn.setEnabled(False)
        if self.helper.load_plan(fileName):
            self.executeBtn.setEnabled(True)

    def plan(self):
        self.helper.plan = self.helper.CPP(self.helper.ifaceLA.loaded_json_wpoints,
                                           self.helper.ifaceRA.loaded_json_wpoints, None)
        if self.helper.plan:
            self.saveBtn.setEnabled(True)
            self.outputText.append("Planning successful.")
            # self.executeBtn.setEnabled(True)
        else:
            self.saveBtn.setEnabled(False)
            self.outputText.append("Planning failed. Try again.")

            # self.executeBtn.setEnabled(False)

    def savePlan(self):
        if self.fileNameEdit.text() == '':
            self.outputText.append('Please enter a file name')
        else:
            self.helper.save_plan(self.helper.plan, self.fileNameEdit.text())
            self.executeBtn.setEnabled(True)
            self.outputText.append("Plan saved successfully. Ready to execute.")

    def executePlan(self):
        thread = threading.Thread(target=self.helper.interact_with_monkey_listener)
        thread.start()

    def startMovement(self):
        self.start = True
        self.startBtn.setEnabled(False)

    def confirmAction(self):
        # Get the selected action and file name
        action = self.optionCombo.currentText()
        fileName = self.fileNameEdit.text()
        if action == 'Collect waypoints':
            self.helper.initial_wp_count = 1
            self.setupCollectWaypointsUI()

            thread = threading.Thread(target=self.helper.collect_wpoints)
            thread.start()


        elif action == 'Load and edit waypoints':
            if self.helper.loadWaypointsFromJSON(fileName):
                self.helper.initial_wp_count = len(self.helper.ifaceLA.loaded_json_wpoints.poses) + 1
                self.setupCollectWaypointsUI()
                thread = threading.Thread(target=self.helper.collect_wpoints)
                thread.start()

        elif action == 'Plan/Execute':
            self.setupPlanningUI()
            # Load a saved plan and execute it


if __name__ == '__main__':
    # Create interface to Moveit:RobotCommander (move-group python interface)

    app = QApplication(sys.argv)
    gui = RobotGUI()

    gui.show()
    sys.exit(app.exec_())
