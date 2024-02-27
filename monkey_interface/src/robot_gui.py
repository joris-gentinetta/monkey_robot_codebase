#!/usr/bin/env python3

import sys
import time
import threading

from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout,
                             QWidget, QTextEdit, QLineEdit, QLabel, QComboBox, QHBoxLayout)
from monkey_interface import MoveGroupInterface, Utils



# Create self.helper class
class RobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        self.interface_left_arm = MoveGroupInterface('monkey_left_arm')
        self.interface_right_arm = MoveGroupInterface('monkey_right_arm')
        self.interface_head = MoveGroupInterface('head')
        self.helper = Utils(self.interface_left_arm, self.interface_right_arm, self.interface_head, self)

        self.add_waypoint = False
        self.save_waypoints = False
        self.cont_waypoints = False

        self.initUI()

    def initUI(self):
        self.setWindowTitle('Robot Control GUI')
        self.setGeometry(100, 100, 600, 400)

        self.layout = QVBoxLayout()

        # Option Selection
        self.optionLabel = QLabel('Select an action:', self)
        self.optionCombo = QComboBox(self)
        self.optionCombo.addItem('Collect waypoints')
        self.optionCombo.addItem('Load and edit waypoints')
        self.optionCombo.addItem('Plan/Execute')

        # File Name Input setup with QHBoxLayout
        self.fileNameLayout = QHBoxLayout()  # Horizontal layout for label and edit
        self.fileNameLabel = QLabel('File name:', self)
        self.fileNameEdit = QLineEdit(self)

        # Initially hide if necessary
        self.fileNameLabel.setVisible(False)
        self.fileNameEdit.setVisible(False)

        # Add the label and edit to the horizontal layout
        self.fileNameLayout.addWidget(self.fileNameLabel)
        self.fileNameLayout.addWidget(self.fileNameEdit)
        # Connect the ComboBox change signal to the slot
        self.optionCombo.currentIndexChanged.connect(self.updateUIBasedOnSelection)

        # Confirm Button
        self.confirmBtn = QPushButton('Confirm', self)
        self.confirmBtn.clicked.connect(self.confirmAction)

        # Text Edit to display output (for demonstration purposes)
        self.outputText = QTextEdit(self)
        self.outputText.setReadOnly(True)

        # Layout Setup
        self.layout.addWidget(self.optionLabel)
        self.layout.addWidget(self.optionCombo)
        self.layout.addWidget(self.fileNameLayout)
        self.layout.addWidget(self.confirmBtn)
        self.layout.addWidget(self.outputText)

        self.mainWidget = QWidget()
        self.mainWidget.setLayout(self.layout)
        self.setCentralWidget(self.mainWidget)

    def updateUIBasedOnSelection(self):
        # Check the current selection
        if self.optionCombo.currentText() == 'Load and edit waypoints':
            # Show the FileName Label and Edit
            self.fileNameLabel.show()
            self.fileNameEdit.show()
        else:
            # Hide the FileName Label and Edit
            self.fileNameLabel.hide()
            self.fileNameEdit.hide()

    def setupCollectWaypointsUI(self):
        widgets = self.layout.children()
        for widgetToRemove in widgets:
            self.layout.removeWidget(widgetToRemove)
            # widgetToRemove.setParent(None)

        # Setup the UI for collecting waypoints
        self.infoLabel = QLabel('Move the robot to desired positions and press "Add Waypoint"', self)
        self.addWaypointBtn = QPushButton('Add Waypoint', self)
        self.addWaypointBtn.setEnabled(False)  # Disable the button initially
        # Connect this button to your method for adding a waypoint
        self.addWaypointBtn.clicked.connect(self.addWaypoint)

        self.saveBtn = QPushButton('Save Waypoints', self)
        self.saveBtn.setEnabled(False)  # Disable the button initially
        self.contBtn = QPushButton('Continue Collecting Waypoints', self)
        self.contBtn.setEnabled(False)  # Disable the button initially

        # Connect this button to your method for saving waypoints
        self.saveBtn.clicked.connect(self.saveWaypoints)
        self.contBtn.clicked.connect(self.contWaypoints)


        # Add widgets to the layout
        self.layout.addWidget(self.infoLabel)
        self.layout.addWidget(self.addWaypointBtn)
        self.layout.addWidget(self.contBtn)
        self.layout.addWidget(self.fileNameLayout)
        self.layout.addWidget(self.saveBtn)
        self.layout.addWidget(self.outputText)


    def setupPlanningUI(self):

        removeWidgets = self.layout.children()
        for widgetToRemove in removeWidgets:
            self.layout.removeWidget(widgetToRemove)
            widgetToRemove.setParent(None)

        self.infoLabel = QLabel('Planning UI', self)

        self.loadWPBtn = QPushButton('Load Waypoints', self)
        self.loadPlanBtn = QPushButton('Load Plan', self)

        self.planBtn = QPushButton('Plan Carthesian Path', self)
        self.planBtn.setEnabled(False)

        self.saveBtn = QPushButton('Save Plan', self)
        self.saveBtn.setEnabled(False)  # Disable the button initially

        self.executeBtn = QPushButton('Execute Plan', self)
        self.executeBtn.setEnabled(False)  # Disable the button initially

        self.startBtn = QPushButton('Start', self)
        self.startBtn.setEnabled(False)  # Disable the button initially
        self.start = False



        # Connect this button to your method for saving waypoints
        self.loadWPBtn.clicked.connect(self.loadWP)
        self.loadPlanBtn.clicked.connect(self.loadPlan)
        self.planBtn.clicked.connect(self.plan)
        self.saveBtn.clicked.connect(self.savePlan)
        self.executeBtn.clicked.connect(self.executePlan)
        self.startBtn.clicked.connect(self.startMovement)

        self.layout.addWidget(self.infoLabel)
        self.layout.addWidget(self.fileNameLayout)
        self.layout.addWidget(self.loadWPBtn)
        self.layout.addWidget(self.loadPlanBtn)
        self.layout.addWidget(self.planBtn)
        self.layout.addWidget(self.saveBtn)
        self.layout.addWidget(self.executeBtn)
        self.layout.addWidget(self.startBtn)
        # self.show()
    def restoreMainUI(self):
        for widgetToRemove in self.layout.children():
            self.layout.removeWidget(widgetToRemove)
            # widgetToRemove.setParent(None)
        # self.layout.addWidget(self.infoLabel)
        self.layout.addWidget(self.optionLabel)
        self.layout.addWidget(self.optionCombo)
        self.layout.addWidget(self.fileNameLabel)
        self.layout.addWidget(self.fileNameEdit)
        self.layout.addWidget(self.confirmBtn)
        self.layout.addWidget(self.outputText)
    def addWaypoint(self):
        # Your logic for adding a waypoint
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
        self.helper.plan = self.helper.CPP(self.helper.ifaceLA.loaded_json_wpoints, self.helper.ifaceRA.loaded_json_wpoints)
        if self.helper.plan:
            self.saveBtn.setEnabled(True)
            # self.executeBtn.setEnabled(True)
        else:
            self.saveBtn.setEnabled(False)
            # self.executeBtn.setEnabled(False)

    def savePlan(self):
        if self.fileNameEdit.text() == '':
            self.outputText.append('Please enter a file name')
        else:
            self.helper.save_plan(self.helper.plan, self.fileNameEdit.text())
            self.executeBtn.setEnabled(True)

    def executePlan(self):
        # self.helper.ifaceLA.move_group.execute(helper.plan,
        #                                   wait=True)  # Waits until feedback from execution is received
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
            #
            # self.setupPlanningUI()
            # # Query save
            #
            # self.interface_left_arm.markerArrayPub.publish(self.interface_left_arm.markerArray)
            # self.interface_right_arm.markerArrayPub.publish(self.interface_right_arm.markerArray)
            # self.interface_head.markerArrayPub.publish(self.interface_head.markerArray)


        elif action == 'Load and edit waypoints':
            if self.helper.loadWaypointsFromJSON(fileName):

                self.helper.initial_wp_count = len(self.helper.ifaceLA.loaded_json_wpoints.poses) + 1
                self.setupCollectWaypointsUI()
                thread = threading.Thread(target=self.helper.collect_wpoints)
                thread.start()

        elif action == 'Plan/Execute':
            self.setupPlanningUI()
            # Load a saved plan and execute it


        # Update the output text with the selected option and file name (for demonstration)
        self.outputText.append(f"Action: {action}, File Name: {fileName}")
        # Here, add your logic to handle the selected action and file name,
        # such as invoking functions from your script or setting up further GUI elements.




if __name__ == '__main__':
    # Create interface to Moveit:RobotCommander (move-group python interface)

    app = QApplication(sys.argv)
    gui = RobotGUI()

    gui.show()
    sys.exit(app.exec_())
