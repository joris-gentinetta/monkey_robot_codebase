import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout,
                             QWidget, QTextEdit, QLineEdit, QLabel, QComboBox)
from monkey_interface import MoveGroupInterface, Utils

# Create interface to Moveit:RobotCommander (move-group python interface)
interface_left_arm = MoveGroupInterface('monkey_left_arm')
interface_right_arm = MoveGroupInterface('monkey_right_arm')
interface_head = MoveGroupInterface('head')

# Create helper class
helper = Utils(interface_left_arm, interface_right_arm, interface_head)
class RobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Robot Control GUI')
        self.setGeometry(100, 100, 600, 400)

        self.layout = QVBoxLayout()

        # Option Selection
        self.optionLabel = QLabel('Select an action:', self)
        self.optionCombo = QComboBox(self)
        self.optionCombo.addItem('Collect waypoints')
        self.optionCombo.addItem('Load and edit saved waypoints')
        self.optionCombo.addItem('Load a saved plan and execute it')

        # File Name Input
        self.fileNameLabel = QLabel('File name:', self)
        self.fileNameEdit = QLineEdit(self)

        # Confirm Button
        self.confirmBtn = QPushButton('Confirm', self)
        self.confirmBtn.clicked.connect(self.confirmAction)

        # Text Edit to display output (for demonstration purposes)
        self.outputText = QTextEdit(self)
        self.outputText.setReadOnly(True)

        # Layout Setup
        self.layout.addWidget(self.optionLabel)
        self.layout.addWidget(self.optionCombo)
        self.layout.addWidget(self.fileNameLabel)
        self.layout.addWidget(self.fileNameEdit)
        self.layout.addWidget(self.confirmBtn)
        self.layout.addWidget(self.outputText)

        self.mainWidget = QWidget()
        self.mainWidget.setLayout(self.layout)
        self.setCentralWidget(self.mainWidget)

    def setupCollectWaypointsUI(self):
        # Clear the current layout
        for i in reversed(range(self.layout.count())):
            widgetToRemove = self.layout.itemAt(i).widget()
            self.layout.removeWidget(widgetToRemove)
            widgetToRemove.setParent(None)

        # Setup the UI for collecting waypoints
        self.infoLabel = QLabel('Move the robot to desired positions and press "Add Waypoint"', self)
        self.addWaypointBtn = QPushButton('Add Waypoint', self)
        self.addWaypointBtn.setEnabled(False)  # Disable the button initially
        # Connect this button to your method for adding a waypoint
        self.addWaypointBtn.clicked.connect(self.addWaypoint)

        self.waypointsList = QTextEdit(self)
        self.waypointsList.setReadOnly(True)

        self.saveBtn = QPushButton('Save Waypoints', self)
        self.saveBtn.setEnabled(False)  # Disable the button initially

        # Connect this button to your method for saving waypoints
        self.saveBtn.clicked.connect(self.saveWaypoints)

        # Add widgets to the layout
        self.layout.addWidget(self.infoLabel)
        self.layout.addWidget(self.addWaypointBtn)
        self.layout.addWidget(self.waypointsList)
        self.layout.addWidget(self.saveBtn)

    def addWaypoint(self):
        # Your logic for adding a waypoint
        self.save_waypoint = True

    def saveWaypoints(self):
        # Your logic for saving waypoints
        print("Waypoints Saved")
        #todo

    def confirmAction(self):
        # Get the selected action and file name
        action = self.optionCombo.currentText()
        fileName = self.fileNameEdit.text()
        if action == 'Collect waypoints':
            self.setupCollectWaypointsUI()

            # Collect waypoints in gui
            # collected_pose_arrayLA, collected_pose_arrayRA, collected_pose_arrayH = helper.collect_wpoints(1)
            collected_pose_arrayLA, collected_pose_arrayRA = helper.collect_wpoints(1, self)
            # Query save
            helper.querySave(collected_pose_arrayLA, collected_pose_arrayRA)  # , collected_pose_arrayH)
            # Make sure that all user set waypoints are being displayed
            interface_left_arm.markerArrayPub.publish(interface_left_arm.markerArray)
            interface_right_arm.markerArrayPub.publish(interface_right_arm.markerArray)
            interface_head.markerArrayPub.publish(interface_head.markerArray)

            # Query decision to plan car. path
            # helper.queryCPP(collected_pose_arrayLA, collected_pose_arrayRA, collected_pose_arrayH)
            helper.queryCPP(collected_pose_arrayLA, collected_pose_arrayRA)
        elif action == 'Load and edit saved waypoints':
            # Query json file name, load poseArray, query appending new poses
            helper.loadWaypointsFromJSON()
        elif action == 'Load a saved plan and execute it':
            # Load a saved plan and execute it
            helper.interact_with_monkey_listener()

        # Update the output text with the selected option and file name (for demonstration)
        self.outputText.append(f"Action: {action}, File Name: {fileName}")
        # Here, add your logic to handle the selected action and file name,
        # such as invoking functions from your script or setting up further GUI elements.


class CollectWaypointsGUI(QWidget):
    def __init__(self, helper, parent=None):
        super().__init__(parent)
        self.helper = helper
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Collect Waypoints')
        self.setGeometry(200, 200, 400, 300)

        layout = QVBoxLayout()

        self.infoLabel = QLabel('Move the robot to desired positions and press "Add Waypoint"', self)
        self.addWaypointBtn = QPushButton('Add Waypoint', self)
        self.addWaypointBtn.clicked.connect(self.addWaypoint)

        self.waypointsList = QTextEdit(self)
        self.waypointsList.setReadOnly(True)

        self.saveBtn = QPushButton('Save Waypoints', self)
        self.saveBtn.clicked.connect(self.saveWaypoints)

        layout.addWidget(self.infoLabel)
        layout.addWidget(self.addWaypointBtn)
        layout.addWidget(self.waypointsList)
        layout.addWidget(self.saveBtn)

        self.setLayout(layout)

    def addWaypoint(self):
        # Placeholder for adding a waypoint
        # You'll need to integrate this with the actual method to add a waypoint
        self.waypointsList.append("Waypoint Added")
        # Example: self.helper.addWaypoint()

    def saveWaypoints(self):
        # Placeholder for saving waypoints
        # You'll need to integrate this with the actual method to save waypoints
        print("Waypoints Saved")
        # Example: self.helper.saveWaypoints()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = RobotGUI()
    gui.show()
    sys.exit(app.exec_())
