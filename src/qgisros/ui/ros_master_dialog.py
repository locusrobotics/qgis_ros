from pathlib import Path
import os

from PyQt5 import uic
from PyQt5.QtWidgets import QDialog
from PyQt5.QtCore import QTimer

from socket import error as socket_error
import rosgraph
import rospy

FORM_CLASS, _ = uic.loadUiType(str(Path(os.path.dirname(__file__)) / 'ros_master_dialog.ui'))


class ROSMasterDialog(QDialog, FORM_CLASS):

    def __init__(self, parent=None):
        super(ROSMasterDialog, self).__init__(parent)
        self.setupUi(self)
        self.dataLoaderWidget.tableWidget.removeColumn(2)  # Don't need "count" field.

        # Check every two seconds for a ROS master.
        self.checkMasterTimer = QTimer()
        self.checkMasterTimer.timeout.connect(self._checkForMaster)
        self.checkMasterTimer.start(2000)

        # Report current state of ROS MASTER search to user.
        masterUri = os.environ.get('ROS_MASTER_URI')
        if masterUri:
            label = 'Searching for ROS Master at: {}...'.format(masterUri)
            self._getAvailableTopics()
        else:
            label = 'Environment variable `ROS_MASTER_URI` not set. Cannot initialize node.'
            self.checkMasterTimer.stop()
        self.searchMasterLabel.setText(label)

    def _checkForMaster(self):
        try:
            rosgraph.Master('/rostopic').getPid()
        except socket_error:
            pass  # No master found.
        else:
            # Master found. Init node then update UI state.
            rospy.init_node('qgis_ros_toolbox')

            self.checkMasterTimer.stop()
            self.currentRosMasterLabel.setText(os.environ['ROS_MASTER_URI'])
            self.stackedWidget.setCurrentWidget(self.dataLoaderWidgetContainer)

    def _getAvailableTopics(self):
        topicMetadata = [(topicName, topicType) for topicName, topicType in rospy.get_published_topics()]
        print(topicMetadata)
        self.dataLoaderWidget.setTopics(topicMetadata)

