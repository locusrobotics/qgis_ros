from pathlib import Path
import os
import threading

from PyQt5 import uic
from PyQt5.QtWidgets import QDialog
from PyQt5.QtCore import QTimer, pyqtSignal

from qgis.core import QgsProject

from socket import error as socket_error
import rosgraph
import rospy

from ..core import TranslatorRegistry


FORM_CLASS, _ = uic.loadUiType(str(Path(os.path.dirname(__file__)) / 'ros_master_dialog.ui'))


class ROSMasterDialog(QDialog, FORM_CLASS):

    layerCreated = pyqtSignal(object)

    def __init__(self, parent=None):
        super(ROSMasterDialog, self).__init__(parent)
        self.setupUi(self)
        self.dataLoaderWidget.tableWidget.removeColumn(3)  # Don't need "count" field.

        self.addLayerButton.clicked.connect(self._createLayerFromSelected)
        self.layerCreated.connect(self._addCreatedLayer)

        # Check every two seconds for a ROS master.
        self.checkMasterTimer = QTimer()
        self.checkMasterTimer.timeout.connect(self._checkForMaster)
        self.checkMasterTimer.start(2000)

        # Update UI depending on which data loading method is selected.
        self.subscribeRadioGroup.buttonClicked.connect(self._updateKeepOlderMessagesCheckbox)

        # Update UI depending on what the selected layer's data type is.
        self.dataLoaderWidget.selectedTopicChanged.connect(self._updateDataLoadingOptions)

        # Report current state of ROS MASTER search to user.
        masterUri = os.environ.get('ROS_MASTER_URI')
        if masterUri:
            label = 'Searching for ROS Master at: {}...'.format(masterUri)
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
            self._getAvailableTopics()

            self.checkMasterTimer.stop()
            self.currentRosMasterLabel.setText(os.environ['ROS_MASTER_URI'])
            self.stackedWidget.setCurrentWidget(self.dataLoaderWidgetContainer)

    def _getAvailableTopics(self):
        topicMetadata = [(topicName, topicType) for topicName, topicType in rospy.get_published_topics()]
        self.dataLoaderWidget.setTopics(topicMetadata)

    def _createLayerFromSelected(self):
        threading.Thread(name='createLayerThread', target=self._createLayerWorker).start()

    def _createLayerWorker(self):
        name, topicType = self.dataLoaderWidget.getSelectedTopic()

        subscribe = self.subscribeRadio.isChecked()
        keepOlderMessages = self.keepOlderMessagesCheckbox.isChecked()

        translator = TranslatorRegistry.instance().get(topicType)
        layer = translator.createLayer(name, subscribe=subscribe, keepOlderMessages=keepOlderMessages)
        self.layerCreated.emit(layer)  # Need to add layer from main thread.

    def _addCreatedLayer(self, layer):
        QgsProject.instance().addMapLayer(layer)

    def _updateKeepOlderMessagesCheckbox(self):
        self.keepOlderMessagesCheckbox.setEnabled(not self.takeLatestRadio.isChecked())

    def _updateDataLoadingOptions(self):
        '''Adjust the available loading options if a raster is selected.

        Raster data types aren't subscriptable. Must use Take Latest instead.
        '''
        name, topicType = self.dataLoaderWidget.getSelectedTopic()
        dataModelType = TranslatorRegistry.instance().get(topicType).dataModelType
        isRaster = dataModelType == 'Raster'

        self.subscribeRadio.setEnabled(not isRaster)
        self.takeLatestRadio.setChecked(isRaster)

        self._updateKeepOlderMessagesCheckbox()
