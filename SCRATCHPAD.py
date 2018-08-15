

# Try to init ROS node or fail out immediately to avoid blocking the UI.
from socket import error as socket_error
import rosgraph
import rospy

try:
    rosgraph.Master('/rostopic').getPid()
except socket_error:
    raise rospy.ROSInitException('Cannot load QGIS ROS. ROS Master was not found.')
else:
    rospy.init_node('qgis_ros_toolbox')


from functools import partial
from pathlib import Path
import os

from PyQt5 import uic, QtCore, QtWidgets
from qgis.core import QgsProject
import rospy
from ..core import TranslatorRegistry


FORM_CLASS, _ = uic.loadUiType(str(Path(os.path.dirname(__file__)) / 'data_loader_widget.ui'))


class DataLoaderWidget(QtWidgets.QWidget, FORM_CLASS):

    def __init__(self, parent=None):
        '''Occurs on init, even if dialog is not shown.'''
        super(DataLoaderWidget, self).__init__(parent)

        self.setupUi(self)

        self.topicList.currentItemChanged.connect(self._onTopicListChange)
        self.createLayerButton.clicked.connect(self._onCreateLayer)
        self.subscribeButton.clicked.connect(
            partial(self._onCreateLayer, subscribe=True))

        self._selectedTopicName = None
        self._selectedTopicType = None

    def showEvent(self, event):
        self._populate_topic_list()

    def _populate_topic_list(self):
        self.topicList.clear()

        topics = rospy.get_published_topics()

        # TODO: sort.
        for t in topics:
            if t[1] in TranslatorRegistry.instance().translatableTypeNames:
                item = QtWidgets.QListWidgetItem(t[0])
                item.setData(QtCore.Qt.UserRole, t[1])
                self.topicList.addItem(item)

    def _onTopicListChange(self, current: QtWidgets.QListWidgetItem, previous: QtWidgets.QListWidgetItem):
        if current is not None:
            topicType = current.data(QtCore.Qt.UserRole)
            topicName = current.text()
            dataModelType = TranslatorRegistry.instance().get(topicType).dataModelType
            self.selectedTopicDetails.setEnabled(True)
        else:
            topicType = None
            topicName = None
            dataModelType = None
            self.selectedTopicDetails.setEnabled(False)

        # Update GUI
        self.topicTypeLabel.setText(topicType or '')
        self.topicNameLabel.setText(topicName or '')
        self.dataModelTypeLabel.setText(dataModelType or '')

        self._selectedTopicName = topicName
        self._selectedTopicType = topicType

    def _onCreateLayer(self, subscribe=False):
        translator = TranslatorRegistry.instance().get(self._selectedTopicType)
        layer = translator.createLayer(self._selectedTopicName, subscribe=subscribe)
        QgsProject.instance().addMapLayer(layer)
