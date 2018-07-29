from functools import partial
from pathlib import Path
import os

from PyQt5 import uic, QtCore, QtWidgets
from qgis.core import QgsProject
import rospy
from .core import TranslatorRegistry


FORM_CLASS, _ = uic.loadUiType(str(Path(os.path.dirname(__file__)) / 'ui' / 'vector_data_dialog.ui'))


class VectorDataDialog(QtWidgets.QDialog, FORM_CLASS):

    def __init__(self, parent=None):
        '''Occurs on init, even if dialog is not shown.'''
        super(VectorDataDialog, self).__init__(parent)

        self.setupUi(self)

        self.topicList.currentItemChanged.connect(self._onTopicListChange)
        self.createLayerButton.clicked.connect(self._onCreateLayer)
        self.subscribeButton.clicked.connect(partial(self._onCreateLayer, subscribe=True))

        self._selectedTopicName = None
        self._selectedTopicType = None

    def showEvent(self, event):
        self._populate_topic_list()
        print('shown!')

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
            dataType = TranslatorRegistry.instance().get(topicType).dataType
            self.selectedTopicDetails.setEnabled(True)
        else:
            topicType = None
            topicName = None
            dataType = None
            self.selectedTopicDetails.setEnabled(False)

        # Update GUI
        self.topicTypeLabel.setText(topicType or '')
        self.topicNameLabel.setText(topicName or '')
        self.dataTypeLabel.setText(dataType or '')

        self._selectedTopicName = topicName
        self._selectedTopicType = topicType

    def _onCreateLayer(self, subscribe=False):
        translator = TranslatorRegistry.instance().get(self._selectedTopicType)
        layer = translator.createLayer(self._selectedTopicName, subscribe=subscribe)
        QgsProject.instance().addMapLayer(layer)
