from pathlib import Path
import os
import threading

from PyQt5 import uic
from PyQt5.QtWidgets import QDialog, QFileDialog
from PyQt5.QtCore import pyqtSignal, pyqtSlot

from qgis.core import QgsProject

from ..core import helpers, TranslatorRegistry

FORM_CLASS, _ = uic.loadUiType(str(Path(os.path.dirname(__file__)) / 'bagfile_dialog.ui'))


class BagfileDialog(QDialog, FORM_CLASS):

    layerCreated = pyqtSignal(object)
    layerLoadProgress = pyqtSignal(int)

    def __init__(self, parent=None):
        super(BagfileDialog, self).__init__(parent)
        self.setupUi(self)

        self.openBagButton.clicked.connect(self._getBagFileTopics)
        self.addTopicButton.clicked.connect(self._createLayerFromSelected)
        self.unloadBagButton.clicked.connect(self._unloadBag)

        self.layerCreated.connect(self.addCreatedLayer)
        self.layerLoadProgress.connect(self.updateLoadProgress)

        self._bagFilePath = None

    def _getBagFileTopics(self):
        # Load topic metadata from bag.
        filePath, _ = QFileDialog.getOpenFileName(parent=self, filter='Bagfiles (*.bag);;All Files (*)')
        topicMetadata = helpers.getTopicsFromBag(filePath)

        # Bag loaded properly. Update components and then show them.
        self._bagFilePath = filePath
        self.currentBagPathLabel.setText(filePath)
        self.dataLoaderWidget.setTopics(topicMetadata)
        self.stackedWidget.setCurrentWidget(self.dataLoaderWidgetContainer)

    def _createLayerFromSelected(self):
        self.addTopicButton.setText('Loading...')
        self.addTopicButton.setEnabled(False)

        t = threading.Thread(name='my_worker', target=self._createLayerWorker)
        t.start()

    def _createLayerWorker(self):
        name, topicType = self.dataLoaderWidget.getSelectedTopic()
        messages = helpers.getBagData(
            self._bagFilePath,
            name,
            sampleInterval=100,
            progressCallback=self.layerLoadProgress.emit
        )
        translator = TranslatorRegistry.instance().get(topicType)
        layer = translator.createLayer(name, rosMessages=messages)
        self.layerCreated.emit(layer)

        self.addTopicButton.setText('Add Layer')
        self.addTopicButton.setEnabled(True)

    @pyqtSlot(object)
    def addCreatedLayer(self, layer):
        QgsProject.instance().addMapLayer(layer)

    @pyqtSlot(int)
    def updateLoadProgress(self, progress):
        self.addTopicButton.setText(str(progress))

    def _unloadBag(self):
        '''Frees up any resources from the bag, resets view and state.'''
        self._bagFilePath = None
        self.currentBagPathLabel.setText('No bag selected')
        self.dataLoaderWidget.setTopics(None)
        self.stackedWidget.setCurrentWidget(self.bagSelectionWidget)
        self.unloadBagButton.setEnabled(False)
