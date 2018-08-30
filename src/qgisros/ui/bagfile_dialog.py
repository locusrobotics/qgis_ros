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
        self.addLayerButton.clicked.connect(self._createLayerFromSelected)
        self.unloadBagButton.clicked.connect(self._unloadBag)

        self.layerCreated.connect(self._addCreatedLayer)
        self.layerLoadProgress.connect(self._updateLoadProgress)
        self.takeRadioGroup.buttonClicked.connect(self._updateRadioSelection)

        self._bagFilePath = None

    def _getBagFileTopics(self):
        # Load topic metadata from bag.
        filePath, _ = QFileDialog.getOpenFileName(parent=self, filter='Bagfiles (*.bag);;All Files (*)')

        if not filePath:
            return

        self._bagFilePath = filePath
        self.openBagButton.setText('Opening...')
        self.openBagButton.setEnabled(False)

        threading.Thread(name='getBagTopicsThread', target=self._getBagContentsWorker).start()

    def _getBagContentsWorker(self):
        topicMetadata = helpers.getTopicsFromBag(self._bagFilePath)
        self.currentBagPathLabel.setText(self._bagFilePath)
        self.dataLoaderWidget.setTopics(topicMetadata)
        self.stackedWidget.setCurrentWidget(self.dataLoaderWidgetContainer)
        self.openBagButton.setText('Open Bag...')
        self.openBagButton.setEnabled(True)
        self.unloadBagButton.setEnabled(True)

    def _createLayerFromSelected(self):
        self.addLayerButton.setText('Loading...')
        self.addLayerButton.setEnabled(False)

        threading.Thread(name='createLayerThread', target=self._createLayerWorker).start()

    def _createLayerWorker(self):
        name, topicType = self.dataLoaderWidget.getSelectedTopic()

        # Defaults for taking all messages.
        sampleInterval = 1
        takeLast = False

        if self.takeSampleRadio.isChecked():  # Take sample.
            sampleInterval = self.sampleIntervalBox.value()
        elif self.takeLastRadio.isChecked():  # Take last.
            takeLast = True

        messages = helpers.getBagData(
            self._bagFilePath,
            name,
            sampleInterval=sampleInterval,
            takeLast=takeLast,
            progressCallback=self.layerLoadProgress.emit
        )

        translator = TranslatorRegistry.instance().get(topicType)
        layer = translator.createLayer(name, rosMessages=messages)
        self.layerCreated.emit(layer)  # Need to add layer from main thread.

        self.addLayerButton.setText('Add Layer')
        self.addLayerButton.setEnabled(True)

    @pyqtSlot(object)
    def _addCreatedLayer(self, layer):
        QgsProject.instance().addMapLayer(layer)

    @pyqtSlot(int)
    def _updateLoadProgress(self, progress):
        self.addLayerButton.setText(str(progress))

    def _unloadBag(self):
        '''Frees up any resources from the bag, resets view and state.'''
        self._bagFilePath = None
        self.currentBagPathLabel.setText('No bag selected')
        self.dataLoaderWidget.setTopics(None)
        self.stackedWidget.setCurrentWidget(self.bagSelectionWidget)
        self.unloadBagButton.setEnabled(False)

    def _updateRadioSelection(self):
        '''When radio group changes, enable/disable the sample interval.'''
        if self.takeSampleRadio.isChecked():
            self.sampleIntervalGroup.setEnabled(True)
        else:
            self.sampleIntervalGroup.setEnabled(False)
