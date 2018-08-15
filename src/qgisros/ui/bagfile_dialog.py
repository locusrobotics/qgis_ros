from pathlib import Path
import os

from PyQt5 import uic
from PyQt5.QtWidgets import QDialog, QFileDialog

from qgis.core import QgsProject

from ..core import helpers, TranslatorRegistry
from .loading_modal import LoadingModal

FORM_CLASS, _ = uic.loadUiType(str(Path(os.path.dirname(__file__)) / 'bagfile_dialog.ui'))


class BagfileDialog(QDialog, FORM_CLASS):

    def __init__(self, parent=None):
        super(BagfileDialog, self).__init__(parent)
        self.setupUi(self)

        self.openBagButton.clicked.connect(self._getBagFileTopics)
        self.AddTopicButton.clicked.connect(self._createLayerFromSelected)
        self.unloadBagButton.clicked.connect(self._unloadBag)

        self._bagFilePath = None
        self._loadingModal = LoadingModal(self)  # self as parent.

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
        name, topicType = self.dataLoaderWidget.getSelectedTopic()
        self._loadingModal.show()
        messages = helpers.getBagData(self._bagFilePath, name)
        translator = TranslatorRegistry.instance().get(topicType)
        layer = translator.createLayer(name, rosMessages=messages)
        QgsProject.instance().addMapLayer(layer)
        self._loadingModal.hide()

    def _unloadBag(self):
        '''Frees up any resources from the bag, resets view and state.'''
        self._bagFilePath = None
        self.currentBagPathLabel.setText('No bag selected')
        self.dataLoaderWidget.setTopics(None)
        self.stackedWidget.setCurrentWidget(self.bagSelectionWidget)
        self.unloadBagButton.setEnabled(False)
