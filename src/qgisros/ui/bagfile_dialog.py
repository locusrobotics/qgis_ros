from pathlib import Path
import os

from PyQt5 import uic
from PyQt5.QtWidgets import QDialog, QFileDialog

from ..core import helpers

FORM_CLASS, _ = uic.loadUiType(str(Path(os.path.dirname(__file__)) / 'bagfile_dialog.ui'))


class BagfileDialog(QDialog, FORM_CLASS):

    def __init__(self, parent=None):
        super(BagfileDialog, self).__init__(parent)
        self.setupUi(self)

        self.openBagButton.clicked.connect(self._getBagFileTopics)
        self.AddTopicButton.clicked.connect(self._addSelectedTopic)

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

    def _addSelectedTopic(self):
        name, topicType = self.dataLoaderWidget.getSelectedTopic()
        data = helpers.getBagData(self._bagFilePath, name)
        print(len(data))
