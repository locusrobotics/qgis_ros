import os
from pathlib import Path

from PyQt5 import uic

from PyQt5.QtWidgets import QWidget, QTableWidgetItem, QHeaderView

from ..core import TranslatorRegistry

FORM_CLASS, _ = uic.loadUiType(str(Path(os.path.dirname(__file__)) / 'data_loader_widget.ui'))


class DataLoaderWidget(QWidget, FORM_CLASS):

    def __init__(self, parent=None):
        super(DataLoaderWidget, self).__init__(parent)

        self.setupUi(self)

    def setTopics(self, topicMetadata):
        '''Populates table with topicMetadata.'''

        # Filter untranslatable topics
        topicMetadata = [t for t in topicMetadata if t.type in TranslatorRegistry.instance().translatableTypeNames]

        # Populate table.
        self.tableWidget.setRowCount(len(topicMetadata))
        for row, metadata in enumerate(topicMetadata):
            self.tableWidget.setItem(row, 0, QTableWidgetItem(metadata.name))
            self.tableWidget.setItem(row, 1, QTableWidgetItem(metadata.type))
            self.tableWidget.setItem(row, 2, QTableWidgetItem(str(metadata.count)))

        header = self.tableWidget.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.Stretch)

    def getSelectedTopic(self):
        row = self.tableWidget.currentRow()
        topicName = self.tableWidget.item(row, 0).text()
        topicType = self.tableWidget.item(row, 1).text()

        return topicName, topicType
