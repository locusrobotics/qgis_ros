import os
from pathlib import Path

from PyQt5 import uic

from PyQt5.QtWidgets import QWidget, QTableWidgetItem, QHeaderView
from PyQt5.QtCore import pyqtSignal

from ..core import TranslatorRegistry

FORM_CLASS, _ = uic.loadUiType(str(Path(os.path.dirname(__file__)) / 'data_loader_widget.ui'))


class DataLoaderWidget(QWidget, FORM_CLASS):

    selectedTopicChanged = pyqtSignal()

    def __init__(self, parent=None):
        super(DataLoaderWidget, self).__init__(parent)
        self._showMessageCount = False
        self.setupUi(self)

        self.tableWidget.itemSelectionChanged.connect(self.selectedTopicChanged)

    def setTopics(self, topicMetadata):
        '''Populates table with a new set of topicMetadata.'''
        self.tableWidget.clearContents()

        if topicMetadata is None:
            return

        # Filter untranslatable topicMetadata
        topicMetadata = [t for t in topicMetadata if t[1] in TranslatorRegistry.instance().translatableTypeNames]

        # Populate table.
        self.tableWidget.setRowCount(len(topicMetadata))
        for row, topicMetadata in enumerate(topicMetadata):
            dataModelType = TranslatorRegistry.instance().get(topicMetadata[1]).dataModelType
            self.tableWidget.setItem(row, 0, QTableWidgetItem(topicMetadata[0]))
            self.tableWidget.setItem(row, 1, QTableWidgetItem(topicMetadata[1]))
            self.tableWidget.setItem(row, 2, QTableWidgetItem(dataModelType))

            try:
                self.tableWidget.setItem(row, 3, QTableWidgetItem(str(topicMetadata[2])))
            except Exception:
                pass  # No data count available.

        header = self.tableWidget.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)

    def getSelectedTopic(self):
        row = self.tableWidget.currentRow()
        topicName = self.tableWidget.item(row, 0).text()
        topicType = self.tableWidget.item(row, 1).text()

        return topicName, topicType

