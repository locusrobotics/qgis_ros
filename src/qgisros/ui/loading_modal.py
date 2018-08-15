from pathlib import Path
import os

from PyQt5 import uic, QtWidgets


FORM_CLASS, _ = uic.loadUiType(str(Path(os.path.dirname(__file__)) / 'loading_modal.ui'))


class LoadingModal(QtWidgets.QDialog, FORM_CLASS):

    def __init__(self, parent=None):
        super(LoadingModal, self).__init__(parent)
        self.setupUi(self)
