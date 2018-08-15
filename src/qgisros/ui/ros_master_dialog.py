from pathlib import Path
import os

from PyQt5 import uic, QtWidgets


FORM_CLASS, _ = uic.loadUiType(str(Path(os.path.dirname(__file__)) / 'ros_master_dialog.ui'))


class ROSMasterDialog(QtWidgets.QDialog, FORM_CLASS):

    def __init__(self, parent=None):
        super(ROSMasterDialog, self).__init__(parent)

        self.setupUi(self)
