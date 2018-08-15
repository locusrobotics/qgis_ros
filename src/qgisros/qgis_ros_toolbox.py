import os.path

from PyQt5 import QtWidgets
from qgis.core import QgsProviderRegistry, QgsProviderMetadata

from .ui import ROSMasterDialog, BagfileDialog
from .core import ROSVectorProvider


class QgisRos(object):
    '''The parent plugin object.

    Initializes both UI and non-UI elements for QGIS-ROS. This includes a toolbar (and associated drop-down menu),
    which are connected to instances of each window, created eagerly. The ROSVectorProvider is also registered,
    allowing data layers to be created using it.

    A ROS node is *not* initialized unless explicitly requested by the user. This allows a user to use ROS bags in an
    environment where a ROS master is not
    '''

    AUTO_REFRESH_INTERVAL = 1000  # milliseconds

    def __init__(self, iface):
        self.iface = iface
        self.plugin_dir = os.path.dirname(__file__)

        # Register all actions for cleanup on plugin unload.
        self.actions = []

        self.menu = 'QGIS ROS'
        self.toolbar = self.iface.addToolBar(u'QgisRos')
        self.toolbar.setObjectName(u'QgisRos')

        # Register the ROS vector data provider.
        metadata = QgsProviderMetadata(
            ROSVectorProvider.providerKey(),
            ROSVectorProvider.description(),
            ROSVectorProvider.createProvider)
        QgsProviderRegistry.instance().registerProvider(metadata)

        # Eager init dialogs and preserve state when re-opened in the future.
        self.bagFileDialog = BagfileDialog()
        self.rosMasterDialog = ROSMasterDialog()

    def initGui(self):
        toolbarButtons = [
            ('Load Bag Data...', self.bagFileDialog),
            ('Load Topic Data', self.rosMasterDialog)
        ]

        # Assemble the toolbar buttons.
        for button in toolbarButtons:
            action = QtWidgets.QAction(button[0], self.iface.mainWindow())
            action.triggered.connect(button[1].show)
            self.toolbar.addAction(action)
            self.iface.addPluginToMenu(self.menu, action)
            self.actions.append(action)

    def unload(self):
        for action in self.actions:
            self.iface.removePluginMenu('QGIS ROS', action)
            self.iface.removeToolBarIcon(action)
        del self.toolbar  # Deref to ensure C++ cleanup.

    def run(self):
        self.dialog.show()
