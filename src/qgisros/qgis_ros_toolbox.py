import os.path
from socket import error as socket_error

from PyQt5 import QtWidgets
from qgis.core import QgsProviderRegistry, QgsProviderMetadata
import rosgraph
import rospy

from .vector_data_dialog import VectorDataDialog
from .core import ROSVectorProvider


class QgisRos(object):

    AUTO_REFRESH_INTERVAL = 1000  # milliseconds

    def __init__(self, iface):
        self.iface = iface
        self.plugin_dir = os.path.dirname(__file__)

        self.actions = []
        self.menu = 'QGIS ROS'
        self.toolbar = self.iface.addToolBar(u'QgisRos')
        self.toolbar.setObjectName(u'QgisRos')

        # Try to init ROS node or fail out immediately to avoid blocking the UI.
        try:
            rosgraph.Master('/rostopic').getPid()
        except socket_error:
            raise rospy.ROSInitException(
                'Cannot load QGIS ROS. No ROS Master was found.')
        else:
            rospy.init_node('qgis_ros_toolbox')

        # Register the ROS vector data provider.
        metadata = QgsProviderMetadata(
            ROSVectorProvider.providerKey(),
            ROSVectorProvider.description(),
            ROSVectorProvider.createProvider
        )
        r = QgsProviderRegistry.instance()
        r.registerProvider(metadata)

        self.dialog = VectorDataDialog()

    def initGui(self):
        action = QtWidgets.QAction('Vector Data', self.iface.mainWindow())
        action.triggered.connect(self.run)
        action.setEnabled(True)

        self.toolbar.addAction(action)
        self.iface.addPluginToMenu(self.menu, action)

    def unload(self):
        for action in self.actions:
            self.iface.removePluginMenu('QGIS ROS', action)
            self.iface.removeToolBarIcon(action)
        del self.toolbar  # Deref to ensure C++ cleanup.

    def run(self):
        self.dialog.show()
        self.dialog.exec_()
