# TODO: monkeypatch function.
import sys
sys.argv = []


def classFactory(iface):
    from .qgis_ros_toolbox import QgisRos
    return QgisRos(iface)
