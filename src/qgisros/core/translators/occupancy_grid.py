import uuid
import subprocess
from catkin.find_in_workspaces import find_in_workspaces as catkin_find
from nav_msgs.msg import OccupancyGrid
from .translator import Translator, RasterTranslatorMixin


class OccupancyGridTranslator(Translator, RasterTranslatorMixin):
    '''A really terrible implementation to generate rasters from ROS topics.

    This is because of an unresolved issue where GDAL's underlying C++ implementation crashes unexplainably
    when used in this context. As its own process it works fine.
    https://issues.qgis.org/issues/19252
    '''

    messageType = OccupancyGrid

    @staticmethod
    def translate(msg, topicName):
        script = catkin_find(project='qgis_ros', path='scripts/topic_to_geotiff.py', first_match_only=True)[0]
        outfile = '/tmp/{}.tif'.format(uuid.uuid4())

        fail = subprocess.call([script, topicName, outfile])
        if fail:
            raise RuntimeError('Could not create the geotiff.')

        return outfile
