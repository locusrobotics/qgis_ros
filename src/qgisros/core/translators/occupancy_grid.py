import os
import uuid
from pathlib import Path
import subprocess

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
        script = str(Path(os.path.dirname(os.path.realpath(__file__))) / '..' / '..' / 'scripts' / 'topic_to_geotiff.py')
        outfile = '/tmp/{}.tif'.format(uuid.uuid4())

        fail = subprocess.call([script, topicName, outfile])
        if fail:
            raise RuntimeError('Could not create the geotiff.')

        return outfile
