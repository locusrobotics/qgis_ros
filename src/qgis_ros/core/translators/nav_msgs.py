import uuid
import json
from pathlib import Path
import subprocess

from catkin.find_in_workspaces import find_in_workspaces as catkin_find
from nav_msgs.msg import OccupancyGrid, Odometry

from .translator import Translator, RasterTranslatorMixin, VectorTranslatorMixin
from ..helpers import quaternionToYaw


class OdometryTranslator(Translator, VectorTranslatorMixin):

    messageType = Odometry
    geomType = Translator.GeomTypes.Point

    @staticmethod
    def translate(msg):
        return [{
            'type': 'Feature',
            'geometry': {
                'type': 'Point',
                'coordinates': [msg.pose.pose.position.x, msg.pose.pose.position.y]
            },
            'properties': {
                'yaw': quaternionToYaw(msg.pose.pose.orientation),
                'stamp': msg.header.stamp.to_sec(),
            }
        }]


class OccupancyGridTranslator(Translator, RasterTranslatorMixin):
    '''A really terrible implementation to generate rasters from ROS topics.

    This is because of an unresolved issue where GDAL's underlying C++ implementation crashes unexplainably
    when used in this context. As its own process it works fine.
    https://issues.qgis.org/issues/19252
    '''

    messageType = OccupancyGrid

    @staticmethod
    def translate(msg):
        datafile = '/tmp/{}.json'.format(uuid.uuid4())

        data = {
            'height': msg.info.height,
            'width': msg.info.width,
            'resolution': msg.info.resolution,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
            'data': msg.data
        }

        # Save the message data to be read by external translator.
        with open(datafile, 'w') as f:
            json.dump(data, f)

        # Run external translator.
        geotiffFilename = '/tmp/{}.tif'.format(uuid.uuid4())

        script_dir = catkin_find(project='qgis_ros', path='scripts', first_match_only=True)[0]
        rasterize_script = Path(script_dir) / 'msg_to_geotiff.py'

        subprocess.check_call([str(rasterize_script), datafile, geotiffFilename])

        # Return filename.
        return geotiffFilename
