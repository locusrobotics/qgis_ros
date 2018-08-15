from geojson import Point, Feature
from nav_msgs.msg import Odometry
from .translator import Translator, VectorTranslatorMixin


class OdometryTranslator(Translator, VectorTranslatorMixin):

    messageType = Odometry
    geomType = Translator.GeomTypes.Point

    @staticmethod
    def translate(msg):
        properties = {
            'yaw': 1.0,  # TODO
            'stamp': msg.header.stamp,

        }
        p = Point((msg.pose.pose.position.x, msg.pose.pose.position.y))
        f = Feature(geometry=p, properties=properties)
        return f  # List of features derived from the message.
