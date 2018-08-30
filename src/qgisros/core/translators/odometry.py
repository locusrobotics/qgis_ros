from nav_msgs.msg import Odometry
from .translator import Translator, VectorTranslatorMixin


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
                'yaw': 1.0,  # TODO
                'stamp': msg.header.stamp.to_sec(),
            }
        }]
