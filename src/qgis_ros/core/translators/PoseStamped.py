from geometry_msgs.msg import PoseStamped
from .translator import Translator, VectorTranslatorMixin
from ..helpers import quaternionToYaw


class PoseStampedTranslator(Translator, VectorTranslatorMixin):

    messageType = PoseStamped
    geomType = Translator.GeomTypes.Point

    @staticmethod
    def translate(msg):
        return [{
            'type': 'Feature',
            'geometry': {
                'type': 'Point',
                'coordinates': [msg.pose.position.x, msg.pose.position.y]
            },
            'properties': {
                'yaw': quaternionToYaw(msg.pose.orientation),
                'stamp': msg.header.stamp.to_sec(),
            }
        }]
