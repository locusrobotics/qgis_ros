from geometry_msgs.msg import Pose2D, PoseStamped
from .translator import Translator, VectorTranslatorMixin
from ..helpers import quaternionToYaw


class Pose2DTranslator(Translator, VectorTranslatorMixin):

    messageType = Pose2D
    geomType = Translator.GeomTypes.Point

    @staticmethod
    def translate(msg):
        return [{
            'type': 'Feature',
            'geometry': {
                'type': 'Point',
                'coordinates': [msg.x, msg.y]
            },
            'properties': {
                'theta': msg.theta
            }
        }]


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
