from geometry_msgs.msg import Pose2D
from .translator import Translator, VectorTranslatorMixin


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
