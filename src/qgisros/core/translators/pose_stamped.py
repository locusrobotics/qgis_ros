from geojson import Point, Feature
from geometry_msgs.msg import PoseStamped
from .translator import Translator, VectorTranslatorMixin


class PoseStampedTranslator(Translator, VectorTranslatorMixin):

    messageType = PoseStamped
    geomType = Translator.GeomTypes.Point

    @staticmethod
    def translate(msg):  # TODO: More DRY.
        p = Point((msg.pose.position.x, msg.pose.position.y))
        f = Feature(geometry=p, properties={'yaw': 1.0})  # TODO YAW
        return [f]  # List of features derived from the message.
