from geojson import Point, Feature, FeatureCollection, dumps
from geometry_msgs.msg import Pose2D
from .translator import Translator, VectorTranslatorMixin


class Pose2DTranslator(Translator, VectorTranslatorMixin):

    messageType = Pose2D
    geomType = Translator.GeomTypes.Point

    @staticmethod
    def translate(msg):
        p = Point((msg.x, msg.y))
        f = Feature(geometry=p, properties={'theta': msg.theta})
        return dumps(FeatureCollection((f,)))
