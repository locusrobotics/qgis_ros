from sensor_msgs.msg import NavSatFix
from .translator import Translator, VectorTranslatorMixin


class NavSatFixTranslator(Translator, VectorTranslatorMixin):

    messageType = NavSatFix
    geomType = Translator.GeomTypes.Point
    crsName = 'wgs84'

    @staticmethod
    def translate(msg):
        return [{
            'type': 'Feature',
            'geometry': {
                'type': 'Point',
                'coordinates': [msg.longitude, msg.latitude]
            },
            'properties': {
                'stamp': msg.header.stamp.to_sec(),
            }
        }]
