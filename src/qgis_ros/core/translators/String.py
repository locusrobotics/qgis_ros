from std_msgs.msg import String
from .translator import Translator, VectorTranslatorMixin


class StringTranslator(Translator, VectorTranslatorMixin):

    messageType = String
    geomType = Translator.GeomTypes.NoGeometry

    @staticmethod
    def translate(msg):
        return [{
            'type': 'Feature',
            'properties': {
                'data': msg.data
            }
        }]
