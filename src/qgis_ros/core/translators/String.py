from std_msgs.msg import String
from .translator import Translator, TableTranslatorMixin


class StringTranslator(Translator, TableTranslatorMixin):

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
