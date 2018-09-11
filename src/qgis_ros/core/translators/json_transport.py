import json_transport
from .translator import Translator, VectorTranslatorMixin


class JSONTransportTranslator(Translator, VectorTranslatorMixin):

    messageType = json_transport.PackedJson

    # geomType = Translator.GeomTypes.Unknown  # Need to detect this from the first message.
    geomType = Translator.GeomTypes.Polygon  # TODO: revert this.

    @staticmethod
    def translate(msg):
        # Attempt to detect GeoJSON in a JSON message.
        msg = msg.data

        if isinstance(msg, list):
            geojson_msg = msg
        elif isinstance(msg, dict) and msg.get('type') == 'FeatureCollection':
            geojson_msg = msg.get('features')
        else:
            raise ValueError('JSON message is not valid GeoJSON.')

        return geojson_msg
