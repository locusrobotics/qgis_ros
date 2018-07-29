import json
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
            geojson_msg = {
                'type': 'FeatureCollection',
                'features': msg
            }
        elif isinstance(msg, dict) and msg.get('type') == 'FeatureCollection':
            geojson_msg = msg
        else:
            raise ValueError('JSON message is not valid GeoJSON.')

        return json.dumps(geojson_msg)
