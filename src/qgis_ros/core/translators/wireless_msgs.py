from wireless_msgs.msg import Connection
from .translator import Translator, TableTranslatorMixin


class ConnectionTranslator(Translator, TableTranslatorMixin):

    messageType = Connection
    geomType = Translator.GeomTypes.NoGeometry

    @staticmethod
    def translate(msg):

        # Some forks of wireless_msgs/Connection have a header.
        try:
            seq = msg.header.seq
            stamp = msg.header.stamp.to_sec()
        except AttributeError:
            seq = None
            stamp = None

        return [{
            'type': 'Feature',
            'properties': {
                'bitrate': msg.bitrate,
                'txpower': msg.txpower,
                'link_quality_raw': msg.link_quality_raw,
                'link_quality': msg.link_quality,
                'signal_level': msg.signal_level,
                'noise_level': msg.noise_level,
                'essid': msg.essid,
                'bssid': msg.bssid,
                'frequency': msg.frequency,
                'seq': seq,
                'stamp': stamp
            }
        }]
