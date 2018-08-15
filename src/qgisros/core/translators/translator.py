from qgis.core import QgsVectorLayer, QgsRasterLayer, QgsWkbTypes
import rospy
from ..crs import PROJ4_SIMPLE
from ..helpers import featuresToQgs


class Translator(object):
    '''A base class for all translators.
    Reimplement `createLayer` to return a new QgsVectorLayer or QgsRasterLayer with the desired data provider.
    Or use one of the existing mixins: VectorTranslatorMixin or RasterTranslatorMixin.

    Reimplement `translate` to accept a ROS message and return a GeoJSON object or filename of GeoTiff.
    '''

    GeomTypes = QgsWkbTypes

    messageType = None
    dataModelType = None
    geomType = GeomTypes.Unknown

    # TODO: what are these for again?
    # @classmethod
    # def createLayer(cls, name, subscribe=False):
    #     raise NotImplementedError(str(cls))

    # @classmethod
    # def translate(msg):
    #     raise NotImplementedError()


class VectorTranslatorMixin(object):

    dataModelType = 'Vector'

    @classmethod
    def createLayer(cls, topicName, subscribe=False, rosMessages=None):
        if rosMessages:
            # Features were passed in, so it's a static data layer.
            geomType = QgsWkbTypes.displayString(cls.geomType)  # Get string version of geomtype enum.
            uri = '{}?crs=PROJ4:{}'.format(geomType, PROJ4_SIMPLE)
            layer = QgsVectorLayer(uri, topicName, 'memory')

            # Convert from ROS messages to GeoJSON Features to QgsFeatures.
            features = []
            for m in rosMessages:
                features += cls.translate(m)

            qgsFeatures, fields = featuresToQgs(features)
            layer.dataProvider().addAttributes(fields)
            layer.dataProvider().addFeatures(qgsFeatures)
            return layer
        else:
            # No features, it must be a ROS topic to get data from.
            uri = '{}?type={}&index=no&subscribe={}'.format(topicName, cls.messageType._type, subscribe)
            return QgsVectorLayer(uri, topicName, 'rosvectorprovider')


class RasterTranslatorMixin(object):

    dataModelType = 'Raster'

    @classmethod
    def createLayer(cls, topicName, rosMessages=None):
        '''Creates a raster layer from a ROS message.
        Unlike vector data, raster layers cannot currently be subscribed to.
        '''
        if rosMessages:
            msg = rosMessages[0]
        else:
            msg = rospy.wait_for_message(topicName, cls.messageType, 10)

        geotiffFilename = cls.translate(msg)
        layer = QgsRasterLayer(geotiffFilename, topicName)
        return layer
