from qgis.core import QgsVectorLayer, QgsRasterLayer, QgsWkbTypes


class Translator(object):
    '''A base class for all translators.
    Reimplement `createLayer` to return a new QgsVectorLayer or QgsRasterLayer with the desired data provider.
    Or use one of the existing mixins: VectorTranslatorMixin or RasterTranslatorMixin.

    Reimplement `translate` to accept a ROS message and return a GeoJSON object or filename of GeoTiff.
    '''

    GeomTypes = QgsWkbTypes

    messageType = None
    dataType = None
    geomType = GeomTypes.Unknown

    # @classmethod
    # def createLayer(cls, name, subscribe=False):
    #     raise NotImplementedError(str(cls))

    # @classmethod
    # def translate(msg):
    #     raise NotImplementedError()


class VectorTranslatorMixin(object):

    dataType = 'Vector'

    @classmethod
    def createLayer(cls, topic_name, subscribe=False, data=None):
        if data:
            # TODO: data was passed in, create the layer with that.
            pass
        else:
            # Create a QgsVectorLayer that gets data from the rosvectorprovider.
            uri = '{}?type={}&index=no&subscribe={}'.format(topic_name, cls.messageType._type, subscribe)
            return QgsVectorLayer(uri, topic_name, 'rosvectorprovider')


class RasterTranslatorMixin(object):

    dataType = 'Raster'

    @classmethod
    def createLayer(cls, topicName, subscribe=False, initialMessage=None):
        if subscribe:
            raise RuntimeError('Cannot subscribe to Raster layers. Not implemented yet.')
        rasterFileName = cls.translate(initialMessage, topicName)
        return QgsRasterLayer(rasterFileName, topicName)
