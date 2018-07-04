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
    def createLayer(cls, name, subscribe=False):
        if subscribe:
            uri = '{}?type={}&index=no'.format(name, cls.messageType._type)
            return QgsVectorLayer(uri, name, 'rosvectorprovider')
        else:
            # TODO: poll with timeout for one message
            # TODO: create a `memory` layer
            # TODO: use translator to populate the memory layer.
            pass


class RasterTranslatorMixin(object):

    dataType = 'Raster'

    @classmethod
    def createLayer(cls, topicName, subscribe=False, initialMessage=None):
        if subscribe:
            raise RuntimeError('Cannot subscribe to Raster layers. Not implemented yet.')

        rasterFileName = cls.translate(initialMessage, topicName)
        return QgsRasterLayer(rasterFileName, topicName)
