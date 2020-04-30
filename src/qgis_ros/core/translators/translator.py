from qgis.core import QgsVectorLayer, QgsRasterLayer, QgsWkbTypes
import rospy
from ..crs import proj4CrsDict
from ..helpers import featuresToQgs


class Translator(object):
    '''A base class for all translators.
    Reimplement `createLayer` to return a new QgsVectorLayer or QgsRasterLayer with the desired data provider.
    Or use one of the existing mixins: VectorTranslatorMixin or RasterTranslatorMixin.

    Reimplement `translate` to accept a ROS message and return a GeoJSON object or filename of GeoTiff.
    '''

    GeomTypes = QgsWkbTypes
    messageType = None
    geomType = GeomTypes.Unknown
    crsName = 'simple'


class RasterTranslatorMixin(object):
    '''Translate input raster data to a GeoTIFF format.

    Does not support subscription because we have not implemented a rosrasterprovider.
    '''

    dataModelType = 'Raster'

    @classmethod
    def createLayer(cls, topicName, rosMessages=None, **kwargs):
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


class VectorTranslatorMixin(object):
    '''handles vector topic data.

    Creates and returns a layer using the rosvectorprovider, which knows how to
    subscribe to or capture the latest message from a vector topic.
    '''

    dataModelType = 'Vector'

    @classmethod
    def createLayer(cls, topicName, rosMessages=None, extraProperties=None, subscribe=False, keepOlderMessages=False,
                    sampleInterval=1):
        if rosMessages:
            # Features were passed in, so it's a static data layer.
            geomType = QgsWkbTypes.displayString(cls.geomType)  # Get string version of geomtype enum.
            uri = '{}?crs=PROJ4:{}'.format(geomType, proj4CrsDict[cls.crsName])
            layer = QgsVectorLayer(uri, topicName, 'memory')

            # Convert from ROS messages to GeoJSON Features to QgsFeatures.
            features = []
            for n, m in enumerate(rosMessages):
                translatedFeatures = cls.translate(m)  # Append one or more features.

                # Optionally merge extra properties like bag timestamps in to features created by this message.
                if extraProperties is not None:
                    for f in translatedFeatures:
                        f['properties'].update(extraProperties[n])

                features += translatedFeatures

            qgsFeatures, fields = featuresToQgs(features)
            layer.dataProvider().addAttributes(fields)
            layer.dataProvider().addFeatures(qgsFeatures)
            layer.updateFields()  # Required, otherwise the layer will not re-read field metadata.
            return layer
        else:
            # No features, it must be a ROS topic to get data from.
            uri = '{}?type={}&index=no&subscribe={}&keepOlderMessages={}&sampleInterval={}&crsName={}'.format(
                topicName,
                cls.messageType._type,
                subscribe,
                keepOlderMessages,
                sampleInterval,
                cls.crsName,
            )
            layer = QgsVectorLayer(uri, topicName, 'rosvectorprovider')

            # Need to monitor when data is changed and call updateFields in order to capture the new fields
            # that are discovered on the first and possibly future messages. Without this, the layer will never
            # expose any of the field data available.
            # TODO: Find a cleaner way to signal this update and only call it when actual field changes occur.
            layer.dataChanged.connect(layer.updateFields)
            return layer


class TableTranslatorMixin(VectorTranslatorMixin):
    '''Handles non-spatial attribute data.

    Behaves the same as a VectorTranslatorMixin because we use the same provider to handle
    subscribing and all that ROS stuff. It just naturally supports GeoJSON Feature data without
    geometry.
    '''

    dataModelType = 'Table'
