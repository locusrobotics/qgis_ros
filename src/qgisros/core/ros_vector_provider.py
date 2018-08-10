# Developed based on: https://github.com/qgis/QGIS/blob/master/tests/src/python/provider_python.py
from threading import RLock
import rospy
from qgis.core import (
    QgsFields,
    QgsVectorLayer,
    QgsFeatureRequest,
    QgsFeature,
    QgsGeometry,
    QgsProject,
    QgsExpression,
    QgsExpressionContext,
    QgsExpressionContextUtils,
    QgsCoordinateTransform,
    QgsRectangle,
    QgsVectorDataProvider,
    QgsAbstractFeatureSource,
    QgsAbstractFeatureIterator,
    QgsFeatureIterator,
    QgsSpatialIndex,
    QgsDataProvider,
    QgsCsException
)

from .crs import simpleCrs
from .translator_registry import TranslatorRegistry
from .helpers import featureCollectionToQgs, parseUrlArgs

# TODO: Make a non-global version.
# This limits all ROSVectorProvider classes from updating the canvas more than once per second.
DATA_UPDATE_THROTTLE = 1.0  # float seconds
global last_global_refresh
last_global_refresh = 0


class ROSVectorProvider(QgsVectorDataProvider):

    @classmethod
    def providerKey(cls):
        return 'rosvectorprovider'

    @classmethod
    def description(cls):
        return 'ROS Vector Provider'

    @classmethod
    def createProvider(cls, uri, providerOptions):
        return ROSVectorProvider(uri, providerOptions)

    def __init__(self, uri='', providerOptions=QgsDataProvider.ProviderOptions()):
        '''Set up the Data Provider.

        uri contains the topic name, topic type.
        Example uri: 'foo/my_pose?type=geometry_msgs/PoseStamped'
        '''
        try:
            self._topic, argString = uri.split('?')
        except IndexError:
            raise ValueError('uri Cannot be parsed. Is it valid? uri: {}'.format(uri))

        # Parse string of arguments into dict of python types.
        args = parseUrlArgs(argString)
        super().__init__(uri)

        self._translator = TranslatorRegistry.instance().get(args['type'])

        # There's no source for a reasonable collection of native types so we just steal from another provider.
        mlayer = QgsVectorLayer('Polygon?crs=epsg:4326', 'ml', 'memory')  # Wrong url but doesn't matter.
        nativeTypes = mlayer.dataProvider().nativeTypes()
        self.setNativeTypes(nativeTypes)

        self._uri = uri
        self._fields = QgsFields()
        self._wkbType = self._translator.geomType  # TODO: if unknown, infer it.
        self._features = {}
        self._extent = QgsRectangle()
        self._extent.setMinimal()
        self._subset_string = ''
        self._spatialindex = None
        self._provider_options = providerOptions
        self.next_feature_id = 0  # TODO: Is there a more contained approach for numbering? Generator?
        self._lock = RLock()
        self._subscriber = None

        if args.get('index'):
            self.createSpatialIndex()

        if args.get('subscribe'):
            self._subscriber = rospy.Subscriber(self._topic, self._translator.messageType, self._handle_message)
        else:
            msg = rospy.wait_for_message(self._topic, self._translator.messageType, timeout=5)
            self._handle_message(msg)

    def _handle_message(self, msg):
        featureCollection = self._translator.translate(msg)
        features, fields = featureCollectionToQgs(featureCollection)

        self._fields = fields

        try:
            self._setFeatures(features)
        except RuntimeError:
            self._cleanup()

        # Throttle data update to avoid over-stressing QGIS runtime. Consider alleviating this.
        global last_global_refresh
        now = rospy.get_time()
        if now - last_global_refresh > DATA_UPDATE_THROTTLE:
            last_global_refresh = now
            self.dataChanged.emit()  # TODO: remove flicker when this happens.

    def _cleanup(self):
        ''' Clean up ROS subscriber connection.
        The provider is owned by the QgsVectorLayer, which is owned by QGIS internals (layer registry)
        When a layer is removed, it gets deleted and cleaned up. However, there's no clean way to perform
        cleanup activities on the Python side before the underlying C++ object is deleted, thus making this
        object unstable.  A RuntimeError is raised when this is detected. We'll perform cleanup at that point.
        '''
        if self._subscriber:
            self._subscriber.unregister()
            self._subscriber = None

    def featureSource(self):
        with self._lock:
            return ROSVectorFeatureSource(self)

    def dataSourceUri(self, expandAuthConfig=True):
        return self._uri

    def storageType(self):
        return "ROS Topic"

    def getFeatures(self, request=QgsFeatureRequest()):
        with self._lock:
            return QgsFeatureIterator(ROSVectorFeatureIterator(ROSVectorFeatureSource(self), request))

    def uniqueValues(self, fieldIndex, limit=1):
        with self._lock:
            results = set()
            if fieldIndex >= 0 and fieldIndex < self._fields.count():
                req = QgsFeatureRequest()
                req.setFlags(QgsFeatureRequest.NoGeometry)
                req.setSubsetOfAttributes([fieldIndex])
                for f in self.getFeatures(req):
                    results.add(f.attributes()[fieldIndex])
            return results

    def wkbType(self):
        return self._wkbType

    def featureCount(self):
        with self._lock:
            if not self.subsetString():
                return len(self._features)
            else:
                req = QgsFeatureRequest()
                req.setFlags(QgsFeatureRequest.NoGeometry)
                req.setSubsetOfAttributes([])
                return len([f for f in self.getFeatures(req)])

    def fields(self):
        with self._lock:
            return self._fields

    def _setFeatures(self, flist, flags=None):
        with self._lock:
            added = False
            f_added = []

            if self._spatialindex is not None:
                for f in self._features.values():
                    self._spatialindex.deleteFeature(f)

            self.next_feature_id = 0
            self._features = {}

            for _f in flist:
                self._features[self.next_feature_id] = _f
                _f.setId(self.next_feature_id)
                self.next_feature_id += 1
                added = True
                f_added.append(_f)

            if self._spatialindex is not None:
                self._spatialindex.insertFeature(_f)

            if f_added:
                self.clearMinMaxCache()
                self.updateExtents()

            return added, f_added

    def allFeatureIds(self):
        with self._lock:
            return list(self._features.keys())

    def subsetString(self):
        return self._subset_string

    def setSubsetString(self, subsetString):
        if subsetString == self._subset_string:
            return True
        self._subset_string = subsetString
        self.updateExtents()
        self.clearMinMaxCache()
        self.dataChanged.emit()
        return True

    def supportsSubsetString(self):
        return True

    def createSpatialIndex(self):
        if self._spatialindex is None:
            self._spatialindex = QgsSpatialIndex()
            for f in self._features.values():
                self._spatialindex.insertFeature(f)
        return True

    def capabilities(self):
        return QgsVectorDataProvider.SelectAtId

    def name(self):
        return self.providerKey()

    def extent(self):
        if self._extent.isEmpty() and self._features:
            self._extent.setMinimal()
            if not self._subset_string:
                # fast way - iterate through all features
                for feat in self._features.values():
                    if feat.hasGeometry():
                        self._extent.combineExtentWith(feat.geometry().boundingBox())
            else:
                for f in self.getFeatures(QgsFeatureRequest().setSubsetOfAttributes([])):
                    if f.hasGeometry():
                        self._extent.combineExtentWith(f.geometry().boundingBox())

        elif not self._features:
            self._extent.setMinimal()
        return QgsRectangle(self._extent)

    def updateExtents(self):
        self._extent.setMinimal()

    def isValid(self):
        return True

    def crs(self):
        return simpleCrs


class ROSVectorFeatureIterator(QgsAbstractFeatureIterator):

    def __init__(self, source, request):
        super().__init__(request)
        self._request = request if request is not None else QgsFeatureRequest()
        self._source = source
        self._index = 0
        self._transform = QgsCoordinateTransform()
        if self._request.destinationCrs().isValid() and self._request.destinationCrs() != self._source._provider.crs():
            self._transform = QgsCoordinateTransform(
                self._source._provider.crs(),
                self._request.destinationCrs(),
                self._request.transformContext()
            )
        try:
            self._filter_rect = self.filterRectToSourceCrs(self._transform)
        except QgsCsException as e:
            self.close()
            return
        self._filter_rect = self.filterRectToSourceCrs(self._transform)
        if not self._filter_rect.isNull():
            self._select_rect_geom = QgsGeometry.fromRect(self._filter_rect)
            self._select_rect_engine = QgsGeometry.createGeometryEngine(self._select_rect_geom.constGet())
            self._select_rect_engine.prepareGeometry()
        else:
            self._select_rect_engine = None
            self._select_rect_geom = None
        self._feature_id_list = None
        if self._filter_rect is not None and self._source._provider._spatialindex is not None:
            self._feature_id_list = self._source._provider._spatialindex.intersects(self._filter_rect)

    def fetchFeature(self, f):
        """fetch next feature, return true on success"""
        # virtual bool nextFeature( QgsFeature &f );
        if self._index < 0:
            f.setValid(False)
            return False
        try:
            found = False
            while not found:
                _f = self._source._features[list(self._source._features.keys())[self._index]]
                self._index += 1

                if self._feature_id_list is not None and _f.id() not in self._feature_id_list:
                    continue

                if not self._filter_rect.isNull():
                    if not _f.hasGeometry():
                        continue
                    if self._request.flags() & QgsFeatureRequest.ExactIntersect:
                        # do exact check in case we're doing intersection
                        if not self._select_rect_engine.intersects(_f.geometry().constGet()):
                            continue
                    else:
                        if not _f.geometry().boundingBox().intersects(self._filter_rect):
                            continue

                self._source._expression_context.setFeature(_f)
                if self._request.filterType() == QgsFeatureRequest.FilterExpression:
                    if not self._request.filterExpression().evaluate(self._source._expression_context):
                        continue
                if self._source._subset_expression:
                    if not self._source._subset_expression.evaluate(self._source._expression_context):
                        continue
                elif self._request.filterType() == QgsFeatureRequest.FilterFids:
                    if not _f.id() in self._request.filterFids():
                        continue
                elif self._request.filterType() == QgsFeatureRequest.FilterFid:
                    if _f.id() != self._request.filterFid():
                        continue
                f.setGeometry(_f.geometry())
                self.geometryToDestinationCrs(f, self._transform)
                f.setFields(_f.fields())
                f.setAttributes(_f.attributes())
                f.setValid(_f.isValid())
                f.setId(_f.id())
                return True
        except IndexError as e:
            f.setValid(False)
            return False

    def __iter__(self):
        """Returns self as an iterator object"""
        self._index = 0
        return self

    def __next__(self):
        """Returns the next value till current is lower than high"""
        f = QgsFeature()
        if not self.nextFeature(f):
            raise StopIteration
        else:
            return f

    def rewind(self):
        """reset the iterator to the starting position"""
        # virtual bool rewind() = 0;
        if self._index < 0:
            return False
        self._index = 0
        return True

    def close(self):
        """end of iterating: free the resources / lock"""
        # virtual bool close() = 0;
        self._index = -1
        return True


class ROSVectorFeatureSource(QgsAbstractFeatureSource):

    def __init__(self, provider):
        super(ROSVectorFeatureSource, self).__init__()
        self._provider = provider
        self._features = provider._features

        self._expression_context = QgsExpressionContext()
        self._expression_context.appendScope(QgsExpressionContextUtils.globalScope())
        self._expression_context.appendScope(QgsExpressionContextUtils.projectScope(QgsProject.instance()))
        self._expression_context.setFields(self._provider.fields())
        if self._provider.subsetString():
            self._subset_expression = QgsExpression(self._provider.subsetString())
            self._subset_expression.prepare(self._expression_context)
        else:
            self._subset_expression = None

    def getFeatures(self, request):
        return QgsFeatureIterator(ROSVectorFeatureIterator(self, request))
