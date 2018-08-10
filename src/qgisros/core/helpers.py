from qgis.PyQt.QtCore import QTextCodec
from qgis.core import QgsJsonUtils


def featureCollectionToQgs(featureCollection):
    '''Accepts a geojson Feature Collection and returns a tuple of (features, fields)'''

    codec = QTextCodec.codecForName("UTF-8")
    fields = QgsJsonUtils.stringToFields(featureCollection, codec)
    features = QgsJsonUtils.stringToFeatureList(featureCollection, fields, codec)

    return (features, fields)


def parseUrlArgs(argString):
    args = argString.split('&')  # list of arg strings.
    args = dict([a.split('=') for a in args])  # dict of args.

    # Convert string booleans.
    for k, v in args.items():
        if v.lower() in ('yes', 'true'):
            args[k] = True
        elif v.lower() in ('no', 'false'):
            args[k] = False

    return args
