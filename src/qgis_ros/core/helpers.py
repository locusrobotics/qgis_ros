import json
import math

from qgis.PyQt.QtCore import QTextCodec
from qgis.core import QgsJsonUtils


def featuresToQgs(features):
    '''Accepts a list of geojson Features and returns a tuple of (features, fields)'''
    fc = {'type': 'FeatureCollection', 'features': features}
    fcString = json.dumps(fc)
    codec = QTextCodec.codecForName("UTF-8")
    fields = QgsJsonUtils.stringToFields(fcString, codec)
    features = QgsJsonUtils.stringToFeatureList(fcString, fields, codec)

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


def quaternionToYaw(q):
    '''Returns the yaw in radians of a quaternion.
    Reimplements part of euler_from_quaternion from the tf package because tf doesn't play well in Python 3.
    '''
    t0 = 2.0 * (q.w * q.z + q.x * q.y)
    t1 = 1.0 - 2.0 * (q.y**2 + q.z**2)
    return math.atan2(t0, t1)
