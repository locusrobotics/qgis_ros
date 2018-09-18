import json
import math

from qgis.PyQt.QtCore import QTextCodec
from qgis.core import QgsJsonUtils

import rosbag

from .translator_registry import TranslatorRegistry


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


def getTopicsFromBag(filePath):
    '''Returns a tuple of tuples (name, type, count) from all topics found in bag.'''
    with rosbag.Bag(filePath, 'r') as bag:
        topicTuples = bag.get_type_and_topic_info()[1]  # Tuple entry 1 is a dict of metadata.

    t = [(k, v.msg_type, v.message_count) for k, v in topicTuples.items()]
    t.sort(key=lambda k: k[0])
    return t


def getBagDataAsLayer(filePath, topicName, topicType, sampleInterval=1, takeLast=False, progressCallback=None):
    '''Returns a collection of messages from a bag.
    Invokes a progress callback every 1000 elements as this can be long running.
    '''
    count = 0
    messages = []
    extraProperties = []

    with rosbag.Bag(filePath, 'r') as bag:
        for topic, message, time in bag.read_messages(topics=topicName):
            count += 1
            if count % sampleInterval == 0:  # Append every sampleInterval message.
                messages.append(message)
                extraProperties.append({'bagTimestamp': time.to_sec()})

            # Invoke progress callback every 1000 messages.
            if count % 1000 == 0 and progressCallback:
                progressCallback(count)

    if takeLast:
        messages = messages[-1]
        extraProperties = extraProperties[-1]

    translator = TranslatorRegistry.instance().get(topicType)
    return translator.createLayer(topicName, rosMessages=messages, extraProperties=extraProperties)


def quaternionToYaw(q):
    '''Returns the yaw in radians of a quaternion.
    Reimplements part of euler_from_quaternion from the tf package because tf doesn't play well in Python 3.
    '''
    t0 = 2.0 * (q.w * q.z + q.x * q.y)
    t1 = 1.0 - 2.0 * (q.y**2 + q.z**2)
    return math.atan2(t0, t1)
