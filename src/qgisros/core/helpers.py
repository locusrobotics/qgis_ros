from collections import namedtuple
import json

from qgis.PyQt.QtCore import QTextCodec
from qgis.core import QgsJsonUtils

import rosbag


TopicMetadata = namedtuple('TopicMetadata', 'name type count')


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
    '''Returns a tuple of TopicMetadata elements from all topics found in bag.'''
    with rosbag.Bag(filePath, 'r') as bag:
        topicTuples = bag.get_type_and_topic_info()[1]  # Tuple entry 1 is a dict of metadata.

    t = [TopicMetadata(k, v.msg_type, v.message_count) for k, v in topicTuples.items()]
    t.sort(key=lambda k: k.name)
    return t


def getBagData(filePath, topicName):
    '''Returns a collection of messages from a bag.'''
    with rosbag.Bag(filePath, 'r') as bag:
        return [message for topic, message, time in bag.read_messages(topics=topicName)]
