import rosbag

from . import TranslatorRegistry


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
