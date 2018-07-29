from .translators import builtinTranslators


class TranslatorRegistry(object):
    '''A registry of all installed ROS data translators.

    Implemented as singleton to remain consistent with how QGIS handles these kinds of components.
    '''

    _instance = None

    @staticmethod
    def instance():
        if TranslatorRegistry._instance is None:
            TranslatorRegistry()
        return TranslatorRegistry._instance

    def __init__(self):
        if TranslatorRegistry._instance is None:
            TranslatorRegistry._instance = self
        else:
            raise Exception('Cannot re-instantiate singleton.')

        self._translators = {}

        # Register all built-in translators.
        for t in builtinTranslators:
            self.register(t)

        # Register extra translators found in arg.
        # TODO: register extras using importlib.
        # TODO: get envvar ROS_QGIS_EXTRA_TRANSLATORS
        # parse envvar separating by comma
        # for each, try to load it with pathlib and iterate over contents, registering each.

    def register(self, translator):
        if translator.messageType._type in self._translators:
            raise ValueError('Type {} already registered.'.format(translator.messageType._type))
        self._translators[translator.messageType._type] = translator

    @property
    def translatableTypeNames(self):
        return self._translators.keys()

    def get(self, name):
        try:
            return self._translators[name]
        except KeyError:
            raise KeyError('{} not available in list of registered translators.'.format(name))
