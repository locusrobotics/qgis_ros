import importlib
import os

from qgis.core import QgsMessageLog

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
            raise Exception('Cannot re-instantiate singleton.')  # TODO: better exception type.

        self._translators = {}

        # Register all built-in translators.
        for t in builtinTranslators:
            self.register(t)

        # Register extra translators found in QGIS_ROS_EXTRA_TRANSLATORS.
        translatorPaths = os.environ.get('QGIS_ROS_EXTRA_TRANSLATORS', '').split(',')
        for p in translatorPaths:
            # Skip empty strings.
            if p == '':
                continue

            try:
                m = importlib.import_module(p)
            except ModuleNotFoundError:
                QgsMessageLog.logMessage('Could not find QGIS_ROS_EXTRA_TRANSLATORS module: {}. Skipping.'.format(p))
                continue

            try:
                translators = getattr(m, 'translators')
            except AttributeError:
                QgsMessageLog.logMessage(
                    'Could not find translator for QGIS_ROS_EXTRA_TRANSLATORS module: {}. Skipping.'.format(p))
                continue

            # Register found translators.
            for t in translators:
                self.register(t)

    def register(self, translator):
        # TODO: log every registration.
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
