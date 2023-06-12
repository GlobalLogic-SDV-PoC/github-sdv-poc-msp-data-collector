import json

from src.config.providers.base_config import BaseConfigKeyProvider


# BaseConfigKeyProvider usage is optional
class ConfigFromSimpleJsonProvider(BaseConfigKeyProvider):
    """
    Allows configuration through the JSON file
    """

    def __init__(self, config_path):
        """
        :param config_path: path to the json with configuration
        """
        with open(config_path) as json_file:
            self._config_data = json.load(json_file)

    def get(self, key):
        """
        Returns config value for the given key
        :param str key: Key to retrieve
        """
        val = self._config_data.get(key)

        return val
