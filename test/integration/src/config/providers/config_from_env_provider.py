import os
from dotenv import load_dotenv

from src.config.providers.base_config import BaseConfigKeyProvider

# BaseConfigKeyProvider usage is optional
class ConfigFromEnvProvider(BaseConfigKeyProvider):
    """
    Allows configuration through the env variables.
    """
    
    def __init__(self, path) -> None:    
        load_dotenv(path)

    def get(self, key):
        """
        Returns config value for the given key
        :param str key: Key to retrieve
        """
        return os.environ.get(key)
