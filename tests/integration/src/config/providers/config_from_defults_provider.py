from src.config.providers.base_config import BaseConfigKeyProvider


# BaseConfigKeyProvider usage is optional
class ConfigFromDefaultsProvider(BaseConfigKeyProvider):
    """
    Allows configuration through the env variables.
    """

    def __init__(self, props) -> None:
        self.props = props

    def get(self, key):
        """
        Returns config value for the given key
        :param str key: Key to retrieve
        """
        return self.props.get(key)
