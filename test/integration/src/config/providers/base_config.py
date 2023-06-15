class BaseConfigKeyProvider:
    """
    Base class for config providers,
    should not be used directly
    """

    def get(self, key):
        """
        Returns config value for the given key
        :param str key: Key to retrieve
        """
        raise NotImplementedError("Not implemented")
