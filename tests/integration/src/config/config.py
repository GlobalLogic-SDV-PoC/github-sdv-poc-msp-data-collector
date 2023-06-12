import os
import sys

from src.config.converters.base_converter import BaseConverter
from src.config.converters.bool_converter import BoolConverter
from src.config.converters.int_converter import IntConverter
from src.config.converters.string_converter import StringConverter
from src.config.providers.config_from_defults_provider import ConfigFromDefaultsProvider
from src.config.providers.config_from_env_provider import ConfigFromEnvProvider
from src.config.providers.config_from_json_provider import ConfigFromSimpleJsonProvider


class Config:
    DEFAULT_ENV = "raspberry"

    def __init__(self) -> None:
        self.conf_dict = {}

        target = os.environ.get("TARGET")
        if target is None:
            target = Config.DEFAULT_ENV

        # json_path = f"src/config/env_configs/{target}.json" #enable once needed
        env_file = os.path.join("data_collection","tests", "integration", "src", ".env")

        # Hierarhy of providers
        self.providers = [
            ConfigFromEnvProvider(env_file),
            # ConfigFromSimpleJsonProvider(json_path), #enable once needed
            ConfigFromDefaultsProvider(
                {
                    "TIMEOUT_COMMON": 30,
                    "PYTHON_VERSION": f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}",
                    "DOCKER_START_TIMEOUT": 60,
                    "AWS_CERTS_FOLDER": os.path.join("data_collection","tests", "integration", "docker_volumes", "aws", "certs"),
                    "AWS_CONFIG_FOLDER": os.path.join("data_collection","tests", "integration", "docker_volumes", "aws", "configs"),
                    "DC_CONFIG_FOLDER": os.path.join("data_collection","tests", "integration", "docker_volumes", "dc", "configs"),
                    "RAM_FILE": os.path.join("data_collection","tests", "integration", "docker_volumes", "dc", "configs", "ram"),
                    "STORAGE_FILE": os.path.join("data_collection","tests", "integration", "docker_volumes", "dc", "configs", "storage"),
                    "TEMP_FILE": os.path.join("data_collection","tests", "integration", "docker_volumes", "dc", "configs", "temp"),
                
                }
            ),
        ]

        self.register("TIMEOUT_COMMON", IntConverter)
        self.register("PYTHON_VERSION", StringConverter)
        self.register("DOCKER_START_TIMEOUT", IntConverter)
        self.register("AWS_REGION", StringConverter)
        self.register("Account_ID", StringConverter)
        self.register("Commit_ID", StringConverter)
        self.register("AWS_CERTS_FOLDER", StringConverter)
        self.register("AWS_CONFIG_FOLDER", StringConverter)
        self.register("DC_CONFIG_FOLDER", StringConverter)
        self.register("RAM_FILE", StringConverter)
        self.register("STORAGE_FILE", StringConverter)
        self.register("TEMP_FILE", StringConverter)
        

    def register(self, name: str, converter: BaseConverter):
        """
        Register name of the key which is used
        in tests
        """

        # Order in self.provider makes difference
        for provider in self.providers:
            val = provider.get(name)

            if val is not None:
                self.conf_dict[name] = converter.convert(val)
                break

        # raise error if no value is found across the providers
        val = self.conf_dict.get(name)
        if val is None:
            raise Exception(f"{name} variable cannot be found in registered config providers")

        print(f"{name} variable is registered in config with value {val} and type {converter.CLASS}")

    def __getattr__(self, name):
        """
        Return existing value
        """

        val = self.conf_dict.get(name)
        if val is None:
            raise Exception(f"{name} variable is not registered in config")

        return self.conf_dict.get(name)


# python way singleton
CONFIG = Config()
