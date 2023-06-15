import json
import os
from data_collection.tests.integration.src.logger import SDVLogger
from data_collection.tests.integration.src.helpers.helpers import write_to_file


LOGGER = SDVLogger.get_logger("CONFIG_DC")


class ConfigDCServiceProvider:
    CONFIG_NAME = 'main_config.json'
    
    VOLUME_RAM_PATH = '/var/testdata/ram'
    VOLUME_TEMP_PATH = '/var/testdata/temp'
    VOLUME_STORAGE_PATH = '/var/testdata/storage'
    VOLUME_CONFIG_PATH = '/var/configs'

    def __init__(self) -> None:
        self.source_conf = None
        self.config_path = None
        self.ram_path = None
        self.temp_path = None
        self.storage_path = None
        self.expected_temp_value = None
        self.expected_storage_value = None
        self.expected_temp_value = None

    def init_config(self):
        crt_folder_path = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(crt_folder_path, ConfigDCServiceProvider.CONFIG_NAME)
        f = open(config_path)
        self.source_conf = json.load(f)

        return self

    def set_scheduled_message_send(self, state):
        LOGGER.info(f"Set new collectors/temp/enabled to: '{state}'")
        self.source_conf['collectors']['temp']['enabled'] = state

        LOGGER.info(f"Set new collectors/storage/enabled to: '{state}'")
        self.source_conf['collectors']['storage']['enabled'] = state

        LOGGER.info(f"Set new collectors/ram/enabled to: '{state}'")
        self.source_conf['collectors']['ram']['enabled'] = state

        return self
    
    def genarate_ram_file(self, path, value):
        LOGGER.info(f"Generating RAM file for tests in path: '{path}'")

        self.expected_ram_value = value

        content = "MemTotal:32644220 kB\nMemFree:985060 kB\nMemAvailable:23372996 kB" 
        self.ram_path = write_to_file(path, content)
        
        return self

    def set_config_ram_path(self, path):
        LOGGER.info(f"Set new collectors/ram/extract_path to: '{path}'")
        self.source_conf['collectors']['ram']['extract_path'] = path

        return self

    def genarate_storage_file(self, path, value):
        LOGGER.info(f"Generating STORAGE file for tests in path: '{path}'")

        self.storage_path = path
        self.expected_storage_value = value

        return self

    def set_config_storage_path(self, path):
        LOGGER.info(f"Set new collectors/storage/extract_path to: '{path}'")
        self.source_conf['collectors']['storage']['extract_path'] = path

        return self

    def genarate_temp_file(self, path, value):
        LOGGER.info(f"Generating TEMP file for tests in path: '{path}'")

        content = value * 1000 
        self.expected_temp_value = content
        
        self.temp_path = write_to_file(path, content)
        
        return self

    def set_config_temp_path(self, path):
        LOGGER.info(f"Set new collectors/temp/extract_path to: '{path}'")
        self.source_conf['collectors']['temp']['extract_path'] = path

        return self

    def generate(self, path_prefix):
        LOGGER.info(f"Generating new config file in folder '{path_prefix}'")

        if not os.path.exists(path_prefix):
            os.makedirs(path_prefix)
        config_path = os.path.join(path_prefix, ConfigDCServiceProvider.CONFIG_NAME)

        with open(config_path, 'w') as fp:
            json.dump(self.source_conf, fp)

        self.config_path = os.path.dirname(
            os.path.abspath(config_path)
        )

        return self