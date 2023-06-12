from data_collection.tests.integration.src.logger import SDVLogger
from data_collection.tests.integration.src.clients.docker.base_docker_client import BaseDockerClient
from data_collection.tests.integration.src.config.config import CONFIG

LOGGER = SDVLogger.get_logger("DataCollectorClient")


class DataCollectorDockerClient(BaseDockerClient):
    container_name = "raspberry-data-collection-service"
    tag = CONFIG.Commit_ID

    def __init__(self, config) -> None:
        super().__init__()
        self.config = config
        self.volumes = None

    def pull(self):
        LOGGER.info(f"Updating {self.container_name} image with tag '{DataCollectorDockerClient.tag}'"),

        repository = f"{CONFIG.Account_ID}.dkr.ecr.{CONFIG.AWS_REGION}.amazonaws.com/{self.container_name}"
        self.client.images.pull(repository, DataCollectorDockerClient.tag)

        return self

    def map_volume(self, volumes):
        LOGGER.info(f"Mapping volumes: {volumes} to container '{self.container_name}'")
        self.volumes = volumes

        return self

    def run(self):
        LOGGER.info(f"Starting '{self.container_name}' container")
        self.container = self.client.containers.run(
            f"{CONFIG.Account_ID}.dkr.ecr.{CONFIG.AWS_REGION}.amazonaws.com/{self.container_name}:{self.tag}",
            detach=True,
            network_mode='host',
            auto_remove=True,
            name=self.container_name,
            volumes=self.volumes,
    # std::ifstream config_file("/var/configs/main_config.json");

        )

        return self

    def ensure_healthy(self, timeout=CONFIG.DOCKER_START_TIMEOUT):
        msg = 'connected: 127.0.0.1:5555'
        res = self.expects_log_message(msg, timeout)
        if res is True:
            LOGGER.info(f"container {self.container_name} is healthy")
        else:
            LOGGER.warning(f"container {self.container_name} is unhealthy")

        return self

        

    def ensure_connected(self, timeout=CONFIG.DOCKER_START_TIMEOUT):
        msgs = [
            'sending: {"action":"subscribe","topic":"/data_collection/query_data/temp"}',
            'sending: {"action":"subscribe","topic":"/data_collection/query_data/storage"}',
            'sending: {"action":"subscribe","topic":"/data_collection/query_data/ram"}',
        ]
        
        res = True
        for msg in msgs:
            res = self.expects_log_message(msg, timeout)
        
        if res is True:
            LOGGER.info(f"container {self.container_name} is connected to aws-client")
        else:
            LOGGER.warning(f"container {self.container_name} is not connected to aws-client")
        
        return self