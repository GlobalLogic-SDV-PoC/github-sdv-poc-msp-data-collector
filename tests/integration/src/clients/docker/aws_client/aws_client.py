from data_collection.tests.integration.src.logger import SDVLogger
from data_collection.tests.integration.src.clients.docker.base_docker_client import BaseDockerClient
from data_collection.tests.integration.src.config.config import CONFIG
import time


LOGGER = SDVLogger.get_logger("AWSDockerClient")

class AWSDockerClient(BaseDockerClient):
    container_name = "raspberry-aws-client"
    tag = 'latest'

    VOLUMES = {
        # 'config': '/var/configs/main_config.json',
        'config': '/var/configs',
        'certs': '/var/security/',
    }

    def __init__(self, thing_name, config) -> None:
        super().__init__()
        self.thing_name = thing_name
        self.config = config
        self.volumes = None

    def map_volume(self, volumes):
        LOGGER.info(f"Mapping volumes: {volumes}")

        self.volumes = volumes
        
        return self

    def pull(self):
        LOGGER.info(f"Updating {self.container_name} image with tag '{AWSDockerClient.tag}'"),

        repository = f"{CONFIG.Account_ID}.dkr.ecr.{CONFIG.AWS_REGION}.amazonaws.com/{self.container_name}"
        self.client.images.pull(repository, AWSDockerClient.tag)

        return self

    def run(self):
        LOGGER.info(f"Starting {self.container_name} container")
        self.container = self.client.containers.run(
            f"{CONFIG.Account_ID}.dkr.ecr.{CONFIG.AWS_REGION}.amazonaws.com/{self.container_name}:{self.tag}",
            detach=True,
            auto_remove=True,
            name=self.container_name,
            tty=True,
            init=True,
            ports={'5555': '5555'},
            volumes=self.volumes,
        )
        
        return self
    
    def ensure_healthy(self, timeout=CONFIG.DOCKER_START_TIMEOUT):
        LOGGER.info(f"Ensure container '{self.container_name}' healthy")

        msg = "Connected."
        res = self.expects_log_message(msg, timeout)
        if res is True:
            LOGGER.info(f"container {self.container_name} is healthy. '{msg}' is found")
        else:
            LOGGER.warning(f"container {self.container_name} is unhealthy. '{msg}' is not found")

        # msg = 'Exception ocurred trying to set connection promise (likely already set)'
        # res = self.expects_log_message(msg, 30)
        # if res is False:
        #     LOGGER.info(f"container {self.container_name} is healthy. '{msg}' is not found")
        # else:
        #     LOGGER.warning(f"container {self.container_name} is unhealthy. '{msg}' is found")


        return self