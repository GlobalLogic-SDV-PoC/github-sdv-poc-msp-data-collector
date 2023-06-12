import docker
from data_collection.tests.integration.src.logger import SDVLogger
import time 
from data_collection.tests.integration.src.config.config import CONFIG
import subprocess

LOGGER = SDVLogger.get_logger("SDVDockerClient")


class BaseDockerClient:
    container_name = None

    def __init__(self) -> None:
        self.client = docker.from_env()
        self.container = None
        BaseDockerClient.aws_ecr_login()

    @staticmethod
    def aws_ecr_login():
        cmd = f"aws ecr get-login-password --region {CONFIG.AWS_REGION} | docker login --username AWS --password-stdin {CONFIG.Account_ID}.dkr.ecr.{CONFIG.AWS_REGION}.amazonaws.com"
        LOGGER.info(f"Trying to loging with {cmd} command")
        raw_output = subprocess.run(args=cmd, check=True, capture_output=True, shell=True)
        LOGGER.info(f"Raw output: {raw_output}")

    def run(self):
        raise NotImplementedError("Run method needs to be implemented")    
    
    def ensure_stop(self):
        for container in self.client.containers.list():
            LOGGER.info(container.name.lower())
            if container.name.lower() == self.container_name.lower():
                LOGGER.info(f"Container {container.id}:{container.name} is running. Stoping it")
                container.stop()

        return self
    
    def ensure_healthy(self, timeout=CONFIG.DOCKER_START_TIMEOUT):
        raise NotImplementedError("Run method needs to be implemented")    

    def stop(self):
        LOGGER.info(f"Stoping {self.container_name} container")
        if self.container is None:
            LOGGER.warning(f"No active {self.container_name} container running")
            return self
        
        self.container.stop()
        
        return self

    def restart(self):
        if self.container is None:
            LOGGER.warning(f"No active {self.container_name} container running. Starting new one")
            return self.run()
        
        self.container.restart()
        
        return self

    def expects_log_message(self, message, timeout=CONFIG.DOCKER_START_TIMEOUT):
        if self.container is None:
            LOGGER.warning(f"No active {self.container_name} container running")
            return False
        
        timeout = time.time() + timeout
        while time.time() < timeout:
            if str(message) in self.container.logs().decode("utf-8"):
                LOGGER.info(f"Message {message} is found in {self.container_name}")
                return True

        LOGGER.warning(f"Message {message} is not found in {self.container_name}")
        return False

