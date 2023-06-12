import boto3


class BaseClient:

    def __init__(self) -> None:
        self.iot_client = boto3.client('iot')

    def connect(self):
        raise NotImplementedError("Method needs to be implemented")
    
    def send_message(self, topic, data):
        raise NotImplementedError("Method needs to be implemented")

    def read_message(self, topic):
        raise NotImplementedError("Method needs to be implemented")

    def terminate(self):
        raise NotImplementedError("Method needs to be implemented")