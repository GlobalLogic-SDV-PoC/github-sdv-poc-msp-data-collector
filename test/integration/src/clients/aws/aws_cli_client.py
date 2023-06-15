from data_collection.tests.integration.src.base_client import BaseClient
from data_collection.tests.integration.src.helpers.helpers import extract_from_dictionary
from data_collection.tests.integration.src.clients.aws import mqtt_connection_builder
import time
import os
import uuid
import json
from awscrt import mqtt

from data_collection.tests.integration.src.logger import SDVLogger

LOGGER = SDVLogger.get_logger("AWS_CLIENT")


class AWSCliClient(BaseClient):
    THING_TYPE = "base-platform-client"
    THING_GROUP_ARN = "arn:aws:iot:eu-west-1:203647640528:thinggroup/model_alpha"
    THING_POLICY_NAME = "iot_upload_file"
    ENDPOINT = 'a2vups147zkgmn-ats.iot.eu-west-1.amazonaws.com'

    def __init__(self) -> None:
        super().__init__()
        self.thing = None
        self.certs = None
        self.certs_folder = None
        self.mqtt_client_id = str(uuid.uuid4())
        self.mqtt_connection = None
        self.mqtt_message_loop = {}

    @property
    def thing_name(self):
        return self.thing['thingName']

    def create_thing(self, name):
        LOGGER.info(f"Creating thing with '{name}' name")
        self.thing = self.iot_client.create_thing(
            thingName=name,
            thingTypeName=self.THING_TYPE,
        )
        LOGGER.info(f"'{self.thing['thingName']}' thing created")

        return self
    
    def add_thing_to_group(self):
        LOGGER.info(f"Adding thing '{self.thing['thingName']}' to a '{AWSCliClient.THING_GROUP_ARN}' group")
        self.iot_client.add_thing_to_thing_group(
            thingGroupArn=AWSCliClient.THING_GROUP_ARN,
            thingName=self.thing['thingName'],
        )
        return self
    
    def delete_thing(self):
        LOGGER.info(f"Deletting thing '{self.thing['thingName']}'")

        self.iot_client.delete_thing(
            thingName=self.thing['thingName'],
        )
        self.thing = None

        return self
    
    def create_certs(self):
        LOGGER.info("Creating certificates")

        self.certs = self.iot_client.create_keys_and_certificate(
            setAsActive=True
        )
        LOGGER.info(f"Certificates created. Payload {self.certs}")

        return self

    def delete_certs(self):
        LOGGER.info("Trying to remove certs")

        LOGGER.info(f"Set state to INACTIVE for cert '{self.certs['certificateId']}'")
        self.iot_client.update_certificate(
            certificateId=self.certs['certificateId'],
            newStatus='INACTIVE'
        )

        LOGGER.info(f"Removing cert '{self.certs['certificateId']}'")
        self.iot_client.delete_certificate(
            certificateId=self.certs['certificateId'],
            forceDelete=False
        )
        self.certs = None

        return self

    def generate_certs_files(self, path_prefix):
        LOGGER.info(f"Creating certificates files in folder {path_prefix}")

        if self.certs is None:
            raise Exception("Cannot generate certificates because they are not generated. Create them first")

        
        certs_to_generate = {
                    # 'AmazonRootCA1.pem': ['keyPair', 'PublicKey'],            
                    'certificate.pem.crt': ['certificatePem'],
                    'private.pem.key': ['keyPair', 'PrivateKey'],
                }
        
        if not os.path.exists(path_prefix):
            os.makedirs(path_prefix)
        
        for cert_name, dict_path in certs_to_generate.items():
            content = extract_from_dictionary(self.certs, dict_path)
            path = os.path.join(path_prefix, cert_name)

            f = open(path, "w")
            f.write(content)
            f.close()
            LOGGER.info(f"File {path} created")

        self.certs_folder = os.path.abspath(path_prefix)

        return self
    
    def attach_policy_to_cert(self):
        LOGGER.info(f"Attach policy '{AWSCliClient.THING_GROUP_ARN}' to a cert {self.certs['certificateArn']}")
        self.iot_client.attach_policy(
            policyName=AWSCliClient.THING_POLICY_NAME,
            target=self.certs['certificateArn']
        )

        return self

    def detach_policy_from_cert(self):
        LOGGER.info(f"Detach policy '{AWSCliClient.THING_POLICY_NAME}' from  a cert'{self.certs['certificateArn']}'")
        self.iot_client.detach_policy(
            policyName=AWSCliClient.THING_POLICY_NAME,
            target=self.certs['certificateArn']
        )

        return self
 

    def attach_cert_to_thing(self):
        LOGGER.info(f"Attach cert '{self.certs['certificateArn']}' to a thing {self.thing['thingName']}")
        self.iot_client.attach_thing_principal(
            thingName=self.thing['thingName'],
            principal=self.certs['certificateArn']
        )

        return self

    def detach_cert_from_thing(self):
        LOGGER.info(f"Detach cert '{self.certs['certificateArn']}' from a thing {self.thing['thingName']}")
        self.iot_client.detach_thing_principal(
            thingName=self.thing['thingName'],
            principal=self.certs['certificateArn']
        )

        return self


    def connect_mqtt(self):
        # Callback when connection is accidentally lost.
        def on_connection_interrupted(connection, error, **kwargs):
            LOGGER.error("Connection interrupted. error: {}".format(error))

        try:
            LOGGER.info(f"Connecting to AWS IOT MQTT endpoint '{AWSCliClient.ENDPOINT}' with Client Id '{self.mqtt_client_id}'")
            self.mqtt_connection = mqtt_connection_builder.mtls_from_path(
                endpoint=AWSCliClient.ENDPOINT,
                cert_filepath=os.path.join(self.certs_folder, 'certificate.pem.crt'),
                pri_key_filepath=os.path.join(self.certs_folder, 'private.pem.key'),
                client_id=self.mqtt_client_id,
                on_connection_interrupted=on_connection_interrupted,
                )
            
            connect_future = self.mqtt_connection.connect()
            # Future.result() waits until a result is available
            connect_future.result()
            LOGGER.info("Connected!")
        except Exception as e:
            LOGGER.exception(e)

        return self

    def subscribe_to_topic(self, message_topic):
        def on_message_received(topic, payload, dup, qos, retain, **kwargs):
            try:
                LOGGER.info("Received message from topic '{}': {}".format(topic, payload))

                if self.mqtt_message_loop.get(topic) is None:
                    self.mqtt_message_loop[topic] = []
                
                self.mqtt_message_loop[topic].append(payload)
            except Exception as e:
                LOGGER.exception(e)

        try:
            LOGGER.info(f"Subscribing to topic '{message_topic}'...")
            subscribe_future, _ = self.mqtt_connection.subscribe(
                topic=message_topic,
                qos=mqtt.QoS.AT_LEAST_ONCE,
                callback=on_message_received)

            subscribe_result = subscribe_future.result()
            self.mqtt_message_loop[message_topic] = []
            LOGGER.info(f"Subscribed with {str(subscribe_result['qos'])}")
        except Exception as e:
            LOGGER.exception(e)

        return self
    
    def list_messages(self):
        return self.mqtt_message_loop

    def publish_message(self, message_topic, message):
        try:
            LOGGER.info(f"Publishing message to topic '{message_topic}': {message}")
            message_json = json.dumps(message)
            self.mqtt_connection.publish(
                topic=message_topic,
                payload=message_json,
                qos=mqtt.QoS.AT_LEAST_ONCE)
            LOGGER.info("Message sent")
        except Exception as e:
            LOGGER.exception(e)

    def read_message(self, topic):
        return "10"
    
    def expects_message(self, topic, message, timeout = 10):
        timeout = time.time() + timeout
        while time.time() < timeout:
            m = self.read_message(topic)
            if m == message:
                LOGGER.info(f"Message {m} retrieved")
                return m
            
        raise TimeoutError(f"Timeout limit reached. No {m} message retrived")
    
    def terminate(self):
        try:
            LOGGER.info("Disconecting from AWS IOT")
            disconnect_future = self.mqtt_connection.disconnect()
            disconnect_future.result()
            LOGGER.info("Disconnected!")
        except Exception as e:
            LOGGER.exception(e)