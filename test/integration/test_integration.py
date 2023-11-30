# import pytest
# from data_collection.tests.integration.src.clients.aws.aws_cli_client import AWSCliClient
# from data_collection.tests.integration.src.clients.docker.aws_client.aws_client import AWSDockerClient
# from data_collection.tests.integration.src.clients.docker.data_collector.dc_client import DataCollectorDockerClient
# import uuid
# from data_collection.tests.integration.src.config.config import CONFIG
# from data_collection.tests.integration.src.providers.configs.aws.config_aws_service_provider import ConfigAwsServiceProvider
# from data_collection.tests.integration.src.providers.configs.dc.config_dc_service_provider import ConfigDCServiceProvider
# import time
# from data_collection.tests.integration.src.logger import SDVLogger


# LOGGER = SDVLogger.get_logger("TESTS")



# @pytest.fixture(scope='session')
# def aws_client():
#     client = AWSCliClient()
#     name = f"auto_test_{uuid.uuid4()}"
#     client \
#         .create_thing(name) \
#         .add_thing_to_group() \
#         .create_certs() \
#         .attach_policy_to_cert() \
#         .attach_cert_to_thing() \
#         .generate_certs_files(CONFIG.AWS_CERTS_FOLDER) \
#         .connect_mqtt()

#     yield client

#     client \
#         .detach_policy_from_cert() \
#         .detach_cert_from_thing() \
#         .delete_certs() \
#         .delete_thing() \
#         .terminate()


# @pytest.fixture(scope='session')
# def aws_docker_cli(aws_client):
#     aws_config = ConfigAwsServiceProvider()
#     aws_config \
#         .init_config() \
#         .set_client_id(aws_client.thing_name) \
#         .generate(CONFIG.AWS_CONFIG_FOLDER)

#     aws_container = AWSDockerClient(aws_client.thing_name, aws_config)
#     aws_container \
#         .ensure_stop() \
#         .pull() \
#         .map_volume([
#             f"{aws_config.config_path}:{AWSDockerClient.VOLUMES['config']}",
#             f"{aws_client.certs_folder}:{AWSDockerClient.VOLUMES['certs']}",
#             ]) \
#         .run() \
#         .ensure_healthy()

#     yield aws_container, aws_client

#     aws_container.stop()
 
# @pytest.fixture
# def aws_dc_dockers(aws_docker_cli):
#     aws_container, aws_client = aws_docker_cli
    
#     dc_config = ConfigDCServiceProvider()
#     dc_config \
#         .init_config() \
#         .set_scheduled_message_send(True) \
#         .genarate_ram_file(CONFIG.RAM_FILE, 10) \
#         .set_config_ram_path(ConfigDCServiceProvider.VOLUME_RAM_PATH) \
#         .genarate_storage_file(CONFIG.STORAGE_FILE, 100) \
#         .set_config_storage_path(ConfigDCServiceProvider.VOLUME_STORAGE_PATH) \
#         .genarate_temp_file(CONFIG.TEMP_FILE, 99) \
#         .set_config_temp_path(ConfigDCServiceProvider.VOLUME_TEMP_PATH) \
#         .generate(CONFIG.DC_CONFIG_FOLDER)

#     dc_container = DataCollectorDockerClient(dc_config)
#     dc_container \
#         .ensure_stop() \
#         .pull() \
#         .map_volume([
#             f"{dc_config.config_path}:{ConfigDCServiceProvider.VOLUME_CONFIG_PATH}",
#             f"{dc_config.ram_path}:{ConfigDCServiceProvider.VOLUME_RAM_PATH}",
#             f"{dc_config.temp_path}:{ConfigDCServiceProvider.VOLUME_TEMP_PATH}",
#             # f"{dc_config.storage_path}:{ConfigDCServiceProvider.VOLUME_STORAGE_PATH}",
#             ]) \
#         .run() \
#         .ensure_healthy() \
#         .ensure_connected()

#     yield dc_container, aws_container, aws_client

#     dc_container.stop()

# def test_ram_negative(aws_dc_dockers):
#     """
#     SDV-1 Negative RAM testing
#     """
#     dc_container, aws_container, aws_client = aws_dc_dockers

#     ram_topic_name_qr = '/data_collection/query_data/temp'
#     ram_topic_name_send = '/data_collection/send_data/temp'

#     aws_client.subscribe_to_topic(ram_topic_name_send)
#     for i in range(10):
#         aws_client.publish_message(
#             ram_topic_name_qr, str(i)
#         )

#         LOGGER.info(f"Messages list: {aws_client.list_messages()}")
#         time.sleep(5)
#     time.sleep(120)

# #     aws ecr get-login-password --region eu-west-1 | docker login --username AWS --password-stdin 203647640528.dkr.ecr.eu-west-1.amazonaws.com

# # docker pull 203647640528.dkr.ecr.eu-west-1.amazonaws.com/raspberry-aws-client:latest
# # docker run -v C:\\Users\\sergii.butenko\\repos\\work\\sdv\\raspberry-data-collection-service\\data_collection\\tests\\integration\\src\\providers\\configs\\aws\\main_config.json:/var/configs/main_config.json -v C:\\Users\\sergii.butenko\\repos\\work\\sdv\\raspberry-data-collection-service\\data_collection\\tests\\integration\\aws\\certs:/var/security/ -p 5555:5555 -t --init 203647640528.dkr.ecr.eu-west-1.amazonaws.com/raspberry-aws-client:latest
# # docker run -v C:\\Users\\sergii.butenko\\repos\\work\\sdv\\raspberry-data-collection-service\\data_collection\\tests\\integration\\src\\providers\\configs\\aws\\main_config.json:/var/configs/main_config.json -p 5555:5555 -t --init 203647640528.dkr.ecr.eu-west-1.amazonaws.com/raspberry-aws-client:latest
# # docker run -v C:\\Users\\sergii.butenko\\repos\\work\\sdv\\raspberry-data-collection-service\\data_collection\\tests\\integration\\aws\\config\\main_config.json:/var/configs/main_config.json -p 5555:5555 -t --init 203647640528.dkr.ecr.eu-west-1.amazonaws.com/raspberry-aws-client:latest


# # docker pull 203647640528.dkr.ecr.eu-west-1.amazonaws.com/raspberry-data-collection-service:40e6716c46e980dd3ec983a110c8bcd8fb575986
# # docker run network=host -v C:\\Users\\sergii.butenko\\repos\\work\\sdv\\raspberry-data-collection-service\\data_collection\tests\integration\docker_volumes\dc\configs\temp:/var/testdata/temp -v C:\\Users\\sergii.butenko\\repos\\work\\sdv\\raspberry-data-collection-service\\data_collection\\tests\\integration\\docker_volumes\\aws\\configs:/var/configs 203647640528.dkr.ecr.eu-west-1.amazonaws.com/raspberry-data-collection-service:40e6716c46e980dd3ec983a110c8bcd8fb575986
