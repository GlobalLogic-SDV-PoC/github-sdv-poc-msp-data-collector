import pytest

# import uuid

# from thirdparty.sdv_testing_tool.sdv_testing_tool.applications.services.data_collector.dc_service_provider import DCServiceProvider
# from thirdparty.sdv_testing_tool.sdv_testing_tool.applications.services.iot_bridge.iot_bridge_service_provider import IotBridgeServiceProvider
# from thirdparty.sdv_testing_tool.sdv_testing_tool.applications.services.ota_updater.ota_updater_service_provider import OtaUpdaterServiceProvider
# from thirdparty.sdv_testing_tool.sdv_testing_tool.applications.cloud_cli.cloud_provider import CloudProvider
from thirdparty.sdv_testing_tool.sdv_testing_tool.config.config import CONFIG
from thirdparty.sdv_testing_tool.sdv_testing_tool.providers.service.logger import SDVLogger

LOGGER = SDVLogger.get_logger("CONFTEST")

@pytest.fixture(scope="module")
def iot_bridge_iot_thing(iot_bridge_service):

    yield iot_bridge_service


@pytest.fixture(scope="module")
def iot_bridge_ota_iot_thing(iot_bridge_iot_thing):
    
    yield iot_bridge_iot_thing


@pytest.fixture(scope="module")
def iot_bridge_dc_iot_thing(dc_service, iot_bridge_service, iot_thing):

    yield dc_service, iot_bridge_service, iot_thing

    dc_service.tear_down()


# @pytest.fixture(scope="session")
# def iot_thing():
#     client = CloudProvider.get_provider(CONFIG.CLOUD_PROVIDER)
#     client = client()

#     thing_name = f"auto_test_{uuid.uuid4()}"

#     client.init_cli().tear_up_thing(thing_name).connect_to_message_bus()

#     yield client

#     client.tear_down_thing().terminate()


# @pytest.fixture(scope="module")
# def iot_bridge_iot_thing(iot_thing):
#     iot_bridge_class = IotBridgeServiceProvider.get_service(platform=CONFIG.PLATFORM, cloud_provider=CONFIG.CLOUD_PROVIDER)
#     iot_bridge_service = iot_bridge_class(thing_name=iot_thing.thing_name, certs_folder=iot_thing.certs_folder)
#     iot_bridge_service.tear_up()

#     yield iot_bridge_service, iot_thing

#     iot_bridge_service.tear_down()


# @pytest.fixture(scope="module")
# def iot_bridge_ota_iot_thing(iot_bridge_iot_thing):
#     iot_bridge_service, iot_thing = iot_bridge_iot_thing

#     ota_class = OtaUpdaterServiceProvider.get_service(platform=CONFIG.PLATFORM, cloud_provider=CONFIG.CLOUD_PROVIDER)
#     ota_service = ota_class()
#     ota_service.tear_up()

#     yield ota_service, iot_bridge_service, iot_thing

#     ota_service.tear_down()


# @pytest.fixture(scope="module")
# def iot_bridge_dc_iot_thing(iot_bridge_iot_thing):
#     iot_bridge_service, iot_thing = iot_bridge_iot_thing

#     dc_class = DCServiceProvider.get_service(platform=CONFIG.PLATFORM, cloud_provider=CONFIG.CLOUD_PROVIDER)
#     dc_service = dc_class()
#     dc_service.tear_up()

#     yield dc_service, iot_bridge_service, iot_thing

#     dc_service.tear_down()