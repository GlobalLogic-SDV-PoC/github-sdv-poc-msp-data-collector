import pytest

from thirdparty.sdv_testing_tool.sdv_testing_tool.config.config import CONFIG
from thirdparty.sdv_testing_tool.sdv_testing_tool.enums.marks import TC_ID, PrimaryComponent, Priority, Suite, SweLevel, Tasks
from thirdparty.sdv_testing_tool.sdv_testing_tool.helpers.customize_marks import SDVMarks
from thirdparty.sdv_testing_tool.sdv_testing_tool.providers.service.logger import SDVLogger
from tests.data.test_cases_data import ram_testdata_negative, ram_testdata_positive, temp_testdata_negative, temp_testdata_positive

LOGGER = SDVLogger.get_logger("test-DATA_COLLECTOR")

@SDVMarks.links("TASK_125")
@SDVMarks.add(SweLevel.INTEGRATION, Priority.P1, Suite.SMOKE, TC_ID.SDV_00001)
@pytest.mark.parametrize("topic, test_data", temp_testdata_positive)
def test_temp_positive(iot_bridge_dc_iot_thing, topic, test_data):
    dc_service, iot_bridge_service, iot_thing = iot_bridge_dc_iot_thing

    root = dc_service.config.source_conf["collectors"]
    topic_name_qr = root["root_query"] + "/" + topic
    topic_name_send = root["root_send"] + "/" + topic

    iot_thing.subscribe_to_topic(topic_name_qr)
    iot_thing.subscribe_to_topic(topic_name_send)
    dc_service.config.generate_dc_temp_file(test_data)

    msg_body = {"msg": f"trigger the '{topic_name_qr}' query"}
    iot_thing.publish_message(topic_name_qr, msg_body)

    expected_message = {topic: dc_service.config.expected_temp_value}
    assert iot_thing.expects_message(topic_name_send, expected_message, strict=False, timeout=10)


@SDVMarks.links("TASK_126")
@SDVMarks.add(SweLevel.INTEGRATION, Priority.P1, Suite.REGRESSION, TC_ID.SDV_00002)
@pytest.mark.parametrize("topic, test_data", temp_testdata_negative)
@pytest.mark.xfail()
def test_temp_negative(iot_bridge_dc_iot_thing, topic, test_data):
    dc_service, iot_bridge_service, iot_thing = iot_bridge_dc_iot_thing

    root = dc_service.config.source_conf["collectors"]
    topic_name_qr = root["root_query"] + "/" + topic
    topic_name_send = root["root_send"] + "/" + topic

    iot_thing.subscribe_to_topic(topic_name_qr)
    iot_thing.subscribe_to_topic(topic_name_send)
    dc_service.config.generate_dc_temp_file(test_data)

    msg_body = {"msg": f"trigger the '{topic_name_qr}' query"}
    iot_thing.publish_message(topic_name_qr, msg_body)

    expected_message = {topic: dc_service.config.expected_temp_value}
    assert iot_thing.expects_message(topic_name_send, expected_message, strict=False, timeout=10)


@SDVMarks.links("TASK_126")
@SDVMarks.add(SweLevel.INTEGRATION, Priority.P1, Suite.SMOKE, TC_ID.SDV_00005)
@pytest.mark.parametrize("topic, test_data", ram_testdata_positive)
def test_ram_positive(iot_bridge_dc_iot_thing, topic, test_data):
    dc_service, iot_bridge_service, iot_thing = iot_bridge_dc_iot_thing

    root = dc_service.config.source_conf["collectors"]
    topic_name_qr = root["root_query"] + "/" + topic
    topic_name_send = root["root_send"] + "/" + topic

    iot_thing.subscribe_to_topic(topic_name_qr)
    iot_thing.subscribe_to_topic(topic_name_send)
    dc_service.config.generate_dc_ram_file(test_data)

    msg_body = {"msg": f"trigger the '{topic_name_qr}' query"}
    iot_thing.publish_message(topic_name_qr, msg_body)

    expected_message = dc_service.config.expected_ram_value
    assert iot_thing.expects_message(topic_name_send, expected_message, strict=False, timeout=10)


@SDVMarks.links("TASK_126")
@SDVMarks.add(SweLevel.INTEGRATION, Priority.P1, Suite.REGRESSION, TC_ID.SDV_00006)
@pytest.mark.parametrize("topic, test_data", ram_testdata_negative)
@pytest.mark.xfail()
def test_ram_negative(iot_bridge_dc_iot_thing, topic, test_data):
    dc_service, iot_bridge_service, iot_thing = iot_bridge_dc_iot_thing

    root = dc_service.config.source_conf["collectors"]
    topic_name_qr = root["root_query"] + "/" + topic
    topic_name_send = root["root_send"] + "/" + topic

    iot_thing.subscribe_to_topic(topic_name_qr)
    iot_thing.subscribe_to_topic(topic_name_send)
    dc_service.config.generate_dc_ram_file(test_data)

    msg_body = {"msg": f"trigger the '{topic_name_qr}' query"}
    iot_thing.publish_message(topic_name_qr, msg_body)

    expected_message = dc_service.config.expected_ram_value
    assert iot_thing.expects_message(topic_name_send, expected_message, strict=False, timeout=10)
