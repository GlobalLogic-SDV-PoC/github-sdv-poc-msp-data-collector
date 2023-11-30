import pytest

from thirdparty.sdv_testing_tool.sdv_testing_tool.config.config import CONFIG
from thirdparty.sdv_testing_tool.sdv_testing_tool.enums.marks import TC_ID, PrimaryComponent, Priority, Suite, SweLevel, Tasks
from thirdparty.sdv_testing_tool.sdv_testing_tool.helpers.customize_marks import SDVMarks
from thirdparty.sdv_testing_tool.sdv_testing_tool.helpers.helpers import get_utc_time_now
from thirdparty.sdv_testing_tool.sdv_testing_tool.providers.service.logger import SDVLogger
from tests.data.test_cases_data import ram_testdata_negative, ram_testdata_positive, temp_testdata_negative, temp_testdata_positive

LOGGER = SDVLogger.get_logger("test-DATA_COLLECTOR")


@SDVMarks.links("TASK_126")
@SDVMarks.add(
    SweLevel.COMPONENT,
    Priority.P1,
    PrimaryComponent.DATA_COLLECTOR,
    Suite.SMOKE,
    TC_ID.SDV_00011,
)
@pytest.mark.parametrize("topic, test_data", temp_testdata_positive)
def test_dc_iotb_temp_positive(iot_bridge_dc_iot_thing, topic, test_data):
    dc_service, iot_bridge_service, iot_thing = iot_bridge_dc_iot_thing

    root = dc_service.config.source_conf["collectors"]
    topic_name_qr = root["root_query"] + "/" + topic
    topic_name_send = root["root_send"] + "/" + topic

    iot_thing.subscribe_to_topic(topic_name_qr)
    iot_thing.subscribe_to_topic(topic_name_send)
    dc_service.config.generate_dc_temp_file(test_data)

    time_utc_now = get_utc_time_now()

    msg_body = {"msg": f"trigger the '{topic_name_qr}' query"}
    iot_thing.publish_message(topic_name_qr, msg_body)

    msg = "\[dcol\] Send collection data: starting..."
    assert dc_service.expects_log_message(msg, since=time_utc_now)

    msg = f'\[ipc\]\[client\] trying to send packet contents: {{"action":"forward","payload_size":.*,"topic":"{topic_name_send}"'
    assert dc_service.expects_log_message(msg, since=time_utc_now)

    msg = "\[dcol\] Send collection data: done."
    assert dc_service.expects_log_message(msg, since=time_utc_now)

    msg = "\[ipc\]\[client\] sent packet"
    assert dc_service.expects_log_message(msg, since=time_utc_now)


@SDVMarks.links("TASK_126")
@SDVMarks.add(
    SweLevel.COMPONENT,
    Priority.P1,
    PrimaryComponent.DATA_COLLECTOR,
    Suite.REGRESSION,
    TC_ID.SDV_00012,
)
@pytest.mark.parametrize("topic, test_data", temp_testdata_negative)
def test_dc_iotb_temp_negative(iot_bridge_dc_iot_thing, topic, test_data):
    dc_service, iot_bridge_service, iot_thing = iot_bridge_dc_iot_thing

    root = dc_service.config.source_conf["collectors"]
    topic_name_qr = root["root_query"] + "/" + topic
    topic_name_send = root["root_send"] + "/" + topic

    iot_thing.subscribe_to_topic(topic_name_qr)
    iot_thing.subscribe_to_topic(topic_name_send)
    dc_service.config.generate_dc_temp_file(test_data)

    time_utc_now = get_utc_time_now()

    msg_body = {"msg": f"trigger the '{topic_name_qr}' query"}
    iot_thing.publish_message(topic_name_qr, msg_body)

    msg = "\[dcol\] Send collection data: starting..."
    assert dc_service.expects_log_message(msg, since=time_utc_now)

    msg = f'\[ipc\]\[client\] trying to send packet contents: {{"action":"forward","payload_size":.*,"topic":"{topic_name_send}"'
    assert dc_service.expects_log_message(msg, since=time_utc_now)

    msg = "\[dcol\] Send collection data: done."
    assert dc_service.expects_log_message(msg, since=time_utc_now)

    msg = "\[ipc\]\[client\] sent packet"
    assert dc_service.expects_log_message(msg, since=time_utc_now)


@SDVMarks.links("TASK_126")
@SDVMarks.add(
    SweLevel.COMPONENT,
    Priority.P1,
    PrimaryComponent.DATA_COLLECTOR,
    Suite.SMOKE,
    TC_ID.SDV_00013,
)
@pytest.mark.parametrize("topic, test_data", ram_testdata_positive)
def test_dc_iotb_ram_positive(iot_bridge_dc_iot_thing, topic, test_data):
    dc_service, iot_bridge_service, iot_thing = iot_bridge_dc_iot_thing

    root = dc_service.config.source_conf["collectors"]
    topic_name_qr = root["root_query"] + "/" + topic
    topic_name_send = root["root_send"] + "/" + topic

    iot_thing.subscribe_to_topic(topic_name_qr)
    iot_thing.subscribe_to_topic(topic_name_send)
    dc_service.config.generate_dc_temp_file(test_data)

    time_utc_now = get_utc_time_now()

    msg_body = {"msg": f"trigger the '{topic_name_qr}' query"}
    iot_thing.publish_message(topic_name_qr, msg_body)

    msg = "\[dcol\] Send collection data: starting..."
    assert dc_service.expects_log_message(msg, since=time_utc_now)

    msg = f'\[ipc\]\[client\] trying to send packet contents: {{"action":"forward","payload_size":.*,"topic":"{topic_name_send}"'
    assert dc_service.expects_log_message(msg, since=time_utc_now)

    msg = "\[dcol\] Send collection data: done."
    assert dc_service.expects_log_message(msg, since=time_utc_now)

    msg = "\[ipc\]\[client\] sent packet"
    assert dc_service.expects_log_message(msg, since=time_utc_now)


@SDVMarks.links("TASK_126")
@SDVMarks.add(
    SweLevel.COMPONENT,
    Priority.P1,
    PrimaryComponent.DATA_COLLECTOR,
    Suite.REGRESSION,
    TC_ID.SDV_00014,
)
@pytest.mark.parametrize("topic, test_data", ram_testdata_negative)
def test_dc_iotb_ram_negative(iot_bridge_dc_iot_thing, topic, test_data):
    dc_service, iot_bridge_service, iot_thing = iot_bridge_dc_iot_thing

    root = dc_service.config.source_conf["collectors"]
    topic_name_qr = root["root_query"] + "/" + topic
    topic_name_send = root["root_send"] + "/" + topic

    iot_thing.subscribe_to_topic(topic_name_qr)
    iot_thing.subscribe_to_topic(topic_name_send)
    dc_service.config.generate_dc_temp_file(test_data)

    time_utc_now = get_utc_time_now()

    msg_body = {"msg": f"trigger the '{topic_name_qr}' query"}
    iot_thing.publish_message(topic_name_qr, msg_body)

    msg = "\[dcol\] Send collection data: starting..."
    assert dc_service.expects_log_message(msg, since=time_utc_now)

    msg = f'\[ipc\]\[client\] trying to send packet contents: {{"action":"forward","payload_size":.*,"topic":"{topic_name_send}"'
    assert dc_service.expects_log_message(msg, since=time_utc_now)

    msg = "\[dcol\] Send collection data: done."
    assert dc_service.expects_log_message(msg, since=time_utc_now)

    msg = "\[ipc\]\[client\] sent packet"
    assert dc_service.expects_log_message(msg, since=time_utc_now)
