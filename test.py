from utils.rclpy_handler import Topic_Service, RCLPY_Handler
from std_msgs.msg import String
from ardupilot_msgs.msg import GlobalPosition
from ardupilot_msgs.srv import ArmMotors, ModeSwitch
import time
import threading

testPublish = Topic_Service("test", GlobalPosition, "publisher")
testSub = Topic_Service("test", GlobalPosition, "subscriber")
testClient = Topic_Service("/ap/arm_motors", ArmMotors, "client")
testClient2 = Topic_Service("/ap/mode_switch", ModeSwitch, "client")

testNode = RCLPY_Handler("testNode")

def test_callback(data : GlobalPosition):
    print("Received: " + str(data.latitude))

testNode.create_topic_publisher(testPublish)
testNode.create_topic_subscriber(testSub, test_callback)
testNode.create_service_client(testClient)
testNode.create_service_client(testClient2)

def test_srv_arm(arm : bool):
    cliData = ArmMotors.Request()
    cliData.arm = arm
    testNode.send_service_request(testClient, cliData)

def test_mode_switch(mode : int):
    cliData = ModeSwitch.Request()
    cliData.mode = mode
    testNode.send_service_request(testClient2, cliData)

def test_method():
    test_mode_switch(16)
    test_srv_arm(True)
    for i in range(10):
        data = GlobalPosition()
        data.latitude = float(i)
        testNode.publish_topic(testPublish, data)
        time.sleep(0.5)
    test_srv_arm(False)
    test_mode_switch(0)
    testNode.disconnect()

test_thread = threading.Thread(target=test_method, daemon=True)
test_thread.start()

testNode.connect()

