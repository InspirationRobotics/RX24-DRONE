from utils.rclpy_handler import RCLPY_Handler, Publisher, Subscriber, Client
from std_msgs.msg import String
from ardupilot_msgs.msg import GlobalPosition
from ardupilot_msgs.srv import ArmMotors, ModeSwitch
import time
import threading

testPublish = Publisher("test", GlobalPosition)
testSub = Subscriber("test", GlobalPosition)
testClient = Client("/ap/arm_motors", ArmMotors)
testClient2 = Client("/ap/mode_switch", ModeSwitch)

testNode = RCLPY_Handler("testNode")

def test_callback(data : GlobalPosition):
    testNode.log("Received: " + str(data.latitude))

def test_callback2(data : GlobalPosition):
    testNode.log("Changed: " + str(data.latitude))

testNode.create_topic_publisher(testPublish)
testNode.create_topic_subscriber(testSub, test_callback)
testNode.edit_topic_subscriber(testSub, test_callback2)
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

