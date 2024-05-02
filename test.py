from utils.rclpy_handler import Topic_Service, RCLPY_Handler
from std_msgs.msg import String
from ardupilot_msgs.msg import GlobalPosition
import time
import threading

testPublish = Topic_Service("test", GlobalPosition, "publisher")
testSub = Topic_Service("test", GlobalPosition, "subscriber")

testNode = RCLPY_Handler("testNode")

def test_callback(data : GlobalPosition):
    print("Received: " + str(data.latitude))

testNode.create_topic_publisher(testPublish)
testNode.create_topic_subscriber(testSub, test_callback)

def test_method():
    for i in range(10):
        data = GlobalPosition()
        data.latitude = float(i)
        testNode.publish_topic(testPublish, data)
        time.sleep(0.5)
    testNode.disconnect()

test_thread = threading.Thread(target=test_method, daemon=True)
test_thread.start()

testNode.connect()

