from utils.rclpy_handler import Topic_Service, RCLPY_Handler
from std_msgs.msg import String
from ardupilot_msgs.msg import GlobalPosition
import time
import threading

testPublish = Topic_Service("test", String, "publisher")
testSub = Topic_Service("test", String, "subscriber")

testNode = RCLPY_Handler("testNode")

def test_callback(data : String):
    print("Received: " + data.data)

testNode.create_topic_publisher(testPublish)
testNode.create_topic_subscriber(testSub, test_callback)

def test_method():
    for i in range(10):
        data = String()
        data.data = str(i)
        testNode.publish_topic(testPublish, data)
        time.sleep(0.5)

test_thread = threading.Thread(target=test_method, daemon=True)
test_thread.start()

testNode.connect()

