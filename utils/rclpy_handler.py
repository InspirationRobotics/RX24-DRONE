import rclpy
import rclpy.client
from rclpy.node import Node
import rclpy.publisher

class Topic_Service:
    def __init__(self, name : str, class_type, usage_type : str):
        self.__name = name
        self.__class_type = class_type
        self.__usage_type = usage_type

    # CLIENT ONLY
    def set_client(self, client : rclpy.client.Client):
        self.__client = client

    def get_client(self) -> rclpy.client.Client:
        return self.__client

    # PUBLISHER ONLY

    def set_publisher(self, publisher : rclpy.publisher.Publisher):
        self.__publisher = publisher

    def get_publisher(self) -> rclpy.publisher.Publisher:
        return self.__publisher

    # SUBSCRIBER ONLY

    def set_data(self, data):
        self.__data = data

    def get_data(self):
        data = self.__data
        self.__data = None
        return data

    def get_data_last(self):
        return self.__data

    # GENERIC FUNCTIONS

    def get_type(self):
        return self.__class_type

    def get_name(self):
        return self.__name
    
    def get_usage_type(self):
        return self.__usage_type


class RCLPY_Handler:
    def __init__(self, node : str):
        rclpy.init()
        self.node = Node(node)
        self.connected = False

    def log(self, msg : str):
        self.node.get_logger().info(msg)

    def connect(self, rate: int = 5):
        self.connected = True
        self.log("rclpy connected!")
        rclpy.spin(self.node)

    def disconnect(self):
        if self.connected:
            self.log("Shutting down rclpy ...")
            self.node.destroy_node()
            rclpy.shutdown()
            self.connected = False

    def create_topic_publisher(self, topic: Topic_Service):
        topic.set_publisher(self.node.create_publisher(topic.get_type(), topic.get_name(), 10))
        
    def publish_topic(self, topic: Topic_Service, data):
        try:
            topic.get_publisher().publish(data)
        except Exception as e:
            self.log("Failed to publish to topic " + topic.get_name())
            self.log(f"ERROR: {e}")

    def create_topic_subscriber(self, topic: Topic_Service, function=None):
        if function == None:
            function = topic.set_data
        self.node.create_subscription(topic.get_type(), topic.get_name(), function, 10)

    def create_service_client(self, topic: Topic_Service):
        topic.set_client(self.node.create_client(topic.get_type(), topic.get_name()))

    def send_service_request(self, service: Topic_Service, data, timeout=30):
        try:
            srv = service.get_name()
            client = service.get_client()

            self.log(f"waiting for ROS service: {srv}")
            client.wait_for_service(timeout_sec=timeout)
            self.log(f"ROS service is up: {srv}")
            call_srv = client.call_async(data)
            return call_srv.result()
        except Exception as e:
            self.log(f"ERROR: {e}")