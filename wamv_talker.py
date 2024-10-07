#  Communicates over the RFD900+ modems with Jetson GPIO

import rclpy
import time
from enum import Enum
from utils.message import Message
from utils.rclpy_handler import RCLPY_Handler
from utils.topic_service import Publisher, Subscriber

import std_msgs.msg

try:
    import Jetson.GPIO as GPIO  # Use Jetson.GPIO 
except ImportError:
    user_input = input("USER: This is meant to be run on a Jetson Nano. Are you sure you want to continue? Y/n")
    if user_input.lower() != "y":
        exit()

# PINOUT (Jetson Nano BCM pin numbers)
gpio_pins = [26, 19, 13, 6, 5]  # BCM numbering

class Wamv_talker():

    class Tranceive_Type(Enum):
        RECEIVE = 0
        TRANSMIT = 1

    def __init__(self, name, refresh_rate=10):
        self.rclpy_handler = RCLPY_Handler()
        self.refresh_rate = refresh_rate

        self.wamv_command_msg = Message("/wamv_comms/mode_set", std_msgs.msg.GoalStatus)

        self.wamv_command_pub = Publisher(self.wamv_command_msg.get_name(), self.wamv_command_msg.get_type())
        self.wamv_status_sub = Subscriber("/wamv_comms/status", std_msgs.msg.GoalStatus)
        
        self.init_topics()
        self.init_gpio()
    
    def enable(self):
        '''Enable and start the node.'''
        self.rclpy_handler.connect()

    def init_gpio(self):
        GPIO.setmode(GPIO.BCM)  # Jetson Nano uses BCM mode
        GPIO.setup(gpio_pins, GPIO.OUT)
        self.rclpy_handler.log(self, 'GPIO pins going OUT')

    def set_gpio_state(self, state):
        '''Set GPIO pins to high or low.'''
        GPIO.output(gpio_pins, GPIO.HIGH if state else GPIO.LOW)
        self.rclpy_handler.log(f"GPIO pins set to {'HIGH' if state else 'LOW'}.")

    def publish_command(self, mode):
        '''Publishes a WAMV command to change the mode.'''
        # Fill the message data (this depends on the structure of GoalStatus)
        data = std_msgs.msg.GoalStatus()
        data.goal_id.id = str(mode)  # Set the mode in the GoalStatus message, adjust as needed
        self.wamv_command_msg.set_data(data)  # Set data inside the Message object

        # Publish the message using the RCLPY handler
        self.rclpy_handler.publish_topic(self.wamv_command_pub, self.wamv_command_msg.get_data())

    def gpio_cleanup(self):
        GPIO.cleanup()
        self.rclpy_handler.log("GPIO cleanup complete")

    def init_topics(self):
        self.rclpy_handler.create_topic_publisher(self.wamv_command_pub)
        self.rclpy_handler.create_topic_subscriber(self.wamv_status_sub, self.status_callback)

    def status_callback(self):
        self.rclpy_handler.log(f"Received WAMV status: {msg.goal_id.id}")
    
    # def test(self):
    #   data = drone.msg.wamv_comms()
    #   data.name = "hello"
    #   self.wamv_message.set_data(data)
    #   self.ros_driver.publish(self.wamv_message)


def set_gpio(gpio, set_input):
    # GPIO setup for Jetson Nano
    state = GPIO.IN if set_input else GPIO.OUT
    for pin in gpio:
        GPIO.setup(pin, state)

def main():
    rclpy.init()
    talker = Wamv_talker("wamv_talker", 50)

    set_gpio(gpio_pins, set_input=False)

    try:
        while not rclpy.is_shutdown():
            talker.enable()

            talker.set_gpio_state(True)
            time.sleep(1)
            talker.set_gpio_state(False)
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down WAMV talker...")

rclpy.shutdown()

if __name__ == "__main__":
    main()
