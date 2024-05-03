# drone_API.py 
# This file is an interface allowing for control with ArduPilot through ROS2 DDS.
# Written by: Eesh Vij (evij@uci.edu)


import math
import rclpy
import time
import errno
import threading

from geopy import distance
from geopy import point

from utils.rclpy_handler import RCLPY_Handler, Publisher, Subscriber, Client

# Message types
# Ardupilot messages
from ardupilot_msgs.msg import GlobalPosition
from ardupilot_msgs.srv import ArmMotors, ModeSwitch
# Geographic messages
from geographic_msgs.msg import GeoPoseStamped, GeoPointStamped
# Geometry messages
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
# Sensor messages
from sensor_msgs.msg import Joy, BatteryState, Imu, NavSatFix
# Built-in messages
from builtin_interfaces.msg import Time
# Standard messages
from std_msgs.msg import String, Float32, Int32, Int64
from rosgraph_msgs.msg import Clock

# Mode constants
MODE_STABILIZE = 0
MODE_GUIDED = 4
MODE_LOITER = 5
MODE_RTL = 6
MODE_LAND = 9

# Publisher topics
PUB_GLOBAL_POSITION = Publisher("/ap/cmd_gps_pose", GlobalPosition)
PUB_CMD_VEL = Publisher("/ap/cmd_vel", TwistStamped)
PUB_JOY = Publisher("/ap/joy", Joy)

# Subscriber topics
SUB_BATTERY = Subscriber("/ap/battery/battery0", BatteryState)
SUB_CLOCK = Subscriber("/ap/clock", Clock)
SUB_GEOPOSE = Subscriber("/ap/geopose/filtered", GeoPoseStamped)
SUB_GEPPOINT = Subscriber("/ap/gps_global_origin/filtered", GeoPointStamped)
SUB_IMU = Subscriber("/ap/imu/experimental/data", Imu)
SUB_NAVSAT = Subscriber("/ap/navsat/navsat0", NavSatFix)
SUB_POSE = Subscriber("/ap/pose/filtered", PoseStamped)
SUB_TIME = Subscriber("/ap/time", Time)
SUB_TWIST = Subscriber("/ap/twist/filtered", TwistStamped)

# Client topics
CLI_ARM = Client("/ap/arm_motors", ArmMotors)
CLI_MODE = Client("/ap/mode_switch", ModeSwitch)

#---------------------------------#

class drone_API:

    def __init__(self, handler: RCLPY_Handler):
        self.handler = handler
        self.init_topics()
        self.conn_thread = threading.Thread(target=self._connect, daemon=True)

    def connect(self):
        self.handler.log("Starting connection thread ...")
        self.conn_thread.start()

    def _connect(self):
        self.handler.connect()

    def disconnect(self):
        return self.handler.disconnect()
    
    def is_connected(self):
        return handler.connected

    def log(self, msg : str):
        self.handler.log(msg)

    def init_topics(self):
        self.init_publishers()
        self.init_subscribers()
        self.init_clients()

    def init_publishers(self):
        publishers = [v for k, v in globals().items() if isinstance(v, Publisher)]
        for pub in publishers:
            self.handler.create_topic_publisher(pub)

    def init_subscribers(self):
        subscribers = [v for k, v in globals().items() if isinstance(v, Subscriber)]
        for sub in subscribers:
            self.handler.create_topic_subscriber(sub)
        self.edit_subscribers()

    def edit_subscribers(self):
        self.handler.edit_topic_subscriber(SUB_BATTERY, self.batt_cb)
        pass

    def batt_cb(self, msg: BatteryState):
        # self.handler.log(f"Battery voltage: {msg.voltage} V")
        pass

    def init_clients(self):
        clients = [v for k, v in globals().items() if isinstance(v, Client)]
        for cli in clients:
            self.handler.create_service_client(cli)

    # DRONE CONTROL FUNCTIONS

    def arm(self):
        while not self.handler.connected: pass
        self.handler.log("Arming motors ...")
        data = ArmMotors.Request()
        data.arm = True
        self.handler.send_service_request(CLI_ARM, data)
        self.handler.log("Motors armed!")

    def disarm(self):
        while not self.handler.connected: pass
        self.handler.log("Disarming motors ...")
        data = ArmMotors.Request()
        data.arm = False
        self.handler.send_service_request(CLI_ARM, data)
        self.handler.log("Motors disarmed!")

    def switch_mode(self, mode : str):
        while not self.handler.connected: pass
        mode_int = globals()["MODE_" + mode.upper()]
        self.handler.log(f"Switching mode to {mode.upper()} ...")
        data = ModeSwitch.Request()
        data.mode = mode_int
        self.handler.send_service_request(CLI_MODE, data)
        self.handler.log(f"Mode switched to {mode.upper()}!")
    
if __name__ == "__main__":
    handler = RCLPY_Handler("drone_API")
    drone = drone_API(handler)
    drone.connect()
    drone.arm()
    drone.switch_mode("guided")
    time.sleep(3)
    drone.switch_mode("loiter")
    drone.disarm()
    drone.disconnect()
    print("Connection closed.")