#This class defines a wrapper for ROS messages, storing the topic name, message type, and data, with methods to set and retrieve the message data, type, and name.

class Message():
    def __init__(self, name: str, class_type):
        self.name = name
        self.class_type = class_type
        self.data = None

    def set_data(self, data):
        self.data = data

    def get_data(self):
        return self.data

    def get_type(self):
        return self.class_type

    def get_name(self):
        return self.name
