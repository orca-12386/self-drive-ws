from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import numpy as np
import yaml

class NodeGlobal:
    obj = None 

    @staticmethod
    def log_info(s):
        NodeGlobal.obj.get_logger().info(s)

    @staticmethod
    def log_error(s):
        NodeGlobal.obj.get_logger().error(s)

class Config:
    config = None
    @staticmethod
    def read_config(filepath):
        def load_yaml_file(file_path):
            try:
                with open(file_path, 'r') as yaml_file:
                    return yaml.safe_load(yaml_file)
            except Exception as e:
                NodeGlobal.log_error(f"Failed to load YAML file: {e}")
                return None
        yaml_file_path = filepath
        with open(yaml_file_path):
            Config.config = load_yaml_file(yaml_file_path)


class Message:
    parsers = [
        (PoseStamped, lambda message: (message.pose.position.x, message.pose.position.y)),
        (OccupancyGrid, lambda message: np.array(message.data).reshape((message.info.height, message.info.width))),
        (Float64, lambda message: message.data)
    ]
    def __init__(self, message, datatype):
        self.message = message
        self.datatype = datatype

    @staticmethod
    def add_parser(datatype, lambda_func):
        Message.parsers.append((datatype, lambda_func))

    @staticmethod
    def static_parse_message(message, datatype):
        for dt, lf in Message.parsers:
            if dt == datatype:
                return lf(message)
        return message
        
    def parse_message(self):
        return Message.static_parse_message(self.message, self.datatype)


class Subscription:
    subs = dict()

    def __init__(self, topic, datatype, obj, maxlength=1):
        self.topic = topic
        self.datatype = datatype
        self.obj = obj
        self.maxlength = maxlength
        self.messages_obj = list()

    @staticmethod
    def create_subscriptions(subscription_info):
        create_subscriber = lambda topic: NodeGlobal.obj.create_subscription(subscription_info[topic][1], subscription_info[topic][0], lambda msg: Subscription.subs[topic].set_message_obj(msg), 10)
        for topic in subscription_info:
            if len(subscription_info[topic])>2:
                length = subscription_info[topic][2]
            else:
                length = 1
            Subscription.subs[topic] = Subscription(subscription_info[topic][0], subscription_info[topic][1], create_subscriber(topic), length)
            NodeGlobal.log_info("Subscribed to "+topic)

    def set_message_obj(self, msg):
        msg_obj = Message(msg, self.datatype)
        self.messages_obj.append(msg_obj)
        if len(self.messages_obj)>self.maxlength:
            self.messages_obj.pop(0)
        
    def get_all_messages_obj(self):
        if(len(self.messages_obj)) > 0:
            return self.messages_obj
        else:
            return None
        
    def get_latest_message_obj(self):
        if len(self.messages_obj) > 0:
            return self.messages_obj[-1]
        else:
            return None
        
    def get_all_messages(self):
        if(len(self.messages_obj)) > 0:
            return [m.message for m in self.messages_obj]
        else:
            return None
        
    def get_latest_message(self):
        if len(self.messages_obj) > 0:
            return self.messages_obj[-1].message
        else:
            return None

    def parse_message(self, message):
        parsed_message = message.parse_message()
        return parsed_message

    def get_all_data(self):
        if len(self.messages_obj) > 0:
            return [message.parse_message() for message in self.get_all_messages_obj()]
        else:
            return None

    def get_latest_data(self):
        if len(self.messages_obj) > 0:
            return self.get_latest_message_obj().parse_message()
        else:
            return None


class Publisher:
    pubs = dict()
    
    @staticmethod
    def create_publishers(publisher_info):
        create_publisher = lambda topic: NodeGlobal.obj.create_publisher(publisher_info[topic][1], publisher_info[topic][0], 10)
        for topic in publisher_info:
            Publisher.pubs[topic] = create_publisher(topic)
            NodeGlobal.log_info("Created publisher to "+topic)