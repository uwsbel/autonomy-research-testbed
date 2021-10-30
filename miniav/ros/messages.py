# General imports
from typing import NamedTuple

class MessageType(NamedTuple):
    """
    ROS bags typically include custom message which are difficult to read directly since the data itself doesn't include information about how it's structured. 
    When writing our own rosbags, we want to store this information to make it easier to parse later. To accomplish this, this class contains information necessary to
    register the message for writing, and then reading later.

    Attributes:
        file (str):     the path to the .msg file that defines the data types
        topic (str):    the topic for which this message exists in the bag
    """ 
    file: str
    topic: str


