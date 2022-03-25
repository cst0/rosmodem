#!/usr/bin/env python3

from genpy.message import get_message_class
import rospy
from abc import ABCMeta, abstractmethod


class PublishWrapper:
    def __init__(self, topic_type: str):
        split = topic_type.strip("\n\r\t ;").split(",")
        self.topic = split[0]
        _type = split[1]

        self.mtype = get_message_class(_type)
        self.pub = rospy.Publisher(self.topic, self.mtype, queue_size=1)


class Modem(metaclass=ABCMeta):
    """
    The Modem class wraps the compression, sending, receiving, and
    decompression of messages on one device for usage on another device. To use
    it, you will need to create a new class which extends this one, and
    override the read and write functions (you may also override the
    initialize/deinitialize functions if your use-case requires it). This class
    also handles presenting mock interfaces for known topics, making the
    modem effectively invisible to your existing ROS interfaces.
    """

    @abstractmethod
    def initialize(self, args):
        """
        Initialize your communications.
        """
        pass

    @abstractmethod
    def deinitialize(self, args):
        """
        Deinitialize your communications.
        """
        pass

    @abstractmethod
    def read(self, max_read_len) -> bytes:
        """
        Read (receive) a set of bytes. This function may block, so it must be threadsafe.
        """
        pass

    @abstractmethod
    def write(self, b: bytes):
        """
        Write (send) a set of bytes.
        """
        pass

    def _call_reads(self, event):
        del event
        read = self.read(self.max_read_len)
        if len(read) > 0:
            self._read_bytes.append(read)

    def _handle_write_queue(self, event):
        del event
        pass

    def _process_queue(self, event):
        del event
        pass

    def place_in_write_queue(self, b):
        if type(b) is str:
            b = b.encode()
        self.write_queue.append(b)

    def _put_msg_in_queue(self, msg):
        self._message_queue.append(msg)

    def _manage_informed_topics(self):
        unpublished_topics_types = set(self._informed_topics) - set(
            self._publishers.keys()
        )
        published_topics_types_dead = set(self._publishers.keys()) - set(
            self._informed_topics
        )
        for topic_type in unpublished_topics_types:
            self._publishers[topic_type] = PublishWrapper(topic_type)
        for topic_type in published_topics_types_dead:
            self._publishers[topic_type].shutdown()
            del self._publishers[topic_type]

    def _inform_topics(self):
        t = rospy.get_published_topics()
        found_topics = [x[0] for x in t]
        found_types = [x[1] for x in t]
        as_strs = "".join(
            [
                "" + found_topics[n] + "," + found_types[n] + ";"
                for n in range(len(found_topics))
            ]  # produces (topic1,type1;topic2,type2;...)
        )

        self.place_in_write_queue("P[" + as_strs + "]P")

    def __init__(self, args):
        self.max_read_len = 1024
        self.write_queue = []

        self._read_bytes = []
        self._message_queue = []
        self._informed_topics = []

        self._publishers = {}
        self._subscribers = {}

        self.initialize(args)
        rospy.on_shutdown(self.deinitialize)
        rospy.spin()
