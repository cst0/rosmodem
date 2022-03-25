#!/usr/bin/env python3

from genpy.message import get_message_class
import rospy
from abc import ABCMeta, abstractmethod


class InterfaceWrapper:
    def populate_from_split(self, topic_type_pair):
        split = topic_type_pair.strip("\n\r\t ;").split(",")
        self.topic = split[0]
        _type = split[1]

        self.mtype = get_message_class(_type)


class PublisherWrapper(InterfaceWrapper):
    def __init__(self, topic_type: str):
        self.populate_from_split(topic_type)
        self.pub = rospy.Publisher(self.topic, self.mtype, queue_size=1)

    def shutdown(self):
        self.pub.unregister()


class SubscriberWrapper(InterfaceWrapper):
    def __init__(self, topic_type: str):
        self.populate_from_split(topic_type)
        self.sub = rospy.Subscriber(self.topic, self.mtype, self._cb, queue_size=1)
        self._has_message = False

    def shutdown(self):
        self.sub.unregister()

    def _cb(self, msg):
        self._has_message = True
        self.msg = msg

    def _get_message(self):
        if self._has_message:
            self._has_message = False
            return self.msg
        return None

    def to_sendable_str(self):
        returnme = ""
        gm = self._get_message()
        if gm:
            returnme += "D[" + self.topic + ";" + gm + "]"


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
        self._write_queue.append(b)

    def _put_msg_in_queue(self, msg):
        self._message_queue.append(msg)

    def _manage_informed_subscribers(self):
        """
        The other end has told us what they want. Let's set up a subscriber
        and send that data over (or remove a subscriber if nobody is using it anymore)
        """
        unsubscribed_topic_type_pairs = set(
            self._informed_subscriber_topic_type_pairs
        ) - set(self._subscribers.keys())
        subscribed_topic_type_pairs_dead = set(self._subscribers.keys()) - set(
            self._informed_subscriber_topic_type_pairs
        )

        for topic_type_pair in unsubscribed_topic_type_pairs:
            self._subscribers[topic_type_pair] = SubscriberWrapper(topic_type_pair)

        for topic_type_pair in subscribed_topic_type_pairs_dead:
            self._subscribers[topic_type_pair].shutdown()
            del self._subscribers[topic_type_pair]

    def _inform_subscribers(self):
        """
        The other end has publishers that correspond with the publishers we're
        actively getting data from here. If anyone subscribes to one of those
        publishers, we need to ask the other side to reach out and actually
        query that data.
        """
        in_use = []
        for p in self._publishers:
            if p.has_subscribers():
                in_use.append(p.topic)
        topiclist = "".join(["" + str(t) + ";" for t in in_use])
        self.place_in_write_queue("S[" + topiclist + "]S")

    def _manage_informed_publishers(self):
        """
        The other end of the modem has told us what they have available.
        Let's make those topics available on our end, too.
        """
        unpublished_topic_type_pairs = set(
            self._informed_publisher_topic_type_pairs
        ) - set(self._publishers.keys())
        published_topic_type_pairs_dead = set(self._publishers.keys()) - set(
            self._informed_publisher_topic_type_pairs
        )

        for topic_type_pair in unpublished_topic_type_pairs:
            self._publishers[topic_type_pair] = PublisherWrapper(topic_type_pair)

        for topic_type_pair in published_topic_type_pairs_dead:
            self._publishers[topic_type_pair].shutdown()
            del self._publishers[topic_type_pair]

    def _inform_publishers(self):
        """
        Inform the other end of the modem what topics we have available on our end.
        """
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

        self._write_queue = []
        self._read_bytes = []
        self._message_queue = []
        self._informed_publisher_topic_type_pairs = []
        self._informed_subscriber_topic_type_pairs = []

        self._publishers = {}
        self._subscribers = {}

        self.initialize(args)
        rospy.on_shutdown(self.deinitialize)
        rospy.spin()
