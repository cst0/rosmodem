#!/usr/bin/env python3

import rospy
from abc import ABCMeta, abstractmethod

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
    def initialize(self, args): pass
    def deinitialize(self, args): pass

    @abstractmethod
    def read(self) -> bytes: pass

    @abstractmethod
    def write(self, b:bytes): pass

    def __init__(self, args):
        self.initialize(args)
        rospy.on_shutdown(self.deinitialize)
        rospy.spin()

