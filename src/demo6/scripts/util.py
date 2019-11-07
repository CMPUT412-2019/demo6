import rospy
from typing import Any, Callable, Optional
from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import Led
import playsound
import numpy as np
import time
from os import path


class SubscriberValue:
    def __init__(self, name, data_class, wait=True, queue_size=1, transform=None):  # type: (str, Any, bool, int, Optional[Callable[[Any], Any]]) -> None
        self._subscriber = rospy.Subscriber(name, data_class, callback=self._callback, queue_size=queue_size)
        self._topic = name
        self._wait = wait
        self._transform = transform
        self._value = None

    def _callback(self, message):
        if self._transform is None:
            self._value = message
        else:
            self._value = self._transform(message)

    def wait(self):
        while self._value is None and not rospy.is_shutdown():
            rospy.loginfo('Waiting for {}...'.format(self._topic))
            rospy.sleep(0.1)
        return self._value

    @property
    def value(self):
        if self._wait:
            self.wait()
        return self._value
