import rospy
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from ros_numpy import numpify
from smach import State, StateMachine

import numpy as np


class FinishedListener:
    def __init__(self):
        self.odom_listener = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.result = None

    def init(self):
        self.result = None

    def odom_callback(self, msg):  # type: (Odometry) -> None
        if msg.pose.pose.position.x < 0:
            self.result = 'finished'

    def __call__(self):
        return self.result

    @property
    def outcomes(self):
        return 'finished',


class BumperListener:
    def __init__(self):
        self.bumper_listener = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)
        self.result = None

    def init(self):
        self.result = None

    def bumper_callback(self, msg):  # type: (BumperEvent) -> None
        if msg.state == msg.PRESSED and msg.bumper == msg.LEFT:
            self.result = 'bumper_left'
        if msg.state == msg.PRESSED and msg.bumper == msg.RIGHT:
            self.result = 'bumper_right'

    def __call__(self):
        return self.result

    @property
    def outcomes(self):
        return 'bumper_left', 'bumper_right'


class CombinedListener:
    def __init__(self, listeners):
        self.listeners = listeners

    def init(self):
        for l in self.listeners:
            l.init()

    def __call__(self):
        result = None
        for l in self.listeners:
            result = l()
            if result is not None:
                break
        return result

    @property
    def outcomes(self):
        outcomes = tuple()
        for l in self.listeners:
            outcomes = outcomes + l.outcomes
        return outcomes


class TimerListener:
    def __init__(self, duration):
        self.duration = duration
        self.start_time = None

    def init(self):
        self.start_time = rospy.get_time()

    def __call__(self):
        if self.start_time + self.duration < rospy.get_time():
            return 'timeout'

    @property
    def outcomes(self):
        return 'timeout',


class MoveForwardAction:
    def __init__(self, v):
        self.v = v
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

    def init(self):
        pass

    def __call__(self):
        t = Twist()
        t.linear.x = self.v
        self.twist_pub.publish(t)


class TurnAction:
    def __init__(self, w):
        self.w = w
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

    def init(self):
        pass

    def __call__(self):
        t = Twist()
        t.angular.z = self.w
        self.twist_pub.publish(t)


class ActionUntil(State):
    def __init__(self, action, until, rate=10):
        super(ActionUntil, self).__init__(outcomes=until.outcomes)
        self.action = action
        self.until = until
        self.rate = rospy.Rate(rate)

    def execute(self, ud):
        self.action.init()
        self.until.init()

        while not rospy.is_shutdown():
            self.action()

            result = self.until()
            if result is not None:
                return result

            self.rate.sleep()


def main():
    v = 0.2
    w = 1

    turn_time = 1.2
    move_time = 1.5

    sm = StateMachine(outcomes=['bumper_left', 'bumper_right', 'finished'])
    with sm:
        StateMachine.add('forward', ActionUntil(MoveForwardAction(v), CombinedListener([FinishedListener(), BumperListener()])), transitions={'finished': None, 'bumper_left': 'turn_left', 'bumper_right': 'turn_right'})

        StateMachine.add('turn_left', ActionUntil(TurnAction(w), TimerListener(turn_time)), transitions={'timeout': 'move_left'})
        StateMachine.add('move_left', ActionUntil(MoveForwardAction(v), TimerListener(move_time)), transitions={'timeout': 'unturn_left'})
        StateMachine.add('unturn_left', ActionUntil(TurnAction(-w), TimerListener(turn_time)), transitions={'timeout': 'forward'})

        StateMachine.add('turn_right', ActionUntil(TurnAction(-w), TimerListener(turn_time)), transitions={'timeout': 'move_right'})
        StateMachine.add('move_right', ActionUntil(MoveForwardAction(v), TimerListener(move_time)), transitions={'timeout': 'unturn_right'})
        StateMachine.add('unturn_right', ActionUntil(TurnAction(w), TimerListener(turn_time)), transitions={'timeout': 'forward'})

    sm.execute()


if __name__ == '__main__':
    rospy.init_node('demo6')
    main()
