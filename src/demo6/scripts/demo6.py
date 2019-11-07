from math import atan2

import rospy
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from ros_numpy import numpify
from smach_ros import IntrospectionServer
from smach import State, StateMachine, Sequence
from tf import transformations

import numpy as np

from marker_tracker import MarkerTracker
from util import SubscriberValue


class FinishedListener:
    def __init__(self):
        self.odom_listener = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.result = None

    def init(self):
        self.result = None

    def odom_callback(self, msg):  # type: (Odometry) -> None
        if msg.pose.pose.position.x < -0.5:
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


class DebouncedButton:
    def __init__(self):
        self._pressed = False
        self._debounced_pressed = False
        self.last_released_time = rospy.get_time()
        self.last_pressed_time = rospy.get_time()

    def press(self):
        self._pressed = True
        self.last_pressed_time = rospy.get_time()

    def release(self):
        self._pressed = False
        self.last_released_time = rospy.get_time()

    @property
    def pressed(self):
        time_since_press = rospy.get_time() - self.last_pressed_time
        time_since_release = rospy.get_time() - self.last_released_time
        if self._pressed == self._debounced_pressed:
            return self._pressed
        if self._pressed and time_since_press > 0.05:
            self._debounced_pressed = self._pressed
        if not self._pressed and time_since_release > 0.05:
            self._debounced_pressed = self._pressed
        return self._debounced_pressed


class NoBumperListener:
    def __init__(self):
        self.bumper_listener = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)
        self.result = None
        self.left = DebouncedButton()
        self.right = DebouncedButton()
        self.center = DebouncedButton()
        self.previously_pressed = False

    def init(self):
        self.left = DebouncedButton()
        self.right = DebouncedButton()
        self.center = DebouncedButton()
        self.previously_pressed = False
        self.result = None

    def bumper_callback(self, msg):  # type: (BumperEvent) -> None
        if msg.bumper == msg.LEFT:
            self.left.press() if msg.state == msg.PRESSED else self.left.release()
        if msg.bumper == msg.RIGHT:
            self.right.press() if msg.state == msg.PRESSED else self.right.release()
        if msg.bumper == msg.CENTER:
            self.center.press() if msg.state == msg.PRESSED else self.center.release()

    def __call__(self):
        print(self.left.pressed)
        if self.left.pressed or self.right.pressed or self.center.pressed:
            self.previously_pressed = True
        elif self.previously_pressed:
            if not (self.left.pressed or self.right.pressed or self.center.pressed):
                self.result = 'released'
        return self.result

    @property
    def outcomes(self):
        return 'released',


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


class FindMarkerListener:
    def __init__(self, marker_tracker):  # type: (MarkerTracker) -> None
        self.marker_tracker = marker_tracker

    def init(self):
        pass

    def __call__(self):
        if self.marker_tracker.get_visible_markers():
            return 'found'

    @property
    def outcomes(self):
        return 'found',


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


class FindMarker(State):
    def __init__(self, marker_tracker):  # type: (MarkerTracker) -> None
        super(FindMarker, self).__init__(outcomes=['ok', 'err'], output_keys=['marker_pose'])
        self.marker_tracker = marker_tracker

    def execute(self, ud):
        while not rospy.is_shutdown():
            try:
                markers = self.marker_tracker.get_visible_markers()
                marker_id = next(iter(markers))
                marker_pose = self.marker_tracker.get_pose(marker_id)
                marker_pose.header.frame_id = 'odom'
                ud.marker_pose = marker_pose
            except StopIteration:
                continue
            return 'ok'


class NavigateToGoalState(State):
    def __init__(self):
        super(NavigateToGoalState, self).__init__(outcomes=['ok', 'err'], input_keys=['target_pose'])
        self.client = SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, ud):
        pose = ud.target_pose  # type: PoseStamped
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = pose.header.frame_id
        goal.target_pose.pose.position = pose.pose.position
        goal.target_pose.pose.orientation = pose.pose.orientation
        self.client.send_goal(goal)
        if self.client.wait_for_result():
            return 'ok'
        else:
            return 'err'


class ChooseNewNavGoalState(State):
    def __init__(self, origin, back_distance):  # type: (np.ndarray, float)
        super(ChooseNewNavGoalState, self).__init__(outcomes=['ok'], input_keys=['marker_pose'], output_keys=['target_pose'])
        self.origin = origin
        self.back_distance = back_distance

    def execute(self, ud):
        marker_pose = ud.marker_pose  # type: PoseStamped

        marker_position = np.array([marker_pose.pose.position.x, marker_pose.pose.position.y])

        r_mo = transformations.unit_vector(marker_position - self.origin)
        goal_position = marker_position + self.back_distance * r_mo

        e_mo = transformations.unit_vector(np.append(r_mo, [0]))
        orientation_facing_marker = np.eye(4)
        orientation_facing_marker[0:3, 0:3] = np.column_stack((-e_mo, np.cross([0, 0, 1], -e_mo), [0, 0, 1]))
        orientation_facing_marker = transformations.quaternion_from_matrix(orientation_facing_marker)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = marker_pose.header.frame_id
        goal_pose.pose.position = Point(goal_position[0], goal_position[1], 0)
        goal_pose.pose.orientation = Quaternion(
            orientation_facing_marker[0],
            orientation_facing_marker[1],
            orientation_facing_marker[2],
            orientation_facing_marker[3],
        )
        # goal_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        ud.target_pose = goal_pose
        return 'ok'


def main():
    v = 0.2
    w = 1

    turn_time = 1.2
    move_time = 1.5

    marker_tracker = MarkerTracker()
    sm = StateMachine(outcomes=['ok', 'err', 'finished'])
    with sm:
        StateMachine.add('find_marker', FindMarker(marker_tracker), transitions={'ok': 'choose_goal', 'err': None})
        StateMachine.add('choose_goal', ChooseNewNavGoalState(np.array([-0.5, 0.]), 1.), transitions={'ok': 'go_to_goal'})
        StateMachine.add('go_to_goal', NavigateToGoalState(), transitions={'ok': 'push_box', 'err': None})
        StateMachine.add('push_box', ActionUntil(MoveForwardAction(v), CombinedListener([FinishedListener(), NoBumperListener()])), transitions={
            'finished': None,
            'released': 'backup',
        })
        StateMachine.add('backup', ActionUntil(MoveForwardAction(-v), TimerListener(5)),
                         transitions={'timeout': 'backup2'})
        StateMachine.add('backup2', ActionUntil(MoveForwardAction(-v), FindMarkerListener(marker_tracker)), transitions={'found': 'find_marker'})

    sis = IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()

    sm.execute()


if __name__ == '__main__':
    rospy.init_node('demo6')
    main()
