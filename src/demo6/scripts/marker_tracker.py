import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
from typing import Optional, Set
from threading import Lock


class MarkerTracker:
    """
    Tracks markers across multiple frames. If a marker is seen in get_visible_markers(), it is guaranteed to have
    a pose available via get_pose().
    """
    def __init__(self):
        self.ar_tags_subscriber = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self._ar_callback)
        self.poses = {}
        self.visible_markers = set()

    def _ar_callback(self, msg):  # type: (AlvarMarkers) -> None
        visible_markers = set()
        for m in msg.markers:
            if m.id != 0 and m.id != 255:
                self.poses[m.id] = m.pose
                visible_markers.add(m.id)
        self.visible_markers = visible_markers

    def get_pose(self, marker_id):  # type: (int) -> Optional[PoseStamped]
        return self.poses.get(marker_id, None)

    def get_visible_markers(self):  # type: () -> Set[int]
        return self.visible_markers
