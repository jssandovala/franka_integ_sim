#!/usr/bin/env python3

import rospy
import tf.transformations

from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, \
    InteractiveMarkerControl, Marker
from geometry_msgs.msg import PoseStamped

class InteractiveMarkerNode:
    def __init__(self):
        rospy.init_node("target_pose_node")
        
        
        self.link_name = rospy.get_param("~link_name")
        self.frame = rospy.get_param("~frame")
        self.scale = rospy.get_param("~scale")

        # Initialize pose
        self.marker_pose = PoseStamped()
        
        self.updating_marker = False

        # publisher
        self.pose_pub = rospy.Publisher(
            self.frame + "_pose", PoseStamped, queue_size=10)
            
        # Subscribe 
        rospy.Subscriber(
            self.frame + "_pose", PoseStamped, self.external_pose_callback)

       
        self.server = InteractiveMarkerServer("target_pose_marker")
        self.create_interactive_marker()


    def create_interactive_marker(self):

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.link_name
        int_marker.scale = self.scale
        int_marker.name = self.frame
        int_marker.description = self.frame
        int_marker.pose = self.marker_pose.pose

        # control = InteractiveMarkerControl()
        # control.orientation.w = 1
        # control.orientation.x = 1
        # control.orientation.y = 0
        # control.orientation.z = 0
        # control.name = "rotate_x"
        # control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        # int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        # control = InteractiveMarkerControl()
        # control.orientation.w = 1
        # control.orientation.x = 0
        # control.orientation.y = 1
        # control.orientation.z = 0
        # control.name = "move_y"
        # control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        # int_marker.controls.append(control)

        # control = InteractiveMarkerControl()
        # control.orientation.w = 1
        # control.orientation.x = 0
        # control.orientation.y = 0
        # control.orientation.z = 1
        # control.name = "rotate_z"
        # control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        # int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        if self.updating_marker:
            return
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.marker_pose.pose = feedback.pose
            self.marker_pose.header.stamp = rospy.Time.now()
            self.pose_pub.publish(self.marker_pose)

    def external_pose_callback(self, msg):
        if self.updating_marker:
            return
        try:
            self.updating_marker = True
            int_marker = self.server.get(self.frame)
            if int_marker:
                int_marker.pose = msg.pose
                self.server.insert(int_marker)
                self.server.applyChanges() 
            self.marker_pose = msg
        finally:
            self.updating_marker = False

if __name__ == "__main__":
    try:
        node = InteractiveMarkerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass