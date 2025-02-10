#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
import tf.transformations
import numpy as np
from threading import Lock

class RvizGazeboBridge:
    def __init__(self):
        rospy.init_node('rviz_gazebo_bridge')
        
        # Get parameters
        self.target_name = rospy.get_param('~target_name', 'cube_test')
        self.pose_topic = rospy.get_param('~pose_topic', 'world_pose')
        self.update_rate = rospy.get_param('~update_rate', 50.0)  # Hz
        
        # Initialize locks and flags
        self.lock = Lock()
        self.last_update_time = rospy.Time.now()
        self.min_update_interval = rospy.Duration(1.0 / self.update_rate)
        self.update_source = None  # Track which callback triggered the update
        
        # Initialize pose storage
        self.current_pose = None
        
        # Create publisher for marker updates
        self.marker_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=1)
        
        # Wait for Gazebo services
        rospy.loginfo("Waiting for Gazebo services...")
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # Create subscribers
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback, queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback, queue_size=1)
        
        rospy.loginfo("RViz-Gazebo bridge initialized")

    def normalize_quaternion(self, pose):
        """Normalize the quaternion of the pose"""
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        norm = np.sqrt(sum(x*x for x in q))
        if norm > 0:
            q = [x/norm for x in q]
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
        return pose

    def poses_significantly_different(self, pose1, pose2, threshold=0.0001):
        """Check if poses are significantly different"""
        if not pose1 or not pose2:
            return True
            
        pos_diff = abs(pose1.position.x - pose2.position.x) + \
                   abs(pose1.position.y - pose2.position.y) + \
                   abs(pose1.position.z - pose2.position.z)
                   
        q1 = [pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w]
        q2 = [pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w]
        dot_product = sum(a*b for a, b in zip(q1, q2))
        angle_diff = abs(1.0 - abs(dot_product))
        
        return pos_diff > threshold or angle_diff > threshold

    def pose_callback(self, msg):
        """Handle pose updates from the interactive marker"""
        with self.lock:
            if self.update_source == 'gazebo':
                self.update_source = None
                return
                
            current_time = rospy.Time.now()
            if (current_time - self.last_update_time) < self.min_update_interval:
                return
                
            # Normalize quaternion
            msg.pose = self.normalize_quaternion(msg.pose)
            
            if not self.poses_significantly_different(msg.pose, self.current_pose):
                return
                
            # Create and send ModelState message
            model_state = ModelState()
            model_state.model_name = self.target_name
            model_state.pose = msg.pose
            model_state.reference_frame = "world"
            
            try:
                self.update_source = 'marker'
                self.set_model_state(model_state)
                self.last_update_time = current_time
                self.current_pose = msg.pose
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to set model state: {e}")
            finally:
                self.update_source = None

    def gazebo_callback(self, msg):
        """Handle updates from Gazebo"""
        try:
            model_index = msg.name.index(self.target_name)
            with self.lock:
                if self.update_source == 'marker':
                    return
                    
                gazebo_pose = msg.pose[model_index]
                
                if not self.poses_significantly_different(gazebo_pose, self.current_pose):
                    return
                
                # Update marker position
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = "world"
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.pose = gazebo_pose
                
                self.update_source = 'gazebo'
                self.marker_pub.publish(pose_msg)
                self.current_pose = gazebo_pose
                
        except ValueError:
            pass
        except Exception as e:
            rospy.logerr(f"Error in Gazebo callback: {e}")
        finally:
            self.update_source = None

if __name__ == '__main__':
    try:
        bridge = RvizGazeboBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass