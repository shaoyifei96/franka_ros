#!/usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from scipy.spatial.transform import Rotation as R

class StonePosePublisher:
    def __init__(self):
        rospy.init_node('stone_pose_publisher')
        
        # Create publisher for stone pose
        self.pose_pub = rospy.Publisher('/natnet_ros/DoorHandle/pose', 
                                      PoseStamped, 
                                      queue_size=10)
        
        # Subscribe to Gazebo model states
        rospy.Subscriber('/gazebo/model_states', 
                        ModelStates, 
                        self._model_states_callback)
        
        # Initialize transform listener
        self.tf_listener = tf.TransformListener()
        
        # Wait for transform to be available
        rospy.sleep(1.0)
        
        # Get the transform from world to mocap_world
        try:
            self.tf_listener.waitForTransform('mocap_world', 'world', 
                                           rospy.Time(), 
                                           rospy.Duration(4.0))
        except (tf.LookupException, tf.ConnectivityException, 
                tf.ExtrapolationException) as e:
            rospy.logerr(f"Failed to get transform: {e}")
            return

    def _model_states_callback(self, msg):
        """Callback for Gazebo model states."""
        try:
            # Find the stone in the model states
            stone_idx = msg.name.index('stone')
            
            # Get stone pose
            stone_pose = msg.pose[stone_idx]
            
            # Create PoseStamped message in world frame
            pose_stamped_world = PoseStamped()
            pose_stamped_world.header.frame_id = 'world'
            pose_stamped_world.header.stamp = rospy.Time.now()
            
            # Set position
            pose_stamped_world.pose.position.x = stone_pose.position.x
            pose_stamped_world.pose.position.y = stone_pose.position.y
            pose_stamped_world.pose.position.z = stone_pose.position.z
            
            # Set orientation
            pose_stamped_world.pose.orientation = stone_pose.orientation
            
            # Transform the pose from world frame to mocap_world frame
            try:
                pose_stamped_mocap = self.tf_listener.transformPose('mocap_world', pose_stamped_world)
                
                # Publish the transformed pose in mocap_world frame
                self.pose_pub.publish(pose_stamped_mocap)
            except (tf.LookupException, tf.ConnectivityException, 
                    tf.ExtrapolationException) as e:
                rospy.logwarn_throttle(1.0, f"Transform failed: {e}")
            
        except ValueError:
            rospy.logwarn_throttle(1.0, "Stone not found in model states")
        except Exception as e:
            rospy.logerr_throttle(1.0, f"Error in model states callback: {e}")

    def run(self):
        """Main loop."""
        rate = rospy.Rate(100)  # 100 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        publisher = StonePosePublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass 