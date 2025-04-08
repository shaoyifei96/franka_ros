#!/usr/bin/env python

import rospy
import tf.transformations
import tf
import numpy as np

from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, \
    InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
from scipy.spatial.transform import Rotation as R

marker_pose = PoseStamped()
pose_pub = None
target_pub = None
target_pose = PoseStamped()
tf_listener = None  # Initialize tf_listener globally

# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]


def publisher_callback(msg, link_name):
    marker_pose.header.frame_id = link_name
    marker_pose.header.stamp = rospy.Time(0)
    pose_pub.publish(marker_pose)


def process_feedback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        # these never run if I don't touch the gui
        # rospy.info_throttle(1.0, "Got update")
        marker_pose.pose.position.x = max([min([feedback.pose.position.x,
                                          position_limits[0][1]]),
                                          position_limits[0][0]])
        marker_pose.pose.position.y = max([min([feedback.pose.position.y,
                                          position_limits[1][1]]),
                                          position_limits[1][0]])
        marker_pose.pose.position.z = max([min([feedback.pose.position.z,
                                          position_limits[2][1]]),
                                          position_limits[2][0]])
        marker_pose.pose.orientation = feedback.pose.orientation # Keep orientation update
    server.applyChanges()


def wait_for_initial_pose(link_name):
    global tf_listener # Use global listener
    msg = rospy.wait_for_message("franka_state_controller/franka_states",
                                 FrankaState)  # type: FrankaState

    # Get transform from O_T_EE (base to EE)
    O_T_EE = np.transpose(np.reshape(msg.O_T_EE, (4, 4)))
    
    # We need pose in link0 frame, not base frame O.
    # Use TF to get the pose relative to link_name (panda_link0)
    initial_pose_base = PoseStamped()
    # Set the known base frame_id explicitly, don't rely on msg.header.frame_id
    initial_pose_base.header.frame_id = "panda_link0" 
    initial_pose_base.header.stamp = rospy.Time(0) # Use latest available transform
    initial_pose_base.pose.position.x = msg.O_T_EE[12]
    initial_pose_base.pose.position.y = msg.O_T_EE[13]
    initial_pose_base.pose.position.z = msg.O_T_EE[14]
    initial_quaternion = tf.transformations.quaternion_from_matrix(O_T_EE)
    initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)
    initial_pose_base.pose.orientation.x = initial_quaternion[0]
    initial_pose_base.pose.orientation.y = initial_quaternion[1]
    initial_pose_base.pose.orientation.z = initial_quaternion[2]
    initial_pose_base.pose.orientation.w = initial_quaternion[3]

    try:
        # Ensure the listener is ready
        tf_listener.waitForTransform(link_name, initial_pose_base.header.frame_id, rospy.Time(0), rospy.Duration(4.0))
        # Transform the initial pose from the robot base frame to the target link_name frame
        initial_pose_link0 = tf_listener.transformPose(link_name, initial_pose_base)
        marker_pose.pose = initial_pose_link0.pose # Set marker pose directly from transformed pose

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("Failed to transform initial pose: %s", e)
        # Fallback or error handling needed here - maybe use O_T_EE directly as a rough estimate?
        rospy.logwarn("Falling back to using O_T_EE for initial pose relative to base frame.")
        marker_pose.pose = initial_pose_base.pose


ee_pose = PoseStamped()  # Global variable to store the current end-effector pose

def franka_state_callback(msg):
    # Update the global ee_pose with the current end-effector pose from FrankaState
    # This pose is typically in the robot's base frame (e.g., panda_link0)
    global ee_pose
    ee_pose.header.frame_id = msg.header.frame_id # Use frame_id from message
    ee_pose.header.stamp = rospy.Time.now() # Use current time for potentially transforming later
    ee_pose.pose.position.x = msg.O_T_EE[12]
    ee_pose.pose.position.y = msg.O_T_EE[13]
    ee_pose.pose.position.z = msg.O_T_EE[14]

    # Extract and normalize the quaternion
    quaternion = tf.transformations.quaternion_from_matrix(
        np.transpose(np.reshape(msg.O_T_EE, (4, 4)))
    )
    quaternion = quaternion / np.linalg.norm(quaternion)
    ee_pose.pose.orientation.x = quaternion[0]
    ee_pose.pose.orientation.y = quaternion[1]
    ee_pose.pose.orientation.z = quaternion[2]
    ee_pose.pose.orientation.w = quaternion[3]

def door_handle_callback(door_handle_pose_mocap):
    global ee_pose, target_pose, tf_listener # Use global listener

    # Assume door_handle_pose_mocap is in 'mocap_world' frame based on context
    # Verify frame_id if possible, default to 'mocap_world' if empty
    if not door_handle_pose_mocap.header.frame_id == "mocap_world":
        rospy.logwarn_throttle(5.0, f"Incoming door handle pose has frame_id {door_handle_pose_mocap.header.frame_id}, that is a problem, assuming 'mocap_world'")
        door_handle_pose_mocap.header.frame_id = "mocap_world"

    try:
        # Transform the mocap pose to the robot's link0 frame
        target_link_frame = rospy.get_param("~link_name") # Get link_name parameter
        tf_listener.waitForTransform(target_link_frame, door_handle_pose_mocap.header.frame_id, door_handle_pose_mocap.header.stamp, rospy.Duration(1.0))
        door_handle_pose_link0 = tf_listener.transformPose(target_link_frame, door_handle_pose_mocap)

    # Normalize quaternions
    # ee_quat = ee_quat / np.linalg.norm(ee_quat)
    # handle_init_rot = door_handle_quat / np.linalg.norm(door_handle_quat)

        handle_pos_link0 = np.array([door_handle_pose_link0.pose.position.x,
                                     door_handle_pose_link0.pose.position.y,
                                     door_handle_pose_link0.pose.position.z])
        handle_quat_link0 = np.array([door_handle_pose_link0.pose.orientation.x,
                                      door_handle_pose_link0.pose.orientation.y,
                                      door_handle_pose_link0.pose.orientation.z,
                                      door_handle_pose_link0.pose.orientation.w])

        handle_rot_link0 = R.from_quat(handle_quat_link0)

        # Define the desired EE orientation relative to the handle
        # These rotations define the EE frame relative to the handle frame
        # Adjust these based on how the handle should be grasped
        rotmat_grasp_offset_1 = np.array( [[0,0,-1],
                                           [0,-1,0],
                                           [-1,0,0]])
        # rotmat_grasp_offset_2 = np.array( [[-1, 0, 0],
        #                                    [0, -1, 0],
        #                                    [0, 0, 1]])
        grasp_offset_rot_1 = R.from_matrix(rotmat_grasp_offset_1)
        # grasp_offset_rot_2 = R.from_matrix(rotmat_grasp_offset_2)

        # Calculate the target EE rotation in the link0 frame
        # Target EE orientation = Handle orientation * Desired Grasp Offset rotations
        target_rot_link0 = handle_rot_link0 * grasp_offset_rot_1 #* grasp_offset_rot_2

        # Calculate the target EE position in the link0 frame
        # Define the desired EE position offset relative to the handle frame (in handle's coords)
        # e.g., move 13cm back along the handle's Z-axis
        position_offset_handle_frame = np.array([0.12, 0, 0.0])
        # Transform this offset vector from handle frame to link0 frame
        position_offset_link0_frame = handle_rot_link0.apply(position_offset_handle_frame)

        # Target EE position = Handle position + Offset (all in link0 frame)
        target_pos_link0 = handle_pos_link0 + position_offset_link0_frame

        # Update the target_pose (used for publishing and potentially smoothing)
        target_pose.header.frame_id = target_link_frame # Frame is panda_link0
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = target_pos_link0[0]
        target_pose.pose.position.y = target_pos_link0[1]
        target_pose.pose.position.z = target_pos_link0[2]
        target_quat_link0 = target_rot_link0.as_quat()
        target_pose.pose.orientation.x = target_quat_link0[0]
        target_pose.pose.orientation.y = target_quat_link0[1]
        target_pose.pose.orientation.z = target_quat_link0[2]
        target_pose.pose.orientation.w = target_quat_link0[3]

        target_pub.publish(target_pose)  # Publish the target pose for debugging

        # Smoothly update the marker pose towards the calculated target pose
        # (Ensure marker_pose is also in target_link_frame (panda_link0))
        marker_pose.header.frame_id = target_link_frame # Ensure marker frame matches
        marker_pose.pose.position.x += (target_pose.pose.position.x - marker_pose.pose.position.x) * 0.05
        marker_pose.pose.position.y += (target_pose.pose.position.y - marker_pose.pose.position.y) * 0.05
        marker_pose.pose.position.z += (target_pose.pose.position.z - marker_pose.pose.position.z) * 0.05
        # Smooth orientation update (using slerp for quaternions)
        q_marker = [marker_pose.pose.orientation.x, marker_pose.pose.orientation.y, marker_pose.pose.orientation.z, marker_pose.pose.orientation.w]
        q_target = [target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w]
        q_interpolated = tf.transformations.quaternion_slerp(q_marker, q_target, 0.1) # Interpolate 10% towards target
        marker_pose.pose.orientation.x = q_interpolated[0]
        marker_pose.pose.orientation.y = q_interpolated[1]
        marker_pose.pose.orientation.z = q_interpolated[2]
        marker_pose.pose.orientation.w = q_interpolated[3]

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr_throttle(1.0, "TF Error in door_handle_callback: %s", e)
    except Exception as e:
        rospy.logerr_throttle(1.0, "Error in door_handle_callback: %s", e)


if __name__ == "__main__":
    rospy.init_node("equilibrium_pose_node")
    tf_listener = tf.TransformListener() # Initialize the listener
    link_name = rospy.get_param("~link_name")

    # Wait for TF buffer to fill
    rospy.sleep(1.0)

    wait_for_initial_pose(link_name) # Pass link_name to use for transform

    # Subscribe to Franka state updates
    rospy.Subscriber("franka_state_controller/franka_states", FrankaState, franka_state_callback)

    pose_pub = rospy.Publisher(
        "equilibrium_pose", PoseStamped, queue_size=10)
    target_pub = rospy.Publisher(
        "target_pose", PoseStamped, queue_size=10)  # For debugging target rotations
    server = InteractiveMarkerServer("equilibrium_pose_marker")

    # Subscribe to the door handle pose topic (e.g., /natnet_ros/DoorHandle/pose)
    # Make sure the topic name matches where the pose is published
    rospy.Subscriber("/natnet_ros/DoorHandle/pose", PoseStamped, door_handle_callback)

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = link_name # Use the link_name parameter
    int_marker.scale = 0.3
    int_marker.name = "equilibrium_pose"
    int_marker.description = ("Equilibrium Pose\nBE CAREFUL! "
                              "If you move the \nequilibrium "
                              "pose the robot will follow it\n"
                              "so be aware of potential collisions")
    # marker_pose should be initialized by wait_for_initial_pose in the correct frame
    int_marker.pose = marker_pose.pose
    # run pose publisher (publishes marker_pose)
    rospy.Timer(rospy.Duration(0.005),
                lambda msg: publisher_callback(msg, link_name)) # Pass link_name

    # --- Interactive Marker Controls ---
    # (Keep existing controls, they operate relative to the marker's frame_id)
    # insert a box (Rotation controls)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

    # (Translation controls)
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
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    # --- End Controls ---

    server.insert(int_marker, process_feedback)

    server.applyChanges()

    rospy.spin()
