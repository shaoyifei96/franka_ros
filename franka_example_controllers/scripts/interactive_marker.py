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
panda_link0_pos = np.array([-2.51, 0.65, 0.90])

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
        # marker_pose.pose.orientation = feedback.pose.orientation
    server.applyChanges()


def wait_for_initial_pose():
    msg = rospy.wait_for_message("franka_state_controller/franka_states",
                                 FrankaState)  # type: FrankaState

    initial_quaternion = \
        tf.transformations.quaternion_from_matrix(
            np.transpose(np.reshape(msg.O_T_EE,
                                    (4, 4))))
    initial_quaternion = initial_quaternion / \
        np.linalg.norm(initial_quaternion)
    marker_pose.pose.orientation.x = initial_quaternion[0]
    marker_pose.pose.orientation.y = initial_quaternion[1]
    marker_pose.pose.orientation.z = initial_quaternion[2]
    marker_pose.pose.orientation.w = initial_quaternion[3]
    marker_pose.pose.position.x = msg.O_T_EE[12]
    marker_pose.pose.position.y = msg.O_T_EE[13]
    marker_pose.pose.position.z = msg.O_T_EE[14]


ee_pose = PoseStamped()  # Global variable to store the current end-effector pose

def franka_state_callback(msg):
    # Update the global ee_pose with the current end-effector pose from FrankaState
    global ee_pose
    ee_pose.header.frame_id = "panda_link0"
    ee_pose.header.stamp = rospy.Time.now()
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

def door_handle_callback(door_handle_pose):
    # Use the global ee_pose to compute the difference
    global ee_pose, target_pose, panda_link0_pos

    # look up transform of panda_link0 in mocap_world frame
    # This function is called when a new door handle pose is received
    # Ensure that ee_pose is available before proceeding
    
    # Convert quaternions to numpy arrays
    ee_quat = np.array([ee_pose.pose.orientation.x, ee_pose.pose.orientation.y,
                        ee_pose.pose.orientation.z, ee_pose.pose.orientation.w])
    door_handle_quat = np.array([door_handle_pose.pose.orientation.x,
                                  door_handle_pose.pose.orientation.y,
                                  door_handle_pose.pose.orientation.z,
                                  door_handle_pose.pose.orientation.w])

    # Normalize quaternions
    ee_quat = ee_quat / np.linalg.norm(ee_quat)
    handle_init_rot = door_handle_quat / np.linalg.norm(door_handle_quat)

    rotmat = np.array( [[0,0,-1],
                        [0,-1,0],
                        [-1,0,0]])  # This is the expected relative rotation matrix for the door handle

    rotmat2 = np.array( [[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
    expected_relative_rot_handle = R.from_matrix(rotmat)
    expected_relative_rot_handle_2 = R.from_matrix(rotmat2)
    # This is to simulate the expected relative rotation of the door handle
    # relative to the end-effector
    target_rot =  R.from_quat(handle_init_rot) * expected_relative_rot_handle * expected_relative_rot_handle_2
    # publish the target rotation as a PoseStamped for debugging
    # target_pose = PoseStamped()
    target_pose.header.frame_id = "panda_link0"  # Set the frame_id to match the end-effector's frame
    target_pose.header.stamp = rospy.Time.now()
    posx  = door_handle_pose.pose.position.x - panda_link0_pos[0]
    posy  = door_handle_pose.pose.position.y - panda_link0_pos[1]
    posz  = door_handle_pose.pose.position.z - panda_link0_pos[2]

    # Compute the offset in the target_rot frame
    offset = np.array([0, 0, -0.13])  # Offset of 0.1 m in the -z direction
    offset_in_target_frame = target_rot.apply(offset)

    # Apply the offset to the target position
    posx += offset_in_target_frame[0]
    posy += offset_in_target_frame[1]
    posz += offset_in_target_frame[2]
    target_pose.pose.position.x = posx  # Adjust the position based on the door handle's position
    target_pose.pose.position.y = posy
    target_pose.pose.position.z = posz  # Adjust the position based on the door handle's position
    target_quat = target_rot.as_quat()
    target_pose.pose.orientation.x = target_quat[0]
    target_pose.pose.orientation.y = target_quat[1]
    target_pose.pose.orientation.z = target_quat[2]
    target_pose.pose.orientation.w = target_quat[3]
    target_pub.publish(target_pose)  # Publish the target pose for debugging

    marker_pose.pose.position.x = marker_pose.pose.position.x + (target_pose.pose.position.x - marker_pose.pose.position.x) * 0.05
    marker_pose.pose.position.y = marker_pose.pose.position.y + (target_pose.pose.position.y - marker_pose.pose.position.y) * 0.05
    marker_pose.pose.position.z = marker_pose.pose.position.z + (target_pose.pose.position.z - marker_pose.pose.position.z) * 0.05
    marker_pose.pose.orientation = target_pose.pose.orientation  # Use target pose orientation for now


if __name__ == "__main__":
    rospy.init_node("equilibrium_pose_node")
    listener = tf.TransformListener()
    link_name = rospy.get_param("~link_name")

    # (panda_link0_pos, _) = listener.lookupTransform('mocap_world', 'panda_link0', rospy.Time(0))


    wait_for_initial_pose()

    # Subscribe to Franka state updates
    rospy.Subscriber("franka_state_controller/franka_states", FrankaState, franka_state_callback)

    pose_pub = rospy.Publisher(
        "equilibrium_pose", PoseStamped, queue_size=10)
    target_pub = rospy.Publisher(
        "target_pose", PoseStamped, queue_size=10)  # For debugging target rotations
    server = InteractiveMarkerServer("equilibrium_pose_marker")

    # Subscribe to the door handle pose topic
    rospy.Subscriber("/natnet_ros/DoorHandle/pose", PoseStamped, door_handle_callback)

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = link_name
    int_marker.scale = 0.3
    int_marker.name = "equilibrium_pose"
    int_marker.description = ("Equilibrium Pose\nBE CAREFUL! "
                              "If you move the \nequilibrium "
                              "pose the robot will follow it\n"
                              "so be aware of potential collisions")
    int_marker.pose = marker_pose.pose
    # run pose publisher
    rospy.Timer(rospy.Duration(0.005),
                lambda msg: publisher_callback(msg, link_name))

    # insert a box
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

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



    server.insert(int_marker, process_feedback)

    server.applyChanges()


    rospy.spin()
