#!/usr/bin/env python3
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import TransformListener, Buffer
from threading import Thread

from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from linkattacher_msgs.srv import AttachLink, DetachLink

def lookup_multiple_transforms():
    # Create a node
    node = Node('aruco_tf_subscriber')

    # Create a Buffer and TransformListener
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    # The ArUco marker IDs and the frame names they are published as
    marker_frames = ['obj_1', 'obj_3', 'obj_49']

    # Loop to periodically check for the transforms
    while rclpy.ok():
        for marker_frame in marker_frames:
            try:
                # Look up the transform from 'base_link' to each ArUco marker frame
                transform = tf_buffer.lookup_transform(
                    'base_link',  # parent frame (you can change this to your reference frame)
                    marker_frame, # child frame (ArUco marker frame)
                    rclpy.time.Time(),  # latest available transform
                    Duration(seconds=1.0)  # timeout duration
                )

                # Access translation and rotation data
                translation = transform.transform.translation
                rotation = transform.transform.rotation

                # Print the transform details
                node.get_logger().info(f'Transform for {marker_frame} received:\n'
                                       f'Translation: x={translation.x}, y={translation.y}, z={translation.z}\n'
                                       f'Rotation: x={rotation.x}, y={rotation.y}, z={rotation.z}, w={rotation.w}')
            except Exception as e:
                node.get_logger().warn(f'Could not get transform for {marker_frame}: {e}')

        # Sleep for a bit before trying again
        rclpy.spin_once(node, timeout_sec=1.0)

# Joint configurations
A = [-0.47, -0.55, 1.29, -2.35, -1.55, 2.93]
B = [0.36, -0.36, 0.90, -2.14, -1.55, 2.93]
C = [-1.34, -1.58, 1.5, -2.95, -1.7, 3.1]
D = [0.01, -1.97, -0.97, -3.31, -1.69, 3.00]
O = [0.0, -2.39, 2.40, -3.15, -1.57, 3.14]

def gripper_magnet_control(node, service_name, model1_name, operation):
    # Create service client
    if operation == "attach":
        service_client = node.create_client(AttachLink, service_name)
    elif operation == "detach":
        service_client = node.create_client(DetachLink, service_name)

    # Wait for the service to be available
    while not service_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(f'{service_name} service not available, waiting...')

    # Prepare the request
    if operation == "attach":
        req = AttachLink.Request()
    elif operation == "detach":
        req = DetachLink.Request()

    req.model1_name = model1_name  # Box name
    req.link1_name = 'link'  # Assumed link name
    req.model2_name = 'ur5'  # Robot name
    req.link2_name = 'wrist_3_link'  # Robot link

    # Call the service synchronously
    result = service_client.call(req)

    # Check the result
    if result is not None:
        node.get_logger().info(f'{operation.capitalize()} for {model1_name} succeeded')
    else:
        node.get_logger().error(f'{operation.capitalize()} for {model1_name} failed')

# def move_to_joint_position(moveit2, node, joint_positions, position_name):
#     node.get_logger().info(f"Moving to {position_name}: {joint_positions}")
#     moveit2.move_to_configuration(joint_positions)
#     moveit2.wait_until_executed()
#     node.get_logger().info(f"Reached {position_name}")

def move_to_pose(moveit2, node, pose, position_name):
    node.get_logger().info(f"Moving to {position_name}: {pose.position.x}, {pose.position.y}, {pose.position.z}")
    moveit2.move_to_pose(pose)
    moveit2.wait_until_executed()
    node.get_logger().info(f"Reached {position_name}")

def main():
    rclpy.init()

    # Create a node for this example
    node = Node("arm_gripper_control")

    # Create callback group for parallel execution
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),  # UR5 joint names
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in the background
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Define poses (translation + rotation) for the boxes
    box_poses = [
        {
            "pose": {"position": [0.5, 0.2, 0.2], "orientation": [0, 0, 0]},  # Box 49
            "position_name": "box49_pick",
            "gripper_action": "attach",
            "box": "box49"
        },
        {
            "pose": {"position": [-0.3, -0.4, 0.1], "orientation": [0, 0, 1.57]},  # Box 3
            "position_name": "box3_pick",
            "gripper_action": "attach",
            "box": "box3"
        },
        {
            "pose": {"position": [0.4, 0.4, 0.3], "orientation": [0, 1.57, 0]},  # Box 1
            "position_name": "box1_pick",
            "gripper_action": "attach",
            "box": "box1"
        }
    ]

    # Loop through each box pose
    for step in box_poses:
        target_position = step["pose"]["position"]
        target_orientation = step["pose"]["orientation"]

        # Create a Pose message
        pose = Pose()
        pose.position.x = target_position[0]
        pose.position.y = target_position[1]
        pose.position.z = target_position[2]

        # Convert Euler angles to quaternion
        quat = quaternion_from_euler(target_orientation[0], target_orientation[1], target_orientation[2])
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        # Move to the pose
        move_to_pose(moveit2, node, pose, step["position_name"])

        # Perform gripper action if defined
        if "gripper_action" in step and "box" in step:
            if step["gripper_action"] == "attach":
                node.get_logger().info(f"Attaching {step['box']} at {step['position_name']}")
                gripper_magnet_control(node, "/GripperMagnetON", step["box"], "attach")
            elif step["gripper_action"] == "detach":
                node.get_logger().info(f"Detaching {step['box']} at {step['position_name']}")
                gripper_magnet_control(node, "/GripperMagnetOFF", step["box"], "detach")

        # Add delay between moves if needed
        rclpy.sleep(0)

    # Shutdown ROS 2
    rclpy.shutdown()

if __name__ == "__main__":
    main()