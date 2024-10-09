#!/usr/bin/env python3
import rclpy
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from threading import Thread

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from linkattacher_msgs.srv import AttachLink, DetachLink

# Joint configurations
A = [-0.47, -0.55, 1.29, -2.35, -1.55, 2.93]
B = [0.36, -0.36, 0.90, -2.14, -1.55, 2.93]
C = [-1.34, -1.58, 1.5, -2.95, -1.7, 3.1]
D = [0.01, -1.97, -0.97, -3.31, -1.69, 3.00]
D1= [0.01, -1.79, -1.32, -3.31, -1.69, 3.00]
D2= [0.01, -2.21, -0.97, -3.31, -1.69, 3.00]
O = [0.0, -2.39, 2.40, -3.15, -1.57, 3.14]

def gripper_magnet_control(node, service_name, model1_name, operation):
    # Create service client
    if operation == "attach":
        service_client = node.create_client(AttachLink, service_name)
    elif operation == "detach":
        service_client = node.create_client(DetachLink, service_name)

    # Wait for the service to be available
    while not service_client.wait_for_service(timeout_sec=0.0):
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

def move_to_position(moveit2, node, joint_positions, position_name):
    node.get_logger().info(f"Moving to {position_name}: {joint_positions}")
    moveit2.move_to_configuration(joint_positions)
    moveit2.wait_until_executed()
    node.get_logger().info(f"Reached {position_name}")

def main():
    rclpy.init()

    # Create node for this example
    node = Node("arm_gripper_control")

    # Create callback group for parallel execution
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
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

    # Movement sequence with gripper actions
    movements = [
        {"position": A, "position_name": "A", "gripper_action": "attach", "box": "box49"},
       # {"position": O, "position_name": "O"},
        {"position": D1, "position_name": "D1", "gripper_action": "detach", "box": "box49"},
       # {"position": O, "position_name": "O"},
        {"position": B, "position_name": "B", "gripper_action": "attach", "box": "box3"},
       # {"position": O, "position_name": "O"},
        {"position": D2, "position_name": "D2", "gripper_action": "detach", "box": "box3"},
       # {"position": O, "position_name": "O"},
        {"position": C, "position_name": "C", "gripper_action": "attach", "box": "box1"},
       # {"position": O, "position_name": "O"},
        {"position": D, "position_name": "D", "gripper_action": "detach", "box": "box1"},
        {"position": O, "position_name": "O"}
    ]

    # Perform each movement in sequence
    for step in movements:
        joint_positions = step["position"]
        position_name = step["position_name"]

        # Move to the desired joint position
        move_to_position(moveit2, node, joint_positions, position_name)

        # Perform gripper action if defined
        if "gripper_action" in step and "box" in step:
            if step["gripper_action"] == "attach":
                node.get_logger().info(f"Attaching {step['box']} at {position_name}")
                gripper_magnet_control(node, "/GripperMagnetON", step["box"], "attach")
            elif step["gripper_action"] == "detach":
                node.get_logger().info(f"Detaching {step['box']} at {position_name}")
                gripper_magnet_control(node, "/GripperMagnetOFF", step["box"], "detach")

        # Short delay between moves for safety
        time.sleep(0)

    # Shutdown ROS 2
    rclpy.shutdown()

if __name__ == "__main__":
    main()
