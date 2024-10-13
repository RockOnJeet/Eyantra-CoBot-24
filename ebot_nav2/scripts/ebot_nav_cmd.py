from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import rclpy
import tf_transformations as tf

rclpy.init()
nav = BasicNavigator()

# initial_pose:
    #   x: 1.84
    #   y: -9.05
    #   z: 0.1
    #   yaw: 3.14
init_pose = PoseStamped()
init_pose.header.frame_id = 'map'
init_pose.header.stamp = nav.get_clock().now().to_msg()
init_pose.pose.position.x = 1.84
init_pose.pose.position.y = -9.05

quat = tf.quaternion_from_euler(0, 0, 3.14) # roll, pitch, yaw
init_pose.pose.orientation.x = quat[0]
init_pose.pose.orientation.y = quat[1]
init_pose.pose.orientation.z = quat[2]
init_pose.pose.orientation.w = quat[3]
nav.setInitialPose(init_pose)
nav.waitUntilNav2Active() # if autostarted, else use lifecycleStartup()

# Goal Poses [x, y, yaw]:
# P1 :   [ -0.12, -2.35, 3.14 ]
# P2 :   [ 1.86, 2.56, 0.97 ]
# P3 :   [ -3.84, 2.64, 2.78 ]
# Make an array of goal poses and move to them one by one
goal_poses = [
    [-0.12, -2.35, 3.14],
    [1.86, 2.56, 0.97],
    [-3.84, 2.64, 2.78]
]

for pose in goal_poses:
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = pose[0]
    goal_pose.pose.position.y = pose[1]

    quat = tf.quaternion_from_euler(0, 0, pose[2]) # roll, pitch, yaw
    goal_pose.pose.orientation.x = quat[0]
    goal_pose.pose.orientation.y = quat[1]
    goal_pose.pose.orientation.z = quat[2]
    goal_pose.pose.orientation.w = quat[3]

    nav.goToPose(goal_pose)
    while not nav.isTaskComplete():
        pass

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
