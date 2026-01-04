import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pose(navigator, x, y, z_rotation):
    """Helper to create a PoseStamped message easily"""
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # Position
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = 0.0
    
    # Orientation (convert z_rotation in degrees to quaternion)
    # If you don't care about rotation, you can leave these as 0,0,0,1
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, z_rotation)
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    
    return goal_pose

def main():
    rclpy.init()
    
    # --- 1. Init Navigator ---
    navigator = BasicNavigator()
    
    # --- 2. Set Initial Pose (Optional but recommended) ---
    # If you launch the sim and the robot is at 0,0, you usually don't need this.
    # But if the robot is "lost", you set this to where the robot actually is.
    # initial_pose = create_pose(navigator, 0.0, 0.0, 0.0)
    # navigator.setInitialPose(initial_pose)
    
    # Wait for navigation to activate
    navigator.waitUntilNav2Active()

    # --- 3. Define the Goal ---
    # Change x=2.0, y=1.0 to wherever you want the robot to go
    goal_pose = create_pose(navigator, 2.0, 1.0, 0.0)

    print("Sending robot to goal...")
    navigator.goToPose(goal_pose)

    # --- 4. Monitor Progress ---
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        # You can print feedback here if you want (e.g. distance remaining)
        # print(f'Distance remaining: {feedback.distance_remaining:.2f}')

    # --- 5. Result ---
    result = navigator.getResult()
    if result == 1: # TaskSucceeded
        print('Goal succeeded!')
    else:
        print(f'Goal failed with result code: {result}')

    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()