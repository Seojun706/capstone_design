#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def move_to_goal(x, y, z, ox, oy, oz, ow):
    # Initialize the ROS node
    rospy.init_node('move_robot_node', anonymous=True)

    # Create a publisher to the /move_base_simple/goal topic
    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    # Wait for the publisher to establish connection
    rospy.sleep(1)

    # Create a PoseStamped message
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"

    goal.pose.position.x = x    
    goal.pose.position.y = y
    goal.pose.position.z = z

    goal.pose.orientation.x = ox
    goal.pose.orientation.y = oy
    goal.pose.orientation.z = oz
    goal.pose.orientation.w = ow

    # Publish the goal
    goal_publisher.publish(goal)
    rospy.loginfo("Goal published!")

if __name__ == '__main__':
    try:
        # Define the target position and orientation
        x = 20.91334342956543
        y = 12.20483112335205
        z = 0.0
        ox = 0.0
        oy = 0.0
        oz = -0.28978452838612234
        ow = 0.9570919115257597

        # Move the robot to the goal
        move_to_goal(x, y, z, ox, oy, oz, ow)
    except rospy.ROSInterruptException:
        pass
