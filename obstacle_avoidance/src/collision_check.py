#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive

class CollisionObjectManager:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('collision_object_manager')

        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.group_name = "manipulator"  # Replace with your planning group
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        self.marker_pub = rospy.Publisher('collision_object_marker', PlanningScene, queue_size=10)


    def add_collision_object(self, object_name, object_pose, object_size):
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.robot.get_planning_frame()
        collision_object.id = object_name

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = object_size

        object_pose_stamped = PoseStamped()
        object_pose_stamped.header.frame_id = self.robot.get_planning_frame()
        object_pose_stamped.pose = object_pose

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(object_pose_stamped.pose)
        collision_object.operation = CollisionObject.ADD

        rospy.sleep(0.5)  # Add a delay to ensure the collision object is added before planning
        self.scene.add_box(object_name, object_pose_stamped, size=object_size)
        
        # Publish collision object marker for visualization in RViz
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        self.marker_pub.publish(planning_scene)

    def check_collision(self):
        return self.group.is_collisions()

if __name__ == '__main__':
    try:
        collision_manager = CollisionObjectManager()

        # Define collision object parameters
        object_name = "cluster_x"
        object_pose = Pose()
        object_pose.position.x = 0.0
        object_pose.position.y = 0.4
        object_pose.position.z = 0.6
        object_size = [0.1, 0.1, 0.1]  # Dimensions of the cube

        # Add collision object to the planning scene
        collision_manager.add_collision_object(object_name, object_pose, object_size)
        
        '''
        # Check for collisions
        collision_result = collision_manager.check_collision()

        if collision_result:
            rospy.logerr("Collision detected!")
        else:
            rospy.loginfo("No collision detected.")
        '''

    except rospy.ROSInterruptException:
        pass
