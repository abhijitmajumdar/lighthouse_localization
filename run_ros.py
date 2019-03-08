#!/usr/bin/env python
import rospy
from positioning.positioning import Position
from positioning.plotting import plot_pose,save_pose_data
from communications.ros_interface import ROSInterface

def localize():
    # Interface to communicate the position and transforms over
    interface = ROSInterface()
    # Get parameters
    port = rospy.get_param('~port', '/dev/ttyS1')
    ekf = bool(rospy.get_param('~ekf', False))
    num_sensors = int(rospy.get_param('~num_sensors', 4))
    num_filter = int(rospy.get_param('~num_filter', 10))
    num_init_itr = int(rospy.get_param('~num_init_itr', 100))
    # The positioning object
    position = Position(port=port,n_sensors=num_sensors,ekf=ekf,gauss_filter=num_filter)
    # Initialize the poisiton of the lighthouse
    position.lock_lighthouse(num_init_itr)
    while(interface.is_alive()):
        # Compute
        position.process_pose()
        position.find_object()
        # Pose
        object_pose = position.get_object_pose_world_frame()
        interface.publish_pose(object_pose,"World Frame")
        # Transforms
        lighthouse = position.get_lighthouse()
        interface.publish_transform(lighthouse, "Lighthouse Frame", "World Frame")
        object = position.get_object()
        interface.publish_transform(object,"Base Frame","Lighthouse Frame")

if __name__=="__main__":
    localize()
