#!/usr/bin/env python
from positioning.utils import Timer,parse_args
from positioning.positioning import Position
from communications.ros_interface import ROSInterface
from communications.ipc_interface import SocketInterface

def localize():
    args = parse_args()
    # The positioning object
    position = Position(port=args.port,n_sensors=args.num_sensors,ekf=args.ekf,gauss_filter=args.num_filter)
    # Interface to communicate the position and transforms over
    interface = ROSInterface() if args.comm=="ros" else SocketInterface(IP="127.0.0.1", PORT=5005)
    timer = Timer()
    # Initialize the poisiton of the lighthouse
    position.lock_lighthouse(100)
    while(interface.is_alive()):
        # Compute
        position.process_pose()
        position.find_object()
        # Pose
        object_pose = position.get_object_pose_world_frame()
        interface.publish_pose(object_pose,"World Frame")
        # Transforms
        if args.show_transforms==True:
            lighthouse = position.get_lighthouse()
            interface.publish_transform(lighthouse, "Lighthouse Frame", "World Frame")
            object = position.get_object()
            interface.publish_transform(object,"Base Frame","Lighthouse Frame")
        # Measure inference time and print debug
        if args.time==True:
            dt = timer.update()
            print "Inference time:",dt
        if args.print_pose==True:
            print object_pose

if __name__=="__main__":
    localize()
