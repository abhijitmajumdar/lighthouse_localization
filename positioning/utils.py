import time
import sys
import argparse

class Timer:
    def __init__(self):
        self._t_log = time.time()

    def update(self,show=False):
        _t_log_now = time.time()
        dt = _t_log_now-self._t_log
        self._t_log = _t_log_now
        if show==True:
            print dt
        return dt

def parse_args():
    parser = argparse.ArgumentParser(description="Lighthouse Localization")
    parser.add_argument("--port", "-p", help="dev port for sensor interface",default='/dev/ttyS1')
    parser.add_argument("--ekf", "-ekf", action='store_true', help="Use EKF for filtering")
    parser.add_argument("--time", "-t", action='store_true', help="Time the inferences")
    parser.add_argument("--show_transforms", "-tr", action='store_true', help="Publish tf transforms")
    parser.add_argument("--print_pose", "-pp", action='store_true', help="Print out pose estimation")
    parser.add_argument("--comm","-c", default="ros", help="Form of communication to use (ros/ipc)")
    parser.add_argument("--num_sensors","-ns", default=4, type=int, help="Number of sensors")
    parser.add_argument("--num_filter","-nf", default=10, type=int, help="Value passed to gaussian filter")
    return parser.parse_args()
