import time
import sys
import signal
import argparse
import numpy as np

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

class SignalHandler:
    def __init__(self):
        self.run = True
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        self.run = False

    def running(self):
        return self.run

def parse_args():
    parser = argparse.ArgumentParser(description="Lighthouse Localization")
    parser.add_argument("--port", "-p", help="dev port for sensor interface",default='/dev/ttyS1')
    parser.add_argument("--ekf", "-ekf", action='store_true', help="Use EKF for filtering")
    parser.add_argument("--time", "-t", action='store_true', help="Time the inferences")
    parser.add_argument("--show_transforms", "-tr", action='store_true', help="Publish tf transforms")
    parser.add_argument("--print_pose", "-pp", action='store_true', help="Print out pose estimation")
    parser.add_argument("--record","-r", default=None, help="Record pose data into file")
    parser.add_argument("--plot", "-plot", default=None, help="Path to plot saved data")
    parser.add_argument("--comm","-c", default="ros", help="Form of communication to use (ros/ipc)")
    parser.add_argument("--num_sensors","-ns", default=4, type=int, help="Number of sensors")
    parser.add_argument("--num_filter","-nf", default=10, type=int, help="Value passed to gaussian filter")
    return parser.parse_args()
