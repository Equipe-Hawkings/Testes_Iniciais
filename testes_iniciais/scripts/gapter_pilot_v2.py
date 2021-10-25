#!/usr/bin/env python3
import rospy
import math
import time
import numpy as np

from control.Principal import Principal
from geometry_msgs.msg import TwistStamped, PoseStamped

class Auto(Principal):
    def __init__(self):
        super(Auto, self).__init__()
    
    def run(self):
        for i in range (150):
            self.setpoint_position_local(0,0,0)
        self.arm(True)
        while not rospy.is_shutdown():
            if(self.current_mode.mode != "OFFBOARD"):
                self.set_mode("OFFBOARD")
            else:  
                self.setpoint_position_local(0,0,2)


        
if __name__ == "__main__":
    rospy.init_node('gapter_pilot_v2')
    drone = Auto()
    drone.run()

