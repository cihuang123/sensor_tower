#!/usr/bin/env python3

import rospy 
# from laser_assembler.srv import AssembleScans2
from sensor_msgs.msg import PointCloud2, LaserScan
from laser_geometry import LaserProjection

class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcpub = rospy.Publisher("/mmwave_scan_pcl",PointCloud2, queue_size=1)
        self.lasersub = rospy.Subscriber("/husky2/RL/scan_mmwave",LaserScan,self.lasercb)

    def lasercb(self, data):
        cloud_out = self.laserProj.projectLaser(data)
        self.pcpub.publish(cloud_out)

if __name__=='__main__':
    rospy.init_node("laser2PointCloud")
    l2pc = Laser2PC()
    rospy.spin()