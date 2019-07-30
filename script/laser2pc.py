#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection


class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
#        self.inMsg = rospy.get_param('~inMsg')
#        self.outMsg = rospy.get_param('~outMsg')
        self.inMsg = '/scan'
        self.outMsg = '/converted_pc'
        self.pcPub = rospy.Publisher(self.outMsg,pc2,queue_size=1)
        self.laserSub = rospy.Subscriber(self.inMsg,LaserScan,self.laserCallback)
        self.count = 0

    def laserCallback(self,data):

        cloud_out = self.laserProj.projectLaser(data)
        cloud_out.header = data.header
        cloud_out.header.frame_id = self.outMsg
        self.pcPub.publish(cloud_out)
        self.count += 1

        print("Publish complete!" + str(self.count))


if __name__=='__main__':
    print("Operating laser scan listener node...")
    rospy.init_node("laser2PointCloud")
    l2pc = Laser2PC()
    rospy.spin()


