#!/usr/bin/env python
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from geometry_msgs.msg import *
from humanoid_nav_msgs.msg import *
from humanoid_nav_msgs.srv import *

class Pose2D(Pose2D):
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        super(Pose2D, self).__init__()
        self.x = round(x, 2)
        self.y = round(y, 2)
        self.theta = round(theta, 2)

class NaoFootsteps(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'nao_footsteps')

        self.connectNaoQi()

        initStiffness = rospy.get_param('~init_stiffness', 0.0)

        # TODO: parameterize
        if initStiffness > 0.0 and initStiffness <= 1.0:
            self.motionProxy.stiffnessInterpolation('Body', initStiffness, 0.5)

		
	    rospy.wait_for_service('plan_footsteps')
        try:
            Plan_Footsteps_Client = rospy.ServiceProxy('plan_footsteps', PlanFootsteps)
            x=Pose2D(0.5,0.3,0)
	    y=Pose2D(2.5,0.3,0)
	    resp1 = Plan_Footsteps_Client(x, y)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
   
        rospy.loginfo("nao_footsteps initialized")
			
    def connectNaoQi(self):
        """(re-) connect to NaoQI"""
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.get_proxy("ALMotion")
        if self.motionProxy is None:
            exit(1)
if __name__ == "__main__":
    walker = NaoFootsteps()
    rospy.loginfo("nao_footsteps running...")
    rospy.spin()

    rospy.loginfo("nao_footsteps stopped.")
    exit(0)
