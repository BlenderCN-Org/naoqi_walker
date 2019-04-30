# -*- encoding: UTF-8 -*-


import rospy

from naoqi_driver.naoqi_node import NaoqiNode


import actionlib

from humanoid_nav_msgs.msg import *
from geometry_msgs.msg import *

# redefine StepTarget by inheritance
class StepTarget(StepTarget):
    def __init__(self, x=0.0, y=0.0, theta=0.0, leg=0.0):
        super(StepTarget, self).__init__()
        self.pose.x = round(x, 4)
        self.pose.y = round(y, 4)
        self.pose.theta = round(theta, 4)
        self.leg = leg

    def __eq__(self, a):
        return (float_equ(self.pose.x, a.pose.x) and
                float_equ(self.pose.y, a.pose.y) and
                float_equ(self.pose.theta, a.pose.theta) and
                self.leg == a.leg)

    def __ne__(self, a):
        return not (self == a)

    def __str__(self):
        return "(%f, %f, %f, %i)" % (self.pose.x, self.pose.y, self.pose.theta,
                                     self.leg)

    def __repr__(self):
        return self.__str__()
# redefine Pose2D by inheritance
class Pose2D(Pose2D):
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        super(Pose2D, self).__init__()
        self.x = round(x, 2)
        self.y = round(y, 2)
        self.theta = round(theta, 2)

    def __eq__(self, a):
        return (float_equ(self.x, a.x) and
                float_equ(self.y, a.y) and
                float_equ(self.theta, a.theta))

    def __ne__(self, a):
        return not (self == a)

    def __str__(self):
        return "(%f, %f, %f, %i)" % (self.x, self.y, self.theta)

    def __repr__(self):
        return self.__str__()

class NaoFootsteps(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'nao_leds')

        self.connectNaoQi()

        # initial stiffness (defaults to 0 so it doesn't strain the robot when
        # no teleoperation is running)
        # set to 1.0 if you want to control the robot immediately
        initStiffness = rospy.get_param('~init_stiffness', 0.0)

        # TODO: parameterize
        if initStiffness > 0.0 and initStiffness <= 1.0:
            self.motionProxy.stiffnessInterpolation('Body', initStiffness, 0.5)

        # last: ROS subscriptions (after all vars are initialized)
        #rospy.Subscriber("footstep", PlanFootsteps, self.handleStep, queue_size=50)
		
		rospy.wait_for_service('plan_footsteps')
        try:
            Plan_Footsteps_Client = rospy.ServiceProxy('plan_footsteps', PlanFootsteps)
            x=Pose2D(0.5,0.3,0)
			y=Pose2D(2.5,0.3,0)
			resp1 = Plan_Footsteps_Client(x, y)
            for data in resp1.footsteps:
				rospy.loginfo("Step leg: %d; target: %f %f %f", data.leg, data.pose.x,
						data.pose.y, data.pose.theta)
				try:
					if data.leg == StepTarget.right:
						leg = [LEG_RIGHT]
					elif data.leg == StepTarget.left:
						leg = [LEG_LEFT]
					else:
						rospy.logerr("Received a wrong leg constant: %d, ignoring step",
									 " command", data.leg)
						return

					footStep = [[data.pose.x, data.pose.y, data.pose.theta]]
					timeList = [STEP_TIME]
					self.motionProxy.setFootSteps(leg, footStep, timeList, False)
					print self.motionProxy.getFootSteps()
					self.motionProxy.waitUntilWalkIsFinished()
					
					# Go to rest position
					# motionProxy.rest()
					return True
				except RuntimeError, e:
					rospy.logerr("Exception caught in handleStep:\n%s", e)
					return False 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
   
        rospy.loginfo("nao_footsteps initialized")
		
    def connectNaoQi(self):
        """(re-) connect to NaoQI"""
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.get_proxy("ALMotion")
        if self.motionProxy is None:
            exit(1)
			
    /*def handleStep(self, data_steps):
	    for data in data_steps.footSteps:
			rospy.loginfo("Step leg: %d; target: %f %f %f", data.leg, data.pose.x,
					data.pose.y, data.pose.theta)
			try:
				if data.leg == StepTarget.right:
					leg = [LEG_RIGHT]
				elif data.leg == StepTarget.left:
					leg = [LEG_LEFT]
				else:
					rospy.logerr("Received a wrong leg constant: %d, ignoring step",
								 " command", data.leg)
					return

				footStep = [[data.pose.x, data.pose.y, data.pose.theta]]
				timeList = [STEP_TIME]
				self.motionProxy.setFootSteps(leg, footStep, timeList, False)
				time.sleep(0.1)
				print self.motionProxy.getFootSteps()
				self.motionProxy.waitUntilWalkIsFinished()
				
				# Go to rest position
				# motionProxy.rest()
				return True
			except RuntimeError, e:
				rospy.logerr("Exception caught in handleStep:\n%s", e)
				return False */
	def stopWalk(self):
        """
        Stops the current walking bahavior and blocks until the clearing is
        complete.
        """
        try:
            self.motionProxy.setWalkTargetVelocity(0.0, 0.0, 0.0, self.stepFrequency)
            self.motionProxy.waitUntilWalkIsFinished()

        except RuntimeError,e:
            print "An error has been caught"
            print e
            return False
        return True

 



if __name__ == "__main__":
    walker = NaoFootsteps()
    rospy.loginfo("nao_footsteps running...")
    rospy.spin()
    rospy.loginfo("nao_footsteps stopping...")
    walker.stopWalk()

    rospy.loginfo("nao_footsteps stopped.")
    exit(0)
	
