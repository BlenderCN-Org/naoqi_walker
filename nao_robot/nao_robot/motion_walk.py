import sys

import motion

import time

from naoqi import ALProxy

def StiffnessOn(proxy):

    pName = "Body"

    pStiffnessLists = 1.0

    pTimeLists = 1.0

    proxy.stiffnessInterpolation(pName, pStiffnessLists, pTimeLists)



def main(robotIP):

    try:

        motionProxy = ALProxy("ALMotion", robotIP, 9559)

    except Exception, e:

        print"could not create proxy to ALMotion"

        print "error was", e



    try:

        postureProxy = ALProxy("ALRobotPosture", robotIP, 9559)

    except Exception, e:

        print "could not create proxy to ALRobotPosture"

        print "error is ", e



        # set nao in stiffness on

        StiffnessOn(motionProxy)



        # send nao to pose init

        postureProxy.goToPosture("StandInit", 0.5);



        # eable arms control by walk algorithm

        motionProxy.setWalkArmsEable(True, True)

        # foot contact protection

        motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])



        # target velocity

        X = -0.5  # backward

        Y = 0.0

        Theta = 0.0

        Frequency = 0.0  # low speed

        motionProxy.setWalkTargetVelocity(X, Y.Theta, Frequency)

        time.sleep(4.0)



        # target velocity

        X = 0.9

        Y = 0.0

        Theta = 0.0

        Frenqency = 1.0  # max speed

        motionProxy.setWalkTargetVelocity(X, Y, Theta, Frenquency)

        time.sleep(2.0)



        # arms user motion

        # arms motion from user have alwalys  priority than walk arms motion

        JoinNames = ["LShouderPitch", "LShouderRoll", "LElbowYaw", "LElbowRoll"]

        Arm1 = [-40, 25, 0, -40]

        Arm1 = [x * motion.TO_RAD for x in Aram1]



        Arm2 = [-40, 50, 0, -80]

        Arm2 = [x * motion.TO_RAD for x in Aram2]



        pFractionMaxSpeed = 0.6



        motionProxy.angleInterpolationWithSpeed(JoinNames, Arms1, pFractionMaxSpeed)

        motionProxy.angleInterpolationWithSpeed(JoinNames, Arms2, pFractionMaxSpeed)

        motionProxy.angleInterpolationWithSpeed(JoinNames, Arms1, pFractionMaxSpeed)



        # end walk

        X = 0.0

        Y = 0.0

        Theta = 0.0

        motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)



if __name__=="__main__":

    robotIP = "127.0.0.1"

    if len(sys.argv) <= 1:

        print

        "useage pyhton motion_walk.py robotIP,default is 127.0.0.1"

    else:

        robotIp = sys.argv[1]

    main(robotIP)


