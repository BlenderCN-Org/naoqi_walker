#-*-encoding:UTF-8-*-
'''control nao's left foot,
    cartesian control:torso and foot trajectories
    '''
 
import sys
import motion
import almath
from naoqi import ALProxy
 
def StiffnessOn(proxy):
    pName="Body"
    pStiffnessLists=1.0
    pTimeLists=1.0
    proxy.stiffnessInterpolation(pName,pStiffnessLists,pTimeLists)
 
def main(robotIP):
    '''example of cartesian foot trajectory
    '''
    try :
        motionProxy=ALProxy("ALMotion",robotIP,9559)
    except Exception,e:
        print "could not create a proxy"
        print "error is ",e
               
 
    try:
        postureProxy=ALProxy("ALRobotPosture",robotIP,9559)
               
    except  Exception ,e:
        print "could not create a proxy"
        print"error is",e
 
    StiffnessOn(motionProxy)
    #send nao to pose init
    postureProxy.goToPosture("StandInit",0.5)
 
    space=motion.FRAME_WORLD
    axisMask=almath.AXIS_MASK_VEL
    isAbsolute=False
    path=[-0.00,0.00,0.00,0.0,0.0,0.0]
    #lower the torso and move the size
    effector="RLeg"
    time=2.0
    motionProxy.positionInterpolation(effector,space,path,axisMask,time,isAbsolute)
 
    #lleg motion
    effector="LLeg"
    path=[-0.05,0.00,0.03,0.0,0.0,0.0]
    times=2.0
 
    motionProxy.positionInterpolation(effector,space,path,axisMask,time,isAbsolute)

    postureProxy.goToPosture("StandInit",0.5)
               
if __name__=="__main__":
    robotIP="127.0.0.1"
    if len(sys.argv)<=1:
        print "usage python robotIP"
    else:
        robotIP=sys.argv[1]
    main(robotIP)
 
 
 
 
 
 
 
 
 
 
               
