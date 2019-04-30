# -*- encoding: UTF-8 -*-

import time
import argparse
from naoqi import ALProxy

def main(robotIP, PORT=9559):
    motionProxy  = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

    # Wake up robot
    motionProxy.wakeUp()

    # Send robot to Pose Init
    postureProxy.goToPosture("StandInit", 0.5)

    # A small step forwards and anti-clockwise with the left foot
    i=0
    while True:
	i+=1
    	# A small step forwards and anti-clockwise with the left foot
    	legName = ["LLeg", "RLeg","LLeg", "RLeg","LLeg", "RLeg","LLeg", "RLeg"]
    	X       = 0.1
    	Y       = 0.1
    	Theta   = 0.3
    	footSteps = [[X, Y, Theta], [X, -Y, Theta],[X, Y, Theta], [X, -Y, Theta],[X, Y, Theta], [X, -Y, Theta],[X, Y, Theta], [X, -Y, Theta]]
    	timeList = [2, 4,6,8,10,12,14,16]
    	clearExisting = False
    	motionProxy.setFootSteps(legName, footSteps, timeList, clearExisting)
    	#motionProxy.post.moveTo(0.3, 0.0, 0)

    	# wait that the move process start running
    	time.sleep(1.0)

    	# get the foot steps vector
    	##footSteps = motionProxy.getFootSteps()
	footStepsRecord=motionProxy.getFootSteps()
	# print the result
    	leftFootWorldPosition = footStepsRecord[0][0]
    	print "leftFootWorldPosition:"
    	print leftFootWorldPosition
    	print ""

    	rightFootWorldPosition = footStepsRecord[0][1]
    	print "rightFootWorldPosition:"
    	print rightFootWorldPosition
    	print ""
    	motionProxy.waitUntilMoveIsFinished()

    # Go to rest position
    #motionProxy.rest()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)
