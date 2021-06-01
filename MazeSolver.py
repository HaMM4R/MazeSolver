#############     REFERENCES     #############

#https://github.com/marc-hanheide/ros_book_sample_code/blob/master/chapter12/src/follower_p.py - Line follower
#https://www.youtube.com/watch?v=eJ4QPrYqMlw - Laser Scanner
#https://www.theconstructsim.com/ros-qa-135-how-to-rotate-a-robot-to-a-desired-heading-using-feedback-from-odometry/ - Odom

import rospy
import numpy as np
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class image_converter:

    roll = pitch = yaw = 0.0
    rotationRadians = 0
    moveSpeed = 0
    detectedObstical = False
    doingLeftTurn = False
    stuckInCorner = False

    greenDetected = False
    onGreen = False

    def __init__(self):

        cv2.startWindowThread()
        self.bridge = CvBridge()

        #Subscribes to laser scan and camera and sets up a publisher to publish twist commands
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
                                           Image, self.callbackRGB)

        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callbackLaserScan)
        

        self.cmd_move = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                           queue_size=1)

        self.twist = Twist()
        self.move_robot()
        

    def callbackLaserScan(self, msg):

        #Makes sure the laser scanner is getting data
        if(len(msg.ranges) != 0):
            self.rotationRadians = 0
            middleLaserScan = len(msg.ranges) / 2

            farLeft = msg.ranges[-1]
            farRight = msg.ranges[0]

            #Only runs if red or green aren't detected
            if(self.detectedObstical == False and self.greenDetected == False and self.onGreen == False):
                
                #If there is an obsticle directly infront of the robot it will stop moving and rotate in
                #Whichever the last direction it was rotating to help move around corners and not get stuck in dead ends
                if(msg.ranges[middleLaserScan] < 0.7):
                    self.moveSpeed = 0
                    if(self.doingLeftTurn):
                        self.rotationRadians += 0.7
                    else:
                        self.rotationRadians -= 0.7
                    stuckInCorner = True
                else:
                    stuckInCorner = False
                
                #If the robot isn't stuck it'll move preferencing left turns
                if(stuckInCorner == False):
                    #If there is a gap on the left the robot will attempt to turn into it, an implementation of the left hand rule
                    if(farLeft > 1.2):
                        self.doingLeftTurn = True
                        self.rotationRadians += 0.2
                        self.moveSpeed = 0.2
                    #Helps to move around right hand corners but only in big open spaces where right is the only option
                    elif(farRight > 1 and farLeft < 2.2):
                        self.doingLeftTurn = False
                        self.rotationRadians -= 0.2
                        self.moveSpeed = 0.2

                    #stops the robot from hitting into walls by rotating away from them if it gets too close
                    if(farRight < 0.5 and farRight < farLeft):
                        self.doingLeftTurn = True
                        self.rotationRadians += 0.5
                        self.moveSpeed = -0#.1
                    elif(farLeft < 0.5 and farLeft < farRight):
                        self.doingLeftTurn = False
                        self.rotationRadians -= 0.5
                        self.moveSpeed = -0#.1
                    else:
                        self.moveSpeed = 0.2
    
    #Callback from the camera
    def callbackRGB(self, data):
        cv2.namedWindow("Image window", 1)
        cv2.namedWindow("Original", 1)
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #Gets the bounds for red colouring
        lower_red = np.array([0,50,50])
        upper_red = np.array([10,255,255])
        mask0 = cv2.inRange(hsv, lower_red, upper_red)

        # upper mask (170-180)
        lower_red = np.array([170,50,50])
        upper_red = np.array([180,255,255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        #Gets the bounds for green colouring
        lower_green = np.array([36, 25, 25])
        upper_green = np.array([70, 255, 255])

        maskGreen = cv2.inRange(hsv, lower_green, upper_green)

        h, w, d = image.shape

        #Will only search the bottom half of the screen for red to avoid moving away from it before it gets close enough to detect turns near the red
        maskRed = mask0+mask1
        searchBottom = 3*h/4 + 60
        maskRed[0:searchBottom, 0:w] = 0
        Mr = cv2.moments(maskRed)

        #If the robot detects red, turn away from it
        Mg = cv2.moments(maskGreen)
        if Mr['m00'] > 0:
            self.detectedObstical = True
            self.rotationRadians -= 10
            self.moveSpeed = 0
        else:
            self.detectedObstical = False
        
        #If the robot detects green move towards it
        if(Mg['m00'] > 0 and self.onGreen == False):
            #Calculates the centre point of the colour detected
            cx = int(Mg['m10']/Mg['m00'])
            cy = int(Mg['m01']/Mg['m00'])

            #sets the robot to move and rotation towards the centre of the colour
            print(cy)
            self.greenDetected = True
            err = cx - w/2
            self.moveSpeed = 0.2
            self.rotationRadians = -float(err) / 100
        else:
            #If it is on the green, stop the robot moving
            if(self.greenDetected):
                self.onGreen = True
                self.rotationRadians = 0
                self.moveSpeed = 0

        cv2.imshow("Image window", maskGreen)

        cv2.waitKey(1)

    #Move the robot based on values decided in previous functions
    def move_robot(self):
        while not rospy.is_shutdown():
            self.twist.linear.x = self.moveSpeed
            self.twist.angular.z = self.rotationRadians
            self.cmd_move.publish(self.twist)


cv2.startWindowThread()
rospy.init_node('image_converter')
imagineC = image_converter()
rospy.spin()
cv2.destroyAllWindows()