#!/usr/bin/env python3
import rospy 
import math
import numpy as np
import tf
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import datetime


class TurnCounter:
    def __init__(self):
        self.rate = rospy.Rate(30)
        self.yaw=0
        self.yaw_ref=0
        self.bridge = CvBridge()
        self.turn_detected = False
        self.right_turns = 0
        self.left_turns = 0
        self.turn_threshold = 7.5
        self.straight_line_threshold = 2
        self.image_frames = []
        self.setSubscribers()

    def setSubscribers(self):
        rospy.Subscriber("/orb_slam3/tracking_image", Image, self.callback)
        self.listener = tf.TransformListener()
    
    def callback(self,data):
        image = data
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        #draw a arrow pointing in the direction of the yaw
        try:
            (trans,rot) = self.listener.lookupTransform('/world',"/camera", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        #check if the transform is valid
        if rot[0]==0 and rot[1]==0 and rot[2]==0 and rot[3]==1:
            self.yaw_ref = 0
            self.yaw = 0
            return
        #transform the rotation to euler angles
        euler = tf.transformations.euler_from_quaternion(rot)
        #transform the euler angles to degrees
        euler = np.degrees(euler)
        #get the yaw
        self.yaw = euler[1]
        #draw a arrow pointing in the direction of the yaw on the top left corner of the image
        cv2.arrowedLine(cv_image,(50,50),(int(50+math.cos(math.radians(self.yaw-90))*50),int(50+math.sin(math.radians(self.yaw-90))*50)),(0,255,0),2)
        
        #compute the moving average of the last 100 yaws
        self.yaw_ref = (self.yaw_ref*99+self.yaw)/100
        
        #draw a arrow pointing in the direction of the moving average yaw on the top right corner of the image
        cv2.arrowedLine(cv_image,(50,50),(int(50+math.cos(math.radians(self.yaw_ref-90))*50),int(50+math.sin(math.radians(self.yaw_ref-90))*50)),(0,0,255),2)
        #display on the bottom left corner of the image the difference between the yaw and the moving average yaw
        #make a strig with the difference between the yaw and the moving average yaw and one decimal place
        cv2.putText(cv_image,str(round(self.yaw-self.yaw_ref,1)),(50,100),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,70,255),2)
        #if the difference is greater than 10 degrees the robot is turning right and if it is less than -10 degrees the robot is turning left
        if self.yaw-self.yaw_ref>self.turn_threshold:
            cv2.putText(cv_image,"Turning right",(10,150),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,70,255),2)
            if not self.turn_detected:
                #count only one time the turning
                self.turn_detected = True
                self.right_turns += 1
        elif self.yaw-self.yaw_ref<-self.turn_threshold:
            cv2.putText(cv_image,"Turning left",(10,150),cv2.FONT_HERSHEY_SIMPLEX,0.8,(255,0,0),2)
            if not self.turn_detected:
                #count only one time the turning
                self.turn_detected = True
                self.left_turns += 1
        
        if abs(self.yaw-self.yaw_ref)<self.straight_line_threshold:
            #reset the turn detection
            self.turn_detected = False
        #count only one time the turning
        #display the number of right and left turns on the bottom right corner of the image
        cv2.putText(cv_image,"Right turns: "+str(self.right_turns),(10,300),cv2.FONT_HERSHEY_SIMPLEX,0.8,(255,0,0),2)
        cv2.putText(cv_image,"Left turns: "+str(self.left_turns),(10,340),cv2.FONT_HERSHEY_SIMPLEX,0.8,(255,0,0),2)
        cv2.imshow("turn_counter",cv_image)
        self.image_frames.append(cv_image)
        cv2.waitKey(1)

    def saveVideo(self, filename):
        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(filename, fourcc, 30.0, (self.image_frames[10].shape[1], self.image_frames[10].shape[0]))
        # Loop through each image frame and write it to the video
        for i in range(len(self.image_frames)):
            out.write(self.image_frames[i])

        # Release everything if job is finished
        out.release() 
    
    def checkPublisher(self,node_name):
        nodes = os.popen("rosnode list").read().splitlines()
        if node_name in nodes:
            rospy.loginfo("Node %s is running", node_name)
            return True
        else:
            rospy.loginfo("Node %s is not running", node_name)
            return False



def count_turns(InputVideoPath,OutputFolderPath):
    rospy.init_node('turn_counter')
    turn_counter = TurnCounter()
    os.system("roslaunch orb_slam3_ros bridgestone_slam.launch video_path:="+InputVideoPath)
    cv2.destroyAllWindows()
    #get date and time to name the video file
    now = datetime.datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
    #save the video
    name = "turns"+now+".mp4"
    name = OutputFolderPath+name
    rospy.loginfo("SAVING VIDEO")
    turn_counter.saveVideo(filename=name)
    return turn_counter.right_turns, turn_counter.left_turns

if __name__ == '__main__':
    right_turns,left_turns=count_turns("/home/caio/bags/ros_pt1.mp4","/home/caio/Videos/turn_counter/")
    print("Right turns: ",right_turns)
    print("Left turns: ",left_turns)