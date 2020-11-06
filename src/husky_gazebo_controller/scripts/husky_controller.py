#!/usr/bin/env python

import sys
import rospkg

# get package path 
ros_pkg = rospkg.RosPack()
pkg_path = ros_pkg.get_path("husky_gazebo_controller")
sys.path.append(pkg_path)

import rospy
import math
import queue
import tf
import os

from scripts.mobilenet_v2_classifier import AudioClassifier
from gazebo_msgs2.msg import SoundWithSource
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from queue import Queue


class HuskyController():

    def __init__(self, nn_classifer, 
                    audio_files_path, avoid_objects):

        # placeholders for husky publisher and Twist message
        self.husky_controller_pub = None
        self.husky_vel_msg = None
        
        # neural network conf
        self.audio_classifier = nn_classifer
        self.audio_files_path = audio_files_path
        self.class_id_map = \
            self.audio_classifier.get_class_id_map()
        self.avoid_objects = avoid_objects
       
        # control flags
        self.controller_started = False
        self.husky_pos = False
        self.MAX_OBJECT_DISTANCE = 3

        # Queue for the audio mensages
        self.queue_msgs = Queue()

        # current husky position and orientation
        self.husky_current_x_pos = 0.0
        self.husky_current_y_pos = 0.0
        self.husky_current_theta_orien = 0.0


    
    def callback_sounds(self, sound_msg):
        
        try:
            self.queue_msgs.put(sound_msg)
        except queue.Full:
            pass
            # Discard queue is full
    
    def callback_husky_pose(self, husky_pose_msg):
        
        # filter the husky pose
        if  husky_pose_msg.child_frame_id == "base_link":
            
            
            position = husky_pose_msg.pose.pose
            quant = position.orientation
            self.husky_current_x_pos = position.position.x
            self.husky_current_y_pos = position.position.y
            # compute the quaternion
            roll, pitch, self.husky_current_theta_orien = \
                    tf.transformations.euler_from_quaternion([quant.x, 
                                       quant.y, quant.z, quant.w])
            
            # set flag to indicate when the husky pose was detected
            self.husky_pos = True


    # move the husky to the goal position
    def move_husky(self, goal_x, goal_y):

        # compute distance between two points
        distance = math.sqrt((goal_x-self.husky_current_x_pos)**2 + \
                            (goal_y-self.husky_current_y_pos)**2)
        
        if abs(distance) > self.MAX_OBJECT_DISTANCE:

            diff_x = goal_x - self.husky_current_x_pos
            diff_y = goal_y - self.husky_current_y_pos
            angle_to_object = math.atan2(diff_y,diff_x)
            
            # rotate husky to the target position
            if abs(angle_to_object-self.husky_current_theta_orien) > 0.1:
                
                self.husky_vel_msg.linear.x = 0.1
                self.husky_vel_msg.angular.z = 2.5

            else: # move the husky to the target position

                self.husky_vel_msg.linear.x = 1.0
                self.husky_vel_msg.angular.z = 0.0
            # send command to move the husky
            self.husky_controller_pub.publish(self.husky_vel_msg)
            return False 

        else:
            # the husky has reached the goal position 
            return True



    def detect_object(self):
        
        r = rospy.Rate(10)
        processing_message = False

        while not rospy.is_shutdown():
            
            if self.husky_pos and not processing_message:
                processing_message = True
                try:
                    sound_msg = self.queue_msgs.get()
                    sound_file_name = sound_msg.sound_file.data
                    goal_x_pos = sound_msg.source.x
                    goal_y_pos = sound_msg.source.y
                    audio_path = os.path.join(self.audio_files_path, sound_file_name)

                    id_class = self.audio_classifier.run_inference(audio_path)
                    label_class = self.audio_classifier.get_class_label(id_class)
                    
                    if label_class in self.avoid_objects:
                        # if a not desire object is detected, 
                        # the husky is move to the oposite direction
                        goal_x_pos*=-1
                        goal_y_pos*=-1
                    rospy.loginfo("Detected class: {}".format(label_class))
                    rospy.loginfo("Goal position x: {}".format(goal_x_pos))
                    rospy.loginfo("Goal position y: {}".format(goal_y_pos))
                
                except queue.Empty:
                    processing_message = False

            if processing_message:
                
                if self.move_husky(goal_x_pos, goal_y_pos):
                    # process next message from the queue
                    processing_message = False

            r.sleep()


    def start_controller(self):
        
        if not self.controller_started:

            # Init husky controller 
            rospy.init_node('husky_controller', anonymous=True)
            # Set Subscribers
            rospy.Subscriber("odom", Odometry, self.callback_husky_pose)
            rospy.Subscriber("sounds", SoundWithSource, self.callback_sounds)

            # Set Publisher
            self.husky_controller_pub = rospy.Publisher('/husky/cmd_vel', Twist, queue_size=10)
            self.husky_vel_msg = Twist()

            try:
                self.detect_object()
            except rospy.ROSInterruptException:
                pass
        else:
            print("The controller was already started..")



if __name__ == '__main__':

        class_id_map = {0:'cat',
                        1:'dog',
                        2:'bicycle',
                        3:'car'}
        
        avoid_objects = ['cat', 'dog']


        # define audio classifier
        classifier = AudioClassifier(checkpoint=os.path.join(pkg_path, 
                        "models/audio_classifier_mobilenet_v2_chkpt.pth.tar"), 
                        class_id_map=class_id_map, num_clases=4)
    
        # Start husky controller
        husky_controller = HuskyController(nn_classifer=classifier,
                                    audio_files_path=os.path.join(pkg_path, 
                                    "../audio_files/"), 
                                    avoid_objects=avoid_objects)

        husky_controller.start_controller()
    