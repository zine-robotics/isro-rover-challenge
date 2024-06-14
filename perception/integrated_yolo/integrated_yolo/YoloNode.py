#!/usr/bin/env python
import os
import numpy as np
import cv2
from ultralytics import YOLO

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ObjectDetectorNode:
    def __init__(self):
        rospy.init_node('object_detector_node', anonymous=True)
        self.bridge = CvBridge()
        self.model = YOLO('isro8l.pt')  
        self.object_pose = rospy.Publisher("/object_pose", dict, queue_size=5)
        self.image_sub_rgb = rospy.Subscriber("/front/stereo_camera/left/rgb", Image, self.rgb_callback)
        self.image_sub_depth = rospy.Subscriber("/front/stereo_camera/left/depth", Image, self.depth_callback)
        self.depth_frame=None
        self.rgb_frame=None
        self.threshold = 0.5 
        self.width = 640
        self.height = 480

    def detect_objects(self):
        color_image = np.asanyarray(self.rgb_frame.get_data())
        color_image = cv2.resize(color_image, (self.width, self.height))
        depth_image = np.asanyarray(self.depth_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        depth_colormap = cv2.resize(depth_colormap, (self.width, self.height))

        results = self.model(color_image)[0]
        for result in results.boxes.data.tolist():
                x1, y1, x2, y2, score, class_id = result
                center = [int((x1 + x2) / 2), int((y1 + y2) / 2)]
                if score>self.threshold and class_id==1:
                    return center
                    
                    
                

    def process_frame(self, frame):
        if self.rgb_frame and self.object_pose:
            center = self.detect_objects(frame)
            if center and (0 <= center[0] < self.depth_frame.width and 0 <= center[1] < self.depth_frame.height):
                distance = self.depth_frame.get_distance(center[0], center[1])
                pose={"x":center[0],"y":center[1],"distance":distance}
                self.object_pose.publish(pose)
      

    def rgb_callback(self, data):
        try:
            self.rgb_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            print(e)
            return
        self.process_frame()

    def depth_callback(self, data):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except Exception as e:
            print(e)
            return
        

if __name__ == '__main__':
    try:
        obj_detector = ObjectDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
