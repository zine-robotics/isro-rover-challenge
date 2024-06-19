# realsense_processor.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO

model = YOLO('/home/zine/final_bot/isro-rover-challenge/perception/new_cylinder_pose_b.pt')
threshold = 0.5
width = 640
height = 480

class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor')
        
        self.rgb_subscriber = self.create_subscription(
            Image,
            '/front/stereo_camera/left/rgb',
            self.rgb_callback,
            10)

        self.depth_subscriber = self.create_subscription(
            Image,
            '/front/stereo_camera/left/depth',
            self.depth_callback,
            10)
        
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/front/stereo_camera/left/camera_info',
            self.camera_info_callback,
            10)
        
        print("OBJECT DETECTOR NODE STARTED")
        
        self.bridge = CvBridge()
        self.intrinsic = None
        self.color_image = None
        self.depth_image = None

        #call loop function
        # self.process_frames()


    def camera_info_callback(self, msg):
        # Extract intrinsic parameters from camera info
        self.intrinsic = np.array(msg.k).reshape((3, 3))
        self.get_logger().info(f"Camera Intrinsic Parameters: {self.intrinsic}")

    def rgb_callback(self, msg):
        # Convert ROS image message to OpenCV image
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.process_frames()

    def depth_callback(self, msg):
        # Convert ROS image message to OpenCV image
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


    def process_frames(self):
        # self.get_logger().info("Function started")
        # # Process the frames if both are available and intrinsic is set
        # while True:
        if self.color_image is not None and self.depth_image is not None and self.intrinsic is not None:
            width, height = 640, 480  # Example resolution, modify as needed
            
            color_image = cv2.resize(self.color_image, (width, height))
            depth_image = cv2.resize(self.depth_image, (width, height))
            
            # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            results = model(color_image)[0]
            # self.get_logger().info()
            # print(results.boxes)
            for result in results.boxes.data.tolist():
                x1, y1, x2, y2, score, class_id = result
                op = f"x1 :{x1} , x2:{x2}, y1:{y1} , y2:{y2}"
                self.get_logger().info(op)
                if score > threshold:  # Assuming 'threshold' is predefined for confidence
                    # Draw the bounding box
                    cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    
                    # Put the class label and confidence score
                    label = f"{model.names[int(class_id)].upper()} {score:.2f}"
                    cv2.putText(color_image, label, (int(x1), int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                    try:
                        keypoints = results.keypoints[int(class_id)]
                        if keypoints is not None:
                            keypoints = keypoints.data.tolist()[0]
                            # print("keypoints is :",keypoints)
                            # bbox_corners[-1].extend(keypoints) 
                            # print("bboxes:",bbox_corners)
                            for kp in keypoints:
                                x, y, confidence = kp[:3]
                                cv2.circle(color_image, (int(x), int(y)), 5, (255, 0, 0), -1)
                    except:
                        self.get_logger().warn("no key points detected")


            cv2.imshow("Color Frame", color_image)
            # cv2.imshow("Depth Frame", depth_colormap)

            if cv2.waitKey(1) == ord('q'):
                cv2.destroyAllWindows()
                rclpy.shutdown()
                # break
        

# def detect_objects(frame, depth_frame, bbox_corners=[], camera_matrix=None):
#     if camera_matrix is None:
#         raise ValueError("Camera matrix must be provided")

#     fx, fy, cx, cy = camera_matrix[0, 0], camera_matrix[1, 1], camera_matrix[0, 2], camera_matrix[1, 2]
    
#     results = model(frame)[0]

#     real_world_bboxes = []

#     for result in results.boxes.data.tolist():
#         x1, y1, x2, y2, score, class_id = result
#         if score > threshold:
#             pixel_corners = [(int(x1), int(y1)), (int(x2), int(y1)), (int(x2), int(y2)), (int(x1), int(y2))]

#             real_world_corners = []
#             for (x, y) in pixel_corners:
#                 # Get the depth value at the pixel (x, y)
#                 z = depth_frame[y, x] * depth_scale  # Assuming depth_frame is in units of millimeters and needs to be scaled to meters
#                 if z == 0:  # If depth is zero, it can't be used for calculations
#                     continue

#                 # Convert the pixel coordinates (x, y) to real-world coordinates (X, Y, Z)
#                 X = (x - cx) * z / fx
#                 Y = (y - cy) * z / fy
#                 real_world_corners.append((X, Y, z))
            
#             if len(real_world_corners) == 4:
#                 real_world_bboxes.append(real_world_corners)
                
#             # Optionally, you can include keypoints processing if needed
#             try:
#                 keypoints = results.keypoints[int(class_id)]
#                 if keypoints is not None:
#                     keypoints = keypoints.data.tolist()[0]
#                     for kp in keypoints:
#                         x, y, confidence = kp[:3]
#                         z = depth_frame[int(y), int(x)] * depth_scale  # Get the depth value for the keypoint
#                         if z == 0:  # If depth is zero, skip this keypoint
#                             continue
                        
#                         X = (x - cx) * z / fx
#                         Y = (y - cy) * z / fy
#                         real_world_corners.append((X, Y, z))
#             except Exception as e:
#                 print(f"Error processing keypoints for class ID {class_id}: {str(e)}")

#     return real_world_bboxes


# def draw_orientation(image, bbox_corners, camera_matrix):
#     # Placeholder function for drawing orientation
#     # Replace with your actual drawing logic
#     return image


def main(args=None):
    rclpy.init(args=args)
    processor = ImageProcessor()
    rclpy.spin(processor)

    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
