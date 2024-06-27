# realsense_processor.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO
import pyrealsense2 as rs
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile, DurabilityPolicy,ReliabilityPolicy
model = YOLO("/home/mahaveer/outside/src/isro-rover-challenge/perception/model/cylinder_optimiser_l.pt")
threshold = 0.5
width = 640
height = 480
bbox = [[(0, 0), (0, 0), (0, 0),(0, 0) ,(0, 0)],[(0,0)],[(0,0)]]

class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor')
        qos_profile = QoSProfile(
            depth=10,  # Queue size
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.rgb_subscriber = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.rgb_callback,
            qos_profile
            )

        self.depth_subscriber = self.create_subscription(
            Image,
            '/camera/realsense_splitter_node/output/depth',
            self.depth_callback,
            qos_profile
            )
        
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.camera_info_callback,
            qos_profile
            )
        
        self.publisher_ = self.create_publisher(Pose, 'object_pose', 10)
        
        self.pick_pose_publisher_ = self.create_publisher(Pose, 'final_pick_pose', 10)
        
        print("OBJECT DETECTOR NODE STARTED")
        
        self.bridge = CvBridge()
        self.intrinsic = None
        self.color_image = None
        self.depth_image = None

        #call loop function        # self.process_frames()


    def camera_info_callback(self, msg):
        # Extract intrinsic parameters from camera info
        self.intrinsic = np.array(msg.k).reshape((3, 3))
        K = msg.k
        msg.d=[0.0,0.0,0.0,0.0,0.0]
        self.camera_intrinsics = rs.intrinsics()
        self.camera_intrinsics.width = width
        self.camera_intrinsics.height = height
        self.camera_intrinsics.fx = K[0]  # Focal length x
        self.camera_intrinsics.fy = K[4]  # Focal length y
        self.camera_intrinsics.ppx = K[2]  # Principal point x
        self.camera_intrinsics.ppy = K[5]  # Principal point y
        
        # Use 'plumb_bob' distortion model (common alias for no distortion or standard pinhole camera)
        self.camera_intrinsics.model = rs.distortion.modified_brown_conrady  # or use rs.distortion.modified_brown_conrady if that's more appropriate
        self.camera_intrinsics.coeffs = msg.d[:5]

        # self.get_logger().info(f"Camera Intrinsic Parameters: {self.intrinsic}")

    def rgb_callback(self, msg):
        # Convert ROS image message to OpenCV image
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.process_frames()

    def depth_callback(self, msg):
        # Convert ROS image message to OpenCV image
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


    def process_frames(self,depth_threshold=1000):
      
        
        if self.color_image is not None and self.depth_image is not None and self.intrinsic is not None:
            width, height = 640, 480  # Example resolution, modify as needed
            
            color_image = cv2.resize(self.color_image, (width, height))
            depth_image = cv2.resize(self.depth_image, (width, height))

            pose_msg = Pose()

            #apply depth masing , on rgb frame , only allow rgb pixels within range for better object detection
            depth_arr = np.array(depth_image)
            mask = depth_arr<depth_threshold
            rgb_mask = np.stack((mask,) * 3, axis=-1)
            masked_color = color_image * rgb_mask

            results = model(masked_color)[0]
       
            for result in results.boxes.data.tolist():
                x1, y1, x2, y2, score, class_id = result
      
                if score > threshold: 
   

                    dx,dy = x1 + (x2-x1)//2,y1+ (y2-y1)//2
                    bbox[0] = [(int(x1), int(y1)), (int(x2), int(y1)), (int(x2), int(y2)), (int(x1), int(y2)) ,(int(dx),int(dy))]
                    
                    worldpts_tf = self.get_world_coordinated(depth_image,dx,dy)
                    pose_msg.position.x = worldpts_tf[2]
                    pose_msg.position.y = -worldpts_tf[0]
                    pose_msg.position.z = -worldpts_tf[1]

                    try:
                        keypoints = results.keypoints[int(class_id)]
                        if keypoints is not None:
                            keypoints = keypoints.data.tolist()[0]
                            kpts = []
                     
                            for kp in keypoints:
                                x, y, confidence = kp[:3]
                                kpts.append((int(x),int(y)))
                            bbox[1] = kpts
                   
                            rvec , bbox[2] = self.get_orientation()
                            pose_msg.orientation.x = rvec[0][0]
                            pose_msg.orientation.y = rvec[1][0]
                            pose_msg.orientation.z = rvec[2][0]
                     


                    except:
                        self.get_logger().warn("no key points detected")

                    self.publisher_.publish(pose_msg)
                    pose_msg.position.x = pose_msg.position.x*100
                    pose_msg.position.y = pose_msg.position.y*100 
                    pose_msg.position.z = pose_msg.position.z*100

                    self.pick_pose_publisher_.publish(pose_msg)

            self.draw_on_frame(color_image)
            # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # cv2.imshow("Depth Frame", depth_colormap)

            if cv2.waitKey(1) == ord('q'):
                cv2.destroyAllWindows()
                rclpy.shutdown()
        

        return bbox
    
    def draw_on_frame(self,color_image):

        
        cv2.rectangle(color_image, bbox[0][0], bbox[0][2], (0, 255, 0), 2) # draw bounding box
        cv2.circle(color_image, bbox[0][4], 2, (255, 0, 0), -1) #center of the bounding box

        # plotting keypoints
        for i, (x,y) in enumerate(bbox[1]):
            # if i==5:
            cv2.circle(color_image, (int(x), int(y)), 2, (255, 0, 0), -1)

        try:
        #draw a bounding cube
            b1,b2,b3,b4,t1,t2,t3,t4 = bbox[2]
        
            cv2.line(color_image, tuple(b1.astype(int)), tuple(b2.astype(int)), (0, 0, 255), 2)
            cv2.line(color_image, tuple(b2.astype(int)), tuple(b3.astype(int)), (0, 0, 255), 2)
            cv2.line(color_image, tuple(b3.astype(int)), tuple(b4.astype(int)), (0, 0, 255), 2)
            cv2.line(color_image, tuple(b4.astype(int)), tuple(b1.astype(int)), (0, 0, 255), 2)

            cv2.line(color_image, tuple(t1.astype(int)), tuple(t2.astype(int)), (0, 0, 255), 2)
            cv2.line(color_image, tuple(t2.astype(int)), tuple(t3.astype(int)), (0, 0, 255), 2)
            cv2.line(color_image, tuple(t3.astype(int)), tuple(t4.astype(int)), (0, 0, 255), 2)
            cv2.line(color_image, tuple(t4.astype(int)), tuple(t1.astype(int)), (0, 0, 255), 2)

            cv2.line(color_image, tuple(t1.astype(int)), tuple(b1.astype(int)), (0, 0, 255), 2)
            cv2.line(color_image, tuple(t2.astype(int)), tuple(b2.astype(int)), (0, 0, 255), 2)
            cv2.line(color_image, tuple(t3.astype(int)), tuple(b3.astype(int)), (0, 0, 255), 2)
            cv2.line(color_image, tuple(t4.astype(int)), tuple(b4.astype(int)), (0, 0, 255), 2)
        except:
            pass

        cv2.imshow("Color Frame", color_image)

    def get_orientation(self):

        dist_coeffs  = None
        radius = 0.03  # radius in meters (half of 5 cm)
        height = 0.15  # height in meters (15 cm)

        # 3D model points of the cylinder
        model_points = np.array([
            [-radius, 0, 0],  
            [0, -radius, 0],   
            [-radius, 0, height],    
            [0, radius, height],   
            [radius, 0, height],         
            [-radius, 0, height],         
            [radius, 0, 0]          
        ])
        # self.get_logger().info(bbox)
            # Pose estimation using solvePnP
        if dist_coeffs is None:
            dist_coeffs = np.zeros((4, 1))


        # Convert points to float32 for cv2 functions
        world_coords = np.array(model_points, dtype=np.float32)
        
        img_points = np.array(bbox[1], dtype=np.float32)

    # Solve PnP to get the rotation and translation vectors
        success, rvec, tvec = cv2.solvePnP(world_coords, img_points, self.intrinsic, dist_coeffs)

        if not success:
            raise ValueError("Pose estimation failed")
          
        axis_points = np.float32([
            [0, 0, height/2], [0.1, 0, height/2], [0, 0.1, height/2], [0, 0, 0.1+height/2]
        ]).reshape(-1, 3)

        _3d_bounding_box = np.float32([
            (-radius,-radius,0),(-radius,radius,0),(radius,radius,0),(radius,-radius,0),
            (-radius,-radius,height),(-radius,radius,height),(radius,radius,height),(radius,-radius,height)
        ])


        projected_points, _ = cv2.projectPoints(_3d_bounding_box, rvec, tvec, self.intrinsic, dist_coeffs)
        projected_points = projected_points.reshape(-1, 2)

        # origin = tuple(projected_points[0].astype(int))
        # x_axis = tuple(projected_points[1].astype(int))
        # y_axis = tuple(projected_points[2].astype(int))
        # z_axis = tuple(projected_points[3].astype(int))

        return rvec,projected_points

    def get_world_coordinated(self,depth_image,x,y):
        depth_value_in_meters = depth_image[int(y-20), int(x)]/1000 # Convert to meters
        if depth_value_in_meters ==0.0:
            depth_value_in_meters = 0.001
        # Project depth pixel to 3D point in camera coordinates
        world_point = rs.rs2_deproject_pixel_to_point(self.camera_intrinsics, (x,y), depth_value_in_meters)
        return world_point

def main(args=None):
    rclpy.init(args=args)
    processor = ImageProcessor()
    rclpy.spin(processor)

    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
