import os
import numpy as np
import cv2
import pyrealsense2 as rs
from ultralytics import YOLO

model = YOLO('isro8l.pt')  
center = []
threshold = 0.1
width = 640
height = 480

def detect_objects(frame):
    global center
    results = model(frame)[0]
    for result in results.boxes.data.tolist():
        x1, y1, x2, y2, score, class_id = result
        center = [int((x1 + x2) / 2), int((y1 + y2) / 2)]
        if score > threshold:
            print("class id:", class_id)
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
            cv2.putText(frame, results.names[int(class_id)].upper(), (int(x1), int(y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)
            width = abs(x2 - x1)
            height = abs(y2 - y1)
            cv2.putText(frame, str(class_id), (int(x1+180), int(y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)
    return frame, center

pipeline = rs.pipeline()
config = rs.config()
bag_file = "D:/ISRO/recorded_feed/20240602_164913.bag"
print(f"Bag file: {bag_file}")
config.enable_device_from_file(bag_file)
print("Enabled device from file")

pipeline.start(config)
print("Pipeline started")

while True:
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    
    
    if not depth_frame or not color_frame:
        print("No depth or color frame")
        continue

    color_image = np.asanyarray(color_frame.get_data())
    color_image = cv2.resize(color_image, (width, height))
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    depth_colormap = cv2.resize(depth_colormap, (width, height))
    
    
    result_frame, center = detect_objects(color_image)
    print("center is:", center)
    if center:
        cv2.circle(depth_colormap, (int(center[0]), int(center[1])), 10, (0, 255, 0), -1)
        if 0 <= center[0] < depth_frame.width and 0 <= center[1] < depth_frame.height:
            distance = depth_frame.get_distance(center[0], center[1])
            print("distance:", distance)
        else:
            print("Center coordinates out of range")

    resized_color_frame = cv2.resize(color_image, (width, height))
    cv2.imshow("Color Frame", color_image)
    
    
    cv2.imshow("Depth Frame", depth_colormap)
    
    
    key = cv2.waitKey(1)
    if key == ord('q'):
        break


pipeline.stop()
cv2.destroyAllWindows()
