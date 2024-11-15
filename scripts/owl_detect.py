import requests
import rospy
import cv_bridge
from sensor_msgs.msg import CompressedImage
from threading import RLock as threading_Lock
import numpy as np
import cv2

url = "localhost:5000/detect"

# 相机内参矩阵K：
fx = 912.8017578125  # 焦距x
fy = 912.8299560546875  # 焦距y
cx = 633.041259765625  # 主点x坐标
cy = 366.2099304199219  # 主点y坐标

# 图像分辨率
width = 1280
height = 720

_cvb = cv_bridge.CvBridge()
_lock = threading_Lock()

def pub_goal(x, y, z):
    pass

def get_depth(u, v):
    depth_roi = last_depth_img[
            max(0, v-2):min(height, v+3),
            max(0, u-2):min(width, u+3)
        ]
    avg_depth = np.mean(depth_roi[depth_roi > 0]) / 1000.0
    return avg_depth

def pixel_to_camera(u, v, depth):
    # 像素坐标转换到相机坐标
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    Z = depth
    return X, Y, Z

def camera_to_robot(X, Y, Z):
    R = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    T = np.array([0.3, 0, 0.07])
    point_camera = np.array([X, Y, Z])
    point_robot = R @ point_camera + T
    return point_robot

def draw_detection(image, detection, depth):
    label = detection['label']
    score = detection['score']
    box = detection['box']
    x1, y1, x2, y2 = map(int, box)
    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(image, f"{label}: {score:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    cv2.putText(image, f"Depth: {depth:.2f}", (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    return image

def image_callback(msg):
    annotated_image = None
    img_base64 = msg.data
    response = requests.post(url, json={'image': img_base64})
    
    if response.status_code == 200:
        result = response.json()
        print("Detection Result:", result)
        if len(result) > 0:
            for detection in result:
                label = detection['label']
                if label == 'glove':
                    box = detection['box']
                    x1, y1, x2, y2 = map(int, box)
                    u, v = (x1 + x2) // 2, (y1 + y2) // 2
                    avg_depth = get_depth(u, v)
                    x_cam, y_cam, z_cam = pixel_to_camera(u, v, avg_depth)
                    x_robot, y_robot, z_robot = camera_to_robot(x_cam, y_cam, z_cam)
                    print(f"Glove detected !!!")
                    print(f"Depth: {avg_depth}")
                    print(f"Pixel: ({u}, {v})")
                    print(f"Camera: ({x_cam}, {y_cam}, {z_cam})")
                    print(f"Robot: ({x_robot}, {y_robot}, {z_robot})")
                    if avg_depth > 0.5:
                        pub_goal(x_robot, y_robot, z_robot)
                        
                    with _lock:
                        annotated_image = _cvb.imgmsg_to_cv2(msg)
                    annotated_image = draw_detection(annotated_image, detection, avg_depth)
                    
        if annotated_image is not None:
            annotated_image_msg = _cvb.cv2_to_imgmsg(annotated_image, "rgb8")
            annotated_image_msg.header = msg.header
            det_pub.publish(annotated_image_msg)
            
    else:
        print("Error:", response.status_code, response.text)
    
def depth_callback(msg):
    global last_depth_img
    with _lock:
        last_depth_img = _cvb.imgmsg_to_cv2(msg)


if __name__ == "__main__":
    last_depth_img = None
    
    rospy.init_node('owl_detector')
    
    # 设置处理频率为1Hz(每秒1次)
    rate = rospy.Rate(1)  
    
    # 设置订阅队列大小为1,丢弃旧的消息
    rospy.Subscriber('/camera/color/image_raw/compressed', CompressedImage, image_callback, queue_size=3)
    rospy.Subscriber('/camera/depth/image_raw/compressed', CompressedImage, depth_callback, queue_size=3)
    det_pub = rospy.Publisher("/detection/image", CompressedImage, queue_size=10)
    
    while not rospy.is_shutdown():
        rate.sleep()  