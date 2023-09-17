#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import torch
from PIL import Image as PILImage
from shapely import Polygon
import cv_bridge
import io

def publish_plt_image():
    # Create a ROS publisher for the Image message

    # Create a CV Bridge to convert the NumPy array to a ROS Image message
    bridge = cv_bridge.CvBridge()

    # Create a simple plot (You can replace this with your actual plotting code)
    x = [1, 2, 3, 4, 5]
    y = [10, 8, 6, 4, 2]
    plt.plot(x, y)

    # Convert the Matplotlib image to a NumPy array
    plt_image_np = np.frombuffer(plt.gcf().canvas.tostring_rgb(), dtype=np.uint8)
    plt_image_np = plt_image_np.reshape(plt.gcf().canvas.get_width_height()[::-1] + (3,))

def draw_all(img, xywh, mask):
    plt.imshow(img)

    # Draw Peduncle
    mask = mask.masks[0]
    polygon = Polygon(mask)
    x, y = polygon.exterior.xy
    plt.fill(x, y, color="blue", alpha=0.5)
    polygon = Polygon(mask)
    plt.plot(*polygon.exterior.xy)

    # draw pepper
    xywh = xywh[0]
    x = int(xywh[0])
    y = int(xywh[1])
    w = int(xywh[2])
    h = int(xywh[3])
    draw_bounding_box(0, x, y, w, h)

    # Save the plot to a BytesIO buffer as a PNG image
    buffer = io.BytesIO()
    plt.savefig(buffer, format='png')
    buffer.seek(0)

    # Convert the buffer to a NumPy array
    buffer_image = np.array(PILImage.open(buffer))

    # Close the plot to free up resources
    plt.close()

    # Convert the NumPy array to a PIL Image
    pil_image = PILImage.fromarray(buffer_image)

    # Return the PIL Image
    return pil_image
def draw_bounding_box(confidence, x, y, w, h, color="blue", fill=False):
    # Get the current reference
    ax = plt.gca()
    # plot the bounding box
    rect = patches.Rectangle((x - w / 2, y - h / 2), w, h, linewidth=2, edgecolor=color, facecolor='none')
    # Add the patch to the Axes
    ax.add_patch(rect)

def draw_bounding_polygon(confidence, mask, img_shape, color="blue", fill=True):
    mask = mask.xy[0]
    polygon = Polygon(mask)
    x, y = polygon.exterior.xy
    if fill:
        plt.fill(x, y, color=color, alpha=0.7)
    polygon = Polygon(mask)
    plt.plot(*polygon.exterior.xy)

class PerceptionNode:
    def __init__(self):
        rospy.init_node('perception_node')
        self.bridge = CvBridge()

        # Define the RealSense image subscriber
        self.image_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback, queue_size=1,buff_size=2**28)
        self.image_publisher = rospy.Publisher('/pepper_yolo_results', Image, queue_size=1)

        curr_path = os.getcwd()
        print(curr_path)

        # Define the YOLO model
        self.yolo_pepper = YOLO('/home/sridevi/iowa_ws/src/ISU_Demo/weights/pepper_fruit_best_4.pt')
        self.yolo_peduncle = YOLO('/home/sridevi/iowa_ws/src/ISU_Demo/weights/pepper_peduncle_best_4.pt')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            # cv2.imshow("Image window", cv_image)
            self.run_yolo(cv_image)

        except Exception as e:
            rospy.logerr("Error converting Image message: {}".format(e))
            return

        # Run YOLO on the image
        detected_image = self.run_yolo(cv_image)


    def run_yolo(self, image):
        results_pepper = self.yolo_pepper.predict(image)
        results_peduncle = self.yolo_peduncle.predict(image)
        pil_image = None
        xywh = None
        mask= None
        for result in results_pepper:
            boxes = result.boxes  # Boxes object for bbox outputs
            box=boxes.xyxy[0].cpu().numpy()#only take the first bb
            if box is not []:
                p1 =(int(box[0]),int(box[1]))
                p2 = (int(box[2]),int(box[3]))
                cv2.rectangle(image,p1,p2,(0,0,255),10)



        for result in results_peduncle:
            mask = result.masks
            boxes = result.boxes
            box_peduncle = boxes.xyxy[0].cpu().numpy()
            if box_peduncle is not []:            
                p3 =(int(box_peduncle[0]),int(box_peduncle[1]))
                p4 = (int(box_peduncle[2]),int(box_peduncle[3]))
                cv2.rectangle(image,p3,p4,(255,0,0),10)
     
        
            

        
            
        image_msg_bb= CvBridge().cv2_to_imgmsg(image,"bgr8")
        print("publish")
        self.image_publisher.publish(image_msg_bb)
        # pil_image = draw_all(image, xywh, mask)
        # np_image = np.array(pil_image)

        # # Convert BGR image to RGB (OpenCV uses BGR by default)
        # rgb_image = np_image
        # # Display the image using cv2.imshow()
        # cv2.imshow('Image', rgb_image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        return image

if __name__ == '__main__':
    try:
        perception_node = PerceptionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
