#!/usr/bin/env python3
"""
node for image segmentation, placement of green line, and calculation of centroid
"""

import cv2
import rclpy
import datetime
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
import image_processing as img_p


class ImageSegment(Node):
    '''
    A segment image, add line, and compute centroid
    '''
    def __init__(self):
        super().__init__('image_segment')
        self.seg_pub = self.create_publisher(Image, '/image/segmented', 10)
        self.centroid_pub = self.create_publisher(PointStamped, '/image/centroid', 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 1)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        '''
        Get  images from topic, segment, and republish
        '''
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        img_segmented = img_p.image_segment(img, [0, 50, 0], [100, 255, 100])

        # compute centroid and publish
        x_centroid = img_p.image_centroid_horizontal(img_segmented)
        centroid = PointStamped()
        centroid.header = msg.header
        centroid.point.x = float(x_centroid)
        centroid.point.y = 0.0
        centroid.point.z = 0.0
        self.centroid_pub.publish(centroid)

        img_seg_color = img_p.image_one_to_three_channels(img_segmented)

        img_w_line = img_p.image_line_vertical(img_seg_color, x_centroid)

        # Convert grayscale to BGR for publishing
        #img_color = cv2.cvtColor(img_w_line, cv2.COLOR_GRAY2BGR)
        seg_msg = self.bridge.cv2_to_imgmsg(img_w_line, 'bgr8')
        self.seg_pub.publish(seg_msg)


def main(args=None):
    rclpy.init(args=args)
    image_segment = ImageSegment()
    rclpy.spin(image_segment)
    image_segment.destroy_node()
    rclpy.shutdown()
