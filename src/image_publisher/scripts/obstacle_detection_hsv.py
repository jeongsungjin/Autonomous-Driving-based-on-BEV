#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class ObstacleDetector:
    def __init__(self):
        rospy.init_node('blue_obstacle_detector')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.marker_pub = rospy.Publisher("/obstacle", MarkerArray, queue_size=1)
        self.cv_image = None
        self.rate = rospy.Rate(30)

        # 초기 HSV 범위 (파란색)
        self.lower_blue = np.array([100, 150, 50])
        self.upper_blue = np.array([140, 255, 255])

        # 트랙바 윈도우
        cv2.namedWindow("Trackbars")
        self.init_trackbars()

    def nothing(self, x):
        pass

    def init_trackbars(self):
        cv2.createTrackbar("Blue Lower H", "Trackbars", self.lower_blue[0], 179, self.nothing)
        cv2.createTrackbar("Blue Lower S", "Trackbars", self.lower_blue[1], 255, self.nothing)
        cv2.createTrackbar("Blue Lower V", "Trackbars", self.lower_blue[2], 255, self.nothing)
        cv2.createTrackbar("Blue Upper H", "Trackbars", self.upper_blue[0], 179, self.nothing)
        cv2.createTrackbar("Blue Upper S", "Trackbars", self.upper_blue[1], 255, self.nothing)
        cv2.createTrackbar("Blue Upper V", "Trackbars", self.upper_blue[2], 255, self.nothing)

    def update_hsv_from_trackbars(self):
        self.lower_blue = np.array([
            cv2.getTrackbarPos("Blue Lower H", "Trackbars"),
            cv2.getTrackbarPos("Blue Lower S", "Trackbars"),
            cv2.getTrackbarPos("Blue Lower V", "Trackbars")
        ])
        self.upper_blue = np.array([
            cv2.getTrackbarPos("Blue Upper H", "Trackbars"),
            cv2.getTrackbarPos("Blue Upper S", "Trackbars"),
            cv2.getTrackbarPos("Blue Upper V", "Trackbars")
        ])

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Error converting image: %s", e)

    def publish_obstacles(self, contours):
        marker_array = MarkerArray()
        for i, cnt in enumerate(contours):
            if cv2.contourArea(cnt) < 3000:  # 너무 작은 물체는 무시
                continue
            x, y, w, h = cv2.boundingRect(cnt)

            marker = Marker()
            marker.header.frame_id = "camera_link"  # 카메라 좌표계에 맞춰 RViz에서 시각화
            marker.header.stamp = rospy.Time.now()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x + w/2) / 100.0   # px → meter 환산 가정
            marker.pose.position.y = float(y + h/2) / 100.0
            marker.pose.position.z = 0.5
            marker.scale.x = float(w) / 100.0
            marker.scale.y = float(h) / 100.0
            marker.scale.z = 0.1
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.8
            marker.lifetime = rospy.Duration(0.1)
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def run(self):
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                self.update_hsv_from_trackbars()

                resized = cv2.resize(self.cv_image, (640, 480))
                hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)

                mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
                result = cv2.bitwise_and(resized, resized, mask=mask)

                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                self.publish_obstacles(contours)

                # 시각화
                # cv2.drawContours(resized, contours, -1, (0, 255, 0), 2)
                # cv2.imshow("Original", resized)
                # cv2.imshow("Blue Mask", mask)
                # cv2.imshow("Masked Output", result)
                # cv2.waitKey(1)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ObstacleDetector()
        node.run()
    except rospy.ROSInterruptException:
        pass
