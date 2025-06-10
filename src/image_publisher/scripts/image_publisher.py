#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from openni import openni2
from openni import _openni2 as c_api

def publish_astra_rgb():
    rospy.init_node('astra_rgb_publisher', anonymous=True)
    image_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    # OpenNI2 초기화
    openni2.initialize("/home/ubuntu/Downloads/OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_a311d/sdk/libs")
    dev = openni2.Device.open_any()

    # RGB 스트림 생성 및 설정
    color_stream = dev.create_color_stream()
    color_stream.set_video_mode(c_api.OniVideoMode(
        pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888,
        resolutionX=640,
        resolutionY=480,
        fps=30
    ))
    color_stream.start()

    rate = rospy.Rate(30)  # 30 FPS

    rospy.loginfo("Astra RGB 스트리밍 시작")

    try:
        while not rospy.is_shutdown():
            frame = color_stream.read_frame()
            frame_data = frame.get_buffer_as_uint8()

            # NumPy 배열로 변환 및 RGB → BGR 변환
            rgb_image = np.frombuffer(frame_data, dtype=np.uint8).reshape((480, 640, 3))
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

            # OpenCV 이미지를 ROS 이미지 메시지로 변환 후 퍼블리시
            image_msg = bridge.cv2_to_imgmsg(bgr_image, "bgr8")
            image_pub.publish(image_msg)

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        color_stream.stop()
        openni2.unload()
        rospy.loginfo("Astra RGB 스트리밍 종료")

if __name__ == '__main__':
    publish_astra_rgb()
