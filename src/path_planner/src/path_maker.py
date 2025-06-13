#!/usr/bin/env python3
import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from path_planner.msg import Waypoint

class PathFitter:
    def __init__(self):
        rospy.init_node('path_fitter')
        self.path_pub = rospy.Publisher('/local_waypoint', Waypoint, queue_size=1)
        self.marker_pub = rospy.Publisher('/fitted_path_marker', MarkerArray, queue_size=1)
        rospy.Subscriber('/waypoint_info', Waypoint, self.callback)
        self.x_array = []
        self.y_array = []
        self.cnt = 0

    def callback(self, msg):
        rospy.loginfo("Received waypoint_info")
        self.x_array = list(msg.x_arr)[0:msg.cnt]
        self.y_array = list(msg.y_arr)[0:msg.cnt]
        self.cnt = msg.cnt

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.cnt < 2:
                rate.sleep()
                continue

            try:
                # Waypoints를 지수 t(=index)에 대한 파라메트릭 다항식으로 맞춤
                x = np.array(self.x_array)
                y = np.array(self.y_array)

                t = np.arange(len(x))  # 0,1,2,...
                # 데이터 수에 따라 다항식 차수 결정(최대 3차)
                deg = min(3, len(x) - 1)

                # x(t), y(t) 각각 적합
                p_x = np.poly1d(np.polyfit(t, x, deg))
                p_y = np.poly1d(np.polyfit(t, y, deg))

                t_new = np.linspace(0, len(x) - 1, 100)
                x_new = p_x(t_new)
                y_new = p_y(t_new)

                # 메시지 구성
                wp_msg = Waypoint()
                wp_msg.x_arr = list(x_new)
                wp_msg.y_arr = list(y_new)
                wp_msg.cnt = len(x_new)

                self.path_pub.publish(wp_msg)

                # MarkerArray 구성
                marker_array = MarkerArray()
                for i, (x_pt, y_pt) in enumerate(zip(x_new, y_new)):
                    m = Marker()
                    m.header.frame_id = "camera_link"
                    m.header.stamp = rospy.Time.now()
                    m.id = i
                    m.type = Marker.CUBE
                    m.action = Marker.ADD
                    m.pose.position.x = x_pt
                    m.pose.position.y = y_pt
                    m.pose.position.z = 0.1
                    m.pose.orientation.w = 1.0
                    m.scale.x = 0.1
                    m.scale.y = 0.1
                    m.scale.z = 0.1
                    m.color.r = 0.0
                    m.color.g = 1.0
                    m.color.b = 0.0
                    m.color.a = 1.0
                    m.lifetime = rospy.Duration(0.5)
                    marker_array.markers.append(m)

                self.marker_pub.publish(marker_array)

            except Exception as e:
                rospy.logwarn(f"Fitting error: {e}")

            rate.sleep()

if __name__ == '__main__':
    try:
        PathFitter().run()
    except rospy.ROSInterruptException:
        pass
