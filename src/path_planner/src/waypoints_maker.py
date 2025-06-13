#!/usr/bin/env python3
import rospy
import math
from visualization_msgs.msg import MarkerArray, Marker
from path_planner.msg import Waypoint

class ObstacleToWaypoint:
    def __init__(self):
        rospy.init_node('obstacle_to_waypoint')
        self.waypoint_pub = rospy.Publisher('/waypoint_info', Waypoint, queue_size=1)
        self.marker_pub = rospy.Publisher('/waypoint_marker', MarkerArray, queue_size=1)
        rospy.Subscriber('/obstacle', MarkerArray, self.obstacle_callback)
        rospy.Subscriber('/car_pos', MarkerArray, self.carpose_callback)

        rospy.Subscriber('/waypoint_info', Waypoint, self.callback)
        
    def carpose_callback(self, msg):
        markers = msg.markers
        if len(markers) == 0:
            return

        self.car_pos = markers[0].pose.position


    def obstacle_callback(self, msg):
        markers = msg.markers
        if len(markers) == 0:
            return

        waypoints_x = []
        waypoints_y = []

        if len(markers) == 1:
            # ✅ 단일 장애물 회피: 원 중심 기준으로 호 (반원 아크) 생성
            obs = markers[0].pose.position
            r = 1.0
            num_points = 20
            start_angle = -math.pi / 2  # -90도
            end_angle = math.pi / 2     # +90도

            for i in range(num_points):
                theta = start_angle + i * (end_angle - start_angle) / (num_points - 1)
                # 원 중심 기준으로 반지름 r의 원 위 점
                x = obs.x + r * math.cos(theta)
                y = obs.y + r * math.sin(theta)
                if y > self.car_pos.y and x < self.car_pos.x:
                    waypoints_x.append(x)
                    waypoints_y.append(y)

        else:
            # ✅ 다중 장애물 회피: 중점에서 오른쪽 offset
            obstacles = sorted([
                (m.pose.position.x, m.pose.position.y) for m in markers
            ], key=lambda p: p[0])  # x 기준 정렬

            for i in range(len(obstacles) - 1):
                x_mid = (obstacles[i][0] + obstacles[i+1][0]) / 2.0
                y_mid = (obstacles[i][1] + obstacles[i+1][1]) / 2.0

                r = 1.0
                theta = 0  # 오른쪽
                x_offset = x_mid + r * math.cos(theta)
                y_offset = y_mid + r * math.sin(theta)

                waypoints_x.append(x_offset)
                waypoints_y.append(y_offset)

        waypoints_x.insert(0, self.car_pos.x)
        waypoints_y.insert(0, self.car_pos.y)
        
        # ✅ 메시지 생성 및 발행
        wp_msg = Waypoint()
        wp_msg.x_arr = waypoints_x
        wp_msg.y_arr = waypoints_y
        wp_msg.cnt = len(waypoints_x)

        self.waypoint_pub.publish(wp_msg)

    def callback(self, msg):
        rospy.loginfo("Received waypoint_info")
        self.x_array = list(msg.x_arr)[0:msg.cnt]
        self.y_array = list(msg.y_arr)[0:msg.cnt]
        self.cnt = msg.cnt

        # ✅ waypoint_info 시각화용 MarkerArray 추가
        marker_array = MarkerArray()
        for i, (x_pt, y_pt) in enumerate(zip(self.x_array, self.y_array)):
            m = Marker()
            m.header.frame_id = "camera_link"
            m.header.stamp = rospy.Time.now()
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x_pt
            m.pose.position.y = y_pt
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color.r = 1.0  # 빨강: 원래 waypoint
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 1.0
            m.lifetime = rospy.Duration(0.5)
            marker_array.markers.append(m)

        self.marker_pub.publish(marker_array)
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        ObstacleToWaypoint().run()
    except rospy.ROSInterruptException:
        pass
