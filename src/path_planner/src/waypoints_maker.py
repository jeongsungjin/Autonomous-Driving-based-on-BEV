#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import MarkerArray
from path_planner.msg import Waypoint

class ObstacleToWaypoint:
    def __init__(self):
        rospy.init_node('obstacle_to_waypoint')
        self.waypoint_pub = rospy.Publisher('/waypoint_info', Waypoint, queue_size=1)
        rospy.Subscriber('/obstacle', MarkerArray, self.obstacle_callback)
        self.rate = rospy.Rate(30)

    def obstacle_callback(self, msg):
        markers = msg.markers
        if len(markers) < 2:
            return  # 장애물이 2개 미만이면 경로 생성 의미 없음

        # 장애물 좌표 추출
        obstacles = sorted([
            (m.pose.position.x, m.pose.position.y) for m in markers
        ], key=lambda p: p[0])  # x축 기준 정렬

        # 장애물 사이 중간 점 생성
        waypoints_x = []
        waypoints_y = []

        for i in range(len(obstacles) - 1):
            x_mid = (obstacles[i][0] + obstacles[i+1][0]) / 2.0
            y_mid = (obstacles[i][1] + obstacles[i+1][1]) / 2.0
            waypoints_x.append(x_mid)
            waypoints_y.append(y_mid)

        # 메시지 생성 및 발행
        wp_msg = Waypoint()
        wp_msg.cnt = len(waypoints_x)
        wp_msg.x_arr = waypoints_x
        wp_msg.y_arr = waypoints_y

        self.waypoint_pub.publish(wp_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        ObstacleToWaypoint().run()
    except rospy.ROSInterruptException:
        pass
