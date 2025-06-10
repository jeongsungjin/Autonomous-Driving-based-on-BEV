#!/usr/bin/env python3
import rospy
import numpy as np
from path_tracking.msg import Waypoint
from std_msgs.msg import Float32

LOOKAHEAD_DISTANCE = 1.0   # 단위: meters
WHEELBASE = 0.3           # 차량 휠베이스 (typical small car or RC car)

class PurePursuitController:
    def __init__(self):
        rospy.init_node('pure_pursuit_controller')
        rospy.Subscriber('/local_waypoint', Waypoint, self.callback)
        self.steer_pub = rospy.Publisher('/steering_angle', Float32, queue_size=1)
        self.throttle_pub = rospy.Publisher('/throttle', Float32, queue_size=1)

        self.x_arr = []
        self.y_arr = []
        self.rate = rospy.Rate(30)

    def callback(self, msg):
        self.x_arr = list(msg.x_arr)
        self.y_arr = list(msg.y_arr)

    def run(self):
        while not rospy.is_shutdown():
            if len(self.x_arr) < 2:
                self.rate.sleep()
                continue

            # 차량 좌표계 기준으로 가장 가까운 lookahead point 찾기
            target_idx = self.find_lookahead_index()

            if target_idx is not None:
                lx = self.x_arr[target_idx]
                ly = self.y_arr[target_idx]

                # Pure Pursuit Steering Angle 계산
                steering_angle = self.compute_steering_angle(lx, ly)
                throttle = 0.4  # 기본 속도 (상황 따라 조절 가능)

                # 제어 명령 퍼블리시
                self.steer_pub.publish(steering_angle)
                self.throttle_pub.publish(throttle)

                rospy.loginfo(f"[PurePursuit] steer: {steering_angle:.3f}, throttle: {throttle:.2f}")

            self.rate.sleep()

    def find_lookahead_index(self):
        for i in range(len(self.x_arr)):
            dx = self.x_arr[i]
            dy = self.y_arr[i]
            dist = np.hypot(dx, dy)
            if dist >= LOOKAHEAD_DISTANCE:
                return i
        return None

    def compute_steering_angle(self, x, y):
        # 차량이 (0, 0) 에 있다고 가정
        # pure pursuit 공식: delta = atan(2L*sin(alpha)/L_d)
        L_d = np.hypot(x, y)
        if L_d == 0:
            return 0.0
        alpha = np.arctan2(y, x)
        delta = np.arctan2(2 * WHEELBASE * np.sin(alpha), L_d)
        return delta

if __name__ == '__main__':
    try:
        controller = PurePursuitController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
