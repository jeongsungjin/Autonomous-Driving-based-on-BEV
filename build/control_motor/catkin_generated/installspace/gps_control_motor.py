import rospy
import redis
import json
import paho.mqtt.client as mqtt
import asyncio
import pigpio
import math
import sys
import select
import time
from std_msgs.msg import Float32
from geopy.distance import geodesic

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

    def pid_control(self, cte):
        self.d_error = cte - self.p_error
        self.p_error = cte
        self.i_error += cte
        return self.kp * self.p_error + self.ki * self.i_error + self.kd * self.d_error

class MotorController:
    def __init__(self):
        rospy.init_node('motor_controller', anonymous=True)
        # Redis 및 MQTT 클라이언트 초기화
        self.redis_client = redis.StrictRedis(host='172.20.10.8', port=6379, db=0)
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("172.20.10.8", 1883, 60)
        self.mqtt_client.loop_start()
        self.pi = pigpio.pi()
        
        if not self.pi.connected:
            rospy.logerr("pigpio 데몬에 연결할 수 없습니다. 데몬을 실행하고 다시 시도해 주세요.")
            rospy.signal_shutdown("pigpio 데몬에 연결할 수 없음")
        
        self.camera_sub = rospy.Subscriber('/lane_x_location', Float32, self.camera_callback)
        self.gps_sub = rospy.Subscriber('/gps/fix', Float32, self.gps_callback)
        
        self.x_location = 256
        self.motor_pwm = 1500
        self.servo_pwm = 1450
        self.obstacle_distance = -1.0
        # 라이다 데이터 초기화
        self.lidar_x = 0.0
        self.lidar_y = 0.0
        self.lidar_z = 0.0
        self.lidar_distance = 0.0
        self.lane_number = 1
        # GPS 데이터 초기화
        self.latitude = 0.0
        self.longitude = 0.0
        self.main_lane_number = 2 
        # GPIO 핀 설정
        self.STEERING_SERVO_PIN = 17
        self.ESC_PIN = 18
        
        self.STEERING_MIN_PULSE_WIDTH = 1200
        self.STEERING_MAX_PULSE_WIDTH = 1850
        self.STEERING_CENTER_PULSE_WIDTH = 1480
        self.ESC_MIN_PULSE_WIDTH = 1200
        self.ESC_MAX_PULSE_WIDTH = 1800
        self.ESC_STOP_PULSE_WIDTH = 1500

        # 상태 플래그 및 데이터 초기화
        self.follow_vehicle_flag = False
        self.processed_timestamps = set()  # 중복 데이터 필터링을 위한 타임스탬프 집합

        # 차량 위치 초기화
        self.main_vehicle_position = None  # 메인 차량의 위치 (GPS 데이터 수신 후 설정)
        self.sub_vehicle_position = None  # 서브 차량의 위치 (GPS 데이터 수신 후 설정)

        # 시나리오 플래그 변수 초기화
        self.avoidance_flag = False
        self.deceleration_flag = False
        self.stop_flag = False

        rospy.loginfo("Motor Controller Node Initialized")
        
    def camera_callback(self, data):
        self.x_location = data.data
    
    def gps_callback(self, data):
        # GPS 데이터 파싱
        gps_data = data.data.split(',')
        if len(gps_data) >= 2:
            self.latitude = float(gps_data[0])
            self.longitude = float(gps_data[1])
            
            self.sub_vehicle_position = (self.latitude, self.longitude)
            
    def calculate_distance(self):
        if self.main_vehicle_position and self.sub_vehicle_position:
            # 두 GPS 위치 간의 거리 계산
            distance = geodesic(self.main_vehicle_position, self.sub_vehicle_position).meters
            rospy.loginfo(f"Distance between vehicles: {distance} meters")
            return distance
        else:
            # rospy.logwarn("Main vehicle position or sub vehicle position is not set")
            return None
       

    def angle_to_pwm(self, angle):
        min_angle = -30
        max_angle = 30
        min_pwm = self.STEERING_MIN_PULSE_WIDTH
        max_pwm = self.STEERING_MAX_PULSE_WIDTH

        normalized_angle = (angle - min_angle) / (max_angle - min_angle)
        pwm_value = min_pwm + (max_pwm - min_pwm) * normalized_angle
        return int(pwm_value)

    def set_servo_pulsewidth(self, pulsewidth):
        pulsewidth = max(self.STEERING_MIN_PULSE_WIDTH, min(self.STEERING_MAX_PULSE_WIDTH, pulsewidth))
        self.pi.set_servo_pulsewidth(self.STEERING_SERVO_PIN, pulsewidth)
        rospy.loginfo(f"Steering PWM: {pulsewidth}")

    def set_esc_pulsewidth(self, pulsewidth):
        pulsewidth = max(self.ESC_MIN_PULSE_WIDTH, min(self.ESC_MAX_PULSE_WIDTH, pulsewidth))
        self.pi.set_servo_pulsewidth(self.ESC_PIN, pulsewidth)
        rospy.loginfo(f"Motor PWM: {pulsewidth}")

    async def handle_redis_data(self):
        data_list = self.redis_client.lrange('vehicle', 0, -1)
        if data_list:
            for data in data_list:
                try:
                    payload_data = json.loads(data.decode('utf-8'))
                    timestamp = payload_data['timestamp']
                    if timestamp in self.processed_timestamps:
                        continue  # 이미 처리된 데이터이면 무시

                    self.processed_timestamps.add(timestamp)
                    #rospy.loginfo(f"Received data from Redis: {payload_data}")

                    # 차량 데이터 업데이트
                    self.main_motor_pwm = payload_data['value'].get('motor_pwm')
                    self.main_servo_pwm = payload_data['value'].get('servo_pwm')
                    self.lidar_x = payload_data['value'].get('lidar_x')
                    self.lidar_y = payload_data['value'].get('lidar_y')
                    self.lidar_z = payload_data['value'].get('lidar_z')
                    self.lidar_distance = payload_data['value'].get('lidar_distance')
                    self.gps_latitude = payload_data['value'].get('gps_latitude')
                    self.gps_longitude = payload_data['value'].get('gps_longitude')
                    self.main_lane_number = payload_data['value'].get('lane_number')

                    if self.gps_latitude and self.gps_longitude:
                        self.main_vehicle_position = (self.gps_latitude, self.gps_longitude)
            
                except json.JSONDecodeError as e:
                    rospy.logerr(f"JSON Decode Error: {e}")
                except Exception as e:
                    rospy.logerr(f"Error processing data: {e}")
                    
    def check_scenario_flags(self):
            # 비차단 키보드 입력 읽기
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == '1':
                    self.avoidance_flag = True
                    self.deceleration_flag =False
                    self.stop_flag = False
                elif key == '2':
                    self.avoidance_flag = False
                    self.deceleration_flag = True
                    self.stop_flag = False
                elif key == '3':
                    self.avoidance_flag = False
                    self.deceleration_flag = False
                    self.stop_flag = True
                elif key == '0':
                    self.avoidance_flag = False
                    self.deceleration_flag = False
                    self.stop_flag = False
                    

    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            asyncio.run(self.handle_redis_data())  # 기존 루프에 handle_redis_data 통합
            self.process_data()  # 프로세스 데이터 함수 호출
            rate.sleep()
        self.set_esc_pulsewidth(1500)
        self.set_servo_pulsewidth(1450)

    def process_data(self):
        if self.x_location is not None:
            pid = PID(1.5, 0.1, 0.03)

            angle = pid.pid_control(self.x_location - 320)
            max_angle = 30.0
            angle = max(min(angle, max_angle), -max_angle)

            self.servo_pwm = self.angle_to_pwm(angle)

            distance_from_main_to_sub = self.calculate_distance()
            
            
                
            if self.stop_flag:
                rospy.loginfo("Stop mode activated")
                self.motor_pwm = 1500
            elif self.deceleration_flag:
                rospy.loginfo("Deceleration Scenario Activated")
                self.motor_pwm = self.main_motor_pwm
            elif self.avoidance_flag:
                rospy.loginfo("Avoidance Scenario Activated")
                self.handle_avoidance_scenario()
                self.motor_pwm = 1600
            else:
                self.motor_pwm = 1600
                if distance_from_main_to_sub is not None:
                    # rospy.loginfo(f"Calculated obstacle distance: {self.lidar_distance} meters")
                    if -2 <= self.lidar_y <= -0.7:
                        self.obstacle_distance = self.lidar_x + distance_from_main_to_sub
                        if 0.0 < self.obstacle_distance <= 5.0:
                            rospy.loginfo("Obstacle detected within 5 meters. Initiating lane change.")
                            self.avoidance_flag = True
                            rospy.loginfo(f"Calculated obstacle distance: {self.obstacle_distance} meters")
                elif self.main_lane_number == self.lane_number:
                    self.deceleration_flag = True
                    if self.main_motor_pwm == 1500:
                        self.stop_flag = True
                        self.deceleration_flag = False

            if self.obstacle_distance is not None:
                self.obstacle_distance = self.lidar_x + (distance_from_main_to_sub if distance_from_main_to_sub is not None else 0.0)
            rospy.loginfo(f"lidar_distance: {self.lidar_distance} meters")
            self.set_servo_pulsewidth(self.servo_pwm)
            self.set_esc_pulsewidth(self.motor_pwm)

    def handle_avoidance_scenario(self):
        rate = rospy.Rate(15)
        loop_duration = 28
        self.lane_number = 1
        self.servo_pwm = self.angle_to_pwm(25)
        self.motor_pwm = 1610
        for _ in range(loop_duration):
            self.set_servo_pulsewidth(self.servo_pwm)
            self.set_esc_pulsewidth(self.motor_pwm)
            rospy.loginfo("Steering pwm : {}".format(self.servo_pwm))
            rospy.loginfo("Motor pwm : {}".format(self.motor_pwm))
            rate.sleep()

        self.avoidance_flag = False

    def cleanup(self):
        rospy.loginfo("Cleaning up...")
        
if __name__ == "__main__":
    try:
        motor_controller = MotorController()
        motor_controller.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Exception occurred: {e}")



