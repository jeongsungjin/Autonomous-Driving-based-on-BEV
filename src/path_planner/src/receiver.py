
# -*- coding: utf-8 -*-

import socket
import json
import time
from Rosmaster_Lib.Rosmaster_Lib import Rosmaster

# --- 설정 ---
SERVER_IP   = "172.20.10.10"     #PC의 Wi-Fi IP 주소 (사용자 환경에 맞게 수정)
SERVER_PORT = 8888              # PC 서버와 동일한 포트
PORT        = "/dev/ttyUSB0"    # Rosmaster 연결 포트
CAR_TYPE    = 1                 # Rosmaster car_type 파라미터
# ------------

class RemoteRosmasterDriver:
    def __init__(self):
        # Rosmaster 초기화
        self.car = Rosmaster(car_type=CAR_TYPE, com=PORT, debug=False)
        self.car.clear_auto_report_data()
        self.car.create_receive_threading()

    def run(self):
        print("RC카 원격 제어 클라이언트를 시작합니다.")

        while True:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    print(f"서버({SERVER_IP}:{SERVER_PORT})에 연결을 시도합니다...")
                    s.connect((SERVER_IP, SERVER_PORT))
                    print("서버에 성공적으로 연결되었습니다! 명령 수신 대기 중...")

                    while True:
                        # 서버로부터 제어 명령 수신
                        data = s.recv(1024)
                        if not data:
                            print("서버와의 연결이 끊겼습니다.")
                            self.car.set_motor(0, 0, 0, 0) # 비상 정지
                            break

                        try:
                            command = json.loads(data.decode('utf-8'))

                            left_speed = command.get("left_speed", 0)
                            right_speed = command.get("right_speed", 0)

                            print(f"수신: L={left_speed}, R={right_speed} → 모터 구동")

                            # Rosmaster 모터 제어 (4개의 모터에 동일한 좌/우 속도 전달)
                            self.car.set_motor(left_speed, left_speed, right_speed, right_speed)

                        except json.JSONDecodeError:
                            print(f"잘못된 JSON 형식의 데이터 수신: {data.decode('utf-8')}")

            except Exception as e:
                print(f"오류 발생: {e}")
                self.car.set_motor(0, 0, 0, 0) # 비상 정지
                print("5초 후 재연결을 시도합니다...")
                time.sleep(5)

if __name__ == '__main__':
    driver = RemoteRosmasterDriver()
    try:
        driver.run()
    except KeyboardInterrupt:
        print("\n사용자에 의해 프로그램이 종료됩니다.")
        driver.car.set_motor(0, 0, 0, 0) # 종료 시 모터 정지



