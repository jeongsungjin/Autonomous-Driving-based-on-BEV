
# -*- coding: utf-8 -*-

import socket
import json
import threading
import sys
import time # 시뮬레이션을 위한 time.sleep 추가
import math # 각속도 계산 등을 위한 math 모듈 추가

# --- 설정 ---
SERVER_IP   = '0.0.0.0' # 모든 IP에서 접속 허용
SERVER_PORT = 8888      # 사용할 포트 번호

# RC카 기구학적 파라미터 (Jetson Nano 클라이언트와 동일하게 설정)
TRACK_WIDTH = 0.15      # RC카 트랙 폭 (미터)
MAX_LINEAR_SPEED = 0.5  # RC카의 최대 선형 속도 (m/s) - Rosmaster의 100에 해당
MAX_MOTOR_CMD = 100     # Rosmaster 모터 명령의 최대값
# ------------

# Pure Pursuit 시뮬레이션을 위한 임시 변수
# 실제 Pure Pursuit 구현 시에는 이 값들이 동적으로 계산되어야 합니다.
current_linear_x = 0.3 # 초기 전진 속도 (m/s)
current_angular_z = 0.0 # 초기 각속도 (rad/s)

def compute_rosmaster_speeds(linear_x, angular_z):
    """
    선속도(linear_x)와 각속도(angular_z)를 Rosmaster의 좌우 바퀴 속도(-100~100)로 변환합니다.
    이 변환 로직은 Jetson Nano 클라이언트 코드와 일치해야 합니다.
    """
    # 좌우 바퀴의 희망 선속도 (m/s)
    left_target_speed_mps = linear_x - (angular_z * TRACK_WIDTH / 2.0)
    right_target_speed_mps = linear_x + (angular_z * TRACK_WIDTH / 2.0)

    # Rosmaster의 -100 ~ 100 범위로 스케일링
    # MAX_LINEAR_SPEED가 0이 아님을 가정
    left_motor_cmd = int((left_target_speed_mps / MAX_LINEAR_SPEED) * MAX_MOTOR_CMD)
    right_motor_cmd = int((right_target_speed_mps / MAX_LINEAR_SPEED) * MAX_MOTOR_CMD)

    # 클램핑: -100 ~ 100 범위로 제한
    left_motor_cmd = max(-MAX_MOTOR_CMD, min(left_motor_cmd, MAX_MOTOR_CMD))
    right_motor_cmd = max(-MAX_MOTOR_CMD, min(right_motor_cmd, MAX_MOTOR_CMD))

    return left_motor_cmd, right_motor_cmd

def run_server():
    global current_linear_x, current_angular_z # 전역 변수 사용 선언

    print("Pure Pursuit 명령 전송 서버를 시작합니다.")
   
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((SERVER_IP, SERVER_PORT))
        s.listen()
        print(f"Jetson의 연결을 {SERVER_PORT} 포트에서 기다립니다...")

        conn, addr = s.accept()
        with conn:
            print(f"Jetson이 연결되었습니다: {addr}")
            print("="*40)
            print("서버가 Pure Pursuit 시뮬레이션 명령을 전송합니다.")
            print("Ctrl + C를 눌러 서버를 종료하세요.")
            print("="*40)

            # 실제 Pure Pursuit 알고리즘은 여기에 구현됩니다.
            # 여기서는 단순히 선속도와 각속도를 변화시키는 시뮬레이션 예시를 보여줍니다.
            # 예를 들어, 일직선 주행, 좌회전, 우회전 등의 시퀀스를 정의할 수 있습니다.
            command_sequence = [
                (0.3, 0.0, 3),  # 3초간 직진 (linear_x=0.3, angular_z=0.0)
                (0.2, 1.0, 2),  # 2초간 좌회전 (linear_x=0.2, angular_z=1.0)
                (0.3, 0.0, 3),  # 3초간 직진
                (0.2, -1.0, 2), # 2초간 우회전 (linear_x=0.2, angular_z=-1.0)
                (0.0, 0.0, 1),  # 1초간 정지
            ]

            while True:
                for linear_val, angular_val, duration in command_sequence:
                    current_linear_x = linear_val
                    current_angular_z = angular_val
                   
                    # 실제 Pure Pursuit에서는 이 부분에서 지속적으로 current_linear_x와 current_angular_z를
                    # RC카의 현재 위치(x,y,yaw)와 목표 웨이포인트를 기반으로 계산하게 됩니다.
                    # 예: target_waypoint = get_next_waypoint(current_pose)
                    #     current_linear_x, current_angular_z = pure_pursuit_calc(current_pose, target_waypoint)

                    left, right = compute_rosmaster_speeds(current_linear_x, current_angular_z)
                   
                    command = {
                        "left_speed": left,
                        "right_speed": right
                    }
                   
                    json_command = json.dumps(command)
                   
                    try:
                        conn.sendall(json_command.encode('utf-8'))
                        print(f"  >> 전송: Lx={current_linear_x:.2f}, Az={current_angular_z:.2f} -> L:{left}, R:{right}")
                    except (socket.error, BrokenPipeError):
                        print("클라이언트와의 연결이 끊겼습니다.")
                        break # 연결이 끊기면 시퀀스 종료
                   
                    time.sleep(duration) # 시뮬레이션에서는 이 시간 동안 같은 명령 전송

                # 시퀀스가 끝나면 다시 반복하거나, 멈추고 종료할 수 있습니다.
                # 여기서는 무한 루프이므로 시퀀스가 끝나면 처음으로 돌아갑니다.

                print("시퀀스 반복 시작...")
                time.sleep(1) # 시퀀스 반복 전 잠시 대기

    print("서버를 종료합니다.")


if __name__ == "__main__":
    try:
        run_server()
    except KeyboardInterrupt:
        print("\n사용자에 의해 서버가 종료되었습니다.")
    except Exception as e:
        print(f"예상치 못한 오류 발생: {e}")



