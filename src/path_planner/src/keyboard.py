# -*- coding: utf-8 -*-

import socket
import json
import threading
import sys

# --- 설정 ---
SERVER_IP   = '0.0.0.0'
SERVER_PORT = 8888
SPEED       = 50
# ------------

def compute_wheel_speeds(key):
    """'w,a,s,d' 키에 따라 좌/우측 바퀴 속도를 계산합니다."""
    key = key.lower().strip() # 입력된 문자를 소문자로 변환하고 공백 제거
    if key == 'w': # 전진
        return SPEED, SPEED
    if key == 's': # 후진
        return -SPEED, SPEED  # 후진 시에는 바퀴 방향이 반대가 될 수 있습니다 (하드웨어에 따라 조정)
        # 만약 둘 다 음수 값이어야 한다면 return -SPEED, -SPEED 로 수정
    if key == 'a': # 좌회전
        return -SPEED, SPEED
    if key == 'd': # 우회전
        return SPEED, -SPEED
    return 0, 0 # 그 외의 모든 입력은 정지

def run_server():
    print("RC카 원격 조종 서버를 시작합니다.")
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((SERVER_IP, SERVER_PORT))
        s.listen()
        print(f"Jetson의 연결을 {SERVER_PORT} 포트에서 기다립니다...")

        conn, addr = s.accept()
        with conn:
            print(f"Jetson이 연결되었습니다: {addr}")
            print("="*40)
            print("이제 명령을 입력하고 Enter를 누르세요.")
            print("  w: 전진, s: 후진, a: 좌회전, d: 우회전")
            print("  (w,a,s,d 외 다른 키 입력 시 정지)")
            print("  프로그램 종료: Ctrl + C")
            print("="*40)

            while True:
                try:
                    # 사용자로부터 명령 입력 (Python 2는 raw_input, Python 3는 input)
                    if sys.version_info.major == 2:
                        key_input = input("명령 입력 > ")
                    else:
                        key_input = input("명령 입력 > ")

                    left, right = compute_wheel_speeds(key_input)
                    
                    command = {
                        "left_speed": left,
                        "right_speed": right
                    }
                    
                    json_command = json.dumps(command)
                    conn.sendall(json_command.encode('utf-8'))
                    print(f"  >> 전송: {json_command}")

                except (socket.error, BrokenPipeError):
                    print("클라이언트와의 연결이 끊겼습니다.")
                    break
                except Exception as e:
                    print(f"오류 발생: {e}")
                    break
    print("서버를 종료합니다.")


if __name__ == "__main__":
    try:
        run_server()
    except KeyboardInterrupt:
        print("\n사용자에 의해 서버가 종료되었습니다.")
