#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import serial
import time

# 시리얼 포트 설정 (포트와 보드레이트를 수정할 수 있습니다)
port = '/dev/ttyS0'  # 또는 '/dev/ttyAMA0'
ser = serial.Serial(port, 9600, timeout=1)
ser.flush()

def parse_nmea(sentence):
    parts = sentence.split(',')
    latitude, longitude = None, None

    if parts[0] == '$GPGGA' or parts[0] == '$GPRMC':
        if parts[0] == '$GPGGA':
            latitude = parts[2]
            lat_dir = parts[3]
            longitude = parts[4]
            lon_dir = parts[5]
        elif parts[0] == '$GPRMC':
            latitude = parts[3]
            lat_dir = parts[4]
            longitude = parts[5]
            lon_dir = parts[6]

        if latitude and longitude and lat_dir and lon_dir:
            # 위도와 경도를 도와 분으로 분리하여 변환
            lat_deg = int(latitude[:2])
            lat_min = float(latitude[2:])
            lon_deg = int(longitude[:3])
            lon_min = float(longitude[3:])

            lat = lat_deg + (lat_min / 60.0)
            lon = lon_deg + (lon_min / 60.0)

            # 남위 또는 서경일 경우 값을 음수로 변환
            if lat_dir == 'S':
                lat = -lat
            if lon_dir == 'W':
                lon = -lon

            return lat, lon
    return None, None

def gps_publisher():
    rospy.init_node('gps_publisher', anonymous=True)
    lat_pub = rospy.Publisher('/gps/latitude', Float32, queue_size=10)
    lon_pub = rospy.Publisher('/gps/longitude', Float32, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').rstrip()
            if line.startswith('$'):
                rospy.loginfo(f"수신된 원본 GPS 데이터: {line}")
                lat, lon = parse_nmea(line)
                if lat is not None and lon is not None:
                    rospy.loginfo(f"위도: {lat}, 경도: {lon}")
                    lat_pub.publish(lat)
                    lon_pub.publish(lon)
                # else:
                    # rospy.logwarn("유효한 GPS 데이터가 없음")
        rate.sleep()

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass

