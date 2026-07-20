# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import csv
import os
import time
import robomaster
from robomaster import robot

# --- ตั้งชื่อไฟล์ CSV แยกตามชนิดเซนเซอร์ ---
FILE_POSITION = "sensor_position.csv"
FILE_ATTITUDE = "sensor_attitude.csv"
FILE_IMU      = "sensor_imu.csv"
FILE_ESC      = "sensor_esc.csv"

# --- ฟังก์ชันสร้างไฟล์และใส่หัวตารางถ้ายังไม่มีไฟล์ ---
def init_csv_file(filename, headers):
    if not os.path.exists(filename):
        with open(filename, mode='w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp"] + headers)

# --- แยก Handler เพื่อบันทึกข้อมูลลง CSV ตามประเภทเซนเซอร์ ---
def sub_position_handler(position_info):
    x, y, z = position_info
    timestamp = time.time()
    print(f"[Position] X:{x:.2f}, Y:{y:.2f}, Z:{z:.2f}")
    with open(FILE_POSITION, mode='a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([timestamp, x, y, z])

def sub_attitude_handler(attitude_info):
    yaw, pitch, roll = attitude_info
    timestamp = time.time()
    print(f"[Attitude] Yaw:{yaw:.2f}, Pitch:{pitch:.2f}, Roll:{roll:.2f}")
    with open(FILE_ATTITUDE, mode='a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([timestamp, yaw, pitch, roll])

def sub_imu_handler(imu_info):
    acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu_info
    timestamp = time.time()
    print(f"[IMU] Acc_X:{acc_x:.2f}, Acc_Y:{acc_y:.2f}, Acc_Z:{acc_z:.2f}")
    with open(FILE_IMU, mode='a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([timestamp, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z])

def sub_esc_handler(esc_info):
    speeds = esc_info[0]     # ความเร็วล้อ 1-4
    positions = esc_info[1]  # ตำแหน่งล้อ 1-4
    timestamp = time.time()
    print(f"[ESC] W1 Speed:{speeds[0]}, W1 Pos:{positions[0]}")
    with open(FILE_ESC, mode='a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([timestamp] + speeds + positions)


if __name__ == '__main__':
    # 1. เริ่มต้นสร้างไฟล์และหัวตาราง CSV
    init_csv_file(FILE_POSITION, ["X", "Y", "Z"])
    init_csv_file(FILE_ATTITUDE, ["Yaw", "Pitch", "Roll"])
    init_csv_file(FILE_IMU, ["Acc_X", "Acc_Y", "Acc_Z", "Gyro_X", "Gyro_Y", "Gyro_Z"])
    init_csv_file(FILE_ESC, ["Speed_W1", "Speed_W2", "Speed_W3", "Speed_W4", "Pos_W1", "Pos_W2", "Pos_W3", "Pos_W4"])

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis

    # 2. เริ่มบอกรับข้อมูลเซนเซอร์ (Subscribe) พร้อมส่งเข้าฟังก์ชันเก็บบันทึก CSV
    ep_chassis.sub_position(freq=5, callback=sub_position_handler) # ปรับ freq ขึ้นเล็กน้อยเพื่อให้เห็นข้อมูลขยับต่อเนื่อง
    ep_chassis.sub_attitude(freq=5, callback=sub_attitude_handler)
    ep_chassis.sub_imu(freq=10, callback=sub_imu_handler)
    ep_chassis.sub_esc(freq=20, callback=sub_esc_handler)

    print("--- เริ่มต้นการเคลื่อนที่ และ เก็บค่าเซนเซอร์ลงไฟล์ CSV ---")

    # 3. สั่งเคลื่อนที่ตามโจทย์บนหน้าจอโปรเจคเตอร์
    # เคลื่อนไปข้างหน้า +x ระยะ 30 cm (0.3 เมตร) ด้วยความเร็ว 0.1 m/s
    ep_chassis.move(x=0.3, y=0, z=0, xy_speed=0.1).wait_for_completed()
    
    # เคลื่อนถอยหลัง -x ระยะ 30 cm (0.3 เมตร) ด้วยความเร็ว 0.1 m/s
    ep_chassis.move(x=-0.3, y=0, z=0, xy_speed=0.1).wait_for_completed()

    # ปล่อยให้ระบบบันทึกค่าเก็บตกอีก 1 วินาทีหลังจากหุ่นยนต์หยุดนิ่ง
    time.sleep(1)

    # 4. ยกเลิกการบอกรับข้อมูลทั้งหมด (Unsubscribe)
    ep_chassis.unsub_esc()
    ep_chassis.unsub_imu()
    ep_chassis.unsub_attitude()
    ep_chassis.unsub_position()

    ep_robot.close()
    print("--- บันทึกข้อมูลเสร็จสิ้น ไฟล์ .csv ทั้งหมดถูกเซฟในโฟลเดอร์เรียบร้อยแล้ว ---")
    