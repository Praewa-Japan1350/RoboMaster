import csv
import time
from robomaster import robot

# -----------------------------
# สร้างไฟล์ CSV
# -----------------------------
acc_file = open("ACC.csv", "w", newline="")
gyro_file = open("GYRO.csv", "w", newline="")
dist_file = open("Distance.csv", "w", newline="")

acc_writer = csv.writer(acc_file)
gyro_writer = csv.writer(gyro_file)
dist_writer = csv.writer(dist_file)

acc_writer.writerow(["Time", "acc_x", "acc_y", "acc_z"])
gyro_writer.writerow(["Time", "gyro_x", "gyro_y", "gyro_z"])
dist_writer.writerow(["Time", "Distance"])

start_time = time.time()

# -----------------------------
# Callback IMU
# -----------------------------
def imu_callback(sub_info):
    acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = sub_info

    t = round(time.time() - start_time, 3)

    acc_writer.writerow([t, acc_x, acc_y, acc_z])
    gyro_writer.writerow([t, gyro_x, gyro_y, gyro_z])

# -----------------------------
# Callback Distance
# -----------------------------
def distance_callback(sub_info):
    distance = sub_info[0]

    t = round(time.time() - start_time, 3)

    dist_writer.writerow([t, distance])

# -----------------------------
# เชื่อมต่อ RoboMaster
# -----------------------------
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")

ep_chassis = ep_robot.chassis
ep_sensor = ep_robot.sensor

# Subscribe Sensor
ep_chassis.sub_imu(freq=20, callback=imu_callback)
ep_sensor.sub_distance(freq=20, callback=distance_callback)

print("Recording Stationary (30 s)...")

# เก็บข้อมูล 30 วินาที (หุ่นไม่เคลื่อนที่)
time.sleep(30)

print("Finished.")

# ยกเลิกการ Subscribe
ep_chassis.unsub_imu()
ep_sensor.unsub_distance()

# ปิดไฟล์
acc_file.close()
gyro_file.close()
dist_file.close()

# ปิดการเชื่อมต่อ
ep_robot.close()

print("ACC.csv, GYRO.csv และ Distance.csv ถูกบันทึกในโฟลเดอร์ scenerio_1")