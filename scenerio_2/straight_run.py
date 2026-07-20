import csv
import time
from robomaster import robot

# Files (saved in this folder)
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

def imu_callback(sub_info):
    acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = sub_info
    t = round(time.time() - start_time, 3)
    acc_writer.writerow([t, acc_x, acc_y, acc_z])
    gyro_writer.writerow([t, gyro_x, gyro_y, gyro_z])

def distance_callback(sub_info):
    distance = sub_info[0]
    t = round(time.time() - start_time, 3)
    dist_writer.writerow([t, distance])

def main():
    # Initialize robot (requires actual RoboMaster connection)
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis
    ep_sensor = ep_robot.sensor

    # Subscribe sensors
    ep_chassis.sub_imu(freq=20, callback=imu_callback)
    ep_sensor.sub_distance(freq=20, callback=distance_callback)

    print("Recording and moving straight 3.0 m at constant speed...")
    # short warmup so callbacks start
    time.sleep(1)

    # Move forward x=3.0 m with constant speed (adjust xy_speed as needed)
    try:
        ep_chassis.move(x=3.0, y=0, z=0, xy_speed=0.4).wait_for_completed()
    except Exception as e:
        print('Movement failed or interrupted:', e)

    print("Movement completed. Stopping subscriptions and saving files...")

    ep_chassis.unsub_imu()
    ep_sensor.unsub_distance()

    acc_file.close()
    gyro_file.close()
    dist_file.close()

    ep_robot.close()

    print("Saved: ACC.csv, GYRO.csv, Distance.csv in scenerio_2 folder")

if __name__ == '__main__':
    main()
