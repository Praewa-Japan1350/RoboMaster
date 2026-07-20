import csv
import time
from robomaster import robot

# Files saved in this folder
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
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis
    ep_sensor = ep_robot.sensor

    ep_chassis.sub_imu(freq=50, callback=imu_callback)
    ep_sensor.sub_distance(freq=50, callback=distance_callback)

    print("Recording scenario 4: straight 3s -> left 90° -> stop")
    time.sleep(1)

    # Run straight for 3 seconds at approximately constant speed
    try:
        ep_chassis.drive_speed(x=0.35, y=0, z=0)
        time.sleep(3)
        ep_chassis.drive_speed(x=0, y=0, z=0)
    except Exception:
        try:
            task = ep_chassis.move(x=1.0, y=0, z=0, xy_speed=0.35)
        except Exception:
            task = None
        time.sleep(3)
        if task is not None:
            try:
                task.stop()
            except Exception:
                pass

    time.sleep(0.2)

    # Turn left 90 degrees
    try:
        ep_chassis.move(x=0, y=0, z=90).wait_for_completed()
    except Exception as e:
        print('Left turn failed:', e)

    time.sleep(0.5)

    print("Scenario 4 finished. Saving files...")

    try:
        ep_chassis.unsub_imu()
    except Exception:
        pass
    try:
        ep_sensor.unsub_distance()
    except Exception:
        pass

    acc_file.close()
    gyro_file.close()
    dist_file.close()
    ep_robot.close()

    print("Saved: ACC.csv, GYRO.csv, Distance.csv in scenerio_4")


if __name__ == '__main__':
    main()
