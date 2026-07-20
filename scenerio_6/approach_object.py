import csv
import time
import os
from robomaster import robot

# สร้างโฟลเดอร์ scenerio_6
run_dir = os.path.dirname(__file__)

print(f"บันทึกไฟล์ลงโฟลเดอร์: {run_dir}")

start_time = time.time()
latest_distance = 9999

f_acc = open(os.path.join(run_dir, 'ACC.csv'), mode='w', newline='')
f_gyro = open(os.path.join(run_dir, 'GYRO.csv'), mode='w', newline='')
f_dist = open(os.path.join(run_dir, 'Distance.csv'), mode='w', newline='')

writer_acc = csv.writer(f_acc)
writer_gyro = csv.writer(f_gyro)
writer_dist = csv.writer(f_dist)

writer_acc.writerow(['Time', 'acc_x', 'acc_y', 'acc_z'])
writer_gyro.writerow(['Time', 'gyro_x', 'gyro_y', 'gyro_z'])
writer_dist.writerow(['Time', 'Distance(mm.)'])

def imu_info_handler(imu_info):
    acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu_info
    current_time = time.time() - start_time
    writer_acc.writerow([f"{current_time:.3f}", acc_x, acc_y, acc_z])
    writer_gyro.writerow([f"{current_time:.3f}", gyro_x, gyro_y, gyro_z])

def tof_data_handler(sub_info):
    global latest_distance
    distance = sub_info[0]
    latest_distance = distance
    current_time = time.time() - start_time
    writer_dist.writerow([f"{current_time:.3f}", distance])

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis

    ep_robot.chassis.sub_imu(freq=50, callback=imu_info_handler)
    ep_robot.sensor.sub_distance(freq=50, callback=tof_data_handler)

    print(">>> วิ่งเข้าใกล้วัตถุ และหยุดที่ระยะเป้าหมาย (29.0 - 31.0 cm)...")
    
    SPEED_FORWARD = 0.2  # ความเร็วเดินหน้าคงที่ (m/s)
    MIN_DIST = 290        # 29.0 cm
    MAX_DIST = 310        # 31.0 cm

    try:
        while True:
            dist = latest_distance

            # เงื่อนไขหยุดเมื่ออยู่ในช่วงระยะ 290 - 310 mm
            if MIN_DIST <= dist <= MAX_DIST:
                ep_chassis.drive_speed(x=0, y=0, z=0)
                print(f"หยุดสำเร็จ! ระยะอยู่ที่เป้าหมาย: {dist/10:.1f} cm ({dist} mm)")
                break

            # ถ้าระยะน้อยกว่า 290 mm (ใกล้เกินไป) ถอยหลังกลับมา
            elif 0 < dist < MIN_DIST:
                ep_chassis.drive_speed(x=-0.05, y=0, z=0)

            # ถ้าระยะมากกว่า 310 mm เดินหน้าเข้าหา
            elif dist > MAX_DIST:
                ep_chassis.drive_speed(x=SPEED_FORWARD, y=0, z=0)

            time.sleep(0.01)

        ep_chassis.drive_speed(x=0, y=0, z=0)
        time.sleep(1)
    except KeyboardInterrupt:
        print("หยุดการทำงานด้วยคีย์บอร์ด")

    ep_robot.chassis.unsub_imu()
    ep_robot.sensor.unsub_distance()
    ep_robot.close()

    f_acc.close()
    f_gyro.close()
    f_dist.close()
    print("บันทึกข้อมูลสำเร็จ!")