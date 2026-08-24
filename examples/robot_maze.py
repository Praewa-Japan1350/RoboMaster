import robomaster
from robomaster import robot
import time
import matplotlib.pyplot as plt

# ==========================================
# 1. การตั้งค่าระบบและตัวแปร
# ==========================================
TARGET_GAP = 10.0      # ระยะกึ่งกลางที่ต้องการจากแต่ละฝั่ง (cm)
FRONT_LIMIT = 15.0     # ระยะเบรกด้านหน้า (cm)
MOVE_SPEED = 0.35      # ความเร็วเดินหน้า (m/s)

path_x = [0.0]
path_y = [0.0]

def sharp_adc_to_cm(adc_val):
    """
    สมการคำนวณระยะทางสำหรับ Sharp GP2Y0A41SK0F (ช่วงวัด 4 - 30 cm)
    """
    voltage = (adc_val / 1023.0) * 3.3
    
    # ถ้าแรงดันต่ำมาก แปลว่าไม่มีกำแพงอยู่ในระยะ 30 cm ให้ตีเป็นพื้นที่โล่ง (35 cm)
    if voltage < 0.35:
        return 35.0  
    
    try:
        # สมการ Power Fit สำหรับรุ่น 0A41SK0F
        distance = 12.08 * (voltage ** -1.058)
    except (ZeroDivisionError, ValueError):
        distance = 35.0
        
    # จำกัดช่วงระยะให้อยู่ในขอบเขตการทำงาน 4 - 35 ซม.
    return max(4.0, min(35.0, distance))

def sub_position_handler(position_info):
    """Callback เก็บพิกัด Odometry แบบ Real-time เพื่อนำไปพล็อตแผนที่"""
    x, y, _ = position_info
    path_x.append(x)
    path_y.append(y)

# ==========================================
# 2. คำสั่งควบคุมแขนกลและมือจับ
# ==========================================
def pick_up_object(arm, gripper):
    """คีบวัตถุที่จุดเริ่มต้นและยกขึ้น"""
    print("[Arm] Picking up object...")
    gripper.open(power=50)
    time.sleep(1)
    arm.moveto(x=160, y=-60).wait_for_completed()
    gripper.close(power=80)
    time.sleep(1.2)
    arm.moveto(x=100, y=80).wait_for_completed()
    print("[Arm] Object secured.")

def drop_off_object(arm, gripper):
    """วางวัตถุที่จุดสิ้นสุด"""
    print("[Arm] Dropping off object...")
    arm.moveto(x=160, y=-60).wait_for_completed()
    gripper.open(power=50)
    time.sleep(1)
    arm.moveto(x=100, y=50).wait_for_completed()
    print("[Arm] Mission complete!")

def generate_map():
    """สร้างและบันทึกภาพแผนที่ 2D ลงในเครื่อง"""
    plt.figure(figsize=(6, 8))
    plt.plot(path_x, path_y, marker='.', color='blue', linestyle='-', label='Trajectory')
    plt.plot(path_x[0], path_y[0], 'go', markersize=10, label='Start')
    plt.plot(path_x[-1], path_y[-1], 'r*', markersize=14, label='Drop Goal')
    plt.title("RoboMaster EP Maze Mapping (10cm Alignment)")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.axis('equal')
    plt.savefig("maze_result_map.png", dpi=300)
    print("[System] Map saved as 'maze_result_map.png'")

# ==========================================
# 3. ระบบนำทางในเขาวงกต (Main Controller)
# ==========================================
def navigate_maze(ep_robot):
    chassis = ep_robot.chassis
    sensor = ep_robot.sensor_adaptor

    start_time = time.time()
    max_duration = 360  # จำกัดเวลาวิ่งไม่เกิน 6 นาที (ตามกฎเกณฑ์ 10 คะแนน)

    print("[Nav] Starting Wall-Following with 3 Adaptors...")
    while time.time() - start_time < max_duration:
        # --- อ่านค่าจาก Sensor Adaptor 3 ตัว ---
        
        # Adaptor ID 1
        ir_right_rear = sensor.get_io(id=1, port=1)             # Port 1: IR ขวา-หลัง
        raw_right_front = sensor.get_adc(id=1, port=2)          # Port 2: Sharp ขวา-หน้า

        # Adaptor ID 2
        raw_left_front = sensor.get_adc(id=2, port=1)           # Port 1: Sharp ซ้าย-หน้า
        dist_front_mm = sensor.get_adc(id=2, port=2)            # Port 2: ToF หน้า

        # Adaptor ID 3
        ir_left_rear = sensor.get_io(id=3, port=1)              # Port 1: IR ซ้าย-หลัง

        # --- แปลงค่าระยะทาง ---
        dist_left_front = sharp_adc_to_cm(raw_left_front)
        dist_right_front = sharp_adc_to_cm(raw_right_front)
        dist_front = dist_front_mm / 10.0  # แปลง mm เป็น cm

        # --- ตรรกะตัดสินใจ (Left-Wall Following + Center Alignment) ---
        
        # 1. ทางแยกซ้ายเปิดโล่ง -> ให้ความสำคัญกับการเลี้ยวซ้ายก่อน
        if dist_left_front > 25.0:
            chassis.drive_speed(x=0, y=0, z=0)
            chassis.move(x=0, y=0, z=90, z_speed=100).wait_for_completed()
            chassis.move(x=0.35, y=0, z=0, xy_speed=0.4).wait_for_completed()

        # 2. ทางข้างหน้าว่าง -> เดินตรงพร้อมสไลด์บาลานซ์ให้อยู่กึ่งกลาง 10 cm
        elif dist_front > FRONT_LIMIT:
            vy = 0.0
            wz = 0.0

            # บาลานซ์กึ่งกลางด้วย Sharp ซ้าย-ขวา
            if dist_left_front <= 25.0 and dist_right_front <= 25.0:
                diff = dist_left_front - dist_right_front
                if abs(diff) > 1.0:
                    vy = diff * 0.015  # สไลด์ Mecanum ชดเชยระยะ
            elif dist_left_front <= 25.0:
                error_left = dist_left_front - TARGET_GAP
                if abs(error_left) > 1.0:
                    vy = -error_left * 0.02

            # ป้องกันท้ายสะบัดชนกำแพงด้วย IR หลัง (0 = เจอสิ่งกีดขวาง)
            if ir_left_rear == 0:
                wz = -15  # หมุนหัวเบนซ้ายเล็กน้อยเพื่อดันท้ายขวาออก
                vy = 0.1  # สไลด์ขวาหลบ
            elif ir_right_rear == 0:
                wz = 15   # หมุนหัวเบนขวาเล็กน้อย
                vy = -0.1 # สไลด์ซ้ายหลบ

            chassis.drive_speed(x=MOVE_SPEED, y=vy, z=wz)
            time.sleep(0.05)

        # 3. ข้างหน้าตัน ซ้ายตัน ขวาเปิด -> เลี้ยวขวา
        elif dist_right_front > 25.0:
            chassis.drive_speed(x=0, y=0, z=0)
            chassis.move(x=0, y=0, z=-90, z_speed=100).wait_for_completed()

        # 4. ทางตันทุกทิศทาง (Dead End) -> หมุนกลับลำ 180 องศา
        else:
            chassis.drive_speed(x=0, y=0, z=0)
            chassis.move(x=0, y=0, z=180, z_speed=100).wait_for_completed()

    chassis.drive_speed(x=0, y=0, z=0)

# ==========================================
# 4. จุดเริ่มต้นโปรแกรม (Execution Flow)
# ==========================================
if __name__ == '__main__':
    ep_robot = robot.Robot()
    
    # "ap" = ต่อตรงเข้า Wi-Fi หุ่น, "sta" = ผ่าน Router วงเดียวกัน
    ep_robot.initialize(conn_type="ap")

    try:
        # ปิดกล้องตามกติกา และเปิดรับข้อมูลพิกัด Odometry
        ep_robot.camera.stop_video_stream()
        ep_robot.chassis.sub_position(freq=5, callback=sub_position_handler)

        # ทำภารกิจตามลำดับ
        pick_up_object(ep_robot.robotic_arm, ep_robot.gripper)
        navigate_maze(ep_robot)
        drop_off_object(ep_robot.robotic_arm, ep_robot.gripper)

    except KeyboardInterrupt:
        print("\n[Safety] Emergency Stop triggered by user!")
        ep_robot.chassis.drive_speed(x=0, y=0, z=0)

    finally:
        # ตัดการเชื่อมต่อและวาดแผนที่เสมอ ไม่ว่าจะจบปกติหรือเกิด Error
        ep_robot.chassis.unsub_position()
        ep_robot.close()
        generate_map()