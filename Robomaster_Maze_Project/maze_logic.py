# -*- coding: utf-8 -*-
"""
============================================================================
ไฟล์: maze_logic.py
คำอธิบาย: คลาส MazeSolver สำหรับประมวลผลอัลกอริทึมแก้ปัญหากลางเขาวงกต
           - อัลกอริทึม Wall Follower (กฎมือขวา Right-Hand Rule)
           - เงื่อนไขสำคัญ: การสไลด์/เลี้ยวเข้าทางแยกจะทำงานเมื่อ IR ด้านหลังส่งค่า 0
             (พ้นมุมกำแพงแล้วเท่านั้น) เพื่อป้องกันไม่ให้ส่วนท้ายหุ่นยนต์เกี่ยว/ชนกำแพง
           - ตรวจสอบการถึงจุดเป้าหมาย: ระยะ Euclidean (x,y) < 10 cm และ ToF หน้า < 15 cm
ภาษาคอมเม้นต์: ภาษาไทย
============================================================================
"""

import math
import time

class MazeSolver:
    """
    คลาสสำหรับตัดสินใจและควบคุมสเตตแมชชีนในการเดินเขาวงกต
    """
    # นิยามสถานะการทำงาน
    STATE_IDLE = "IDLE"
    STATE_NAVIGATING = "NAVIGATING"
    STATE_TARGET_REACHED = "TARGET_REACHED"

    def __init__(self, target_x: float, target_y: float):
        """
        :param target_x: พิกัดเป้าหมายแกน X (เมตร)
        :param target_y: พิกัดเป้าหมายแกน Y (เมตร)
        """
        self.target_x = float(target_x)
        self.target_y = float(target_y)
        self.state = self.STATE_IDLE

        # เกณฑ์ระยะทางสำหรับเซนเซอร์
        self.WALL_THRESHOLD_CM = 25.0  # ระยะที่ถือว่ามีกำแพงอยู่ข้างๆ
        self.FRONT_LIMIT_CM = 25.0     # ระยะเบรกกำแพงด้านหน้า (cm) (เพิ่มระยะเบรกให้ไกลขึ้น)
        self.MOVE_SPEED = 0.20         # ความเร็วเดินหน้า (ลดลงเพื่อไม่ให้ชนก่อนเซนเซอร์อ่านทัน)

    def is_target_reached(self, current_x: float, current_y: float, front_tof: float) -> bool:
        """
        ตรวจสอบว่าหุ่นยนต์ถึงเป้าหมายแล้วหรือยัง
        เงื่อนไข:
        1. ระยะทาง Euclidean ทางพิกัด Odometry จาก (current_x, current_y) ถึง (target_x, target_y) น้อยกว่า 10 cm (0.10 m)
        2. เซนเซอร์ ToF ด้านหน้าระยะน้อยกว่า 15 cm (ชน/จ่อเป้าหมาย)
        """
        dx = current_x - self.target_x
        dy = current_y - self.target_y
        euclidean_dist_m = math.sqrt(dx * dx + dy * dy)
        euclidean_dist_cm = euclidean_dist_m * 100.0

        # แสดง Log ข้อมูลการเข้าใกล้เป้าหมาย
        if euclidean_dist_cm < 30.0:
            print(f"[MazeSolver] เข้าใกล้เป้าหมาย: ระยะ Odometry = {euclidean_dist_cm:.1f} cm, ToF หน้า = {front_tof:.1f} cm")

        if euclidean_dist_cm < 10.0 and front_tof < self.FRONT_LIMIT_CM:
            return True
        return False

    def update(self, sensors, robot_controller) -> str:
        """
        อัปเดตสเตตการตัดสินใจเดินเขาวงกตตามกฎมือขวา (Right-Hand Rule)
        :param sensors: วัตถุ SensorHub
        :param robot_controller: วัตถุ RobotController
        :return: สเตตปัจจุบัน ("NAVIGATING" หรือ "TARGET_REACHED")
        """
        # อ่านค่าเซนเซอร์ล่าสุดผ่าน SensorHub
        sensors.update_sensors()

        front_tof = sensors.get_front_tof()
        sharp_left = sensors.get_sharp_left()
        sharp_right = sensors.get_sharp_right()
        rear_ir_l = sensors.get_rear_ir_left()
        rear_ir_r = sensors.get_rear_ir_right()

        # [Debug] พิมพ์ค่าเซนเซอร์ทุกตัวออกมาดูแบบ Real-time
        print(f"[Sensors Debug] หน้า(ToF)={front_tof:.1f}cm | ขวา(Sharp)={sharp_right:.1f}cm | ซ้าย(Sharp)={sharp_left:.1f}cm | ท้ายขวา(IR)={rear_ir_r} | ท้ายซ้าย(IR)={rear_ir_l}")

        # อ่านพิกัดปัจจุบันจาก Odometry
        curr_x, curr_y = robot_controller.get_current_position()

        # 1. ตรวจสอบเงื่อนไขการถึงจุดเป้าหมาย
        if self.is_target_reached(curr_x, curr_y, front_tof):
            print("[MazeSolver] *** ภารกิจสำเร็จ: ถึงจุดหมายเรียบร้อยแล้ว! ***")
            self.state = self.STATE_TARGET_REACHED
            robot_controller.stop()
            return self.state

        self.state = self.STATE_NAVIGATING

        # --------------------------------------------------------------------
        # ตรรกะตัดสินใจเดินเขาวงกตด้วยกฎมือขวา (Right-Hand Rule)
        # --------------------------------------------------------------------
        
        # 1. เช็คด้านขวา ถ้าขวาว่างให้เลี้ยวขวา (กรณีที่เซนเซอร์มองเห็นทางแยกทันที)
        if sharp_right > self.WALL_THRESHOLD_CM:
            print("[MazeSolver] พบทางแยกขวาเปิดจากเซนเซอร์ด้านข้าง -> เลี้ยวขวา")
            robot_controller.stop()
            robot_controller.turn_right_90()
            robot_controller.move_forward_distance(0.2)
            
        # 2. ถ้าข้างหน้าว่าง ให้ตรงไปโดยใช้ PID เลี้ยงตัวให้อยู่กึ่งกลางกำแพง
        elif front_tof > self.FRONT_LIMIT_CM:
            robot_controller.move_forward_with_pid(self.MOVE_SPEED, sharp_left, sharp_right)
            
        # 3. ถ้าข้างหน้าตัน! (เจอหลุมพรางรถยาว) หุ่นจะหยุดและ "หมุนสแกน" หาทางเลี้ยวเอง
        else:
            print(f"[MazeSolver] ทางข้างหน้าตัน (ToF = {front_tof:.1f}cm)! กำลังสแกนหาทางเลี้ยว...")
            robot_controller.stop()
            
            # --- สเตป 1: สแกนทางขวา ---
            print(" -> หมุนขวา 90 องศา เพื่อเช็คทางขวา")
            robot_controller.turn_right_90()
            time.sleep(0.5)
            sensors.update_sensors()
            if sensors.get_front_tof() > self.FRONT_LIMIT_CM:
                print(" -> ทางขวาโล่ง! เดินหน้าต่อไป")
                return self.STATE_NAVIGATING
                
            # --- สเตป 2: สแกนทางซ้าย (ถ้าขวาตัน) ---
            print(" -> ขวาตัน! หมุนกลับ 180 องศา เพื่อเช็คทางซ้าย")
            robot_controller.turn_180()
            time.sleep(0.5)
            sensors.update_sensors()
            if sensors.get_front_tof() > self.FRONT_LIMIT_CM:
                print(" -> ทางซ้ายโล่ง! เดินหน้าต่อไป")
                return self.STATE_NAVIGATING
                
            # --- สเตป 3: ทางตันทุกด้าน (Dead End) ---
            print(" -> ซ้ายก็ตัน! เป็นทางตัน (Dead End) หมุนซ้าย 90 องศาเพื่อกลับหลังหัน")
            robot_controller.turn_left_90()
            
        return self.STATE_NAVIGATING
