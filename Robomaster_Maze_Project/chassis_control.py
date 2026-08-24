# -*- coding: utf-8 -*-
"""
============================================================================
ไฟล์: chassis_control.py
คำอธิบาย: คลาส RobotController ทำหน้าที่ควบคุมการเคลื่อนที่ของหุ่นยนต์ RoboMaster EP
           - ควบคุมระบบขับเคลื่อนล้อ Mecanum (การสไลด์ซ้าย-ขวา Strafe)
           - ระบบ PID Controller รักษาระยะห่างกึ่งกลางระหว่างกำแพง 2 ฝั่ง
           - การรับพิกัด Odometry (x, y) แบบ Real-time ผ่าน RoboMaster SDK
           - การคีบและวางวัตถุด้วย Robotic Arm และ Gripper
ภาษาคอมเม้นต์: ภาษาไทย
============================================================================
"""

import time
import threading

class RobotController:
    """
    คลาสสำหรับควบคุมฐานหุ่นยนต์ (Chassis), แขนกล (Robotic Arm) และมือจับ (Gripper)
    ผ่าน RoboMaster SDK
    """

    def __init__(self, ep_robot, kp: float = 0.02, ki: float = 0.0001, kd: float = 0.005):
        """
        :param ep_robot: วัตถุ robot.Robot() จาก RoboMaster SDK
        :param kp: ค่า Proportional Gain ของ PID
        :param ki: ค่า Integral Gain ของ PID
        :param kd: ค่า Derivative Gain ของ PID
        """
        self._robot = ep_robot
        self._chassis = ep_robot.chassis
        self._arm = ep_robot.robotic_arm
        self._gripper = ep_robot.gripper

        # ตัวแปรระบบ PID
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = time.time()
        self._target_gap = 15.0  # ระยะเป้าหมายจากกำแพงแต่ละฝั่ง (เพิ่มเป็น 15cm ไม่ให้ชิดซ้าย/ขวาเกินไป)

        # ข้อมูลการติดตามเส้นทาง (Odometry Trajectory Tracking)
        self._trajectory_lock = threading.Lock()
        self._path_x = [0.0]
        self._path_y = [0.0]

    def start_position_subscription(self, freq: int = 5):
        """ เริ่มต้นการสมัครรับข้อมูล Odometry (พิกัด x, y) จากหุ่นยนต์ """
        try:
            self._chassis.sub_position(freq=freq, callback=self._position_callback)
            print("[RobotController] เริ่มต้นติดตามพิกัด Odometry (sub_position)")
        except Exception as e:
            print(f"[RobotController Error] ไม่สามารถ sub_position ได้: {e}")

    def stop_position_subscription(self):
        """ ยกเลิกการรับข้อมูล Odometry """
        try:
            self._chassis.unsub_position()
            print("[RobotController] ยกเลิกการติดตามพิกัด Odometry เรียบร้อย")
        except Exception as e:
            print(f"[RobotController Error] ไม่สามารถ unsub_position ได้: {e}")

    def _position_callback(self, position_info):
        """ Callback เมื่อหุ่นยนต์ส่งพิกัดตำแหน่งกลับมา """
        x, y, _ = position_info
        with self._trajectory_lock:
            self._path_x.append(x)
            self._path_y.append(y)

    def get_trajectory(self):
        """ คืนค่าอาเรย์คู่พิกัด [(x0, y0), (x1, y1), ...] ที่บันทึกไว้ """
        with self._trajectory_lock:
            return list(zip(self._path_x, self._path_y))

    def get_current_position(self):
        """ คืนค่าพิกัดปัจจุบัน (x, y) """
        with self._trajectory_lock:
            return self._path_x[-1], self._path_y[-1]

    # ------------------------------------------------------------------------
    # ระบบคำนวณ PID Controller สำหรับประคองหุ่นยนต์ให้อยู่กึ่งกลางกำแพง
    # ------------------------------------------------------------------------
    def calculate_pid(self, sharp_left: float, sharp_right: float, max_wall_dist: float = 25.0) -> float:
        """
        คำนวณความเร็วในการสไลด์ (Vy) ด้วย PID Controller
        - ถ้ามีกำแพงทั้งสองฝั่ง (<= max_wall_dist): ให้รักษาระยะให้อยู่ตรงกลาง (error = left - right)
        - ถ้ามีกำแพงฝั่งเดียว: ให้รักษาระยะห่างจากกำแพงฝั่งนั้นให้ได้ target_gap (10 cm)
        :return: ความเร็วสไลด์ Vy (m/s)
        """
        now = time.time()
        dt = now - self._last_time
        if dt <= 0:
            dt = 0.05

        error = 0.0
        # กรณี 1: มีกำแพงทั้งสองฝั่ง -> ปรับให้อยู่กึ่งกลาง
        if sharp_left <= max_wall_dist and sharp_right <= max_wall_dist:
            error = sharp_right - sharp_left

        # กรณี 2: มีเฉพาะกำแพงซ้าย -> ปรับระยะห่างกำแพงซ้ายเท่ากับ target_gap
        elif sharp_left <= max_wall_dist:
            error = self._target_gap - sharp_left

        # กรณี 3: มีเฉพาะกำแพงขวา -> ปรับระยะห่างกำแพงขวาเท่ากับ target_gap
        elif sharp_right <= max_wall_dist:
            error = sharp_right - self._target_gap

        # กรณี 4: ไม่มีกำแพงรอบข้างเลย -> ไม่มี Error
        else:
            error = 0.0
            self._integral = 0.0

        # เพิ่ม Deadband: ถ้าระยะคลาดเคลื่อนไม่เกิน 3 cm ถือว่าหุ่นอยู่ตรงกลางแล้ว ไม่ต้องขยับแกว่งซ้ายขวา
        if abs(error) < 3.0:
            error = 0.0

        # คำนวณ P, I, D Components (ปรับลด kp ลงในโค้ดคำนวณเลยเพื่อให้แก้ศูนย์นิ่มนวลขึ้น)
        p_term = (self.kp * 0.25) * error  # ลดความไว (kp) ลง 4 เท่า ป้องกันอาการส่ายเป็นงู
        self._integral += error * dt
        # Clamp integral เพื่อป้องกัน Integral Windup
        self._integral = max(-5.0, min(5.0, self._integral))
        i_term = self.ki * self._integral

        derivative = (error - self._last_error) / dt
        d_term = self.kd * derivative

        self._last_error = error
        self._last_time = now

        # รวมผลลัพธ์ Vy (ความเร็วสไลด์ซ้าย-ขวา)
        vy = p_term + i_term + d_term

        # จำกัดความเร็วสไลด์ให้อยู่ในช่วงที่พอดี (-0.15 ถึง +0.15 m/s)
        # ถ้าตั้งน้อยไป (0.08) หุ่นจะสู้แรงไถลธรรมชาติไม่ได้ ทำให้เป๋เข้ากำแพง
        vy = max(-0.15, min(0.15, vy))
        return vy

    # ------------------------------------------------------------------------
    # คำสั่งควบคุมการขับเคลื่อน Mecanum (Kinematics Strafing & Forward)
    # ------------------------------------------------------------------------
    def move_forward_with_pid(self, vx: float, sharp_left: float, sharp_right: float):
        """
        เดินหน้าด้วยความเร็ว vx (m/s) พร้อมใช้ PID สไลด์ชดเชยทิศทาง Vy
        """
        vy = self.calculate_pid(sharp_left, sharp_right)
        # สั่งงานล้อ Mecanum: drive_speed(x=เดินหน้า, y=สไลด์ข้าง, z=หมุนตัว)
        self._chassis.drive_speed(x=vx, y=vy, z=0.0)

    def strafe_left(self, speed: float = 0.35, distance: float = 0.25):
        """ สไลด์ออกไปทางซ้ายตามระยะทางที่กำหนด (ใช้ Mecanum Strafe) """
        self._chassis.drive_speed(x=0, y=0, z=0)
        time.sleep(0.05)
        self._chassis.move(x=0.0, y=-distance, z=0, xy_speed=speed).wait_for_completed()

    def strafe_right(self, speed: float = 0.35, distance: float = 0.25):
        """ สไลด์ออกไปทางขวาตามระยะทางที่กำหนด (ใช้ Mecanum Strafe) """
        self._chassis.drive_speed(x=0, y=0, z=0)
        time.sleep(0.05)
        self._chassis.move(x=0.0, y=distance, z=0, xy_speed=speed).wait_for_completed()

    def turn_right_90(self):
        """ หมุนตัวไปทางขวา 90 องศา """
        self._chassis.drive_speed(x=0, y=0, z=0)
        # ถอยหลังนิดนึงเพื่อไม่ให้หน้ารถขูดกำแพงตอนหมุน
        self._chassis.move(x=-0.10, y=0, z=0, xy_speed=0.5).wait_for_completed()
        self._chassis.move(x=0, y=0, z=-90, z_speed=100).wait_for_completed()

    def turn_left_90(self):
        """ หมุนตัวไปทางซ้าย 90 องศา """
        self._chassis.drive_speed(x=0, y=0, z=0)
        # ถอยหลังนิดนึงเพื่อไม่ให้หน้ารถขูดกำแพงตอนหมุน
        self._chassis.move(x=-0.10, y=0, z=0, xy_speed=0.5).wait_for_completed()
        self._chassis.move(x=0, y=0, z=90, z_speed=100).wait_for_completed()

    def turn_180(self):
        """ หมุนกลับลำ 180 องศา """
        self._chassis.drive_speed(x=0, y=0, z=0)
        # ถอยหลังนิดนึงเผื่อหน้ารถชนกำแพงอยู่ ล้อจะได้ไม่ติด
        self._chassis.move(x=-0.15, y=0, z=0, xy_speed=0.5).wait_for_completed()
        self._chassis.move(x=0, y=0, z=180, z_speed=100).wait_for_completed()

    def stop(self):
        """ หยุดการเคลื่อนที่ทั้งหมด """
        self._chassis.drive_speed(x=0, y=0, z=0)

    # ------------------------------------------------------------------------
    # คำสั่งควบคุมแขนกล (Robotic Arm) และมือจับ (Gripper)
    # ------------------------------------------------------------------------
    def grab_target(self):
        """ คีบวัตถุที่จุดเริ่มต้น และยกขึ้นเตรียมเคลื่อนที่ """
        print("[RobotController] กำลังคีบวัตถุ...")
        if self._gripper and self._arm:
            self._gripper.open(power=50)
            time.sleep(0.8)
            self._arm.moveto(x=160, y=-60).wait_for_completed()
            self._gripper.close(power=80)
            time.sleep(1.0)
            self._arm.moveto(x=100, y=180).wait_for_completed()
            print("[RobotController] คีบวัตถุสำเร็จ!")

    def release_target(self):
        """ วางวัตถุเมื่อถึงจุดเป้าหมาย """
        print("[RobotController] กำลังวางวัตถุที่เป้าหมาย...")
        if self._gripper and self._arm:
            self._arm.moveto(x=160, y=-60).wait_for_completed()
            self._gripper.open(power=50)
            time.sleep(0.8)
            self._arm.moveto(x=100, y=50).wait_for_completed()
            print("[RobotController] วางวัตถุเรียบร้อยแล้ว!")
