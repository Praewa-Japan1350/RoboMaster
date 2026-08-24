# -*- coding: utf-8 -*-
"""
============================================================================
ไฟล์: main.py
คำอธิบาย: สคริปต์หลัก (Main Entry Point) สำหรับควบคุมหุ่นยนต์แก้ปัญหากลางเขาวงกต
           ใช้ RoboMaster SDK แบบ 100% Pure Python (ไม่ต้องมี Microcontroller)
           1. รับค่าพิกัดเป้าหมาย (target_x, target_y) จากผู้ใช้ทาง Terminal
           2. เริ่มต้นเชื่อมต่อกับ RoboMaster EP SDK
           3. เรียกใช้วัตถุ SensorHub, RobotController, MazeSolver
           4. สั่งคีบวัตถุที่จุดเริ่มต้น
           5. วนลูปประมวลผลเซนเซอร์ และนำทางตามเขาวงกตด้วยกฎมือขวา
           6. เมื่อถึงเป้าหมาย: หยุดหุ่นยนต์ วางวัตถุ และบันทึกแผนที่ 2D Trajectory
ภาษาคอมเม้นต์: ภาษาไทย
============================================================================
"""

import time
import sys
import os

# นำเข้าไลบรารี RoboMaster SDK
import robomaster
from robomaster import robot

# นำเข้าโมดูลภายในโปรเจกต์
from sensors import SensorHub
from chassis_control import RobotController
from maze_logic import MazeSolver
from map_plotter import plot_maze_trajectory

def main():
    print("==========================================================")
    print("   RoboMaster EP Maze Solver - Pure Python SDK System     ")
    print("==========================================================")

    # 1. รับค่าพิกัดเป้าหมาย (target_x, target_y) จากผู้ใช้งานผ่าน Terminal
    try:
        raw_x = input(">> ป้อนค่าพิกัดเป้าหมาย X (เมตร) [เช่น 1.5]: ").strip()
        raw_y = input(">> ป้อนค่าพิกัดเป้าหมาย Y (เมตร) [เช่น 2.0]: ").strip()
        target_x = float(raw_x) if raw_x else 1.5
        target_y = float(raw_y) if raw_y else 2.0
    except ValueError:
        print("[System Error] ค่าพิกัดไม่ถูกต้อง! ใช้ค่าเริ่มต้น: Target (1.5, 2.0)")
        target_x, target_y = 1.5, 2.0

    print(f"\n[System] ตั้งค่าเป้าหมาย: Target X = {target_x} m, Target Y = {target_y} m\n")

    # 2. เริ่มต้นการเชื่อมต่อ RoboMaster EP Robot
    ep_robot = robot.Robot()
    print("[System] กำลังเชื่อมต่อกับหุ่นยนต์ RoboMaster EP (AP Mode)...")
    ep_robot.initialize(conn_type="ap")

    # 3. สร้างวัตถุระบบ (Instantiate Core Components)
    sensor_hub = SensorHub(ep_robot)
    robot_controller = RobotController(ep_robot)
    maze_solver = MazeSolver(target_x, target_y)

    try:
        # ปิดวิดีโอสตรีมตามกฎเกณฑ์ (เพื่อประหยัด Bandwidth ของระบบ)
        ep_robot.camera.stop_video_stream()

        # เริ่มติดตามพิกัด Odometry
        robot_controller.start_position_subscription(freq=10)

        # 4. สั่งแขนกลคีบวัตถุที่จุดเริ่มต้น
        robot_controller.grab_target()

        # 5. ลูปการทำงานหลัก (Main Loop Execution)
        print("\n[System] เริ่มต้นการนำทางในเขาวงกต (Maze Solving Loop Running)...")
        start_time = time.time()
        max_duration_sec = 360 # จำกัดเวลาการทำงานสูงสุด 6 นาที

        while time.time() - start_time < max_duration_sec:
            # --- ประมวลผลตรรกะเขาวงกตและอัปเดตเซนเซอร์ ---
            status = maze_solver.update(sensor_hub, robot_controller)

            # เช็คว่าถึงเป้าหมายหรือยัง
            if status == MazeSolver.STATE_TARGET_REACHED:
                print("\n[System] สเตตการทำงาน: ถึงจุดหมายแล้ว!")
                break

            time.sleep(0.1) # Loop Delay 10Hz (ลดความถี่เพื่อไม่ให้ CAN Bus ของหุ่นยนต์ Overload จนค่าเซนเซอร์ ToF ดีเลย์)

        # 6. เมื่อถึงจุดหมาย: วางวัตถุ
        robot_controller.stop()
        robot_controller.release_target()

    except KeyboardInterrupt:
        print("\n[Safety Warning] การทำงานถูกยกเลิกฉุกเฉินโดยผู้ใช้งาน (Ctrl+C)!")
        robot_controller.stop()

    except Exception as ex:
        print(f"\n[System Error] เกิดข้อผิดพลาดไม่คาดคิด: {ex}")
        robot_controller.stop()

    finally:
        print("\n[System] กำลังคืนทรัพยากรและปิดการเชื่อมต่อ...")
        robot_controller.stop_position_subscription()

        # ดึงข้อมูลการเดินทางและสร้างภาพแผนที่ 2D Trajectory
        trajectory = robot_controller.get_trajectory()
        plot_maze_trajectory(trajectory, target_x, target_y, save_path="maze_result_map.png")

        # ปิดการเชื่อมต่อหุ่นยนต์
        ep_robot.close()
        print("[System] ปิดการเชื่อมต่อระบบเรียบร้อยแล้ว!")

if __name__ == '__main__':
    main()
