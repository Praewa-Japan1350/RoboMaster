# -*- coding: utf-8 -*-
"""
============================================================================
ไฟล์: map_plotter.py
คำอธิบาย: โมดูลสำหรับการวาดและบันทึกกราฟ 2D Trajectory การเคลื่อนที่ของหุ่นยนต์
           - แสดงจุดเริ่มต้น (0,0) และจุดหมายปลายทาง (target_x, target_y)
           - อัตราส่วนกราฟเป็น 1:1 (Aspect Ratio Equal)
           - ใช้ไลบรารี Matplotlib
ภาษาคอมเม้นต์: ภาษาไทย
============================================================================
"""

import matplotlib.pyplot as plt

def plot_maze_trajectory(trajectory_coords: list, target_x: float, target_y: float, save_path: str = "maze_result_map.png"):
    """
    สร้างและบันทึกภาพกราฟแสดงเส้นทางการเคลื่อนที่ของหุ่นยนต์ในเขาวงกต
    :param trajectory_coords: ลิสต์ของคู่พิกัด [(x0, y0), (x1, y1), ...]
    :param target_x: พิกัด X ของเป้าหมาย (เมตร)
    :param target_y: พิกัด Y ของเป้าหมาย (เมตร)
    :param save_path: ชื่อไฟล์ภาพที่จะบันทึก
    """
    if not trajectory_coords:
        print("[MapPlotter Warning] ไม่มีข้อมูลพิกัดสำหรับวาดกราฟ!")
        return

    # แยกพิกัด X และ Y ออกเป็นสองอาเรย์
    path_x = [pt[0] for pt in trajectory_coords]
    path_y = [pt[1] for pt in trajectory_coords]

    # ตั้งค่าขนาดรูปกราฟ
    plt.figure(figsize=(7, 8))

    # 1. วาดเส้นทางการเคลื่อนที่ (Trajectory Line)
    plt.plot(path_x, path_y, marker='.', color='#1f77b4', linestyle='-', linewidth=1.5, label='Robot Path')

    # 2. ทำเครื่องหมายจุดเริ่มต้น (0, 0) สีเขียว
    plt.plot(path_x[0], path_y[0], 'go', markersize=10, label='Start (0,0)')

    # 3. ทำเครื่องหมายจุดสิ้นสุด/เป้าหมาย (target_x, target_y) ดาวสีแดง
    plt.plot(target_x, target_y, 'r*', markersize=14, label=f'Target ({target_x:.2f}, {target_y:.2f})')

    # 4. วาดตำแหน่งสุดท้ายจริงที่หุ่นยนต์ไปถึง
    plt.plot(path_x[-1], path_y[-1], 'mo', markersize=8, label=f'Actual Stop ({path_x[-1]:.2f}, {path_y[-1]:.2f})')

    # การตกแต่งกราฟ
    plt.title("RoboMaster EP Maze Navigation Trajectory", fontsize=14, fontweight='bold')
    plt.xlabel("Position X (meters)", fontsize=12)
    plt.ylabel("Position Y (meters)", fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc='upper right')

    # กำหนดอัตราส่วนแกน X และ Y ให้เท่ากัน (1:1 Aspect Ratio)
    plt.axis('equal')

    # บันทึกรูปภาพ
    try:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"[MapPlotter] บันทึกภาพแผนที่เส้นทางสำเร็จเรียบร้อยที่ไฟล์: '{save_path}'")
    except Exception as e:
        print(f"[MapPlotter Error] เกิดข้อผิดพลาดในการบันทึกรูปภาพ: {e}")

    # ปิด Figure เพื่อคืนหน่วยความจำ
    plt.close()
