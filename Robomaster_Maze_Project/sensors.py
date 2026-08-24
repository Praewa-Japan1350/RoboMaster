# -*- coding: utf-8 -*-
"""
============================================================================
ไฟล์: sensors.py
คำอธิบาย: คลาส SensorHub อ่านค่าเซนเซอร์ 5 ตัวโดยตรงผ่าน RoboMaster SDK Sensor Adaptors
           1. Sharp IR ซ้ายหน้า (Analog get_adc - Adaptor ID 2, Port 1)
           2. Sharp IR ขวาหน้า (Analog get_adc - Adaptor ID 1, Port 2)
           3. ToF ด้านหน้า (get_adc - Adaptor ID 2, Port 2)
           4. Digital IR หลังซ้าย (get_io - Adaptor ID 3, Port 1)
           5. Digital IR หลังขวา (get_io - Adaptor ID 1, Port 1)
           มีระบบ Median Filter ใน Python และแปลงค่าแรงดันไฟฟ้าเป็นระยะทาง (cm)
           คำนวณตามสมการ: Distance = (12.0 / Voltage) - 0.42
ภาษาคอมเม้นต์: ภาษาไทย
============================================================================
"""

import threading
import collections

class SensorHub:
    """
    คลาสจัดการการอ่านค่าและประมวลผลสัญญาณจากเซนเซอร์ของ RoboMaster EP
    แบบ Pure Python โดยตรงผ่าน SDK (ไม่ต้องใช้ Microcontroller)
    """

    def __init__(self, ep_robot=None):
        """
        :param ep_robot: วัตถุ robot.Robot() จาก RoboMaster SDK
        """
        self._robot = ep_robot
        self._sensor_adaptor = ep_robot.sensor_adaptor if ep_robot else None
        self._lock = threading.Lock()

        # เริ่มต้น Subscribe ข้อมูล ToF เซนเซอร์ที่ต่อผ่าน CAN Bus
        if self._robot:
            try:
                self._robot.sensor.sub_distance(freq=50, callback=self._tof_callback)
            except Exception as e:
                print(f"[SensorHub] ไม่สามารถเชื่อมต่อ ToF Sensor ได้: {e}")

        # บัฟเฟอร์สำหรับทำ Median Filter (เก็บค่าล่าสุด 5 ค่าสำหรับสัญญาณ Analog)
        self._buf_sharp_left = collections.deque(maxlen=5)
        self._buf_sharp_right = collections.deque(maxlen=5)

        # ข้อมูลค่าเซนเซอร์ปัจจุบัน
        self._front_tof_cm = 255.0       # ระยะ ToF หน้า (cm)
        self._sharp_left_cm = 35.0       # ระยะ Sharp IR ซ้ายหน้า (cm)
        self._sharp_right_cm = 35.0      # ระยะ Sharp IR ขวาหน้า (cm)
        self._rear_ir_left = 1           # สถานะ IR หลังซ้าย (0 = พ้นกำแพง, 1 = ปกติ)
        self._rear_ir_right = 1          # สถานะ IR หลังขวา (0 = พ้นกำแพง, 1 = ปกติ)

    def set_robot(self, ep_robot):
        """ กำหนดวัตถุ ep_robot ในกรณีที่ไม่ได้ใส่ใน __init__ """
        self._robot = ep_robot
        self._sensor_adaptor = ep_robot.sensor_adaptor if ep_robot else None
        if self._robot:
            try:
                self._robot.sensor.sub_distance(freq=50, callback=self._tof_callback)
            except:
                pass

    def _tof_callback(self, sub_info):
        """ Callback สำหรับรับค่าระยะจาก ToF Sensor ผ่าน CAN Bus """
        if sub_info:
            distance_mm = sub_info[0] # ระยะทางจาก ToF ตัวที่ 1
            raw_tof_cm = (distance_mm / 10.0) if distance_mm is not None else 255.0
            
            with self._lock:
                self._front_tof_cm = raw_tof_cm

    @staticmethod
    def sharp_adc_to_cm(adc_val: float) -> float:
        """
        แปลงค่า ADC (10-bit: 0 - 1023) ของ RoboMaster Sensor Adaptor เป็นระยะทาง cm
        สูตร: Voltage = (ADC / 1023.0) * 3.3
              Distance = (12.0 / Voltage) - 0.42
        """
        if adc_val is None:
            return 35.0

        # แปลงค่า ADC เป็น แรงดันไฟฟ้า (0 - 3.3V)
        voltage = (adc_val / 1023.0) * 3.3

        # ถ้าแรงดันต่ำมาก แสดงว่าไม่มีกำแพงอยู่ในระยะตรวจจับ ให้คืนค่าระยะสูงสุด (35 cm)
        if voltage < 0.35:
            return 35.0

        try:
            # คำนวณระยะทางตามสูตร Distance = (12.0 / Voltage) - 0.42
            distance = (12.0 / voltage) - 0.42
        except (ZeroDivisionError, ValueError):
            distance = 35.0

        # จำกัดขอบเขตระยะให้อยู่ในช่วง 4 - 35 cm
        return max(4.0, min(35.0, distance))

    def _apply_median_filter(self, deque_buffer, new_val: float) -> float:
        """ ประมวลผล Median Filter จากบัฟเฟอร์ข้อมูลเพื่อลด Noise """
        deque_buffer.append(new_val)
        sorted_vals = sorted(list(deque_buffer))
        mid_index = len(sorted_vals) // 2
        return sorted_vals[mid_index]

    def update_sensors(self):
        """
        อ่านค่าจาก Sensor Adaptor 2 ตัวตามพอร์ตที่กำหนด
        - Adaptor 1 (ขวา): Port 1 = IR หลังขวา (IO), Port 2 = Sharp ขวาหน้า (ADC)
        - Adaptor 2 (ซ้าย): Port 1 = IR หลังซ้าย (IO), Port 2 = Sharp ซ้ายหน้า (ADC)
        """
        if not self._sensor_adaptor:
            return

        try:
            # 1. อ่านค่าจาก Adaptor ID 1 (ฝั่งขวา)
            raw_ir_r = self._sensor_adaptor.get_io(id=1, port=1)
            raw_sharp_r = self._sensor_adaptor.get_adc(id=1, port=2)

            # 2. อ่านค่าจาก Adaptor ID 2 (ฝั่งซ้าย)
            raw_sharp_l = self._sensor_adaptor.get_adc(id=2, port=1)

            # 3. อ่านค่าจาก Adaptor ID 3 (IR หลังซ้าย)
            raw_ir_l = self._sensor_adaptor.get_io(id=3, port=1)
            # (ToF ดึงค่าผ่าน CAN Bus อัตโนมัติใน Callback แล้ว)

            # --- ประมวลผลคำนวณระยะทาง ---
            calc_dist_l = self.sharp_adc_to_cm(raw_sharp_l)
            calc_dist_r = self.sharp_adc_to_cm(raw_sharp_r)

            # กรอง Noise ด้วย Median Filter
            filtered_dist_l = self._apply_median_filter(self._buf_sharp_left, calc_dist_l)
            filtered_dist_r = self._apply_median_filter(self._buf_sharp_right, calc_dist_r)

            # แปลงระยะ ToF จาก mm เป็น cm 
            # (อัปเดตผ่าน _tof_callback อัตโนมัติแล้ว จึงข้ามการกำหนดค่าตรงนี้ไป)
            
            # อัปเดตข้อมูลแบบ Thread-safe
            with self._lock:
                self._sharp_left_cm = filtered_dist_l
                self._sharp_right_cm = filtered_dist_r
                # ข้ามอัปเดต self._front_tof_cm เพราะใช้ Callback
                self._rear_ir_left = int(raw_ir_l) if raw_ir_l is not None else 1
                self._rear_ir_right = int(raw_ir_r) if raw_ir_r is not None else 1

        except Exception as e:
            print(f"[SensorHub Error] เกิดข้อผิดพลาดในการอ่านค่าจาก Sensor Adaptor: {e}")

    # ------------------------------------------------------------------------
    # Getter Methods (Thread-Safe)
    # ------------------------------------------------------------------------
    def get_front_tof(self) -> float:
        """ คืนค่าระยะทางเซนเซอร์ ToF ด้านหน้า (หน่วย: cm) """
        with self._lock:
            return self._front_tof_cm

    def get_sharp_left(self) -> float:
        """ คืนค่าระยะทาง Sharp IR ด้านซ้ายหน้า (หน่วย: cm) """
        with self._lock:
            return self._sharp_left_cm

    def get_sharp_right(self) -> float:
        """ คืนค่าระยะทาง Sharp IR ด้านขวาหน้า (หน่วย: cm) """
        with self._lock:
            return self._sharp_right_cm

    def get_rear_ir_left(self) -> int:
        """ คืนค่าสถานะ Digital IR ด้านหลังซ้าย (0 = พ้นกำแพง/เจอสิ่งกีดขวาง, 1 = ปกติ) """
        with self._lock:
            return self._rear_ir_left

    def get_rear_ir_right(self) -> int:
        """ คืนค่าสถานะ Digital IR ด้านหลังขวา (0 = พ้นกำแพง/เจอสิ่งกีดขวาง, 1 = ปกติ) """
        with self._lock:
            return self._rear_ir_right

    def get_all_sensors(self) -> dict:
        """ คืนค่าเซนเซอร์ทั้งหมดในรูปแบบ Dictionary """
        with self._lock:
            return {
                "front_tof": self._front_tof_cm,
                "sharp_left": self._sharp_left_cm,
                "sharp_right": self._sharp_right_cm,
                "rear_ir_left": self._rear_ir_left,
                "rear_ir_right": self._rear_ir_right
            }
