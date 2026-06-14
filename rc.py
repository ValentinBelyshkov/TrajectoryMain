#!/usr/bin/env python3
"""
Проверка аппаратных каналов RealSense камеры.
Установка: pip install pyrealsense2  (или sudo apt install python3-librealsense2)
"""
import pyrealsense2 as rs

STREAM_NAMES = {
    rs.stream.depth: "🌊 Depth (глубина)",
    rs.stream.color: "🎨 Color (RGB)",
    rs.stream.infrared: "👁️ Infrared (ИК)",
    rs.stream.gyro: "🔄 Gyroscope (гироскоп)",
    rs.stream.accel: "📉 Accelerometer (акселерометр)",
    rs.stream.pose: "📍 Pose (визуальный одометр)"
}

def check_realsense_hardware():
    ctx = rs.context()
    devices = ctx.query_devices()

    if len(devices) == 0:
        print("❌ Камеры RealSense не найдены. Проверьте USB-подключение.")
        return

    for dev in devices:
        print(f"\n📷 Модель: {dev.get_info(rs.camera_info.name)}")
        print(f"   Серийник: {dev.get_info(rs.camera_info.serial_number)}")
        print(f"   FW:       {dev.get_info(rs.camera_info.firmware_version)}")
        
        sensors = dev.query_sensors()
        found_streams = set()
        
        for sensor in sensors:
            profiles = sensor.get_stream_profiles()
            for p in profiles:
                found_streams.add(p.stream_type())
                
        print("\n🔌 Доступные аппаратные каналы:")
        has_imu = False
        for st in sorted(found_streams, key=lambda x: x.value):
            name = STREAM_NAMES.get(st, f"📡 {st.name}")
            print(f"   {name}")
            if st in (rs.stream.gyro, rs.stream.accel):
                has_imu = True
                
        print("\n" + "="*40)
        if has_imu:
            print("✅ IMU (гироскоп + акселерометр): ПРИСУТСТВУЕТ")
        else:
            print("❌ IMU (гироскоп + акселерометр): ОТСУТСТВУЕТ")
        print("="*40)

if __name__ == "__main__":
    check_realsense_hardware()
