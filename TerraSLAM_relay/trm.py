#!/usr/bin/env python3
from pymavlink import mavutil
import time

PORT = "/dev/ttyTHS1"
BAUD = 115200

master = mavutil.mavlink_connection(
    PORT,
    baud=BAUD,
    source_system=10,
    source_component=191
)

master.wait_heartbeat()
print(f"Полётник найден: sysid={master.target_system}")

def send_heartbeat():
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )

def send_named_float(name: str, value: float):
    """Отправка NAMED_VALUE_FLOAT обратно в GCS"""
    # name максимум 10 символов
    name_bytes = name.encode('utf-8')[:10].ljust(10, b'\0')
    master.mav.named_value_float_send(
        int(time.time() * 1000) % 4294967295,  # time_boot_ms
        name_bytes,
        float(value)
    )
    print(f"→ Ответ отправлен: [{name}] = {value}")

send_heartbeat()
print("Отправил heartbeat от companion\n")

last_hb = time.time()

while True:
    # Heartbeat раз в секунду
    if time.time() - last_hb > 1.0:
        send_heartbeat()
        last_hb = time.time()

    msg = master.recv_match(type='NAMED_VALUE_FLOAT', blocking=False, timeout=0.05)
    if not msg:
        continue

    name = msg.name.decode('utf-8').strip('\x00') if isinstance(msg.name, bytes) else str(msg.name).strip('\x00')
    value = msg.value
    print(f"← Получено: [{name:15}] = {value}")

    # === Обработка команд ===
    if name == "ORB_CMD":
        cmd = int(value)

        if cmd == 1:          # Start ORB-SLAM
            print("  → Команда: START ORB-SLAM")
            # Здесь твой код запуска
            send_named_float("ORB_RES", 1.0)   # 1 = running

        elif cmd == 0:        # Stop ORB-SLAM
            print("  → Команда: STOP ORB-SLAM")
            # Здесь твой код остановки
            send_named_float("ORB_RES", 2.0)   # 0 = stopped

        elif cmd == 2:        # Reset map
            print("  → Команда: RESET MAP")
            send_named_float("ORB_RES", 3.0)

        else:
            print(f"  → Неизвестная команда: {cmd}")
            send_named_float("ORB_STATUS", -1.0)  # ошибка

    # Можно добавить другие команды по имени
    # elif name == "CAM_CMD":
    #     ...
