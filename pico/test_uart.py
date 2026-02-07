"""
Test UART Pico → Pi5
Envoie des données simulées pour vérifier la liaison
"""

import time
import math
from machine import Pin, UART
from lib.protocole import (
    build_frame,
    convert_accel_g_to_mg,
    convert_gyro_dps_to_raw,
    convert_coord_to_microdeg,
    convert_alt_m_to_dm,
    convert_speed_ms_to_cms,
    convert_heading_to_raw
)

# UART vers Pi5
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
led = Pin(25, Pin.OUT)

print("=" * 40)
print("  Test UART Pico -> Pi5")
print("  Donnees simulees")
print("=" * 40)

seq = 0

while True:
    # Simuler IMU (légère variation)
    acc_x = 0.02 * math.sin(seq * 0.1)
    acc_y = 0.01 * math.cos(seq * 0.1)
    acc_z = 1.0 + 0.005 * math.sin(seq * 0.05)

    gyr_x = 2.0 * math.sin(seq * 0.2)
    gyr_y = 1.5 * math.cos(seq * 0.15)
    gyr_z = 0.5 * math.sin(seq * 0.1)

    # Simuler GPS (mouvement circulaire autour de Paris)
    angle = seq * 0.01
    lat = 48.8566 + 0.0001 * math.sin(angle)
    lon = 2.3522 + 0.0001 * math.cos(angle)
    alt = 35.0
    speed = 1.5
    course = (math.degrees(angle) + 90) % 360

    # Construire et envoyer la trame
    frame = build_frame(
        seq=seq % 65536,
        acc_x=convert_accel_g_to_mg(acc_x),
        acc_y=convert_accel_g_to_mg(acc_y),
        acc_z=convert_accel_g_to_mg(acc_z),
        gyr_x=convert_gyro_dps_to_raw(gyr_x),
        gyr_y=convert_gyro_dps_to_raw(gyr_y),
        gyr_z=convert_gyro_dps_to_raw(gyr_z),
        latitude=convert_coord_to_microdeg(lat),
        longitude=convert_coord_to_microdeg(lon),
        altitude=convert_alt_m_to_dm(alt),
        vitesse=convert_speed_ms_to_cms(speed),
        cap=convert_heading_to_raw(course)
    )

    uart.write(frame)

    # Feedback
    led.toggle()
    print(f"[{seq:5d}] TX 32 bytes | Acc:{acc_z:.2f}g | GPS:{lat:.4f},{lon:.4f}")

    seq += 1
    time.sleep(1)
