"""
Application principale Raspberry Pi Pico
Robot Autonome - Lecture capteurs et transmission UART vers Pi5

Hardware:
- GPS NEO-6M sur UART1 (TX=GP4, RX=GP5) @ 9600 baud
- Communication Pi5 sur UART0 (TX=GP0, RX=GP1) @ 115200 baud
- BMI160 IMU sur I2C1 (SDA=GP14, SCL=GP15) @ 400kHz

Protocole: STX/ETX + CRC8 (compatible TestInterfaceUART)

Câblage Pico → Pi5 (UART3):
- GP0 (TX) → GPIO9 (RXD3) du Pi5
- GP1 (RX) ← GPIO8 (TXD3) du Pi5
- GND → GND
"""

import time
from machine import Pin, UART, I2C

# Import des modules locaux
from lib.protocole import (
    build_frame,
    convert_accel_g_to_mg,
    convert_gyro_dps_to_raw,
    convert_coord_to_microdeg,
    convert_alt_m_to_dm,
    convert_speed_ms_to_cms,
    convert_heading_to_raw
)
from lib.bmi160_raw import BMI160Raw
from lib.gps_reader import GPSReader
from lib.micropyGPS import MicropyGPS


# === Configuration Hardware ===

# Communication Pi5 (UART0)
PI5_UART_ID = 0
PI5_TX_PIN = 0   # GP0 → Pi5 GPIO9 (RX)
PI5_RX_PIN = 1   # GP1 ← Pi5 GPIO8 (TX)
PI5_BAUDRATE = 115200

# GPS (UART1)
GPS_UART_ID = 1
GPS_TX_PIN = 4
GPS_RX_PIN = 5
GPS_BAUDRATE = 9600

# IMU (I2C1)
IMU_I2C_ID = 1
IMU_SDA_PIN = 14
IMU_SCL_PIN = 15
IMU_FREQ = 400000
IMU_ADDR = 0x68

# Fréquence de mise à jour
UPDATE_RATE = 10  # Hz
UPDATE_INTERVAL_MS = 1000 // UPDATE_RATE


def init_hardware():
    """
    Initialise tout le hardware

    Returns:
        tuple: (uart_pi5, imu, gps, led)
    """
    print("\n=== Initialisation Hardware ===")

    # LED intégrée
    led = Pin(25, Pin.OUT)
    led.on()

    # UART vers Pi5
    print(f"UART Pi5: UART{PI5_UART_ID} @ {PI5_BAUDRATE} (GP{PI5_TX_PIN}/GP{PI5_RX_PIN})")
    uart_pi5 = UART(PI5_UART_ID, baudrate=PI5_BAUDRATE,
                    tx=Pin(PI5_TX_PIN), rx=Pin(PI5_RX_PIN))

    # GPS
    print(f"GPS: UART{GPS_UART_ID} @ {GPS_BAUDRATE} (GP{GPS_TX_PIN}/GP{GPS_RX_PIN})")
    uart_gps = UART(GPS_UART_ID, baudrate=GPS_BAUDRATE,
                    tx=Pin(GPS_TX_PIN), rx=Pin(GPS_RX_PIN))
    gps_parser = MicropyGPS(local_offset=0, location_formatting='dd')
    gps = GPSReader(uart_gps, gps_parser)

    # IMU
    print(f"IMU: I2C{IMU_I2C_ID} @ {IMU_FREQ}Hz, addr 0x{IMU_ADDR:02X}")
    i2c = I2C(IMU_I2C_ID, scl=Pin(IMU_SCL_PIN), sda=Pin(IMU_SDA_PIN), freq=IMU_FREQ)
    devices = i2c.scan()
    print(f"  I2C devices: {[hex(d) for d in devices]}")

    imu = BMI160Raw(i2c, addr=IMU_ADDR)
    if not imu.has_error():
        imu.calibrate()

    led.off()
    print("=== Hardware OK ===\n")

    return uart_pi5, imu, gps, led


def main():
    """Boucle principale"""

    print("\n" + "=" * 50)
    print("  ROBOT AUTONOME - Pico Sensor Hub")
    print("  Protocole: STX/ETX + CRC8")
    print("=" * 50)

    # Initialisation
    uart_pi5, imu, gps, led = init_hardware()

    # Compteurs
    seq = 0
    packets_sent = 0
    last_print = time.ticks_ms()

    print(f"Boucle @ {UPDATE_RATE} Hz - Ctrl+C pour arreter\n")

    try:
        while True:
            loop_start = time.ticks_ms()

            # Lire capteurs
            imu_data = imu.read_calibrated()
            gps_data = gps.read()

            # Construire la trame
            frame = build_frame(
                seq=seq % 65536,
                acc_x=convert_accel_g_to_mg(imu_data['acc_x']),
                acc_y=convert_accel_g_to_mg(imu_data['acc_y']),
                acc_z=convert_accel_g_to_mg(imu_data['acc_z']),
                gyr_x=convert_gyro_dps_to_raw(imu_data['gyr_x']),
                gyr_y=convert_gyro_dps_to_raw(imu_data['gyr_y']),
                gyr_z=convert_gyro_dps_to_raw(imu_data['gyr_z']),
                latitude=convert_coord_to_microdeg(gps_data['lat']),
                longitude=convert_coord_to_microdeg(gps_data['lon']),
                altitude=convert_alt_m_to_dm(gps_data['alt']),
                vitesse=convert_speed_ms_to_cms(gps_data['speed']),
                cap=convert_heading_to_raw(gps_data['course']),
                satellites=gps_data['satellites'],
                fix_quality=1 if gps_data['valid'] else 0
            )

            # Envoyer
            uart_pi5.write(frame)
            packets_sent += 1
            seq += 1

            # LED: clignote si GPS fix
            if gps_data['valid']:
                led.toggle()
            else:
                led.off()

            # Affichage debug (1x/sec)
            now = time.ticks_ms()
            if time.ticks_diff(now, last_print) >= 1000:
                gps_status = "FIX" if gps_data['valid'] else "---"
                imu_status = "OK" if imu_data['valid'] else "ERR"

                print(f"[{seq:5d}] GPS:{gps_status}({gps_data['satellites']}sat) IMU:{imu_status} TX:{packets_sent}")
                print(f"  Acc: {imu_data['acc_x']:+.2f} {imu_data['acc_y']:+.2f} {imu_data['acc_z']:+.2f} g")
                print(f"  Gyr: {imu_data['gyr_x']:+.1f} {imu_data['gyr_y']:+.1f} {imu_data['gyr_z']:+.1f} dps")
                if gps_data['valid']:
                    print(f"  GPS: {gps_data['lat']:.6f}, {gps_data['lon']:.6f}")
                print()
                last_print = now

            # Timing
            elapsed = time.ticks_diff(time.ticks_ms(), loop_start)
            sleep_time = UPDATE_INTERVAL_MS - elapsed
            if sleep_time > 0:
                time.sleep_ms(sleep_time)

    except KeyboardInterrupt:
        print("\nArret demande")
        led.off()

    print(f"\nTotal TX: {packets_sent}")
    print("Programme termine")


if __name__ == "__main__":
    main()
