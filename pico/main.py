"""
Raspberry Pi Pico main application
Autonomous Robot - Sensor reading and UART transmission to Pi5

Hardware:
- GPS NEO-6M on UART1 (TX=GP4, RX=GP5) @ 9600 baud
- Pi5 communication on UART0 (TX=GP0, RX=GP1) @ 115200 baud
- IMU on I2C1 (SDA=GP14, SCL=GP15) @ 400kHz
  BNO085 at 0x4A (preferred — absolute heading via mag fusion)
  BMI160 at 0x68 (fallback — accel + gyro, GPS COG for heading)

Protocol: STX/ETX + CRC8 (compatible with TestInterfaceUART)

Wiring Pico → Pi5 (UART3):
- GP0 (TX) → GPIO9 (RXD3) on Pi5
- GP1 (RX) ← GPIO8 (TXD3) on Pi5
- GND → GND
"""

import time
from machine import Pin, UART, I2C

# Import local modules
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


# === Hardware Configuration ===

# Pi5 communication (UART0)
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
IMU_ADDR_BNO085 = 0x4A  # BNO085 default (PS0=low, PS1=low)
IMU_ADDR_BMI160 = 0x68  # BMI160 default (SDO=low)

# Update frequency
UPDATE_RATE = 10  # Hz
UPDATE_INTERVAL_MS = 1000 // UPDATE_RATE


def init_hardware():
    """
    Initialize all hardware. Auto-detects BNO085 (0x4A) first,
    falls back to BMI160 (0x68) if not found.

    Returns:
        tuple: (uart_pi5, imu, imu_type, gps, led)
               imu_type is 'BNO085' or 'BMI160'
    """
    print("\n=== Initialisation Hardware ===")

    # Built-in LED
    led = Pin(25, Pin.OUT)
    led.on()

    # UART to Pi5
    print(f"UART Pi5: UART{PI5_UART_ID} @ {PI5_BAUDRATE} (GP{PI5_TX_PIN}/GP{PI5_RX_PIN})")
    uart_pi5 = UART(PI5_UART_ID, baudrate=PI5_BAUDRATE,
                    tx=Pin(PI5_TX_PIN), rx=Pin(PI5_RX_PIN))

    # GPS
    print(f"GPS: UART{GPS_UART_ID} @ {GPS_BAUDRATE} (GP{GPS_TX_PIN}/GP{GPS_RX_PIN})")
    uart_gps = UART(GPS_UART_ID, baudrate=GPS_BAUDRATE,
                    tx=Pin(GPS_TX_PIN), rx=Pin(GPS_RX_PIN))
    gps_parser = MicropyGPS(local_offset=0, location_formatting='dd')
    gps = GPSReader(uart_gps, gps_parser)

    # IMU — try BNO085 first, fall back to BMI160
    print(f"IMU: I2C{IMU_I2C_ID} @ {IMU_FREQ}Hz")
    i2c = I2C(IMU_I2C_ID, scl=Pin(IMU_SCL_PIN), sda=Pin(IMU_SDA_PIN), freq=IMU_FREQ)
    devices = i2c.scan()
    print(f"  I2C devices: {[hex(d) for d in devices]}")

    if IMU_ADDR_BNO085 in devices:
        from lib.bno085 import BNO085
        imu = BNO085(i2c, addr=IMU_ADDR_BNO085, rate_hz=50)
        imu_type = 'BNO085'
        print(f"  IMU: BNO085 @ 0x{IMU_ADDR_BNO085:02X}")
    else:
        imu = BMI160Raw(i2c, addr=IMU_ADDR_BMI160)
        if not imu.has_error():
            imu.calibrate()
        imu_type = 'BMI160'
        print(f"  IMU: BMI160 @ 0x{IMU_ADDR_BMI160:02X}")

    led.off()
    print("=== Hardware OK ===\n")

    return uart_pi5, imu, imu_type, gps, led


def main():
    """Main loop"""

    print("\n" + "=" * 50)
    print("  ROBOT AUTONOME - Pico Sensor Hub")
    print("  Protocole: STX/ETX + CRC8")
    print("=" * 50)

    # Initialization
    uart_pi5, imu, imu_type, gps, led = init_hardware()

    # Counters
    seq = 0
    packets_sent = 0
    last_print = time.ticks_ms()

    print(f"Boucle @ {UPDATE_RATE} Hz [{imu_type}] - Ctrl+C pour arreter\n")

    try:
        while True:
            loop_start = time.ticks_ms()

            # Read GPS
            gps_data = gps.read()

            # Read IMU and prepare frame values
            if imu_type == 'BNO085':
                # Drain buffered fusion reports to get the latest yaw value.
                # BNO085 runs at 50 Hz; the main loop runs at 10 Hz, so up to
                # 5 reports may be waiting.
                for _ in range(10):
                    if not imu.update():
                        break
                acc_x_mg = acc_y_mg = acc_z_mg = 0
                gyr_x_raw = gyr_y_raw = gyr_z_raw = 0
                # Use BNO085 absolute yaw; fall back to GPS COG until valid
                heading   = imu.yaw if imu.valid else gps_data['course']
                imu_valid = imu.valid
            else:
                imu_data  = imu.read_calibrated()
                acc_x_mg  = convert_accel_g_to_mg(imu_data['acc_x'])
                acc_y_mg  = convert_accel_g_to_mg(imu_data['acc_y'])
                acc_z_mg  = convert_accel_g_to_mg(imu_data['acc_z'])
                gyr_x_raw = convert_gyro_dps_to_raw(imu_data['gyr_x'])
                gyr_y_raw = convert_gyro_dps_to_raw(imu_data['gyr_y'])
                gyr_z_raw = convert_gyro_dps_to_raw(imu_data['gyr_z'])
                heading   = gps_data['course']
                imu_valid = imu_data['valid']

            # Build the frame
            frame = build_frame(
                seq=seq % 65536,
                acc_x=acc_x_mg,
                acc_y=acc_y_mg,
                acc_z=acc_z_mg,
                gyr_x=gyr_x_raw,
                gyr_y=gyr_y_raw,
                gyr_z=gyr_z_raw,
                latitude=convert_coord_to_microdeg(gps_data['lat']),
                longitude=convert_coord_to_microdeg(gps_data['lon']),
                altitude=convert_alt_m_to_dm(gps_data['alt']),
                vitesse=convert_speed_ms_to_cms(gps_data['speed']),
                cap=convert_heading_to_raw(heading),
                satellites=gps_data['satellites'],
                fix_quality=1 if gps_data['valid'] else 0
            )

            # Send
            uart_pi5.write(frame)
            packets_sent += 1
            seq += 1

            # LED: blink if GPS fix
            if gps_data['valid']:
                led.toggle()
            else:
                led.off()

            # Debug display (1x/sec)
            now = time.ticks_ms()
            if time.ticks_diff(now, last_print) >= 1000:
                gps_status = "FIX" if gps_data['valid'] else "---"
                imu_status = "OK" if imu_valid else "ERR"

                print(f"[{seq:5d}] GPS:{gps_status}({gps_data['satellites']}sat) {imu_type}:{imu_status} TX:{packets_sent}")
                if imu_type == 'BNO085':
                    if imu.valid:
                        cal_str = "CAL" if imu.is_calibrated() else f"acc={imu.heading_accuracy_deg:.0f}deg"
                        print(f"  Yaw:{imu.yaw:6.1f}deg  Pitch:{imu.pitch:+5.1f}  Roll:{imu.roll:+5.1f}  [{cal_str}]")
                else:
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
