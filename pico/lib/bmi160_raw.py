"""
BMI160 driver for Raspberry Pi Pico (MicroPython)
Raw accelerometer + gyroscope data reading
"""

import time

# BMI160 registers
BMI160_CMD = 0x7E
BMI160_DATA = 0x0C  # Gyro X LSB (6 bytes gyro + 6 bytes accel)
BMI160_CMD_ACC_NORMAL = 0x11
BMI160_CMD_GYR_NORMAL = 0x15


class BMI160Raw:
    """BMI160 driver for raw acc/gyr data"""

    def __init__(self, i2c, addr=0x68):
        """
        Initialize the BMI160 sensor

        Args:
            i2c: MicroPython I2C instance
            addr: I2C address (0x68 by default, 0x69 if SDO=HIGH)
        """
        self.i2c = i2c
        self.addr = addr
        self._error = False

        # Default sensitivities (range +/-2g, +/-250 deg/s)
        self.accel_scale = 16384.0  # LSB/g
        self.gyro_scale = 131.0     # LSB/(°/s)

        # Calibration offsets
        self.ax_off = 0
        self.ay_off = 0
        self.az_off = 0
        self.gx_off = 0
        self.gy_off = 0
        self.gz_off = 0

        self._init_sensor()

    def _init_sensor(self):
        """Initialize BMI160 in normal mode (acc + gyro)"""
        try:
            # Enable accelerometer
            self.i2c.writeto_mem(self.addr, BMI160_CMD, bytes([BMI160_CMD_ACC_NORMAL]))
            time.sleep(0.05)
            # Enable gyroscope
            self.i2c.writeto_mem(self.addr, BMI160_CMD, bytes([BMI160_CMD_GYR_NORMAL]))
            time.sleep(0.1)
            self._error = False
            print("BMI160: OK (acc+gyro)")
        except Exception as e:
            print(f"BMI160: Erreur init - {e}")
            self._error = True

    def calibrate(self, samples=50):
        """
        At-rest calibration (sensor horizontal and stationary)

        Args:
            samples: Number of samples to average
        """
        if self._error:
            return

        print("BMI160: Calibration...")
        ax_sum = ay_sum = az_sum = 0
        gx_sum = gy_sum = gz_sum = 0

        for _ in range(samples):
            data = self.read_raw()
            ax_sum += data['ax']
            ay_sum += data['ay']
            az_sum += data['az']
            gx_sum += data['gx']
            gy_sum += data['gy']
            gz_sum += data['gz']
            time.sleep(0.01)

        self.ax_off = ax_sum // samples
        self.ay_off = ay_sum // samples
        self.az_off = az_sum // samples - int(self.accel_scale)  # Z = 1g at rest
        self.gx_off = gx_sum // samples
        self.gy_off = gy_sum // samples
        self.gz_off = gz_sum // samples

        print(f"  Acc offsets: {self.ax_off}, {self.ay_off}, {self.az_off}")
        print(f"  Gyr offsets: {self.gx_off}, {self.gy_off}, {self.gz_off}")

    def read_raw(self):
        """
        Read raw data (LSB)

        Returns:
            dict: {ax, ay, az, gx, gy, gz} in LSB
        """
        try:
            # Read 12 bytes: GYR_X(2) + GYR_Y(2) + GYR_Z(2) + ACC_X(2) + ACC_Y(2) + ACC_Z(2)
            data = self.i2c.readfrom_mem(self.addr, BMI160_DATA, 12)

            def to_signed(lsb, msb):
                val = lsb | (msb << 8)
                return val - 65536 if val >= 32768 else val

            gx = to_signed(data[0], data[1])
            gy = to_signed(data[2], data[3])
            gz = to_signed(data[4], data[5])
            ax = to_signed(data[6], data[7])
            ay = to_signed(data[8], data[9])
            az = to_signed(data[10], data[11])

            self._error = False
            return {'ax': ax, 'ay': ay, 'az': az, 'gx': gx, 'gy': gy, 'gz': gz}

        except Exception as e:
            self._error = True
            return {'ax': 0, 'ay': 0, 'az': 0, 'gx': 0, 'gy': 0, 'gz': 0}

    def read_calibrated(self):
        """
        Read data with offsets applied, in physical units

        Returns:
            dict: {acc_x, acc_y, acc_z} in g, {gyr_x, gyr_y, gyr_z} in deg/s, valid
        """
        raw = self.read_raw()

        # Accelerometer in g
        acc_x = (raw['ax'] - self.ax_off) / self.accel_scale
        acc_y = (raw['ay'] - self.ay_off) / self.accel_scale
        acc_z = (raw['az'] - self.az_off) / self.accel_scale

        # Gyroscope in deg/s
        gyr_x = (raw['gx'] - self.gx_off) / self.gyro_scale
        gyr_y = (raw['gy'] - self.gy_off) / self.gyro_scale
        gyr_z = (raw['gz'] - self.gz_off) / self.gyro_scale

        return {
            'acc_x': acc_x, 'acc_y': acc_y, 'acc_z': acc_z,
            'gyr_x': gyr_x, 'gyr_y': gyr_y, 'gyr_z': gyr_z,
            'valid': not self._error
        }

    def has_error(self):
        """Return True if communication error"""
        return self._error


# Module test
if __name__ == "__main__":
    from machine import I2C, Pin

    print("=== Test BMI160 Raw ===")

    i2c = I2C(1, scl=Pin(15), sda=Pin(14), freq=400000)
    devices = i2c.scan()
    print(f"I2C devices: {[hex(d) for d in devices]}")

    if 0x68 in devices:
        imu = BMI160Raw(i2c, addr=0x68)
        imu.calibrate()

        print("\nLecture (Ctrl+C pour arreter)...")
        while True:
            data = imu.read_calibrated()
            print(f"Acc: {data['acc_x']:+.2f} {data['acc_y']:+.2f} {data['acc_z']:+.2f} g | "
                  f"Gyr: {data['gyr_x']:+.1f} {data['gyr_y']:+.1f} {data['gyr_z']:+.1f} dps")
            time.sleep(0.1)
    else:
        print("BMI160 non trouve")
