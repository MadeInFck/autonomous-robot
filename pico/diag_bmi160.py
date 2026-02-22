"""
BMI160 Diagnostic - Raspberry Pi Pico
Run directly in Thonny (not via main.py)

Hardware: I2C1, SDA=GP14, SCL=GP15 (robot wiring)
"""

from machine import I2C, Pin
import struct
import time

# --- Configuration (matches robot hardware) ---
BMI160_ADDR = 0x68
I2C_ID      = 1
SDA_PIN     = 14
SCL_PIN     = 15

SEP = "=" * 45

# ── Try I2C at 100 kHz first, fallback to 400 kHz ──
print(SEP)
print("SCAN I2C (100 kHz)")
print(SEP)
i2c = I2C(I2C_ID, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=100000)
devices = i2c.scan()
print(f"Devices at 100kHz: {[hex(d) for d in devices]}")

if BMI160_ADDR not in devices:
    print("Not found at 100kHz, trying 400kHz...")
    i2c = I2C(I2C_ID, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=400000)
    devices = i2c.scan()
    print(f"Devices at 400kHz: {[hex(d) for d in devices]}")

if BMI160_ADDR not in devices:
    print(f"BMI160 not found at 0x{BMI160_ADDR:02x} — check wiring")
    raise SystemExit
print(f"BMI160 found at 0x{BMI160_ADDR:02x}")

# ── Chip ID ─────────────────────────────────────
print(SEP)
print("CHIP ID (register 0x00)")
print(SEP)
try:
    chip_id = i2c.readfrom_mem(BMI160_ADDR, 0x00, 1)[0]
    print(f"Chip ID: 0x{chip_id:02x}  (BMI160=0xD1, MPU6050=0x68, ICM42688=0x47)")
    if chip_id == 0xD1:
        print("Confirmed: genuine BMI160")
    elif chip_id == 0x68:
        print("WARNING: looks like MPU-6050, not BMI160 — wrong driver!")
    else:
        print(f"Unknown chip ID 0x{chip_id:02x} — may be a clone")
except OSError as e:
    print(f"EIO reading chip ID: {e}")
    print("Trying raw read (no register)...")
    try:
        raw = i2c.readfrom(BMI160_ADDR, 1)
        print(f"Raw byte: 0x{raw[0]:02x}")
    except OSError as e2:
        print(f"Raw read also failed: {e2}")
    print()
    print("Possible causes:")
    print("  1. Module needs CS pin pulled HIGH for I2C mode")
    print("  2. Module is SPI-only on this board")
    print("  3. Pull-up resistors missing/too weak")
    raise SystemExit


def pmu_status():
    pmu = i2c.readfrom_mem(BMI160_ADDR, 0x03, 1)[0]
    acc = (pmu >> 4) & 0x03
    gyr = (pmu >> 2) & 0x03
    acc_s = "NORMAL" if acc == 1 else ("LOW" if acc == 2 else "SUSPEND")
    gyr_s = "NORMAL" if gyr == 1 else ("FAST" if gyr == 3 else "SUSPEND")
    print(f"  PMU 0x{pmu:02x} | Acc={acc_s}({acc}) Gyr={gyr_s}({gyr})")
    return acc, gyr


def soft_reset(delay_s=0.15):
    i2c.writeto_mem(BMI160_ADDR, 0x7E, bytes([0xB6]))
    time.sleep(delay_s)


# ── PMU before init ──────────────────────────────
print(SEP)
print("PMU STATUS BEFORE INIT")
print(SEP)
pmu_status()

# ── Init sequence ────────────────────────────────
print(SEP)
print("INIT SEQUENCE")
print(SEP)

soft_reset(0.15)
print("After soft reset:")
pmu_status()

i2c.writeto_mem(BMI160_ADDR, 0x7E, bytes([0x11]))  # ACC normal
time.sleep(0.1)
print("After ACC normal (0x11):")
acc_mode, _ = pmu_status()

# Gyro: up to 3 attempts, last one tries fast_start first
gyr_mode = 0
for attempt in range(3):
    if attempt < 2:
        i2c.writeto_mem(BMI160_ADDR, 0x7E, bytes([0x15]))  # GYR normal
    else:
        print("  Last attempt: fast_start (0x17) -> normal (0x15)")
        i2c.writeto_mem(BMI160_ADDR, 0x7E, bytes([0x17]))
        time.sleep(0.05)
        i2c.writeto_mem(BMI160_ADDR, 0x7E, bytes([0x15]))
    for _ in range(20):
        time.sleep(0.01)
        pmu = i2c.readfrom_mem(BMI160_ADDR, 0x03, 1)[0]
        if ((pmu >> 2) & 0x03) == 1:
            break
    print(f"After GYR cmd attempt {attempt+1}:")
    _, gyr_mode = pmu_status()
    if gyr_mode == 1:
        break
    time.sleep(0.05)

# ── Gyro self-test ───────────────────────────────
print(SEP)
print("GYRO SELF-TEST")
print(SEP)
soft_reset(0.15)
i2c.writeto_mem(BMI160_ADDR, 0x7E, bytes([0x11]))
time.sleep(0.1)
i2c.writeto_mem(BMI160_ADDR, 0x7E, bytes([0x15]))
time.sleep(0.2)
i2c.writeto_mem(BMI160_ADDR, 0x6D, bytes([0x10]))  # trigger gyro self-test
time.sleep(0.5)
status = i2c.readfrom_mem(BMI160_ADDR, 0x1B, 1)[0]
gyr_test_ok = bool((status >> 1) & 0x01)
print(f"STATUS 0x1B: 0x{status:02x}  Gyro self-test: {'PASS' if gyr_test_ok else 'FAIL'}")

# ── 10 raw data samples ──────────────────────────
print(SEP)
print("RAW DATA (10 samples)")
print(SEP)
soft_reset(0.15)
i2c.writeto_mem(BMI160_ADDR, 0x7E, bytes([0x11]))
time.sleep(0.1)
i2c.writeto_mem(BMI160_ADDR, 0x7E, bytes([0x15]))
time.sleep(0.2)
print("Final PMU:")
acc_mode, gyr_mode = pmu_status()
print()

gx_vals = []
for i in range(10):
    raw = i2c.readfrom_mem(BMI160_ADDR, 0x0C, 12)
    gx, gy, gz, ax, ay, az = struct.unpack('<hhhhhh', raw)
    gx_vals.append(gx)
    print(f"#{i+1:2d} | Gyr {gx:7d} {gy:7d} {gz:7d} | Acc {ax:7d} {ay:7d} {az:7d}")
    time.sleep(0.1)

# ── Verdict ──────────────────────────────────────
print(SEP)
print("VERDICT")
print(SEP)
gyro_all_zero = all(v == 0 for v in gx_vals)
if gyr_mode == 1 and not gyro_all_zero:
    print("Gyro: OK — normal mode, non-zero data")
elif gyr_mode == 1 and gyro_all_zero:
    print("Gyro: PARTIAL — normal mode but data all zero (hardware defect)")
else:
    print("Gyro: DEAD — cannot enter normal mode")
if acc_mode == 1:
    print("Accel: OK")
else:
    print("Accel: ERROR")

