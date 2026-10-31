"""
Mecanum motor controller for Raspberry Pi 5
Driver board: HW-249 with L9110S (5V power supply)

Mecanum wheel configuration (top view):
    FL ──────── FR
    │    ↑     │
    │  FRONT   │
    │          │
    RL ──────── RR

Each motor has 2 pins:
- IA: forward
- IB: reverse
"""

import time
import logging
import threading
from dataclasses import dataclass

logger = logging.getLogger(__name__)

# Import gpiod for Pi5
try:
    import gpiod
    from gpiod.line import Direction, Value
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False
    logger.warning("gpiod non disponible - mode simulation")


@dataclass
class MotorPins:
    """Pin configuration for a motor"""
    ia: int  # Forward pin
    ib: int  # Reverse pin


MOTORS = ('FL', 'FR', 'RL', 'RR')

# Default GPIO configuration for HW-249/L9110S
DEFAULT_MOTOR_CONFIG = {
    'FL': MotorPins(ia=17, ib=27),   # Front Left  - GPIO17, GPIO27
    'FR': MotorPins(ia=22, ib=23),   # Front Right - GPIO22, GPIO23
    'RL': MotorPins(ia=24, ib=25),   # Rear Left   - GPIO24, GPIO25
    'RR': MotorPins(ia=5, ib=6),     # Rear Right  - GPIO5, GPIO6
}

CHIP = "/dev/gpiochip0"


class MecanumController:
    """
    Controller for Mecanum wheeled robot using gpiod

    The H-bridge inputs are plain GPIOs: speed is a software duty cycle
    (see _pwm_loop).
    """

    # 20 steps/cycle = 5% duty-cycle resolution, matching the UI's speed
    # increment (+/- 5%).
    PWM_HZ = 100
    PWM_STEPS = 20

    def __init__(self, motor_config: dict = None):
        self.config = motor_config or DEFAULT_MOTOR_CONFIG
        self.request = None
        self._lock = threading.Lock()

        # Current motor state
        self._motor_speeds = {m: 0 for m in MOTORS}
        self._motor_directions = {m: True for m in MOTORS}

        # Software PWM
        self._pwm_thread = None
        self._pwm_running = False
        self._changed = threading.Event()  # set on every command: wakes an idle PWM thread
        self._pin_values = {}  # {pin: last written Value} — skip redundant syscalls

        self._init_gpio()

    def _init_gpio(self):
        """Initializes GPIO pins with gpiod"""
        if not HAS_GPIO:
            logger.info("Mode simulation - pas de GPIO")
            return

        try:
            gpio_config = {}
            for pins in self.config.values():
                for pin in (pins.ia, pins.ib):
                    gpio_config[pin] = gpiod.LineSettings(
                        direction=Direction.OUTPUT,
                        output_value=Value.INACTIVE
                    )

            self.request = gpiod.request_lines(
                CHIP,
                consumer="mecanum_motors",
                config=gpio_config
            )

            logger.info("GPIO initialisés (gpiod) — " + ", ".join(
                f"{m}: {self.config[m].ia}/{self.config[m].ib}" for m in MOTORS))

            self._pwm_running = True
            self._pwm_thread = threading.Thread(target=self._pwm_loop, daemon=True)
            self._pwm_thread.start()

        except Exception as e:
            # Lines held by another process (a second robot instance) or a
            # wrong chip: fail loudly. Falling back to simulation here made
            # the app report the motors as working while nothing could move.
            self.request = None
            raise RuntimeError(f"GPIO moteurs indisponibles: {e}") from e

    def _write_pin(self, pin: int, value):
        """Writes a GPIO only when its state changes (one syscall per real edge)."""
        if self._pin_values.get(pin) is not value:
            self.request.set_value(pin, value)
            self._pin_values[pin] = value

    def _drive(self, pins: MotorPins, forward: bool):
        self._write_pin(pins.ia, Value.ACTIVE if forward else Value.INACTIVE)
        self._write_pin(pins.ib, Value.INACTIVE if forward else Value.ACTIVE)

    def _coast(self, pins: MotorPins):
        self._write_pin(pins.ia, Value.INACTIVE)
        self._write_pin(pins.ib, Value.INACTIVE)

    def _pwm_loop(self):
        """Software PWM at PWM_HZ, waking up on edges only.

        A motor at duty n/PWM_STEPS is driven during the first n steps of each
        cycle. Rather than ticking every step (2000 wake-ups/s, GIL taken each
        time), the loop switches every running motor on at the start of the
        cycle, then sleeps straight to each distinct switch-off point: a few
        wake-ups per cycle, same waveform. With every motor stopped it blocks
        until the next command and costs nothing.
        """
        period = 1.0 / self.PWM_HZ
        step = period / self.PWM_STEPS
        cycle_start = time.perf_counter()

        while self._pwm_running:
            with self._lock:
                self._changed.clear()
                plan = [
                    (self.config[m],
                     round(self._motor_speeds[m] / 100 * self.PWM_STEPS),
                     self._motor_directions[m])
                    for m in MOTORS
                ]

            if not any(on_steps for _, on_steps, _ in plan):
                for pins, _, _ in plan:
                    self._coast(pins)
                self._changed.wait(timeout=0.5)
                cycle_start = time.perf_counter()
                continue

            for pins, on_steps, forward in plan:
                if on_steps:
                    self._drive(pins, forward)
                else:
                    self._coast(pins)

            for off_step in sorted({n for _, n, _ in plan if 0 < n < self.PWM_STEPS}):
                self._sleep_until(cycle_start + off_step * step)
                for pins, on_steps, _ in plan:
                    if on_steps == off_step:
                        self._coast(pins)

            cycle_start += period
            now = time.perf_counter()
            if now - cycle_start > period:
                cycle_start = now  # fell behind (GIL, load): resync, no burst of short cycles
            self._sleep_until(cycle_start)

    @staticmethod
    def _sleep_until(deadline: float):
        delay = deadline - time.perf_counter()
        if delay > 0:
            time.sleep(delay)

    def _apply(self, commands: dict):
        """Sets {motor: (speed 0-100 %, forward)} for all motors at once.

        One lock for the four wheels: the PWM thread never mixes a new speed
        on one wheel with the previous one on another. Repeated identical
        commands (watchdog stops, joystick heartbeat) don't wake it up.
        """
        changed = False
        with self._lock:
            for motor, (speed, forward) in commands.items():
                speed = max(0, min(100, speed))
                if (speed, forward) != (self._motor_speeds[motor], self._motor_directions[motor]):
                    self._motor_speeds[motor] = speed
                    self._motor_directions[motor] = forward
                    changed = True
        if changed:
            self._changed.set()

    def stop(self):
        """Stops all motors immediately"""
        self._apply({m: (0, True) for m in MOTORS})

    def move(self, vx: float, vy: float, omega: float, speed: int = 100):
        """
        Omnidirectional movement with joystick

        Args:
            vx: Lateral velocity (-1 to 1) - left/right
            vy: Longitudinal velocity (-1 to 1) - forward/backward
            omega: Rotation (-1 to 1) - rotate left/right
            speed: Max speed (0-100%)
        """
        # Clamp inputs
        vx = max(-1, min(1, vx))
        vy = max(-1, min(1, vy))
        omega = max(-1, min(1, omega))

        # Mecanum wheel speeds
        wheels = {
            'FL': vy + vx + omega,
            'FR': vy - vx - omega,
            'RL': vy - vx + omega,
            'RR': vy + vx - omega,
        }

        # Normalize if > 1, then apply speed and direction
        max_val = max(1, *(abs(w) for w in wheels.values()))
        self._apply({m: (int(abs(w) / max_val * speed), w >= 0) for m, w in wheels.items()})

    def forward(self, speed: int = 50):
        self.move(0, 1, 0, speed)

    def backward(self, speed: int = 50):
        self.move(0, -1, 0, speed)

    def strafe_left(self, speed: int = 50):
        self.move(-1, 0, 0, speed)

    def strafe_right(self, speed: int = 50):
        self.move(1, 0, 0, speed)

    def rotate_left(self, speed: int = 50):
        self.move(0, 0, -1, speed)

    def rotate_right(self, speed: int = 50):
        self.move(0, 0, 1, speed)

    def get_status(self) -> dict:
        """Returns the current motor state"""
        with self._lock:
            return {
                m: {'speed': self._motor_speeds[m], 'forward': self._motor_directions[m]}
                for m in MOTORS
            }

    def close(self):
        """Releases GPIO resources"""
        self.stop()
        self._pwm_running = False
        self._changed.set()
        if self._pwm_thread:
            self._pwm_thread.join(timeout=1)

        if self.request is not None:
            # PWM thread is dead: drive every pin LOW ourselves so the H-bridge
            # inputs are not left ACTIVE (or floating) between join and release.
            for pins in self.config.values():
                try:
                    self.request.set_value(pins.ia, Value.INACTIVE)
                    self.request.set_value(pins.ib, Value.INACTIVE)
                except Exception:
                    pass
            try:
                self.request.release()
            except Exception:
                pass
            self.request = None


# Module test
if __name__ == "__main__":
    print("=" * 50)
    print("  Test Contrôleur Mecanum (gpiod)")
    print("=" * 50)

    controller = MecanumController()

    print("\nTest des mouvements (Ctrl+C pour arrêter)...")

    try:
        movements = [
            ("Avant 50%", lambda: controller.forward(50)),
            ("Arrière 50%", lambda: controller.backward(50)),
            ("Strafe Gauche", lambda: controller.strafe_left(50)),
            ("Strafe Droite", lambda: controller.strafe_right(50)),
            ("Rotation Gauche", lambda: controller.rotate_left(40)),
            ("Rotation Droite", lambda: controller.rotate_right(40)),
        ]

        for name, action in movements:
            print(f"\n{name}...")
            action()
            status = controller.get_status()
            for motor, state in status.items():
                direction = "▲" if state['forward'] else "▼"
                print(f"  {motor}: {state['speed']:3d}% {direction}")
            time.sleep(2)
            controller.stop()
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\nArrêt demandé")

    controller.close()
    print("\nTest terminé")
