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
import threading
from dataclasses import dataclass

# Import gpiod for Pi5
try:
    import gpiod
    from gpiod.line import Direction, Value
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False
    print("Warning: gpiod non disponible - mode simulation")


@dataclass
class MotorPins:
    """Pin configuration for a motor"""
    ia: int  # Forward pin
    ib: int  # Reverse pin


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

    Note: This version uses simple ON/OFF.
    For speed control, a software duty cycle is used.
    """

    def __init__(self, motor_config: dict = None):
        self.config = motor_config or DEFAULT_MOTOR_CONFIG
        self.request = None
        self._lock = threading.Lock()

        # Current motor state
        self._motor_speeds = {'FL': 0, 'FR': 0, 'RL': 0, 'RR': 0}
        self._motor_directions = {'FL': True, 'FR': True, 'RL': True, 'RR': True}

        # Software PWM
        self._pwm_thread = None
        self._pwm_running = False
        self._pwm_states = {}  # {motor: {'speed': 0-100, 'forward': True}}

        self._init_gpio()

    def _init_gpio(self):
        """Initializes GPIO pins with gpiod"""
        if not HAS_GPIO:
            print("Mode simulation - pas de GPIO")
            return

        try:
            # Collect all pins
            gpio_config = {}
            for name, pins in self.config.items():
                gpio_config[pins.ia] = gpiod.LineSettings(
                    direction=Direction.OUTPUT,
                    output_value=Value.INACTIVE
                )
                gpio_config[pins.ib] = gpiod.LineSettings(
                    direction=Direction.OUTPUT,
                    output_value=Value.INACTIVE
                )

            self.request = gpiod.request_lines(
                CHIP,
                consumer="mecanum_motors",
                config=gpio_config
            )

            print("GPIO initialisés (gpiod)")
            print(f"  FL: GPIO{self.config['FL'].ia}/{self.config['FL'].ib}")
            print(f"  FR: GPIO{self.config['FR'].ia}/{self.config['FR'].ib}")
            print(f"  RL: GPIO{self.config['RL'].ia}/{self.config['RL'].ib}")
            print(f"  RR: GPIO{self.config['RR'].ia}/{self.config['RR'].ib}")

            # Start the software PWM thread
            self._start_pwm_thread()

        except Exception as e:
            print(f"Erreur init GPIO: {e}")
            self.request = None

    def _start_pwm_thread(self):
        """Starts the software PWM thread"""
        self._pwm_running = True
        self._pwm_thread = threading.Thread(target=self._pwm_loop, daemon=True)
        self._pwm_thread.start()

    def _pwm_loop(self):
        """Software PWM loop - 100 Hz"""
        period = 0.01  # 10ms = 100 Hz
        step = 0.001   # 1ms steps (100 steps per period = 0-100%)

        while self._pwm_running:
            for i in range(100):  # 100 steps per cycle
                if not self._pwm_running:
                    break

                with self._lock:
                    for motor in ['FL', 'FR', 'RL', 'RR']:
                        speed = self._motor_speeds[motor]
                        forward = self._motor_directions[motor]
                        pins = self.config[motor]

                        if self.request is None:
                            continue

                        if speed == 0:
                            # Stop
                            self.request.set_value(pins.ia, Value.INACTIVE)
                            self.request.set_value(pins.ib, Value.INACTIVE)
                        elif i < speed:
                            # ON phase
                            if forward:
                                self.request.set_value(pins.ia, Value.ACTIVE)
                                self.request.set_value(pins.ib, Value.INACTIVE)
                            else:
                                self.request.set_value(pins.ia, Value.INACTIVE)
                                self.request.set_value(pins.ib, Value.ACTIVE)
                        else:
                            # OFF phase (for PWM)
                            self.request.set_value(pins.ia, Value.INACTIVE)
                            self.request.set_value(pins.ib, Value.INACTIVE)

                time.sleep(step)

    def _set_motor(self, motor: str, speed: int, forward: bool):
        """
        Sets the speed and direction of a motor

        Args:
            motor: Motor name (FL, FR, RL, RR)
            speed: Speed 0-100 (%)
            forward: True = forward, False = reverse
        """
        if motor not in self.config:
            return

        speed = max(0, min(100, speed))

        with self._lock:
            self._motor_speeds[motor] = speed
            self._motor_directions[motor] = forward

    def stop(self):
        """Stops all motors immediately"""
        for motor in ['FL', 'FR', 'RL', 'RR']:
            self._set_motor(motor, 0, True)

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

        # Calculate Mecanum wheel speeds
        fl = vy + vx + omega
        fr = vy - vx - omega
        rl = vy - vx + omega
        rr = vy + vx - omega

        # Normalize if > 1
        max_val = max(abs(fl), abs(fr), abs(rl), abs(rr), 1)
        fl /= max_val
        fr /= max_val
        rl /= max_val
        rr /= max_val

        # Apply speed and direction
        self._set_motor('FL', int(abs(fl) * speed), fl >= 0)
        self._set_motor('FR', int(abs(fr) * speed), fr >= 0)
        self._set_motor('RL', int(abs(rl) * speed), rl >= 0)
        self._set_motor('RR', int(abs(rr) * speed), rr >= 0)

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
                'FL': {'speed': self._motor_speeds['FL'], 'forward': self._motor_directions['FL']},
                'FR': {'speed': self._motor_speeds['FR'], 'forward': self._motor_directions['FR']},
                'RL': {'speed': self._motor_speeds['RL'], 'forward': self._motor_directions['RL']},
                'RR': {'speed': self._motor_speeds['RR'], 'forward': self._motor_directions['RR']},
            }

    def close(self):
        """Releases GPIO resources"""
        self._pwm_running = False
        if self._pwm_thread:
            self._pwm_thread.join(timeout=1)

        self.stop()

        if self.request is not None:
            try:
                self.request.release()
            except:
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
