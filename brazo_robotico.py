#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
brazo_robotico.py
=================

Brazo robótico de 3 grados de libertad (base, hombro y codo) controlado con
una PCA9685 y las librerías de Adafruit.

La cámara va montada en la pinza (extremo del brazo). Este archivo NO mueve el
brazo a partir de la cámara: solo controla los servos y, opcionalmente, abre la
cámara para detectar rojo, amarillo y verde.

Sistema de coordenadas (cm)
---------------------------
Origen: centro del eje de la base, a la altura del suelo.
    X: hacia adelante
    Y: hacia la izquierda
    Z: hacia arriba

Articulaciones
--------------
    base    : giro alrededor de Z (yaw)
    hombro  : inclinación del brazo (pitch)
    codo    : inclinación del antebrazo (pitch)

La cinemática inversa es analítica (2 eslabones + giro de base). Con 3 DOF se
puede posicionar la punta en XYZ, pero NO orientar la cámara de forma
independiente: su inclinación depende del hombro y del codo.

Uso rápido
----------
    python3 brazo_robotico.py --sim          # probar sin hardware
    python3 brazo_robotico.py --test         # autocomprobación de la IK
    python3 brazo_robotico.py                # Raspberry Pi real
    python3 brazo_robotico.py --sim --camera 0

Desde otro programa:

    from brazo_robotico import RobotArm, create_driver, default_config

    arm = RobotArm(default_config(), create_driver(simulate=True))
    arm.home()
    arm.move_to(20, 0, 25, duration=2.0)
    print(arm.get_position())
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

# =============================================================================
# 1. CONFIGURACIÓN FÍSICA
# =============================================================================
# Medidas reales del brazo, en centímetros.

BASE_HEIGHT_CM = 15.0   # altura del eje del hombro sobre la base
UPPER_ARM_CM = 10.0     # distancia hombro -> codo
FOREARM_CM = 10.0       # distancia codo -> centro de la cámara/pinza

# Canales de la PCA9685. Cámbialos si cableas distinto.
CHANNEL_BASE = 0
CHANNEL_SHOULDER = 1
CHANNEL_ELBOW = 2

# Calibración de pulso para servos MG995 (cuentas de 12 bits a 60 Hz).
# A 60 Hz el periodo es 16.667 ms y cada cuenta son ~4.07 us:
#   500 us  -> 123 cuentas
#   2500 us -> 614 cuentas
# Es el rango típico del MG995. Si algún servo zumba o se traba en los
# extremos, usa un rango más conservador: 147 (600 us) .. 590 (2400 us).
PULSE_MIN = 123
PULSE_MAX = 614

# Límites lógicos de cada articulación, en grados.
# 0 = posición neutra. Ajusta según cómo queden montados los servos.
BASE_MIN, BASE_MAX = -90.0, 90.0
SHOULDER_MIN, SHOULDER_MAX = -30.0, 90.0
ELBOW_MIN, ELBOW_MAX = -90.0, 90.0

# Pose de reposo (base, hombro, codo) en grados lógicos.
HOME_POSE = (0.0, 30.0, 60.0)

# Movimiento suave: grados por paso y pausa entre pasos.
MAX_STEP_DEG = 1.0
STEP_DELAY_S = 0.02
DEFAULT_SPEED_DEG_S = 45.0


# =============================================================================
# 2. CONFIGURACIÓN DE SERVOS Y ARTICULACIONES
# =============================================================================

@dataclass(frozen=True)
class ServoConfig:
    """Cómo se traduce un ángulo de servo (0-180) a un pulso de la PCA9685."""

    channel: int
    pulse_min: int = PULSE_MIN
    pulse_max: int = PULSE_MAX
    angle_min: float = 0.0
    angle_max: float = 180.0
    offset: float = 0.0      # corrección mecánica en grados
    invert: bool = False     # invierte el sentido de giro

    def angle_to_pulse(self, angle: float) -> int:
        angle = min(max(angle, self.angle_min), self.angle_max)
        servo_angle = (180.0 - angle) if self.invert else angle
        servo_angle = min(max(servo_angle + self.offset, 0.0), 180.0)
        t = servo_angle / 180.0
        pulse = self.pulse_min + t * (self.pulse_max - self.pulse_min)
        return int(round(pulse))


@dataclass(frozen=True)
class JointConfig:
    """Articulación lógica: ángulo de la articulación -> ángulo del servo."""

    name: str
    servo: ServoConfig
    zero: float = 90.0       # ángulo de servo para la posición neutra (0 lógico)
    sign: float = 1.0        # +1 o -1 según el montaje
    min_deg: float = -90.0
    max_deg: float = 90.0

    def joint_to_servo(self, joint_deg: float) -> float:
        return self.zero + self.sign * joint_deg

    def servo_to_joint(self, servo_deg: float) -> float:
        return (servo_deg - self.zero) / self.sign


@dataclass(frozen=True)
class ArmConfig:
    base_height: float
    upper_arm: float
    forearm: float
    joints: Tuple[JointConfig, JointConfig, JointConfig]
    home: Tuple[float, float, float]
    max_step_deg: float = MAX_STEP_DEG
    step_delay: float = STEP_DELAY_S
    default_speed: float = DEFAULT_SPEED_DEG_S

    @property
    def reach_min(self) -> float:
        return abs(self.upper_arm - self.forearm)

    @property
    def reach_max(self) -> float:
        return self.upper_arm + self.forearm


def default_config() -> ArmConfig:
    base = JointConfig(
        name="base",
        servo=ServoConfig(channel=CHANNEL_BASE),
        zero=90.0,
        sign=1.0,
        min_deg=BASE_MIN,
        max_deg=BASE_MAX,
    )
    shoulder = JointConfig(
        name="hombro",
        servo=ServoConfig(channel=CHANNEL_SHOULDER),
        zero=90.0,
        sign=1.0,
        min_deg=SHOULDER_MIN,
        max_deg=SHOULDER_MAX,
    )
    elbow = JointConfig(
        name="codo",
        servo=ServoConfig(channel=CHANNEL_ELBOW),
        zero=90.0,
        sign=1.0,
        min_deg=ELBOW_MIN,
        max_deg=ELBOW_MAX,
    )
    return ArmConfig(
        base_height=BASE_HEIGHT_CM,
        upper_arm=UPPER_ARM_CM,
        forearm=FOREARM_CM,
        joints=(base, shoulder, elbow),
        home=HOME_POSE,
        max_step_deg=MAX_STEP_DEG,
        step_delay=STEP_DELAY_S,
        default_speed=DEFAULT_SPEED_DEG_S,
    )


# =============================================================================
# 3. DRIVERS: HARDWARE REAL Y SIMULACIÓN
# =============================================================================

class ServoDriver:
    """Interfaz mínima que deben cumplir los drivers."""

    def set_pulse(self, channel: int, pulse: int) -> None:
        raise NotImplementedError

    def close(self) -> None:
        pass


class PCA9685Driver(ServoDriver):
    """Driver real con Adafruit Blinka + adafruit_pca9685."""

    def __init__(self, address: int = 0x40, frequency: int = 60) -> None:
        import board
        import busio
        from adafruit_pca9685 import PCA9685

        self._i2c = busio.I2C(board.SCL, board.SDA)
        self._pca = PCA9685(self._i2c, address=address)
        self._pca.frequency = frequency

    def set_pulse(self, channel: int, pulse: int) -> None:
        pulse = max(0, min(4095, int(pulse)))
        # duty_cycle es de 16 bits; el pulso está en cuentas de 12 bits.
        self._pca.channels[channel].duty_cycle = pulse << 4

    def close(self) -> None:
        try:
            for channel in range(16):
                self._pca.channels[channel].duty_cycle = 0
        except Exception:
            pass
        try:
            self._pca.deinit()
        except Exception:
            pass


class SimulatedDriver(ServoDriver):
    """Driver de simulación: guarda los pulsos y no toca hardware."""

    def __init__(self, verbose: bool = False) -> None:
        self.verbose = verbose
        self.pulses: dict[int, int] = {}

    def set_pulse(self, channel: int, pulse: int) -> None:
        self.pulses[channel] = int(pulse)
        if self.verbose:
            print(f"  [sim] canal {channel} -> pulso {pulse}")

    def close(self) -> None:
        self.pulses.clear()


def create_driver(simulate: bool = False) -> ServoDriver:
    """Crea el driver real o el simulado."""

    if simulate:
        return SimulatedDriver()
    return PCA9685Driver()


# =============================================================================
# 4. ERRORES
# =============================================================================

class ArmError(Exception):
    """Error base del brazo."""


class UnreachableError(ArmError):
    """La coordenada está fuera del alcance físico."""


class JointLimitError(ArmError):
    """La solución requiere un ángulo fuera de los límites."""


# =============================================================================
# 5. BRAZO: CINEMÁTICA Y MOVIMIENTO
# =============================================================================

class RobotArm:
    """Brazo de 3 DOF con cinemática inversa y movimiento suave."""

    def __init__(self, config: Optional[ArmConfig] = None,
                 driver: Optional[ServoDriver] = None) -> None:
        self.cfg = config or default_config()
        self.driver = driver if driver is not None else SimulatedDriver()
        self._lock = threading.RLock()
        # No se puede leer la posición real de un servo. Asumimos que el brazo
        # está en la pose de reposo al arrancar.
        self._joints: list[float] = list(self.cfg.home)

    # ------------------------------------------------------------------ IK/FK
    def forward_kinematics(
        self, joints: Optional[Sequence[float]] = None
    ) -> Tuple[float, float, float]:
        """Ángulos lógicos (base, hombro, codo) -> coordenada (x, y, z) en cm."""

        base, shoulder, elbow = joints if joints is not None else self._joints
        b = math.radians(base)
        a = math.radians(shoulder)
        g = math.radians(elbow)

        r = (self.cfg.upper_arm * math.cos(a)
             + self.cfg.forearm * math.cos(a + g))
        z = (self.cfg.base_height
             + self.cfg.upper_arm * math.sin(a)
             + self.cfg.forearm * math.sin(a + g))
        x = r * math.cos(b)
        y = r * math.sin(b)
        return (x, y, z)

    def inverse_kinematics(
        self, x: float, y: float, z: float, elbow: str = "auto"
    ) -> Tuple[float, float, float]:
        """Coordenada (x, y, z) en cm -> ángulos lógicos.

        ``elbow`` puede ser "auto", "up" o "down". Con "auto" se intenta la
        solución de codo arriba y, si no cabe en los límites, la de codo abajo.
        """

        if z < 0:
            raise UnreachableError(
                f"z={z:.2f} cm está por debajo del suelo (z debe ser >= 0)."
            )

        r = math.hypot(x, y)
        zr = z - self.cfg.base_height
        d = math.hypot(r, zr)

        l1, l2 = self.cfg.upper_arm, self.cfg.forearm
        if d > self.cfg.reach_max + 1e-6:
            raise UnreachableError(
                f"El punto está a {d:.2f} cm; el alcance máximo es "
                f"{self.cfg.reach_max:.2f} cm."
            )
        if d < self.cfg.reach_min - 1e-6:
            raise UnreachableError(
                f"El punto está a {d:.2f} cm; el alcance mínimo es "
                f"{self.cfg.reach_min:.2f} cm."
            )

        cos_gamma = (d * d - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        cos_gamma = max(-1.0, min(1.0, cos_gamma))
        gamma_abs = math.acos(cos_gamma)

        base_deg = math.degrees(math.atan2(y, x))

        if elbow == "auto":
            signs = (1.0, -1.0)
        elif elbow == "up":
            signs = (1.0,)
        elif elbow == "down":
            signs = (-1.0,)
        else:
            raise ValueError("elbow debe ser 'auto', 'up' o 'down'.")

        candidates: list[Tuple[float, float, float]] = []
        for sign in signs:
            gamma = sign * gamma_abs
            alpha = (math.atan2(zr, r)
                     - math.atan2(l2 * math.sin(gamma),
                                  l1 + l2 * math.cos(gamma)))
            candidates.append(
                (base_deg, math.degrees(alpha), math.degrees(gamma))
            )

        for candidate in candidates:
            if self._within_limits(candidate):
                return candidate

        detalle = ", ".join(
            f"(base={b:.1f}, hombro={s:.1f}, codo={e:.1f})"
            for b, s, e in candidates
        )
        raise JointLimitError(
            "La coordenada es alcanzable, pero requiere ángulos fuera de los "
            f"límites: {detalle}. Revisa los límites o la calibración."
        )

    # -------------------------------------------------------------- límites
    def _within_limits(self, joints: Sequence[float]) -> bool:
        for joint, value in zip(self.cfg.joints, joints):
            if value < joint.min_deg - 1e-6 or value > joint.max_deg + 1e-6:
                return False
            servo_angle = joint.joint_to_servo(value)
            if servo_angle < 0.0 or servo_angle > 180.0:
                return False
        return True

    def _limit_message(self, joints: Sequence[float]) -> str:
        partes = []
        for joint, value in zip(self.cfg.joints, joints):
            estado = "OK" if joint.min_deg <= value <= joint.max_deg else "FUERA"
            partes.append(
                f"{joint.name}={value:.1f}° "
                f"(límite {joint.min_deg:.0f}..{joint.max_deg:.0f}, {estado})"
            )
        return "; ".join(partes)

    # -------------------------------------------------------------- movimiento
    def _apply(self, joints: Sequence[float]) -> None:
        for joint, value in zip(self.cfg.joints, joints):
            servo_angle = joint.joint_to_servo(value)
            pulse = joint.servo.angle_to_pulse(servo_angle)
            self.driver.set_pulse(joint.servo.channel, pulse)
        self._joints = list(joints)

    def move_joints(
        self,
        base: float,
        shoulder: float,
        elbow: float,
        duration: Optional[float] = None,
        speed: Optional[float] = None,
    ) -> Tuple[float, float, float]:
        """Mueve las articulaciones con interpolación suave.

        ``duration`` en segundos; si se omite, se calcula con ``speed``
        (grados/segundo). ``duration`` tiene prioridad sobre ``speed``.
        """

        target = [float(base), float(shoulder), float(elbow)]
        if not self._within_limits(target):
            raise JointLimitError(self._limit_message(target))

        with self._lock:
            start = list(self._joints)
            max_delta = max(abs(t - s) for s, t in zip(start, target))

            if max_delta < 1e-9:
                # Aun sin movimiento, escribe los pulsos para dejar los servos
                # en la posición conocida.
                self._apply(target)
                return self.get_position()

            if duration is None:
                v = speed if speed is not None else self.cfg.default_speed
                duration = max_delta / max(float(v), 1e-6)

            duration = max(float(duration), 0.0)
            steps = max(1, int(math.ceil(max_delta / self.cfg.max_step_deg)))
            dt = duration / steps

            for i in range(1, steps + 1):
                t = i / steps
                # smoothstep: arranque y frenada suaves
                eased = t * t * (3.0 - 2.0 * t)
                pose = [s + (tgt - s) * eased for s, tgt in zip(start, target)]
                self._apply(pose)
                if dt > 0:
                    time.sleep(dt)

            self._joints = target
            return self.get_position()

    def move_to(
        self,
        x: float,
        y: float,
        z: float,
        duration: Optional[float] = None,
        speed: Optional[float] = None,
        elbow: str = "auto",
    ) -> Tuple[float, float, float]:
        """Mueve la punta a (x, y, z) en cm."""

        target = self.inverse_kinematics(x, y, z, elbow=elbow)
        return self.move_joints(*target, duration=duration, speed=speed)

    def home(
        self, duration: Optional[float] = None, speed: Optional[float] = None
    ) -> Tuple[float, float, float]:
        """Va a la pose de reposo."""

        return self.move_joints(*self.cfg.home, duration=duration, speed=speed)

    def set_servo_angle(self, channel: int, angle: float) -> None:
        """Mueve un servo directamente (0-180°), para calibrar.

        No actualiza la pose lógica. Después llama a ``home()`` o a
        ``move_joints()`` para volver a una pose conocida.
        """

        angle = min(max(float(angle), 0.0), 180.0)
        for joint in self.cfg.joints:
            if joint.servo.channel == channel:
                pulse = joint.servo.angle_to_pulse(angle)
                self.driver.set_pulse(channel, pulse)
                return
        canales = [j.servo.channel for j in self.cfg.joints]
        raise ValueError(
            f"El canal {channel} no está configurado. Canales: {canales}"
        )

    # ------------------------------------------------------------------ estado
    def get_position(self) -> Tuple[float, float, float]:
        with self._lock:
            return self.forward_kinematics(self._joints)

    def get_joints(self) -> Tuple[float, float, float]:
        with self._lock:
            return tuple(self._joints)  # type: ignore[return-value]

    def status(self) -> dict:
        with self._lock:
            joints = list(self._joints)
            x, y, z = self.forward_kinematics(joints)
            servo_angles = [
                joint.joint_to_servo(value)
                for joint, value in zip(self.cfg.joints, joints)
            ]
            pulses = {
                joint.name: joint.servo.angle_to_pulse(angle)
                for joint, angle in zip(self.cfg.joints, servo_angles)
            }
            return {
                "joints": tuple(joints),
                "servo_angles": tuple(servo_angles),
                "pulses": pulses,
                "xyz": (x, y, z),
            }

    def close(self) -> None:
        self.driver.close()


# =============================================================================
# 6. VISIÓN: DETECCIÓN DE ROJO, AMARILLO Y VERDE
# =============================================================================

# Rangos HSV de OpenCV (H: 0-179, S: 0-255, V: 0-255).
COLOR_RANGES = {
    "rojo": (
        ((0, 100, 100), (10, 255, 255)),
        ((170, 100, 100), (180, 255, 255)),
    ),
    "amarillo": (
        ((20, 100, 100), (35, 255, 255)),
    ),
    "verde": (
        ((40, 70, 70), (85, 255, 255)),
    ),
}

COLOR_BGR = {
    "rojo": (0, 0, 255),
    "amarillo": (0, 255, 255),
    "verde": (0, 255, 0),
}


def detect_colors(frame, min_area: float = 500.0) -> list[dict]:
    """Devuelve una lista de detecciones: color, área, caja y centro."""

    import cv2
    import numpy as np

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    detections: list[dict] = []

    for name, ranges in COLOR_RANGES.items():
        mask = None
        for low, high in ranges:
            partial = cv2.inRange(
                hsv,
                np.array(low, dtype=np.uint8),
                np.array(high, dtype=np.uint8),
            )
            mask = partial if mask is None else cv2.bitwise_or(mask, partial)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            detections.append({
                "color": name,
                "area": area,
                "bbox": (x, y, w, h),
                "center": (x + w / 2.0, y + h / 2.0),
            })

    detections.sort(key=lambda d: d["area"], reverse=True)
    return detections


def draw_detections(frame, detections: Sequence[dict]):
    """Dibuja las cajas y etiquetas sobre el frame."""

    import cv2

    for det in detections:
        x, y, w, h = det["bbox"]
        color = COLOR_BGR[det["color"]]
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        label = f"{det['color']} ({int(det['area'])})"
        cv2.putText(
            frame, label, (x, max(y - 8, 15)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2,
        )
    return frame


def run_camera(camera_index: int = 0, width: int = 640,
               height: int = 480) -> None:
    """Abre la cámara y muestra la detección de colores (no mueve el brazo)."""

    try:
        import cv2
    except ImportError:
        print("OpenCV no está instalado.")
        print("En la Raspberry Pi: sudo apt install python3-opencv python3-numpy")
        return

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"No se pudo abrir la cámara {camera_index}.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    print("Cámara abierta. Pulsa ESC o 'q' para salir.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("No se pudo leer un frame de la cámara.")
                break

            detections = detect_colors(frame)
            draw_detections(frame, detections)

            resumen = ", ".join(
                f"{d['color']}:{int(d['area'])}" for d in detections[:3]
            ) or "sin colores"
            cv2.putText(
                frame, resumen, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2,
            )
            cv2.imshow("Brazo - deteccion de colores", frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


# =============================================================================
# 7. AUTOCONFIGURACIÓN Y AUTOTEST
# =============================================================================

def print_config(cfg: ArmConfig) -> None:
    print("Configuración del brazo")
    print("-" * 40)
    print(f"  Altura del hombro : {cfg.base_height:.1f} cm")
    print(f"  Brazo (L1)        : {cfg.upper_arm:.1f} cm")
    print(f"  Antebrazo (L2)    : {cfg.forearm:.1f} cm")
    print(f"  Alcance           : {cfg.reach_min:.1f} .. {cfg.reach_max:.1f} cm")
    for joint in cfg.joints:
        print(
            f"  {joint.name:<8} canal {joint.servo.channel}: "
            f"{joint.min_deg:.0f}..{joint.max_deg:.0f}° lógicos"
        )
    print(f"  Home              : {cfg.home}")


def self_test() -> int:
    """Comprueba la IK/FK en simulación, sin hardware."""

    print("Autocomprobación (simulación)")
    print("-" * 40)
    arm = RobotArm(default_config(), SimulatedDriver())
    ok = True

    poses = [
        (0.0, 30.0, 60.0),
        (0.0, 45.0, 30.0),
        (30.0, 20.0, 40.0),
        (-45.0, 10.0, 50.0),
        (80.0, 0.0, 30.0),
    ]

    for pose in poses:
        x, y, z = arm.forward_kinematics(pose)
        try:
            solved = arm.inverse_kinematics(x, y, z, elbow="auto")
            x2, y2, z2 = arm.forward_kinematics(solved)
            error = math.dist((x, y, z), (x2, y2, z2))
            estado = "OK" if error < 1e-6 else "ERROR"
            if error >= 1e-6:
                ok = False
            print(
                f"  [{estado}] pose={pose} -> XYZ=({x:.2f}, {y:.2f}, {z:.2f}) "
                f"-> IK={tuple(round(v, 2) for v in solved)} "
                f"(error {error:.2e} cm)"
            )
        except ArmError as exc:
            ok = False
            print(f"  [ERROR] pose={pose}: {exc}")

    # Un punto claramente fuera de alcance debe fallar.
    try:
        arm.inverse_kinematics(100.0, 0.0, 10.0)
        ok = False
        print("  [ERROR] un punto fuera de alcance no lanzó UnreachableError")
    except UnreachableError:
        print("  [OK] punto fuera de alcance rechazado")

    # Un movimiento suave en simulación debe terminar en el destino.
    try:
        target = (15.0, 5.0, 25.0)
        arm.move_to(*target, duration=0.05)
        x, y, z = arm.get_position()
        error = math.dist(target, (x, y, z))
        estado = "OK" if error < 1e-6 else "ERROR"
        if error >= 1e-6:
            ok = False
        print(
            f"  [{estado}] move_to({target}) -> ({x:.2f}, {y:.2f}, {z:.2f}) "
            f"(error {error:.2e} cm)"
        )
    except ArmError as exc:
        ok = False
        print(f"  [ERROR] move_to: {exc}")

    print("-" * 40)
    print("Resultado:", "TODO OK" if ok else "HAY ERRORES")
    return 0 if ok else 1


# =============================================================================
# 8. CONSOLA INTERACTIVA
# =============================================================================

def print_help() -> None:
    print("""
Comandos:
  x y z [seg]     Mover la punta a esa coordenada en cm (ej: 20 0 25 2)
  angles b h c    Mover por ángulos lógicos: base, hombro, codo
  servo c a       Mover el servo del canal c a un ángulo 0-180 (calibrar)
  home            Ir a la pose de reposo
  pos             Mostrar articulaciones, servos y XYZ actuales
  limits          Mostrar límites y alcance
  cam             Abrir la cámara (rojo, amarillo, verde)
  config          Mostrar la configuración física
  help            Esta ayuda
  quit            Salir
""".strip())


def print_status(arm: RobotArm) -> None:
    s = arm.status()
    base, shoulder, elbow = s["joints"]
    x, y, z = s["xyz"]
    print(
        f"  Articulaciones: base={base:.1f}° hombro={shoulder:.1f}° "
        f"codo={elbow:.1f}°"
    )
    print(
        "  Servos: "
        + " ".join(
            f"{name}={angle:.1f}°"
            for name, angle in zip(("base", "hombro", "codo"),
                                   s["servo_angles"])
        )
    )
    print(f"  XYZ: ({x:.2f}, {y:.2f}, {z:.2f}) cm")
    print(f"  Pulsos: {s['pulses']}")


def _is_number(text: str) -> bool:
    try:
        float(text)
        return True
    except ValueError:
        return False


def run_console(arm: RobotArm, camera_index: int) -> None:
    print_config(arm.cfg)
    print()
    print_help()

    try:
        arm.home(duration=1.0)
    except ArmError as exc:
        print(f"No se pudo ir a home: {exc}")

    while True:
        try:
            line = input("\nbrazo> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()

        try:
            if cmd in ("q", "quit", "exit", "salir"):
                break
            elif cmd in ("h", "help", "ayuda"):
                print_help()
            elif cmd in ("home", "inicio"):
                arm.home()
                print("  En pose de reposo.")
            elif cmd in ("pos", "posicion", "posición"):
                print_status(arm)
            elif cmd in ("limits", "limites", "límites"):
                print_config(arm.cfg)
            elif cmd in ("config", "configuracion", "configuración"):
                print_config(arm.cfg)
            elif cmd in ("cam", "camera", "cámara", "camara"):
                run_camera(camera_index)
            elif cmd in ("angles", "angulos", "ángulos"):
                if len(parts) != 4:
                    print("  Uso: angles <base> <hombro> <codo>")
                    continue
                arm.move_joints(*map(float, parts[1:4]))
                print_status(arm)
            elif cmd == "servo":
                if len(parts) != 3:
                    print("  Uso: servo <canal> <ángulo 0-180>")
                    continue
                arm.set_servo_angle(int(parts[1]), float(parts[2]))
                print(f"  Canal {parts[1]} -> {parts[2]}°")
            elif len(parts) in (3, 4) and all(_is_number(p) for p in parts):
                x, y, z = map(float, parts[:3])
                duration = float(parts[3]) if len(parts) == 4 else None
                arm.move_to(x, y, z, duration=duration)
                print_status(arm)
            else:
                print("  Comando no reconocido. Escribe 'help'.")
        except UnreachableError as exc:
            print(f"  Fuera de alcance: {exc}")
        except JointLimitError as exc:
            print(f"  Límite de articulación: {exc}")
        except ValueError as exc:
            print(f"  Dato inválido: {exc}")
        except ArmError as exc:
            print(f"  Error: {exc}")


# =============================================================================
# 9. MAIN
# =============================================================================

def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Brazo robótico de 3 DOF con PCA9685."
    )
    parser.add_argument(
        "--sim", action="store_true",
        help="No usa hardware; simula los pulsos de la PCA9685.",
    )
    parser.add_argument(
        "--test", action="store_true",
        help="Ejecuta la autocomprobación de la IK y sale.",
    )
    parser.add_argument(
        "--camera", type=int, default=0,
        help="Índice de la cámara USB (por defecto 0).",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    if args.test:
        return self_test()

    cfg = default_config()

    try:
        driver = create_driver(simulate=args.sim)
    except Exception as exc:
        print("No se pudo inicializar la PCA9685.")
        print(f"Detalle: {exc}")
        print()
        print("Revisa:")
        print("  - I2C habilitado (sudo raspi-config)")
        print("  - Cableado SDA/SCL y alimentación")
        print("  - Librerías: pip install adafruit-blinka "
              "adafruit-circuitpython-pca9685")
        print()
        print("Para probar sin hardware: python3 brazo_robotico.py --sim")
        return 1

    arm = RobotArm(cfg, driver)

    if args.sim:
        print("Modo SIMULACIÓN: no se mueve ningún servo real.")
    else:
        print("Modo HARDWARE: cuidado con los límites mecánicos.")

    try:
        run_console(arm, args.camera)
    finally:
        arm.close()

    print("Programa terminado.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
