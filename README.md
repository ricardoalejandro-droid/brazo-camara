# Brazo robótico de 3 DOF con PCA9685

Brazo de **3 grados de libertad** (base, hombro y codo) controlado desde una
Raspberry Pi con una **PCA9685** y las librerías de Adafruit.

Incluye:

- Cinemática inversa analítica: le das `(x, y, z)` en cm y calcula los ángulos.
- Movimiento suave con límites de articulación y comprobación de alcance.
- Modo simulación para probar sin hardware.
- Consola interactiva y API de Python para importar.
- Cámara USB con detección de **rojo, amarillo y verde** (solo muestra; no
  mueve el brazo).

---

## 1. Cableado

### PCA9685

| PCA9685 | Raspberry Pi |
|---|---|
| VCC | 3.3 V |
| GND | GND |
| SCL | GPIO 3 (SCL) |
| SDA | GPIO 2 (SDA) |

Dirección I²C por defecto: `0x40`.

### Servos

| Servo | Canal PCA9685 |
|---|---|
| Base (giro) | 0 |
| Hombro | 1 |
| Codo | 2 |

- Alimenta los servos con una **fuente externa de 5–6 V** capaz de entregar la
  corriente que piden. No los alimentes desde el 5 V de la Raspberry Pi.
- Une el **GND de la fuente de servos con el GND de la Raspberry Pi**.
- Si usas servos grandes (MG996R), pon un condensador de 470–1000 µF en la
  alimentación.

Habilita I²C:

```bash
# Opción no interactiva (0 = habilitar)
sudo raspi-config nonint do_i2c 0
sudo reboot
```

O de forma interactiva:

```bash
sudo raspi-config
# Interface Options -> I2C -> Yes
sudo reboot
```

Comprueba que la PCA9685 aparece:

```bash
i2cdetect -y 1
# Debe aparecer 40
```

Si no aparece `40`:

- Revisa VCC, GND, SDA y SCL.
- Comprueba que la PCA9685 tenga alimentación lógica (VCC).
- Verifica que no haya otro dispositivo usando la dirección `0x40`.
- Prueba con otro cable o módulo.

---

## 2. Instalación

```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy python3-libgpiod \
    libgpiod3 python3-smbus i2c-tools

python3 -m pip install --break-system-packages \
    adafruit-blinka adafruit-circuitpython-pca9685
```

> Debian 13 marca el Python del sistema como "externally managed", por eso se
> usa `--break-system-packages`. Si prefieres no tocar el Python del sistema,
> crea un entorno virtual con `python3 -m venv .venv` y instala ahí.

O con el archivo de dependencias:

```bash
python3 -m pip install --break-system-packages -r requirements.txt
```

---

## 3. Configuración física (importante)

Abre `brazo_robotico.py` y ajusta la sección **1. CONFIGURACIÓN FÍSICA**:

```python
BASE_HEIGHT_CM = 15.0   # altura del eje del hombro sobre la base
UPPER_ARM_CM = 10.0     # distancia hombro -> codo
FOREARM_CM = 10.0       # distancia codo -> centro de la cámara/pinza
```

> Estas son las medidas de tu brazo (15 / 10 / 10 cm). Si cambias la mecánica,
> vuelve a medir y actualiza estos valores.

Canales y calibración:

```python
CHANNEL_BASE = 0
CHANNEL_SHOULDER = 1
CHANNEL_ELBOW = 2

# MG995: 500-2500 us a 60 Hz -> 123-614 cuentas de 12 bits
PULSE_MIN = 123
PULSE_MAX = 614
```

A 60 Hz el periodo es 16.667 ms y cada cuenta son ~4.07 µs. El rango
`123..614` corresponde a **500–2500 µs**, el rango típico del MG995.

Si un servo zumba, se calienta o se traba en los extremos, usa un rango más
conservador:

```python
PULSE_MIN = 147   # 600 us
PULSE_MAX = 590   # 2400 us
```

> **Alimentación:** el MG995 puede pedir 1–2 A por servo al arrancar. Usa una
> fuente externa de 5–6 V con al menos 3–5 A para los tres servos, y une el GND
> de esa fuente con el GND de la Raspberry Pi.

Límites lógicos:

```python
BASE_MIN, BASE_MAX = -90.0, 90.0
SHOULDER_MIN, SHOULDER_MAX = -30.0, 90.0
ELBOW_MIN, ELBOW_MAX = -90.0, 90.0
```

Ajusta según el montaje real. Si una articulación gira al revés, cambia el
`sign` de `JointConfig` a `-1`; si queda desalineada, usa `offset` en
`ServoConfig`.

> Con `SHOULDER_MIN = -30` el brazo no baja hasta el suelo lejos de la base.
> Si tu mecánica permite bajar más sin chocar con la base, cambia ese valor a
> `-60` o `-90` y vuelve a probar en simulación antes de usar hardware.

---

## 4. Uso

### Probar sin hardware (simulación)

```bash
python3 brazo_robotico.py --sim
```

### Autocomprobación de la cinemática

```bash
python3 brazo_robotico.py --test
```

### Raspberry Pi real

```bash
python3 brazo_robotico.py
```

### Cámara

Dentro de la consola:

```text
cam
```

O directamente:

```bash
python3 brazo_robotico.py --sim --camera 0
```

> `--camera` solo se usa al abrir la cámara. La detección muestra cajas y el
> nombre del color; no mueve el brazo.

### Comandos de la consola

```text
x y z [seg]     Mover la punta a esa coordenada en cm (ej: 20 0 25 2)
angles b h c    Mover por ángulos lógicos: base, hombro, codo
servo c a       Mover el servo del canal c a 0-180° (calibración)
home            Ir a la pose de reposo
pos             Mostrar articulaciones, servos y XYZ actuales
limits          Mostrar límites y alcance
cam             Abrir la cámara
config          Mostrar la configuración física
help            Ayuda
quit            Salir
```

Ejemplo:

```text
brazo> 20 0 25 2
brazo> 15 10 30
brazo> pos
```

---

## 5. API de Python

```python
from brazo_robotico import RobotArm, create_driver, default_config

arm = RobotArm(default_config(), create_driver(simulate=True))
arm.home()
arm.move_to(20, 0, 25, duration=2.0)
print(arm.get_position())

arm.move_joints(base=0, shoulder=45, elbow=30, duration=1.0)
print(arm.status())

arm.close()
```

Para hardware real: `create_driver(simulate=False)`.

### Errores que puedes capturar

```python
from brazo_robotico import UnreachableError, JointLimitError

try:
    arm.move_to(100, 0, 10)
except UnreachableError as exc:
    print("Fuera de alcance:", exc)
except JointLimitError as exc:
    print("Límite:", exc)
```

---

## 6. Sistema de coordenadas

- Origen: centro del eje de la base, a la altura del suelo.
- **X**: hacia adelante.
- **Y**: hacia la izquierda.
- **Z**: hacia arriba.
- Unidades: centímetros.

La cinemática inversa usa dos eslabones (brazo y antebrazo) más el giro de la
base. Con 3 DOF puedes posicionar la punta en XYZ, pero **no orientar la cámara
de forma independiente**: su inclinación depende del hombro y del codo. Para
corregirla haría falta un cuarto servo en la muñeca.

---

## 7. Seguridad

- Antes de conectar los servos, verifica que el brazo tenga espacio libre.
- Empieza con `--sim` y revisa los ángulos antes de usar hardware.
- Mantén los límites conservadores al principio.
- Si algo se traba, corta la alimentación de los servos inmediatamente.
- La PCA9685 no sabe en qué posición están los servos al arrancar. El programa
  asume que el brazo está en la pose de reposo (`HOME_POSE`). Colócalo ahí
  antes de iniciar, o llama a `home()` con cuidado.

---

## 8. Próximos pasos posibles

- Añadir un servo de pinza (canal 3).
- Añadir un servo de muñeca para orientar la cámara.
- Mover el brazo hacia el color detectado usando la posición del color en la
  imagen y una calibración cámara–brazo.
- Guardar trayectorias y reproducirlas.
