# Plataforma Gough-Stewart 6-DOF | GUI de Control y Seguimiento de Trayectorias

Interfaz gráfica de usuario (GUI) nativa en **Python 3 y PySide6** diseñada para una plataforma paralela Gough-Stewart de 6 grados de libertad (6-DOF) orientada a tareas de seguimiento de trayectorias espaciales (*Trajectory Tracking*), control cartesiano y orientación azimutal/elevación (*pointing/solar/antenna tracking*).

---

## 🚀 Características Principales

- **Arquitectura Asíncrona sin Bloqueo de UI**: Comunicación serie mediante `SerialManager` desacoplado en un `QThread` dedicado usando `PySide6.QtSerialPort` y con emulador/simulador virtual en bucle cerrado integrado.
- **Renderizado 3D en Tiempo Real**: Visor OpenGL con `pyqtgraph.opengl.GLViewWidget` que dibuja la base fija ($B_i$), plataforma móvil ($A_i$), los 6 actuadores lineales telescópicos con código de color dinámico y el vector de apuntamiento normal.
- **Cinemática Inversa Completa**:
  - **Modo Cartesiano (6-DOF)**: `pose_to_q(x, y, z, roll, pitch, yaw)` calcula las longitudes de actuadores $q_{mm}$, vector de traslación $d_a$ y matriz de rotación $R_{3\times 3}$.
  - **Modo Track (Alpha / Beta)**: `inverse_kinematics(y=[alpha, beta])` mapea ángulos de elevación y azimut en el espacio de tarea.
  - **Interpolación de Trayectorias**: `SimpleCubicSpline(t, y)` para perfiles suaves continuos en velocidad (precesión circular, lemniscata/ocho, espiral, respuesta al escalón o waypoints CSV).
  - **Análisis de Singularidades y Jacobiano**: `InvJac(da, R)` evalúa el Jacobiano inverso $6\times 6$, índice de manipulabilidad de Yoshikawa y número de condición normalizado $\kappa(J^{-1})$.
- **Telemetría en Vivo (30 Hz)**: Osciloscopio con `pyqtgraph.PlotWidget` para monitoreo de longitudes, errores de seguimiento $e_i = q^{des} - q^{act}$ (preparado para controladores Sliding-PI) y esfuerzo de control.
- **Monitoreo de Actuadores**: 6 barras de nivel con umbrales de zona segura, límites mecánicos de carrera y delta respecto al origen nominal.
- **Estilo Visual Industrial Dark Mode**: Paleta de alta tecnología (#121820, #1A232E, acentos en cian #00E5FF, verde esmeralda #00E676 y rojo parada #E53935).

---

## 📁 Estructura del Proyecto

```text
Plataforma Gough-Stewart_/
├── config/
│   ├── __init__.py
│   └── parameters.py        # Geometría de base/plataforma, límites de carrera, baudrates
├── core/
│   ├── __init__.py
│   ├── kinematics.py        # pose_to_q, inverse_kinematics, InvJac, SimpleCubicSpline
│   └── serial_manager.py    # SerialManager asíncrono sobre QThread con simulador
├── ui/
│   ├── __init__.py
│   ├── styles.py            # QSS Industrial Dark Theme completo y tokens de color
│   ├── widgets/
│   │   ├── __init__.py
│   │   ├── actuator_bars.py # Indicadores visuales de carrera para L1..L6
│   │   ├── gl_view.py       # Visor 3D OpenGL interactivo de la plataforma
│   │   ├── led_indicator.py # Badges LED y widgets de estado industrial
│   │   └── telemetry_plot.py# Gráficos multi-canal de telemetría a 30 Hz
│   └── main_window.py       # Ventana principal integrando todas las secciones
├── main.py                  # Punto de entrada de la aplicación
└── README.md                # Documentación del sistema
```

---

## 🛠 Requisitos e Instalación

### Dependencias
- Python 3.10+ (probado en Python 3.12)
- `PySide6`
- `pyqtgraph`
- `PyOpenGL`
- `numpy`
- `scipy`

Si aún no están instaladas en el entorno:
```bash
pip install PySide6 pyqtgraph PyOpenGL PyOpenGL_accelerate numpy scipy pyserial
```

---

## ▶ Ejecución

Para iniciar la aplicación:
```bash
python main.py
```

### Operación sin Hardware Físico (Modo Simulación)
1. En la barra superior, seleccione el puerto `SIMULADOR VIRTUAL (6-DOF Mock)`.
2. Presione **CONECTAR**.
3. El simulador responderá inmediatamente con dinámica de actuadores en bucle cerrado de segundo orden con ruido Gaussiano.
4. Puede mover los sliders cartesianos o ejecutar trayectorias en la pestaña **Modo Track (Alpha / Beta)** y observar el visor 3D y la telemetría en tiempo real.

---

## 📡 Protocolo de Comunicación Serie

La comunicación utiliza tramas ASCII de bajo retardo y alta fiabilidad:

- **Envío de Posición (PC -> Microcontrolador)**:
  ```text
  POS <q1> <q2> <q3> <q4> <q5> <q6>\n
  ```
  Ejemplo: `POS 331.73 331.73 331.73 331.73 331.73 331.73\n` (longitudes en milímetros).

- **Telemetría de Retorno (Microcontrolador -> PC)**:
  ```text
  TEL <q1> <q2> <q3> <q4> <q5> <q6>\n
  ```

- **Comandos de Control**:
  - `HOME\n`: Retorno a posición home nominal.
  - `ESTOP\n`: Parada de emergencia inmediata.
  - `RESET_SAFE\n`: Restablecimiento de modo seguro tras paro.
  - `CALIBRATE\n`: Rutina de auto-calibración.
  - `ZERO\n`: Puesta a cero de encoders / sensores de longitud.
