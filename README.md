# Plataforma Gough-Stewart 6-DOF | GUI de Control y Seguimiento de Trayectorias

Interfaz gráfica de usuario (GUI) nativa multiplataforma (**Windows 10/11 y Ubuntu Linux**) en **Python 3 y PySide6** diseñada para una plataforma paralela Gough-Stewart de 6 grados de libertad (6-DOF) con **control directo por USB de 6 tarjetas Actuonix LAC** (mediante Hub USB de 6 puertos).

---

## 🚀 Características Principales

- **Control Directo por USB Hub (6x Actuonix LAC)**:
  - Comunicación USB Bulk nativa de 10 bits (`0..1023`) con paquetes de 3 bytes `[Control, Data_Low, Data_High]` (`VID=0x04D8`, `PID=0xFC5F`).
  - Auto-detección e identificación de cada actuador (`L1..L6`) al abrir la app mediante `0x51 GET_SERIAL_NUM` y grabación de ID en EEPROM (`0x50 SET_SERIAL_NUM`).
  - Carga automática de configuración (`0x01 SET_ACCURACY` a `0x0F SET_AVERAGE_ADC`) al iniciar el sistema.
  - Envío de consigna y lectura simultánea de posición real en el mismo paquete (`0x20 SET_POSITION`) más consulta directa (`0x10 GET_FEEDBACK`).
  - Modo Simulador Virtual 6-DOF integrado para pruebas sin hardware físico.
- **Renderizado 3D en Tiempo Real**: Visor OpenGL con `pyqtgraph.opengl.GLViewWidget` (compatible con Windows y Ubuntu X11/Wayland).
- **Cinemática Directa, Inversa y Jacobiano Completos**:
  - **Modo Cartesiano (6-DOF)**: `pose_to_q(x, y, z, roll, pitch, yaw)`
  - **Modo Track Semiesfera ($\alpha / \beta$)**: `inverse_kinematics(y=[alpha, beta])`
  - **Reproducción de Trayectorias y Archivos CSV (MATLAB)**: `SimpleCubicSpline(t, y)`
  - **Análisis de Singularidades y Jacobiano**: `InvJac(da, R)`

---

## 🛠 Instalación y Ejecución

### Opción A: Ubuntu / Debian Linux (Recomendado — Nativo con `libusb`)

En Ubuntu no necesitas instalar drivers externos ni Zadig. El repositorio incluye un instalador automático que configura las librerías de Qt6/OpenGL, instala la regla `udev` (`99-actuonix-lac.rules`) para dar acceso sin `sudo` a las 6 tarjetas Actuonix LAC y crea el entorno virtual:

```bash
git clone https://github.com/PyMonkey81/gsp.git
cd gsp
bash setup_ubuntu.sh
```

Para ejecutar la aplicación en Ubuntu:
```bash
source .venv/bin/activate
python main.py
```

### Opción B: Windows 10 / 11

```powershell
git clone https://github.com/PyMonkey81/gsp.git
cd gsp
pip install -r requirements.txt
python main.py
```
> **Nota para Windows (64 bits):** Si Windows tiene cargado el driver antiguo de Microchip (`mchpusb.sys`), abre **[Zadig](https://zadig.akeo.ie/)**, selecciona la tarjeta LAC (`04D8:FC5F`) y haz clic en *"Replace Driver -> WinUSB"*.

---

## 🔍 Diagnóstico Rápido de Tarjetas LAC por USB

Para probar 1 o más tarjetas LAC desde la terminal (leer feedback ADC `0..1023`, mover un actuador o asignarle su número de serie `1..6`):

```bash
python test_lac_usb.py
```
