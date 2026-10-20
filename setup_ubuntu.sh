#!/usr/bin/env bash
# ==============================================================================
# Script de Instalación Automática para Ubuntu / Debian Linux
# Plataforma Gough-Stewart 6-DOF (6x Actuonix LAC por Hub USB)
# ==============================================================================
set -e

echo "=================================================================="
echo " Configurando Plataforma Gough-Stewart 6-DOF para Ubuntu Linux"
echo "=================================================================="

# 1. Instalar dependencias del sistema para PySide6 (Qt6), OpenGL y libusb-1.0
echo "[1/4] Instalando librerías del sistema (libusb, OpenGL, Qt6 XCB)..."
sudo apt-get update
sudo apt-get install -y \
    python3 \
    python3-venv \
    python3-pip \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    libgl1 \
    libegl1 \
    libxcb-cursor0 \
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-xinerama0 \
    libxcb-xfixes0 \
    libdbus-1-3

# 2. Instalar regla udev para las 6 tarjetas Actuonix LAC (VID=04d8, PID=fc5f)
echo "[2/4] Configurando permisos USB (udev) para tarjetas Actuonix LAC (04d8:fc5f)..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sudo cp "${SCRIPT_DIR}/99-actuonix-lac.rules" /etc/udev/rules.d/99-actuonix-lac.rules
sudo chmod 644 /etc/udev/rules.d/99-actuonix-lac.rules
sudo usermod -aG plugdev "$USER" || true
sudo udevadm control --reload-rules
sudo udevadm trigger

# 3. Crear entorno virtual e instalar paquetes de Python
echo "[3/4] Creando entorno virtual (.venv) e instalando requirements.txt..."
cd "${SCRIPT_DIR}"
if [ ! -d ".venv" ]; then
    python3 -m venv .venv
fi
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

echo "=================================================================="
echo " [4/4] ¡Instalación en Ubuntu completada con éxito!"
echo " Para iniciar la plataforma ejecuta:"
echo "   source .venv/bin/activate"
echo "   python main.py"
echo " Para diagnóstico rápido de tarjetas LAC por USB:"
echo "   python test_lac_usb.py"
echo "=================================================================="
