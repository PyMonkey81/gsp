"""
Real-time 3D OpenGL visualization widget for Gough-Stewart 6-DOF Platform.
Built with pyqtgraph.opengl.GLViewWidget, fully expanded viewport, and centered camera.
"""

from typing import Optional, Union
import numpy as np
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QFrame, QSizePolicy
)
from PySide6.QtGui import QVector3D
from PySide6.QtCore import Qt

import pyqtgraph.opengl as gl

from config.parameters import BASE_JOINTS, PLATFORM_JOINTS, Z_HOME, Q_MIN, Q_MAX, Q_SAFE_MIN, Q_SAFE_MAX
from ui.styles import (
    COLOR_CARD_BG, COLOR_BORDER, COLOR_CYAN, COLOR_EMERALD,
    COLOR_EMERGENCY, COLOR_WARNING, COLOR_TEXT_MUTED
)

# Mechanical center of the platform mechanism (base at z=0, platform nominal at z=300)
ROBOT_CENTER = QVector3D(0.0, 0.0, float(Z_HOME / 2.0))
DEFAULT_CAM_DISTANCE = 550.0
DEFAULT_CAM_ELEVATION = 22.0
DEFAULT_CAM_AZIMUTH = 45.0


class Platform3DView(QWidget):
    """
    Interactive 3D viewport rendering base, mobile platform, and 6 dynamic actuators.
    Fully expanding viewport with compact overlay controls and centered camera target.
    """
    def __init__(self, base_joints: Optional[np.ndarray] = None, platform_joints: Optional[np.ndarray] = None, parent=None):
        super().__init__(parent)
        self.base_joints = np.asarray(base_joints if base_joints is not None else BASE_JOINTS, dtype=np.float64)
        self.platform_joints = np.asarray(platform_joints if platform_joints is not None else PLATFORM_JOINTS, dtype=np.float64)

        # Ensure container widget expands in both dimensions
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        # Zero-margin, zero-spacing layout to eliminate any gaps
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Ultra-compact Camera Presets Bar
        cam_bar = QFrame()
        cam_bar.setFixedHeight(26)
        cam_bar.setStyleSheet(f"""
            QFrame {{
                background-color: #141B24;
                border-bottom: 1px solid {COLOR_BORDER};
                padding: 0px 6px;
            }}
        """)
        cam_layout = QHBoxLayout(cam_bar)
        cam_layout.setContentsMargins(6, 1, 6, 1)
        cam_layout.setSpacing(4)

        title_lbl = QLabel("VISOR 3D")
        title_lbl.setStyleSheet(f"font-weight: 800; font-size: 10px; color: {COLOR_CYAN}; letter-spacing: 0.5px;")
        cam_layout.addWidget(title_lbl)
        cam_layout.addStretch()

        btn_iso = QPushButton("ISO")
        btn_top = QPushButton("TOP")
        btn_front = QPushButton("FRONT")
        btn_reset = QPushButton("RESET")
        for b in (btn_iso, btn_top, btn_front, btn_reset):
            b.setFixedHeight(20)
            b.setStyleSheet(f"""
                QPushButton {{
                    background-color: #1B2430;
                    border: 1px solid #37474F;
                    border-radius: 3px;
                    color: #ECEFF1;
                    font-size: 10px;
                    font-weight: 700;
                    padding: 1px 7px;
                    min-height: 18px;
                }}
                QPushButton:hover {{
                    background-color: #263342;
                    border-color: {COLOR_CYAN};
                    color: {COLOR_CYAN};
                }}
            """)
            cam_layout.addWidget(b)

        layout.addWidget(cam_bar, stretch=0)

        # GLViewWidget with full expansion
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.gl_widget.setBackgroundColor("#10151C")
        
        # Center camera precisely on mechanism midpoint (0, 0, 150 mm)
        self.gl_widget.opts['center'] = ROBOT_CENTER
        self.gl_widget.setCameraPosition(
            pos=ROBOT_CENTER,
            distance=DEFAULT_CAM_DISTANCE,
            elevation=DEFAULT_CAM_ELEVATION,
            azimuth=DEFAULT_CAM_AZIMUTH
        )
        layout.addWidget(self.gl_widget, stretch=1)

        # Wire camera preset buttons with centered focus
        btn_iso.clicked.connect(lambda: self.gl_widget.setCameraPosition(
            pos=ROBOT_CENTER, distance=DEFAULT_CAM_DISTANCE, elevation=22, azimuth=45
        ))
        btn_top.clicked.connect(lambda: self.gl_widget.setCameraPosition(
            pos=ROBOT_CENTER, distance=DEFAULT_CAM_DISTANCE, elevation=90, azimuth=0
        ))
        btn_front.clicked.connect(lambda: self.gl_widget.setCameraPosition(
            pos=ROBOT_CENTER, distance=DEFAULT_CAM_DISTANCE, elevation=0, azimuth=0
        ))
        btn_reset.clicked.connect(lambda: self.gl_widget.setCameraPosition(
            pos=ROBOT_CENTER, distance=DEFAULT_CAM_DISTANCE, elevation=DEFAULT_CAM_ELEVATION, azimuth=DEFAULT_CAM_AZIMUTH
        ))

        self._init_scene()

    def _init_scene(self):
        """Construct reference grid, coordinate axes, and initial platform geometry."""
        # Ground Grid (z = 0)
        grid = gl.GLGridItem(size=QVector3D(800, 800, 1))
        grid.setSpacing(50, 50, 1)
        grid.setColor((38, 50, 56, 160))
        self.gl_widget.addItem(grid)

        # World Coordinate Axes at (0, 0, 0)
        axes = gl.GLAxisItem(size=QVector3D(80, 80, 80))
        self.gl_widget.addItem(axes)

        # Base Anchor Points
        self.base_scatter = gl.GLScatterPlotItem(
            pos=self.base_joints,
            size=10,
            color=(0.0, 0.9, 1.0, 0.9),  # Cyan
            pxMode=True
        )
        self.gl_widget.addItem(self.base_scatter)

        # Base Perimeter Polygon Ring (close loop: 0->1->2->3->4->5->0)
        base_closed = np.vstack([self.base_joints, self.base_joints[0]])
        self.base_ring = gl.GLLinePlotItem(
            pos=base_closed,
            color=(0.0, 0.6, 0.7, 0.8),
            width=2.5,
            antialias=True
        )
        self.gl_widget.addItem(self.base_ring)

        # Mobile Platform Anchor Scatter Plot (initial pose at z_home)
        init_ai = np.array([0.0, 0.0, Z_HOME]) + self.platform_joints
        self.platform_scatter = gl.GLScatterPlotItem(
            pos=init_ai,
            size=11,
            color=(0.0, 0.9, 0.46, 1.0),  # Emerald
            pxMode=True
        )
        self.gl_widget.addItem(self.platform_scatter)

        # Mobile Platform Perimeter Ring
        plat_closed = np.vstack([init_ai, init_ai[0]])
        self.platform_ring = gl.GLLinePlotItem(
            pos=plat_closed,
            color=(0.0, 0.9, 1.0, 1.0),
            width=3.0,
            antialias=True
        )
        self.gl_widget.addItem(self.platform_ring)

        # Normal vector indicator line (pointing direction)
        self.normal_ray = gl.GLLinePlotItem(
            pos=np.array([[0.0, 0.0, Z_HOME], [0.0, 0.0, Z_HOME + 90.0]]),
            color=(0.0, 0.9, 0.46, 1.0),  # Emerald ray
            width=3.5,
            antialias=True
        )
        self.gl_widget.addItem(self.normal_ray)

        # 6 Actuator Lines connecting B_i to A_i
        self.actuator_lines = []
        for i in range(6):
            pts = np.vstack([self.base_joints[i], init_ai[i]])
            line = gl.GLLinePlotItem(
                pos=pts,
                color=(0.0, 0.9, 1.0, 0.95),  # Cyan
                width=3.5,
                antialias=True
            )
            self.gl_widget.addItem(line)
            self.actuator_lines.append(line)

    def update_platform(self, da: np.ndarray, R: np.ndarray, q_mm: Union[list, np.ndarray]):
        """
        Fast GPU buffer update of the mobile platform and 6 actuators.
        """
        da = np.asarray(da, dtype=np.float64)
        R = np.asarray(R, dtype=np.float64)
        q_arr = np.asarray(q_mm, dtype=np.float64)

        # Transformed mobile platform anchor positions: A_i = da + R * P_i
        ai = da + (self.platform_joints @ R.T)

        # Update Platform Scatter
        self.platform_scatter.setData(pos=ai)

        # Update Platform Ring
        plat_closed = np.vstack([ai, ai[0]])
        self.platform_ring.setData(pos=plat_closed)

        # Update Normal Vector Ray
        normal_vec = R @ np.array([0.0, 0.0, 90.0])
        self.normal_ray.setData(pos=np.vstack([da, da + normal_vec]))

        # Update 6 Actuator lines with dynamic safety color coding
        for i in range(6):
            pts = np.vstack([self.base_joints[i], ai[i]])
            qi = q_arr[i] if i < len(q_arr) else 332.0

            # Dynamic color based on mechanical extension limits
            if qi < Q_MIN or qi > Q_MAX:
                color = (0.9, 0.1, 0.1, 1.0)  # Red warning
            elif qi < Q_SAFE_MIN or qi > Q_SAFE_MAX:
                color = (1.0, 0.7, 0.0, 1.0)  # Amber warning
            else:
                color = (0.0, 0.9, 1.0, 0.95)  # Cyan optimal

            self.actuator_lines[i].setData(pos=pts, color=color)
