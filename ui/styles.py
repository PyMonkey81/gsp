"""
Industrial Dark Mode Style Sheet and Visual Tokens for PySide6 Gough-Stewart GUI.
"""

# Color Palette Constants
COLOR_BG_DARK = "#121820"
COLOR_PANEL_BG = "#1A232E"
COLOR_CARD_BG = "#151D26"
COLOR_BORDER = "#263238"
COLOR_BORDER_LIGHT = "#37474F"

COLOR_CYAN = "#00E5FF"
COLOR_CYAN_DIM = "#0097A7"
COLOR_EMERALD = "#00E676"
COLOR_EMERALD_DIM = "#00A854"
COLOR_EMERGENCY = "#E53935"
COLOR_EMERGENCY_HOVER = "#FF1744"
COLOR_WARNING = "#FFB300"

COLOR_TEXT_PRIMARY = "#ECEFF1"
COLOR_TEXT_SECONDARY = "#90A4AE"
COLOR_TEXT_MUTED = "#546E7A"

FONT_MAIN = "'Segoe UI', 'Ubuntu', 'Roboto', 'Inter', 'DejaVu Sans', -apple-system, sans-serif"
FONT_MONO = "'Consolas', 'Ubuntu Mono', 'Cascadia Code', 'Fira Code', 'DejaVu Sans Mono', 'Courier New', monospace"

INDUSTRIAL_DARK_STYLESHEET = f"""
/* =========================================================================
   GLOBAL RESET & MAIN WINDOW
   ========================================================================= */
QMainWindow, QWidget {{
    background-color: {COLOR_BG_DARK};
    color: {COLOR_TEXT_PRIMARY};
    font-family: {FONT_MAIN};
    font-size: 12px;
    selection-background-color: {COLOR_CYAN_DIM};
    selection-color: #FFFFFF;
}}

QWidget:focus {{
    outline: none;
}}

/* =========================================================================
   FRAMES, PANELS & CARDS
   ========================================================================= */
QFrame#TopBar {{
    background-color: {COLOR_PANEL_BG};
    border-bottom: 2px solid {COLOR_BORDER};
    padding: 6px 12px;
}}

QFrame#PanelCard {{
    background-color: {COLOR_PANEL_BG};
    border: 1px solid {COLOR_BORDER};
    border-radius: 6px;
}}

QGroupBox {{
    background-color: {COLOR_PANEL_BG};
    border: 1px solid {COLOR_BORDER};
    border-radius: 6px;
    margin-top: 22px;
    padding: 12px 8px 8px 8px;
    font-weight: 600;
    font-size: 11px;
    color: {COLOR_CYAN};
    text-transform: uppercase;
    letter-spacing: 0.5px;
}}

QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 14px;
    padding: 2px 8px;
    background-color: {COLOR_CARD_BG};
    border: 1px solid {COLOR_BORDER};
    border-radius: 4px;
}}

/* =========================================================================
   BUTTONS
   ========================================================================= */
QPushButton {{
    background-color: {COLOR_CARD_BG};
    border: 1px solid {COLOR_BORDER_LIGHT};
    border-radius: 4px;
    color: {COLOR_TEXT_PRIMARY};
    padding: 6px 14px;
    font-weight: 600;
    font-size: 11px;
    min-height: 20px;
}}

QPushButton:hover {{
    background-color: #212C3B;
    border-color: {COLOR_CYAN};
    color: {COLOR_CYAN};
}}

QPushButton:pressed {{
    background-color: #111822;
    border-color: {COLOR_CYAN_DIM};
}}

QPushButton:disabled {{
    background-color: #161D24;
    border-color: #1C262E;
    color: {COLOR_TEXT_MUTED};
}}

/* Primary Action Button (Connect, Start, etc.) */
QPushButton#PrimaryButton {{
    background-color: #0B3A42;
    border: 1px solid {COLOR_CYAN};
    color: {COLOR_CYAN};
}}

QPushButton#PrimaryButton:hover {{
    background-color: #0F515C;
    color: #FFFFFF;
}}

/* Success Emerald Button */
QPushButton#SuccessButton {{
    background-color: #0C3823;
    border: 1px solid {COLOR_EMERALD};
    color: {COLOR_EMERALD};
}}

QPushButton#SuccessButton:hover {{
    background-color: #135A38;
    color: #FFFFFF;
}}

/* Emergency Stop Button */
QPushButton#EmergencyButton {{
    background-color: {COLOR_EMERGENCY};
    border: 2px solid #FF5252;
    border-radius: 5px;
    color: #FFFFFF;
    font-size: 13px;
    font-weight: 800;
    letter-spacing: 1px;
    padding: 8px 18px;
}}

QPushButton#EmergencyButton:hover {{
    background-color: {COLOR_EMERGENCY_HOVER};
    border-color: #FF8A80;
}}

QPushButton#EmergencyButton:pressed {{
    background-color: #B71C1C;
}}

/* Warning / Pause Button */
QPushButton#WarningButton {{
    background-color: #382A0C;
    border: 1px solid {COLOR_WARNING};
    color: {COLOR_WARNING};
}}

QPushButton#WarningButton:hover {{
    background-color: #574112;
    color: #FFFFFF;
}}

/* =========================================================================
   INPUTS: SPINBOXES, COMBOBOXES, LINEEDITS
   ========================================================================= */
QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {{
    background-color: {COLOR_CARD_BG};
    border: 1px solid {COLOR_BORDER};
    border-radius: 4px;
    color: {COLOR_TEXT_PRIMARY};
    padding: 4px 8px;
    min-height: 22px;
}}

QComboBox:hover, QSpinBox:hover, QDoubleSpinBox:hover, QLineEdit:hover {{
    border-color: {COLOR_BORDER_LIGHT};
}}

QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus, QLineEdit:focus {{
    border-color: {COLOR_CYAN};
}}

QComboBox::drop-down {{
    subcontrol-origin: padding;
    subcontrol-position: top right;
    width: 20px;
    border-left: 1px solid {COLOR_BORDER};
}}

QComboBox QAbstractItemView {{
    background-color: {COLOR_PANEL_BG};
    border: 1px solid {COLOR_BORDER_LIGHT};
    selection-background-color: #1E3040;
    selection-color: {COLOR_CYAN};
    color: {COLOR_TEXT_PRIMARY};
    padding: 4px;
}}

/* =========================================================================
   SLIDERS (Cyan Industrial Rail)
   ========================================================================= */
QSlider::groove:horizontal {{
    height: 6px;
    background: #10161E;
    border: 1px solid {COLOR_BORDER};
    border-radius: 3px;
}}

QSlider::sub-page:horizontal {{
    background: {COLOR_CYAN_DIM};
    border-radius: 3px;
}}

QSlider::handle:horizontal {{
    background: qradialgradient(cx:0.5, cy:0.5, radius:0.5, fx:0.5, fy:0.5, stop:0 {COLOR_CYAN}, stop:1 #00838F);
    border: 1px solid #FFFFFF;
    width: 14px;
    margin-top: -5px;
    margin-bottom: -5px;
    border-radius: 7px;
}}

QSlider::handle:horizontal:hover {{
    background: #E0F7FA;
    border-color: {COLOR_CYAN};
}}

/* =========================================================================
   TABS
   ========================================================================= */
QTabWidget::pane {{
    border: 1px solid {COLOR_BORDER};
    background-color: {COLOR_PANEL_BG};
    border-radius: 0 0 6px 6px;
    top: -1px;
}}

QTabBar::tab {{
    background-color: {COLOR_CARD_BG};
    border: 1px solid {COLOR_BORDER};
    border-bottom: none;
    border-top-left-radius: 5px;
    border-top-right-radius: 5px;
    padding: 8px 16px;
    margin-right: 2px;
    color: {COLOR_TEXT_SECONDARY};
    font-weight: 600;
    font-size: 11px;
}}

QTabBar::tab:selected {{
    background-color: {COLOR_PANEL_BG};
    color: {COLOR_CYAN};
    border-top: 2px solid {COLOR_CYAN};
}}

QTabBar::tab:hover:!selected {{
    background-color: #1E2835;
    color: {COLOR_TEXT_PRIMARY};
}}

/* =========================================================================
   PROGRESS BARS
   ========================================================================= */
QProgressBar {{
    border: 1px solid {COLOR_BORDER};
    border-radius: 4px;
    background-color: {COLOR_CARD_BG};
    text-align: center;
    color: #FFFFFF;
    font-weight: 600;
    font-size: 10px;
}}

QProgressBar::chunk {{
    background-color: {COLOR_CYAN};
    border-radius: 3px;
}}

/* =========================================================================
   TERMINAL / TEXT EDIT
   ========================================================================= */
QTextEdit#ConsoleLog {{
    background-color: #0E1318;
    border: 1px solid {COLOR_BORDER};
    border-radius: 4px;
    color: #80D8FF;
    font-family: {FONT_MONO};
    font-size: 11px;
    line-height: 1.4;
    padding: 6px;
}}

QLabel#MatrixDisplay {{
    background-color: #0E1318;
    border: 1px solid {COLOR_BORDER};
    border-radius: 4px;
    color: #B0BEC5;
    font-family: {FONT_MONO};
    font-size: 11px;
    padding: 6px;
}}

/* =========================================================================
   STATUS BAR & SCROLLBARS
   ========================================================================= */
QStatusBar {{
    background-color: {COLOR_PANEL_BG};
    border-top: 1px solid {COLOR_BORDER};
    color: {COLOR_TEXT_SECONDARY};
    font-size: 11px;
}}

QScrollBar:vertical {{
    background: {COLOR_BG_DARK};
    width: 8px;
    margin: 0;
}}

QScrollBar::handle:vertical {{
    background: {COLOR_BORDER_LIGHT};
    min-height: 20px;
    border-radius: 4px;
}}

QScrollBar::handle:vertical:hover {{
    background: {COLOR_CYAN_DIM};
}}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0px;
}}

QScrollBar:horizontal {{
    background: {COLOR_BG_DARK};
    height: 8px;
    margin: 0;
}}

QScrollBar::handle:horizontal {{
    background: {COLOR_BORDER_LIGHT};
    min-width: 20px;
    border-radius: 4px;
}}

QScrollBar::handle:horizontal:hover {{
    background: {COLOR_CYAN_DIM};
}}

QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
    width: 0px;
}}

QSplitter::handle {{
    background-color: {COLOR_BORDER};
}}
"""
