"""
Dialogs package for Gough-Stewart Platform GUI.
"""

from ui.dialogs.parameters_dialog import ParametersDialog, load_parameters, save_parameters
from ui.dialogs.serial_config_dialog import SerialConfigDialog

__all__ = ["ParametersDialog", "SerialConfigDialog", "load_parameters", "save_parameters"]
