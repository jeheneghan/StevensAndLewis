"""model package for the StevensAndLewis project.

Auto-discovers submodules for convenient imports and exposes basic package
metadata and a safe logger for library use.
"""

# auto-discover submodules (populates __all__)
import pkgutil

__all__ = [name for _, name, _ in pkgutil.iter_modules(__path__)]

# package metadata
__version__ = "0.0.0"

# safe logger for libraries
import logging
logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler())
