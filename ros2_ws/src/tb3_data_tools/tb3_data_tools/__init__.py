"""TurtleBot3 data collection helpers."""

from importlib.metadata import version, PackageNotFoundError

try:
    __version__ = version('tb3_data_tools')
except PackageNotFoundError:  # pragma: no cover - during local editing
    __version__ = '0.0.0'

__all__ = ['__version__']
