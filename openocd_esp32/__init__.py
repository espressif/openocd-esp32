# SPDX-License-Identifier: GPL-2.0-or-later

"""OpenOCD for Espressif chips."""

import os
import subprocess
import sys
from pathlib import Path

try:
    from openocd_esp32._version import __version__
except ImportError:
    __version__ = "0.0.0.dev0"


def get_openocd_path() -> Path:
    """Return path to the openocd binary."""
    pkg_dir = Path(__file__).parent
    if sys.platform == "win32":
        return pkg_dir / "bin" / "openocd.exe"
    return pkg_dir / "bin" / "openocd"


def get_scripts_path() -> Path:
    """Return path to the OpenOCD scripts directory."""
    return Path(__file__).parent / "share" / "openocd" / "scripts"


def main() -> None:
    """Run openocd with the bundled scripts path.

    On POSIX, replaces the current process via os.execv and does not return.
    On Windows, runs openocd as a subprocess and exits with its return code.
    """
    openocd = get_openocd_path()
    scripts = get_scripts_path()

    if not openocd.exists():
        print(
            f"Error: OpenOCD binary not found at {openocd}.\n"
            "This installation appears to be incomplete. "
            "Reinstall the platform-specific wheel from PyPI.",
            file=sys.stderr,
        )
        sys.exit(1)

    args = [str(openocd), "-s", str(scripts), *sys.argv[1:]]

    sys.stdout.flush()
    sys.stderr.flush()

    if sys.platform == "win32":
        try:
            sys.exit(subprocess.run(args).returncode)
        except KeyboardInterrupt:
            sys.exit(130)

    os.execv(args[0], args)
