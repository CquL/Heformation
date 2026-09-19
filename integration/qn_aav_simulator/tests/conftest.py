"""Import the source package without requiring a ROS installation.

``mrta_python`` lives beside this package in ``integration/`` and is a path-based
package, so the same conftest makes it importable: the task line joins the two.
"""

from pathlib import Path
import sys

_HERE = Path(__file__).resolve()
sys.path.insert(0, str(_HERE.parents[1] / "src"))
sys.path.insert(0, str(_HERE.parents[2]))
